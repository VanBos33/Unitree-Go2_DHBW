#!/usr/bin/env python3
"""
SDK-Seite der Fahr-Brücke: UDP (localhost:9870) -> Unitree Sport-API

Befehle (JSON per UDP):
  {"x": .., "y": .., "z": ..}   -> Move (laufen)            [von go2_cmd_vel.py]
  {"stop": true}                -> StopMove
  {"action": "sit"}             -> hinsetzen                [von go2_mission.py]
  {"action": "stand"}           -> aufstehen + Lauf-Modus
  Auf "action" kommt eine Antwort an den Absender: {"ack": "sit", "ok": true, "info": ".."}

- OHNE go2_env.sh starten (neues Terminal, nur ssh), sonst stört die ROS-Umgebung das SDK.
- Watchdog: 0,5 s kein Befehl -> StopMove. Gilt auch, wenn die ROS-Seite abstürzt.
- Solange der Hund sitzt, werden alle Fahrbefehle ignoriert.

Start:
  python3 ~/unitree_ws/src/go2_bridge/go2_sport_sender.py
  python3 ~/unitree_ws/src/go2_bridge/go2_sport_sender.py --sit-mode standdown   # hinlegen statt sitzen
"""
import argparse
import json
import os
import socket
import time

for k in ("CYCLONEDDS_URI",):   # ROS-Einstellungen nicht ans SDK weiterreichen
    os.environ.pop(k, None)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

PORT = 9870
TIMEOUT = 0.5
RATE = 20.0


def call(sport, name):
    """SDK-Funktion aufrufen, falls die Firmware/SDK-Version sie kennt."""
    fn = getattr(sport, name, None)
    if fn is None:
        raise RuntimeError(f"SportClient kennt '{name}' nicht")
    return fn()


def do_sit(sport, mode):
    sport.StopMove()
    time.sleep(0.3)
    if mode == "sit":
        r = call(sport, "Sit")
        time.sleep(2.5)
        return r == 0, f"Sit() Rückgabe {r}"
    r = call(sport, "StandDown")
    time.sleep(2.5)
    return r == 0, f"StandDown() Rückgabe {r}"


def do_stand(sport, mode):
    if mode == "sit":
        r = call(sport, "RiseSit")
        time.sleep(2.0)
    else:
        r = call(sport, "StandUp")
        time.sleep(2.0)
    # In den Lauf-Modus wechseln, sonst wird Move nur zur Körperneigung
    r2 = sport.BalanceStand()
    time.sleep(1.0)
    # RiseSit()/StandUp() melden -1, wenn der Hund schon steht. Entscheidend ist,
    # dass BalanceStand klappt – dann steht er sicher im Lauf-Modus.
    ok = (r2 == 0)
    return ok, f"{'RiseSit' if mode == 'sit' else 'StandUp'}() Rückgabe {r}, BalanceStand() Rückgabe {r2}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sit-mode", choices=["sit", "standdown"], default="sit",
                    help="sit = Sit()/RiseSit(), standdown = hinlegen mit StandDown()/StandUp()")
    args = ap.parse_args()

    ChannelFactoryInitialize(0)          # "auto" – wie beim Kamera-Stream
    sport = SportClient()
    sport.SetTimeout(3.0)
    sport.Init()
    try:
        r = sport.BalanceStand()
        print("BalanceStand gesendet (Rückgabe %s)" % (r,), flush=True)
        time.sleep(1.0)
    except Exception as e:
        print("BalanceStand fehlgeschlagen:", e, flush=True)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", PORT))
    sock.settimeout(1.0 / RATE)
    print(f"SportClient bereit. Warte auf Befehle (UDP {PORT}), Watchdog {TIMEOUT} s, "
          f"Sitz-Modus '{args.sit_mode}'.", flush=True)

    cmd = {"x": 0.0, "y": 0.0, "z": 0.0}
    last = 0.0
    moving = False
    sitting = False
    n = 0
    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                msg = json.loads(data)
                action = msg.get("action")
                if action in ("sit", "stand"):
                    if moving:
                        sport.StopMove()
                        moving = False
                    try:
                        if action == "sit":
                            ok, info = do_sit(sport, args.sit_mode)
                            sitting = sitting or ok
                        else:
                            ok, info = do_stand(sport, args.sit_mode)
                            if ok:
                                sitting = False
                    except Exception as e:
                        ok, info = False, str(e)
                    print(f"Aktion '{action}': {'OK' if ok else 'FEHLER'} ({info})", flush=True)
                    sock.sendto(json.dumps({"ack": action, "ok": ok, "info": info}).encode(), addr)
                    cmd = {"x": 0.0, "y": 0.0, "z": 0.0}
                    last = 0.0
                elif msg.get("stop"):
                    last = 0.0
                else:
                    cmd = msg
                    last = time.time()
                    n += 1
                    if n == 1:
                        print("Erster Befehl empfangen:", cmd, flush=True)
            except socket.timeout:
                pass
            alive = (time.time() - last) < TIMEOUT
            zero = cmd.get("x", 0.0) == 0.0 and cmd.get("y", 0.0) == 0.0 and cmd.get("z", 0.0) == 0.0
            if sitting:
                pass                      # im Sitzen nie laufen
            elif alive and not zero:
                sport.Move(cmd["x"], cmd["y"], cmd["z"])
                moving = True
            elif moving:
                sport.StopMove()
                moving = False
                print("Stopp.", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        sport.StopMove()
        print("Beendet -> StopMove gesendet.", flush=True)


if __name__ == "__main__":
    main()
