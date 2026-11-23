#!/usr/bin/env python3
"""
Test: Hinsetzen und Aufstehen über go2_sport_sender.py (der muss laufen!).

  python3 ~/unitree_ws/src/go2_bridge/go2_sit_test.py          # sitzen, 5 s warten, aufstehen
  python3 ~/unitree_ws/src/go2_bridge/go2_sit_test.py sit      # nur hinsetzen
  python3 ~/unitree_ws/src/go2_bridge/go2_sit_test.py stand    # nur aufstehen
"""
import json
import socket
import sys
import time

PORT = 9870


def send(sock, action):
    print(f"-> {action} ...", flush=True)
    sock.sendto(json.dumps({"action": action}).encode(), ("127.0.0.1", PORT))
    try:
        data, _ = sock.recvfrom(1024)
        ack = json.loads(data)
        print(f"<- {'OK' if ack.get('ok') else 'FEHLER'}: {ack.get('info')}", flush=True)
        return ack.get("ok", False)
    except socket.timeout:
        print("<- keine Antwort. Läuft go2_sport_sender.py?", flush=True)
        return False


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(10.0)
    what = sys.argv[1] if len(sys.argv) > 1 else "both"
    if what in ("sit", "both"):
        send(sock, "sit")
    if what == "both":
        time.sleep(5.0)
    if what in ("stand", "both"):
        send(sock, "stand")


if __name__ == "__main__":
    main()
