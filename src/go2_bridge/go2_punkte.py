#!/usr/bin/env python3
"""
Feste Punkte (z. B. vor Fenstern) speichern und anfahren.

  speichern NAME   Hund per Fernbedienung vors Fenster stellen und ausrichten, dann aufrufen:
                   aktuelle Position + Blickrichtung (TF map -> base_link) wird als NAME gespeichert.
                   Braucht laufende Bridge + Nav2 und eine gesetzte Startpose.
  fahre NAME       Punkt anfahren (über die Mission, Port 8081): hinlaufen, ausrichten, hinsetzen
  liste            alle gespeicherten Punkte anzeigen
  loeschen NAME    Punkt entfernen

Datei: maps/raum3_punkte.yaml (kommt mit ins Git, damit alle dieselben Punkte haben)

Kurzform über go2.sh:  go2 punkt speichern fenster_1 | go2 fahre fenster_1 | go2 punkte
"""
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request

import yaml

HIER = os.path.dirname(os.path.abspath(__file__))
DATEI = os.environ.get("GO2_PUNKTE", os.path.join(HIER, "maps", "raum3_punkte.yaml"))
MISSION = os.environ.get("GO2_MISSION_URL", "http://127.0.0.1:8081")


def laden():
    try:
        with open(DATEI) as f:
            return yaml.safe_load(f) or {}
    except FileNotFoundError:
        return {}


def schreiben(punkte):
    os.makedirs(os.path.dirname(DATEI), exist_ok=True)
    with open(DATEI, "w") as f:
        f.write("# Feste Ziele für go2_mission.py (x/y in m im Frame map, yaw_deg = Blickrichtung)\n")
        yaml.safe_dump(punkte, f, sort_keys=True, allow_unicode=True)


def aktuelle_pose(timeout=5.0):
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    import tf2_ros

    rclpy.init()
    node = Node("go2_punkte")
    buf = tf2_ros.Buffer()
    tfl = tf2_ros.TransformListener(buf, node)  # noqa: F841 (muss am Leben bleiben)
    ende = time.time() + timeout
    try:
        while time.time() < ende:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                tf = buf.lookup_transform("map", "base_link", Time())
                q = tf.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                return tf.transform.translation.x, tf.transform.translation.y, math.degrees(yaw)
            except Exception:
                pass
        return None
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def cmd_speichern(name):
    pose = aktuelle_pose()
    if pose is None:
        print("❌ Keine Position (TF map -> base_link). Laufen Bridge und Nav2? Startpose in Foxglove gesetzt?")
        return 1
    x, y, yaw = pose
    punkte = laden()
    alt = name in punkte
    punkte[name] = {"x": round(x, 3), "y": round(y, 3), "yaw_deg": round(yaw, 1),
                    "gespeichert": time.strftime("%Y-%m-%d %H:%M")}
    schreiben(punkte)
    print(f"✅ {'Aktualisiert' if alt else 'Gespeichert'}: {name}  x={x:.2f} m  y={y:.2f} m  Blick {yaw:.0f}°")
    print(f"   Datei: {DATEI}")
    return 0


def cmd_fahre(name):
    url = f"{MISSION}/fahre?name={urllib.request.quote(name)}"
    try:
        with urllib.request.urlopen(url, timeout=3) as r:
            antwort = json.loads(r.read())
        print(f"✅ Fahre zu {antwort.get('fahre')} – Ablauf im Tab 6:Mission")
        return 0
    except urllib.error.HTTPError as e:
        info = json.loads(e.read() or b"{}")
        print(f"❌ {info.get('fehler', e)}. Vorhanden: {', '.join(info.get('punkte', [])) or 'keine'}")
    except Exception as e:
        print(f"❌ Mission nicht erreichbar ({e.__class__.__name__}). Läuft go2_mission.py?")
    return 1


def cmd_liste():
    punkte = laden()
    if not punkte:
        print(f"Noch keine Punkte gespeichert ({DATEI}).")
        print("Hund vors Fenster stellen, dann: go2 punkt speichern fenster_1")
        return 0
    print(f"{'Name':<16}{'x [m]':>8}{'y [m]':>8}{'Blick':>8}   gespeichert")
    for name, p in sorted(punkte.items()):
        print(f"{name:<16}{p['x']:>8.2f}{p['y']:>8.2f}{p['yaw_deg']:>7.0f}°   {p.get('gespeichert', '')}")
    return 0


def cmd_loeschen(name):
    punkte = laden()
    if punkte.pop(name, None) is None:
        print(f"❌ '{name}' gibt es nicht.")
        return 1
    schreiben(punkte)
    print(f"✅ '{name}' gelöscht.")
    return 0


def main():
    a = sys.argv[1:]
    if not a or a[0] in ("-h", "--help", "hilfe"):
        print(__doc__)
        return 0
    if a[0] == "liste":
        return cmd_liste()
    if len(a) < 2:
        print(f"Name fehlt, z. B.: {a[0]} fenster_1")
        return 1
    befehl = {"speichern": cmd_speichern, "fahre": cmd_fahre, "loeschen": cmd_loeschen}.get(a[0])
    if befehl is None:
        print(f"Unbekannt: {a[0]}")
        print(__doc__)
        return 1
    return befehl(a[1])


if __name__ == "__main__":
    sys.exit(main())
