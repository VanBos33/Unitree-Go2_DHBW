# maps – gespeicherte SLAM-Karten

Alle Karten wurden mit `slam_toolbox` (Einstellungen `../slam_go2.yaml`) und dem dichten Scan aus `go2_scan_accum.py` aufgenommen. Die Auflösung beträgt **0,05 m pro Pixel**.

| Karte | Dateien | Ursprung (`origin`) | Verwendung |
|---|---|---|---|
| `raum1` | `raum1.pgm`, `raum1.yaml`, `raum1_slam.data`, `raum1_slam.posegraph` | `[-8.398, -4.061, 0]` | Erste Karte vom 02.11.2026: Klassenzimmer, ca. 9 × 8 m, geschlossene Wände, vier Tischreihen als Punktreihen, Tür/Nische unten rechts |
| `raum2` | `raum2.pgm`, `raum2.yaml`, `raum2_slam.data`, `raum2_slam.posegraph` | `[-1.961, -9.816, 0]` | Zweite Aufnahme |
| `raum3` | `raum3.pgm`, `raum3.yaml` | `[-8.398, -4.061, 0]` | **Standardkarte für Nav2** (`go2_nav2.launch.py`): Klassenzimmer mit vier Tischreihen |

## Dateitypen

| Datei | Inhalt | Wofür |
|---|---|---|
| `*.pgm` | Belegungsbild (weiß = frei, schwarz = belegt, grau = unbekannt) | Nav2-Map-Server |
| `*.yaml` | Metadaten: Bild, Auflösung, Ursprung, Schwellwerte (`occupied_thresh 0.65`, `free_thresh 0.196`, `mode: trinary`) | Nav2-Map-Server |
| `*_slam.data`, `*_slam.posegraph` | Serialisierter Posen-Graph von slam_toolbox | Weiterkartieren oder Lokalisieren mit slam_toolbox |

## Karte für Nav2 wählen

```bash
source ~/unitree_ws/src/go2_bridge/go2_env.sh
ros2 launch ~/unitree_ws/src/go2_bridge/go2_nav2.launch.py map:=$HOME/unitree_ws/src/go2_bridge/maps/raum1.yaml
```

Ohne `map:=` wird `raum3.yaml` geladen.

## Neue Karte aufnehmen

1. Bridge mit dichtem Scan starten:
   ```bash
   source ~/unitree_ws/src/go2_bridge/go2_env.sh
   ros2 launch ~/unitree_ws/src/go2_bridge/go2_bridge.launch.py accum_window:=0.6
   ```
2. SLAM starten (zweites Fenster):
   ```bash
   source ~/unitree_ws/src/go2_bridge/go2_env.sh
   ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$HOME/unitree_ws/src/go2_bridge/slam_go2.yaml
   ```
3. Foxglove-Bridge starten und in der Foxglove-App `/map` und `/scan` einblenden (Display frame `map`).
4. Den Hund per Fernbedienung **langsam** fahren und **sehr langsam drehen**: erst an den Wänden entlang, zum Startpunkt zurück (Schleife schließen), dann durch die Gänge.
5. Speichern, **solange SLAM noch läuft** (drittes Fenster):
   ```bash
   source ~/unitree_ws/src/go2_bridge/go2_env.sh
   ros2 run nav2_map_server map_saver_cli -f ~/unitree_ws/src/go2_bridge/maps/raum4
   ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
     "{filename: '$HOME/unitree_ws/src/go2_bridge/maps/raum4_slam'}"
   ls ~/unitree_ws/src/go2_bridge/maps/
   ```

## Karte auf den Mac holen

Im Mac-Terminal, ohne `ssh`. Die Anführungszeichen sind wegen des `*` nötig:

```bash
scp "fynn@100.121.161.1:~/unitree_ws/src/go2_bridge/maps/raum3.*" ~/Desktop/
```

## Hinweise

- Eine Karte, die aussieht wie ein „ausgeleerter Farbeimer“ (strahlenförmige weiße Fläche, kaum Wände), entsteht ohne `go2_scan_accum.py` oder bei zu schnellem Drehen.
- Der Scan erfasst nur Hindernisse **10–40 cm über dem Boden**. Tischplatten erscheinen deshalb nur als Punktreihen (Tischbeine), nicht als Fläche.
