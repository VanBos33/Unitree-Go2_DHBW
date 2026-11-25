#!/usr/bin/env bash
# =============================================================================
#  go2 – startet, überwacht und beendet alle Programme für den Go2 in EINER
#  tmux-Sitzung "go2" (ein Fenster mit Tabs statt vieler Terminal-Fenster).
#
#  go2 start          alles in der richtigen Reihenfolge starten
#  go2 oeffnen        Sitzung anzeigen (startet vorher alles, falls nötig)
#  go2 status         Übersicht: was läuft, was nicht
#  go2 stop           alles sauber beenden (hängende Programme werden erzwungen)
#  go2 restart        stop + start (z. B. nachdem der Hund aus war)
#  go2 neu <teil>     ein Programm neu starten (bridge, sport, cmd_vel, nav2,
#                     foxglove, mission, kamera, dashboard)
#  go2 reset          nach dem Setzen der Startpose: Costmaps leeren
#  go2 abbruch        laufende Fahrt abbrechen
#  go2 sitz / steh    Hund von Hand hinsetzen / aufstehen lassen
#  go2 punkt speichern fenster_1   aktuelle Position + Blickrichtung als Punkt speichern
#  go2 punkt loeschen fenster_1    Punkt entfernen
#  go2 punkte         gespeicherte Punkte anzeigen
#  go2 fahre fenster_1             Punkt anfahren, ausrichten, hinsetzen (+ Kamera-KI am Mac)
#  go2 hilfe          diese Hilfe
#
#  Dashboard im Browser am Mac:  http://100.121.161.1:8082   (Tab 9:Dashboard)
#
#  In der Sitzung: unten auf die Tabs klicken. Fenster schließen/ssh weg ->
#  alles läuft auf dem Pi weiter; "go2 oeffnen" bringt dich zurück.
# =============================================================================

SESSION="go2"
WS="${GO2_WS:-$HOME/unitree_ws}"
BR="${GO2_BRIDGE_DIR:-$WS/src/go2_bridge}"
YOLO="${GO2_YOLO_DIR:-$WS/src/yolo_zeugs}"
HUND_IP="${GO2_HUND_IP:-192.168.123.161}"
ENV_SH="$BR/go2_env.sh"
SELF="$BR/go2.sh"

# --- Programme: Name | braucht go2_env.sh | Ordner | Befehl | "fertig"-Muster | Zeitlimit (s)
NAMEN=(Bridge Sport cmd_vel Nav2 Foxglove Mission Kamera Dashboard)
declare -A MIT_ENV=(  [Bridge]=1 [Sport]=0 [cmd_vel]=1 [Nav2]=1 [Foxglove]=1 [Mission]=1 [Kamera]=0 [Dashboard]=1 )
declare -A ORDNER=(   [Bridge]="$BR" [Sport]="$BR" [cmd_vel]="$BR" [Nav2]="$BR" [Foxglove]="$BR" [Mission]="$BR" [Kamera]="$YOLO" [Dashboard]="$BR" )
declare -A BEFEHL=(
  [Bridge]="ros2 launch $BR/go2_bridge.launch.py accum_window:=0.25"
  [Sport]="python3 $BR/go2_sport_sender.py"
  [cmd_vel]="python3 $BR/go2_cmd_vel.py"
  [Nav2]="ros2 launch $BR/go2_nav2.launch.py"
  [Foxglove]="ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
  [Mission]="python3 $BR/go2_mission.py"
  [Kamera]="python3 $YOLO/go2_tuer_stream.py auto --ohne-ki"
  [Dashboard]="python3 $BR/go2_dashboard_server.py"
)
declare -A FERTIG=(
  [Bridge]="Strahlen mit Treffer"
  [Sport]="SportClient bereit"
  [cmd_vel]="Bereit: /cmd_vel"
  [Nav2]="Managed nodes are active"
  [Foxglove]="[Ll]istening|8765"
  [Mission]="Bereit\. Ziele"
  [Kamera]="8080"
  [Dashboard]="Dashboard bereit"
)
declare -A ZEIT=( [Bridge]=40 [Sport]=30 [cmd_vel]=20 [Nav2]=120 [Foxglove]=30 [Mission]=20 [Kamera]=40 [Dashboard]=30 )
# Sport-Sender und Kamera nutzen das Unitree-SDK -> NIEMALS mit go2_env.sh (CycloneDDS-Konflikt)

GRUEN=$'\e[32m'; ROT=$'\e[31m'; GELB=$'\e[33m'; FETT=$'\e[1m'; AUS=$'\e[0m'

hilfe() { awk 'NR>2 && /^# ====/ {exit} NR>2' "$SELF" | sed 's/^#  \{0,1\}//; s/^#$//'; }

sitzung_da() { tmux has-session -t "$SESSION" 2>/dev/null; }

# tmux mit sauberer Umgebung starten: keine ROS-Variablen aus der aufrufenden Shell erben
sitzung_anlegen() {
  env -i HOME="$HOME" USER="$USER" LOGNAME="${LOGNAME:-$USER}" SHELL=/bin/bash \
      LANG="${LANG:-C.UTF-8}" TERM="${TERM:-xterm-256color}" \
      PATH="$HOME/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
      tmux new-session -d -s "$SESSION" -n Status -x 200 -y 50
  tmux set -t "$SESSION" mouse on
  tmux set -t "$SESSION" history-limit 20000
  tmux set -t "$SESSION" status-position bottom
  tmux set -t "$SESSION" status-style "bg=colour236,fg=colour250"
  tmux set -t "$SESSION" status-left "#[bg=colour31,fg=colour231,bold] Go2 #[default] "
  tmux set -t "$SESSION" status-left-length 10
  tmux set -t "$SESSION" status-right "#[fg=colour245]Tab anklicken · Strg+b d = verlassen (läuft weiter) "
  tmux set -t "$SESSION" status-right-length 60
  tmux set -t "$SESSION" window-status-format " #I:#W "
  tmux set -t "$SESSION" window-status-current-format "#[bg=colour31,fg=colour231,bold] #I:#W "
  tmux set -t "$SESSION" renumber-windows off
  tmux set -t "$SESSION" remain-on-exit off
  tmux send-keys -t "$SESSION:Status" "while true; do clear; bash '$SELF' status; sleep 3; done" Enter
}

fenster_da() { tmux list-windows -t "$SESSION" -F '#W' 2>/dev/null | grep -qx "$1"; }
pane_pid()   { tmux display-message -p -t "$SESSION:$1" '#{pane_pid}' 2>/dev/null; }
laeuft()     { local p; p=$(pane_pid "$1") || return 1; [ -n "$p" ] && pgrep -P "$p" >/dev/null 2>&1; }
ausgabe()    { tmux capture-pane -p -J -S -3000 -t "$SESSION:$1" 2>/dev/null; }

nummer() { [ "$1" = Dashboard ] && { echo 9; return; }   # Tab 8 bleibt "Befehle"
  local i=1; for n in "${NAMEN[@]}"; do [ "$n" = "$1" ] && { echo $i; return; }; i=$((i+1)); done; }

fenster_anlegen() {
  local name=$1 nr; nr=$(nummer "$name")
  fenster_da "$name" || tmux new-window -d -t "$SESSION:$nr" -n "$name"
}

befehle_fenster() {
  fenster_da Befehle && return
  tmux new-window -d -t "$SESSION:8" -n Befehle
  tmux send-keys -t "$SESSION:Befehle" "source '$ENV_SH'; cd '$BR'; clear; bash '$SELF' hilfe" Enter
}

starte_teil() {
  local name=$1
  fenster_anlegen "$name"
  if laeuft "$name"; then echo "  $name läuft schon"; return 0; fi
  tmux send-keys -t "$SESSION:$name" "clear" Enter
  sleep 0.3
  tmux clear-history -t "$SESSION:$name"
  local cmd="cd '${ORDNER[$name]}' && ${BEFEHL[$name]}"
  [ "${MIT_ENV[$name]}" = 1 ] && cmd="source '$ENV_SH' && $cmd"
  tmux send-keys -t "$SESSION:$name" "$cmd" Enter
  printf '  %-9s startet ' "$name"
  local t=0 limit=${ZEIT[$name]}
  while [ $t -lt $((limit*2)) ]; do
    if ausgabe "$name" | grep -Eq "${FERTIG[$name]}"; then
      echo "${GRUEN}✅ bereit${AUS}"; return 0
    fi
    if [ $t -gt 4 ] && ! laeuft "$name"; then
      echo "${ROT}❌ beendet – Tab '$name' ansehen${AUS}"; return 1
    fi
    sleep 0.5; t=$((t+1))
    [ $((t % 4)) -eq 0 ] && printf '.'
  done
  echo "${GELB}⚠️  noch keine Bereit-Meldung nach ${limit}s – Tab '$name' ansehen${AUS}"
  return 1
}

prozessbaum() { local c; for c in $(pgrep -P "$1"); do prozessbaum "$c"; done; echo "$1"; }

stoppe_teil() {
  local name=$1 p
  fenster_da "$name" || return 0
  laeuft "$name" || return 0
  p=$(pane_pid "$name")
  printf '  %-9s wird beendet ' "$name"
  tmux send-keys -t "$SESSION:$name" C-c
  local t=0
  while [ $t -lt 30 ] && pgrep -P "$p" >/dev/null 2>&1; do sleep 0.5; t=$((t+1)); [ $((t % 4)) -eq 0 ] && printf '.'; done
  if pgrep -P "$p" >/dev/null 2>&1; then
    local pids="" c
    for c in $(pgrep -P "$p"); do pids="$pids $(prozessbaum "$c")"; done
    # shellcheck disable=SC2086
    kill -9 $pids 2>/dev/null
    echo "${GELB}erzwungen${AUS}"
  else
    echo "${GRUEN}ok${AUS}"
  fi
}

hund_erreichbar() { [ "${GO2_OHNE_PING:-0}" = 1 ] || ping -c 1 -W 2 "$HUND_IP" >/dev/null 2>&1; }

cmd_start() {
  echo "${FETT}Go2 starten${AUS}"
  if ! hund_erreichbar; then
    echo "${ROT}❌ Hund ($HUND_IP) antwortet nicht auf ping.${AUS}"
    echo "   Hund eingeschaltet und aufgestanden? LAN-Kabel steckt? Danach erneut: go2 start"
    return 1
  fi
  echo "  Hund      ${GRUEN}✅ erreichbar${AUS}"
  sitzung_da || sitzung_anlegen
  local n
  for n in "${NAMEN[@]}"; do starte_teil "$n"; done
  befehle_fenster
  tmux select-window -t "$SESSION:Status"
  echo
  echo "Fertig. Dashboard: ${FETT}http://100.121.161.1:8082${AUS} (oder Foxglove)."
  echo "Startpose setzen (Dashboard: Rechtsklick → „Hund steht hier“), dann ${FETT}go2 reset${AUS}, dann Ziel."
}

cmd_stop() {
  sitzung_da || { echo "Keine go2-Sitzung aktiv."; return 0; }
  echo "${FETT}Go2 beenden${AUS}"
  local i
  for (( i=${#NAMEN[@]}-1; i>=0; i-- )); do stoppe_teil "${NAMEN[$i]}"; done
  tmux kill-session -t "$SESSION" 2>/dev/null
  echo "Alles beendet."
}

cmd_status() {
  local jetzt; jetzt=$(date +%H:%M:%S)
  echo "${FETT}Go2 – Status ($jetzt)${AUS}"
  echo "────────────────────────────────────────────────"
  local ms
  ms=$(ping -c 1 -W 1 "$HUND_IP" 2>/dev/null | sed -n 's/.*time=\([0-9.]*\).*/\1/p')
  if [ -n "$ms" ]; then printf '  %-10s %s✅ erreichbar (%s ms)%s\n' "Hund" "$GRUEN" "$ms" "$AUS"
  else printf '  %-10s %s❌ nicht erreichbar%s\n' "Hund" "$ROT" "$AUS"; fi
  if ! sitzung_da; then echo "  Programme  ${ROT}❌ keine Sitzung – go2 start${AUS}"; return; fi
  local n info out
  for n in "${NAMEN[@]}"; do
    info=""
    if ! fenster_da "$n"; then printf '  %-10s –\n' "$n"; continue; fi
    out=$(ausgabe "$n" | tail -400)
    case $n in
      Bridge)  info=$(echo "$out" | grep -o '/scan: [0-9]*/360' | tail -1) ;;
      Sport)   info=$(echo "$out" | grep -E "Aktion '(sit|stand)': OK" | tail -1 | grep -q sit && echo "sitzt" || echo "steht") ;;
      Nav2)    echo "$out" | grep -q "Managed nodes are active" && info="aktiv" ;;
      Mission) info=$(echo "$out" | grep -o '\-> \[[A-Z_]*\]' | tail -1 | tr -d '[]>-' | xargs) ;;
      Kamera)  info="http://100.121.161.1:8080" ;;
      Dashboard) info="http://100.121.161.1:8082" ;;
    esac
    if laeuft "$n"; then printf '  %-10s %s✅ läuft%s  %s\n' "$n" "$GRUEN" "$AUS" "$info"
    else printf '  %-10s %s❌ gestoppt%s  (Tab %s ansehen)\n' "$n" "$ROT" "$AUS" "$n"; fi
  done
  echo "────────────────────────────────────────────────"
  echo "  Tabs unten anklicken · Befehle im Tab 8:Befehle"
}

cmd_oeffnen() {
  sitzung_da || cmd_start || { echo; read -rp "Enter zum Schließen ..." _; return 1; }
  if [ -n "$TMUX" ]; then tmux switch-client -t "$SESSION"; else exec tmux attach -t "$SESSION"; fi
}

cmd_neu() {
  local teil=${1,,} n ziel=""
  for n in "${NAMEN[@]}"; do [ "${n,,}" = "$teil" ] && ziel=$n; done
  [ -z "$ziel" ] && { echo "Unbekannt: '$1'. Möglich: ${NAMEN[*],,}"; return 1; }
  sitzung_da || sitzung_anlegen
  stoppe_teil "$ziel"
  starte_teil "$ziel"
}

ros_env() { # shellcheck disable=SC1090
  source "$ENV_SH" >/dev/null; }

case "${1:-hilfe}" in
  start)    cmd_start ;;
  stop)     cmd_stop ;;
  restart)  cmd_stop; sleep 2; cmd_start ;;
  status)   cmd_status ;;
  oeffnen|logs|attach) cmd_oeffnen ;;
  neu)      cmd_neu "$2" ;;
  reset)    bash "$BR/reset_nav2_after_pose.sh" ;;
  abbruch)  ros_env; ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}" ;;
  sitz|steh) # über die Mission (dann weiß sie Bescheid und die Kamera-KI startet), sonst direkt
            if python3 -c "import urllib.request,sys; urllib.request.urlopen('http://127.0.0.1:8081/$1', timeout=2)" 2>/dev/null; then
              echo "✅ $1 – über die Mission (Tab 6:Mission)"
            else
              echo "Mission nicht erreichbar – direkt an den Sport-Sender"
              python3 "$BR/go2_sit_test.py" "$([ "$1" = sitz ] && echo sit || echo stand)"
            fi ;;
  punkt)    case "$2" in
              speichern) ros_env; python3 "$BR/go2_punkte.py" speichern "$3" ;;
              loeschen)  python3 "$BR/go2_punkte.py" loeschen "$3" ;;
              *) echo "go2 punkt speichern NAME  |  go2 punkt loeschen NAME"; exit 1 ;;
            esac ;;
  punkte)   python3 "$BR/go2_punkte.py" liste ;;
  fahre)    python3 "$BR/go2_punkte.py" fahre "$2" ;;
  hilfe|-h|--help) hilfe ;;
  *) echo "Unbekannter Befehl: $1"; hilfe; exit 1 ;;
esac
