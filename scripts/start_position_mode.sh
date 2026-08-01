#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

TERMINAL_PIDS=()
TERMINAL_CMD=""
TMUX_SESSION="${TMUX_SESSION:-position_mode_slam}"

find_terminal() {
  if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    if command -v tmux >/dev/null 2>&1; then
      TERMINAL_CMD="tmux"
      return
    fi

    echo "No graphical display detected and tmux is not installed." >&2
    echo "Install tmux or run the three scripts manually." >&2
    exit 1
  fi

  if command -v gnome-terminal >/dev/null 2>&1; then
    TERMINAL_CMD="gnome-terminal"
  elif command -v konsole >/dev/null 2>&1; then
    TERMINAL_CMD="konsole"
  elif command -v xfce4-terminal >/dev/null 2>&1; then
    TERMINAL_CMD="xfce4-terminal"
  elif command -v xterm >/dev/null 2>&1; then
    TERMINAL_CMD="xterm"
  else
    echo "No supported terminal emulator found." >&2
    echo "Install one of: gnome-terminal, konsole, xfce4-terminal, xterm" >&2
    exit 1
  fi
}

cleanup() {
  trap - EXIT INT TERM HUP

  if [[ "$TERMINAL_CMD" == "tmux" ]]; then
    if tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
      tmux kill-session -t "$TMUX_SESSION" 2>/dev/null || true
    fi
    return
  fi

  if ((${#TERMINAL_PIDS[@]} == 0)); then
    return
  fi

  echo
  echo "Stopping launched Position-mode terminals..."
  for pid in "${TERMINAL_PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done

  for pid in "${TERMINAL_PIDS[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
}

launch_terminal() {
  local title="$1"
  local script_name="$2"
  local repo_quoted
  local command

  printf -v repo_quoted "%q" "$BASE_REPO_ROOT"
  command="cd $repo_quoted && bash ./scripts/$script_name; status=\$?; echo; echo '$script_name exited with status '\$status; echo 'Close this terminal or press Enter.'; read -r _"

  case "$TERMINAL_CMD" in
    gnome-terminal)
      gnome-terminal --wait --title="$title" -- bash -lc "$command" &
      ;;
    konsole)
      konsole --nofork --workdir "$BASE_REPO_ROOT" -p "tabtitle=$title" -e bash -lc "$command" &
      ;;
    xfce4-terminal)
      xfce4-terminal --disable-server --title="$title" --command="bash -lc $(printf "%q" "$command")" &
      ;;
    xterm)
      xterm -T "$title" -e bash -lc "$command" &
      ;;
  esac

  TERMINAL_PIDS+=("$!")
}

tmux_command() {
  local script_name="$1"
  local prefix="${2:-}"
  local repo_quoted

  printf -v repo_quoted "%q" "$BASE_REPO_ROOT"
  printf 'cd %s && %sbash ./scripts/%s; status=$?; echo; echo "%s exited with status $status"; echo "Press Enter to close this pane."; read -r _' \
    "$repo_quoted" "$prefix" "$script_name" "$script_name"
}

start_tmux_session() {
  if tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
    echo "tmux session already exists: $TMUX_SESSION" >&2
    echo "Close it first with: tmux kill-session -t $TMUX_SESSION" >&2
    exit 1
  fi

  tmux new-session -d -s "$TMUX_SESSION" -n position_mode "$(tmux_command micro_dds.sh)"
  tmux split-window -t "$TMUX_SESSION:0" -h "$(tmux_command run_mid360_fastlio2.sh "OPEN_RVIZ=false ")"
  tmux split-window -t "$TMUX_SESSION:0.1" -v "$(tmux_command run_px4_position_mode.sh)"
  tmux set-window-option -t "$TMUX_SESSION:0" remain-on-exit on >/dev/null
  tmux select-layout -t "$TMUX_SESSION:0" tiled >/dev/null

  echo "Started tmux session: $TMUX_SESSION"
  echo "Panes:"
  echo "  bash ./scripts/micro_dds.sh"
  echo "  OPEN_RVIZ=false bash ./scripts/run_mid360_fastlio2.sh"
  echo "  bash ./scripts/run_px4_position_mode.sh"
  echo
  echo "Close this terminal to stop all panes, or press Ctrl+B then type :kill-session."

  tmux attach-session -t "$TMUX_SESSION"
}

find_terminal
trap cleanup EXIT INT TERM HUP

echo "Starting PX4 Position-mode SLAM stack with $TERMINAL_CMD..."
echo "Close this launcher terminal or press Ctrl+C to stop all launched terminals."

if [[ "$TERMINAL_CMD" == "tmux" ]]; then
  start_tmux_session
  exit 0
fi

launch_terminal "PX4 XRCE Agent" "micro_dds.sh"
sleep 1
launch_terminal "MID360 FAST-LIO2" "run_mid360_fastlio2.sh"
sleep 1
launch_terminal "PX4 Position VIO Bridge" "run_px4_position_mode.sh"

echo
echo "Started:"
echo "  bash ./scripts/micro_dds.sh"
echo "  bash ./scripts/run_mid360_fastlio2.sh"
echo "  bash ./scripts/run_px4_position_mode.sh"
echo
echo "This launcher does not start EGO, GVF, ISMC, or Offboard control."
echo "Waiting. Press Ctrl+C here to stop all three terminals."

while true; do
  sleep 1
done
