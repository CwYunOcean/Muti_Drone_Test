#!/usr/bin/env bash
set -euo pipefail

# Configure chrony time sync for the multi-drone EGO-Swarm setup.
# The swarm broadcast path drops trajectories when clock skew > 0.25 s,
# so both drones must sync to the same reference (drone 0 acts as server).
#
# Usage:
#   sudo ./setup_chrony_time_sync.sh server                # on drone 0
#   sudo ./setup_chrony_time_sync.sh client <drone0_ip>    # on drone 1
#
# Verify afterwards:
#   chronyc tracking          # "System time" offset should be < 10 ms
#   chronyc sources -v        # client should list drone 0 as ^* source

ROLE="${1:-}"
SERVER_IP="${2:-}"
CONF="/etc/chrony/chrony.conf"
MARKER="# --- drone-swarm time sync ---"

if [[ "$ROLE" != "server" && "$ROLE" != "client" ]]; then
  echo "Usage: $0 server | client <drone0_ip>" >&2
  exit 1
fi
if [[ "$ROLE" == "client" && -z "$SERVER_IP" ]]; then
  echo "client role needs the drone 0 IP: $0 client <drone0_ip>" >&2
  exit 1
fi
if [[ $EUID -ne 0 ]]; then
  echo "Run with sudo." >&2
  exit 1
fi

if ! command -v chronyd >/dev/null 2>&1; then
  apt-get update && apt-get install -y chrony
fi

cp -n "$CONF" "$CONF.bak" || true

# Remove a previous block from this script, then append the fresh one.
sed -i "/$MARKER start/,/$MARKER end/d" "$CONF"

if [[ "$ROLE" == "server" ]]; then
  cat >> "$CONF" <<EOF
$MARKER start
# Serve time to the swarm even when offline (local stratum fallback).
allow 0.0.0.0/0
local stratum 8
$MARKER end
EOF
else
  cat >> "$CONF" <<EOF
$MARKER start
# Follow drone 0 tightly; step immediately on large offsets at startup.
server $SERVER_IP iburst minpoll 2 maxpoll 4 prefer
makestep 1.0 -1
$MARKER end
EOF
fi

systemctl restart chrony
sleep 2
chronyc tracking || true
echo "Done ($ROLE). Check offset with: chronyc tracking"
