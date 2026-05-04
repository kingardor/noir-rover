#!/usr/bin/env bash
# Tail all native service logs with coloured prefixes.
cd "$(dirname "$0")/.."
mkdir -p logs
touch logs/vision.log

tail -n 30 -f logs/vision.log 2>/dev/null \
  | awk '
      /==> logs\/vision/ { svc="vision"; next }
      svc=="vision" { printf "\033[34m[vision]\033[0m %s\n", $0 }
      !svc          { print $0 }
    '
