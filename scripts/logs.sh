#!/usr/bin/env bash
# Tail all native service logs with coloured prefixes.
cd "$(dirname "$0")/.."
mkdir -p logs
touch logs/vision.log logs/tts.log logs/stt.log logs/mcp.log

tail -n 30 -f logs/vision.log logs/tts.log logs/stt.log logs/mcp.log 2>/dev/null \
  | awk '
      /==> logs\/vision/ { svc="vision"; next }
      /==> logs\/tts/    { svc="tts";    next }
      /==> logs\/stt/    { svc="stt";    next }
      /==> logs\/mcp/    { svc="mcp";    next }
      svc=="vision" { printf "\033[34m[vision]\033[0m %s\n", $0 }
      svc=="tts"    { printf "\033[32m[tts]   \033[0m %s\n", $0 }
      svc=="stt"    { printf "\033[33m[stt]   \033[0m %s\n", $0 }
      svc=="mcp"    { printf "\033[35m[mcp]   \033[0m %s\n", $0 }
      !svc          { print $0 }
    '
