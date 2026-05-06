#!/usr/bin/env bash
until python3 -c '
import urllib.request, json, sys
try:
    r = json.loads(urllib.request.urlopen("http://localhost:8014/audio/health", timeout=2).read())
    sys.exit(0 if r.get("stt_ready") and r.get("tts_ready") else 1)
except:
    sys.exit(1)
' 2>/dev/null; do
    sleep 3
done
