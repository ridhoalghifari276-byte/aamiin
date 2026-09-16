"""Instantaneous A/V rates, from differences between /health polls."""
import json
import sys
import time
import urllib.request

DEV = "bodycam-01"
N = int(sys.argv[1]) if len(sys.argv) > 1 else 8
GAP = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0


def poll():
    with urllib.request.urlopen("http://127.0.0.1:2222/health", timeout=8) as r:
        return json.load(r)


prev = poll()
st = prev.get("device_state", {}).get(DEV, {})
print(f"device_state: stream={st.get('stream')} audio={st.get('audio')} "
      f"video={st.get('video')} visual={st.get('visual')} rssi={st.get('rssi')}")
print(f"{'window':>8} {'audio Hz':>9} {'of 16k':>7} {'video fps':>10} {'video kbps':>11}")

for _ in range(N):
    time.sleep(GAP)
    cur = poll()
    pc, cc = prev["counters"], cur["counters"]
    dt = cc["t"] - pc["t"]
    if dt <= 0:
        continue
    da = cc["audio_samples"].get(DEV, 0) - pc["audio_samples"].get(DEV, 0)
    df = cc["video_frames"].get(DEV, 0) - pc["video_frames"].get(DEV, 0)
    db = cc["video_bytes"].get(DEV, 0) - pc["video_bytes"].get(DEV, 0)
    hz = da / dt
    print(
        f"{dt:7.1f}s {hz:9.0f} {hz / 16000 * 100:6.0f}% "
        f"{df / dt:10.1f} {db * 8 / 1000 / dt:11.1f}"
    )
    prev = cur
