"""Pick the jitter-buffer size for 120 ms packets, and check the hum fix.

Part 1 simulates the worklet's depth/trim loop against 120 ms arrivals. The
trim steers the *mean* depth, and depth swings by a whole packet, so the
trough sits about half a packet under target; if that trough reaches the
rescue threshold the pitch gets yanked and the listener hears a warble at the
packet rate.

Part 2 feeds speech followed by a long silence through VoiceChain and watches
what the gain does once the talking stops.
"""
import os
import tempfile

import numpy as np

os.environ.setdefault("DATA_DIR", tempfile.mkdtemp(prefix="tune-"))
from app import SAMPLE_RATE, VoiceChain  # noqa: E402

SR, QUANTUM, SRC = 48000, 128, SAMPLE_RATE


def sim(pkt_ms, target_ms, jitter_ms, max_ms, clamp=0.01, low_f=0.3):
    pkt = int(SRC * pkt_ms / 1000)
    target = target_ms * SRC / 1000.0
    base = SRC / SR
    depth_k = QUANTUM / SR / 2.0
    trim_k = QUANTUM / SR / 4.0
    hard_max = max(target_ms, max_ms) * SRC / 1000.0
    w, r, trim, avg = target, 0.0, 0.0, target
    rng = np.random.RandomState(0)
    steps, depths, rescues, overruns = [], [], 0, 0
    nxt = 0.0
    for q in range(int(30 * SR / QUANTUM)):
        t = q * QUANTUM / SR
        while t >= nxt:
            w += pkt
            nxt += pkt / SRC + rng.uniform(-jitter_ms, jitter_ms) / 1000.0
        if w - r > min(hard_max, target * 3):
            r, avg, overruns = w - target, target, overruns + 1
        depth = w - r
        avg += (depth - avg) * depth_k
        low = target * low_f
        if depth < low:
            rescues += 1
            trim += (-0.15 * (1 - depth / low) - trim) * 0.5
        else:
            want = max(-clamp, min(clamp, (avg - target) / (target * 12)))
            trim += (want - trim) * trim_k
        steps.append(base * (1 + trim))
        depths.append(depth)
        r += QUANTUM * base * (1 + trim)
    skip = int(5 * SR / QUANTUM)
    rate = np.array(steps[skip:]) / base
    d = np.array(depths[skip:]) / SRC * 1000.0
    x = rate - rate.mean()
    win = np.hanning(x.size)
    amp = 2 * np.abs(np.fft.rfft(x * win)) / win.sum() * 100
    freq = np.fft.rfftfreq(x.size, QUANTUM / SR)
    pr = 1000.0 / pkt_ms
    band = (freq > pr * 0.9) & (freq < pr * 1.1)
    return (float(np.max(amp[band])) if band.any() else 0.0,
            d.min(), d.max(), rescues, overruns)


print("120 ms packets — packet-rate wobble must stay near zero\n")
print(f"  {'target':>7} {'ceil':>6} {'jitter':>7} {'wobble':>9} "
      f"{'depth ms':>12} {'rescue':>7} {'overrun':>8}")
for tgt, mx in ((180, 500), (250, 600), (250, 700), (320, 800)):
    for jit in (15, 60):
        wob, dmin, dmax, res, ovr = sim(120, tgt, jit, mx)
        flag = "" if (wob < 0.02 and res == 0 and ovr == 0) else "  <-- trips"
        print(f"  {tgt:>7} {mx:>6} {jit:>6}ms {wob:>8.3f}% "
              f"{dmin:>5.0f}-{dmax:<6.0f} {res:>7} {ovr:>8}{flag}")

print("\n\nhum after speaking — gain must come back down when talking stops\n")
rate = SAMPLE_RATE
t = np.arange(rate * 24) / rate
# Syllables with gaps, not a continuous tone: 2 s of unbroken sound drives the
# minimum-statistics floor tracker up to the speech level itself, so a steady
# sine tests the floor tracker rather than the gain. 8 s of talking, then 16 s
# of nothing but room noise.
rs1 = np.random.RandomState(1)
syll = ((t % 0.65) < 0.40) & (t < 8.0)
sig = 16000 * np.sin(2 * np.pi * 300 * t) * syll + 120 * rs1.randn(t.size)
pcm = np.clip(sig, -32768, 32767).astype(np.int16)

PKT = int(rate * 0.125)
vc = VoiceChain()
out = []
marks = {}
for i in range(0, pcm.size - PKT, PKT):
    out.append(vc.process(pcm[i : i + PKT].tobytes()))
    secs = (i + PKT) / rate
    for at in (7.9, 9, 11, 14, 18, 22, 23):
        if at <= secs < at + 0.13:
            marks[at] = (vc.gain, vc.speech, vc.floor, vc.duck, vc.hang)
y = np.frombuffer(b"".join(out), dtype="<i2").astype(np.float64)

print(f"  {'t':>5} {'gain':>7} {'speech':>8} {'floor':>7} {'duck':>6} {'hang':>5}  what")
for at in sorted(marks):
    g, sp, fl, dk, hg = marks[at]
    what = "still speaking" if at < 8 else f"{at - 8:.0f}s after last word"
    print(f"  {at:>4.0f}s {g:>7.2f} {sp:>8.0f} {fl:>7.0f} {dk:>6.2f} {hg:>5}  {what}")


def dbfs(a):
    return 20 * np.log10(max(float(np.sqrt(np.mean(a * a))), 1e-9) / 32768)


# What the listener actually hears, and how it compares to the input. The test
# that matters: silence must not come out louder than it went in.
sp_out = y[int(rate * 2) : int(rate * 7.5)]
sil_out = y[int(rate * 14) :]
sil_in = pcm[int(rate * 14) :].astype(np.float64)
print(f"\n  speech out  {dbfs(sp_out):6.1f} dBFS")
print(f"  silence in  {dbfs(sil_in):6.1f} dBFS")
print(f"  silence out {dbfs(sil_out):6.1f} dBFS   "
      f"({dbfs(sil_out) - dbfs(sil_in):+.1f} dB vs input — must not be positive)")
print(f"  silence peak |sample| {int(np.max(np.abs(sil_out)))}  (must be 0)")
print(f"  speech-to-silence separation {dbfs(sp_out) - dbfs(sil_out):.1f} dB")

# And the first word after the pause must not be faded in.
first = y[int(rate * 0.0) : int(rate * 0.45)]
later = y[int(rate * 5.0) : int(rate * 5.45)]
print(f"\n  first syllable {dbfs(first):6.1f} dBFS vs a later one {dbfs(later):6.1f} dBFS"
      f"  ({dbfs(first) - dbfs(later):+.1f} dB)")
