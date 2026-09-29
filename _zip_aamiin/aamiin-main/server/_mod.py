"""Is the whine amplitude modulation at the chunk rate?

Captures the raw mic tap and the processed stream at once, then looks at the
spectrum of each one's ENVELOPE. A steady noise floor has a flat envelope
spectrum; anything pulsing shows a peak at its repetition rate. Chunks are
250 ms, so a chain that updates once per chunk would show up at 4 Hz.
"""
import asyncio
import sys

import numpy as np
import websockets

RATE = 16000
SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
BASE = "ws://127.0.0.1:2222/ws/liveaudio?device=bodycam-01"


async def grab(url, secs):
    out = []
    async with websockets.connect(url, max_size=None) as ws:
        loop = asyncio.get_event_loop()
        end = loop.time() + secs
        while loop.time() < end:
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=6.0)
            except asyncio.TimeoutError:
                break
            if isinstance(msg, bytes):
                out.append((loop.time(), msg))
    return out


async def main():
    return await asyncio.gather(
        grab(BASE + "&raw=1", SECS), grab(BASE, SECS)
    )


raw_pkts, proc_pkts = asyncio.run(main())


def envelope_spectrum(x, label):
    if x.size < RATE * 4:
        print(f"  {label}: only {x.size / RATE:.1f}s, too short")
        return
    # Envelope: RMS in 10 ms hops -> 100 Hz sample rate.
    hop = 160
    n = x.size // hop
    env = np.sqrt(np.mean(x[: n * hop].reshape(n, hop).astype(np.float64) ** 2, axis=1))
    env -= env.mean()
    if env.std() < 1e-9:
        print(f"  {label}: envelope is flat")
        return
    win = np.hanning(env.size)
    mag = np.abs(np.fft.rfft(env * win))
    freq = np.fft.rfftfreq(env.size, hop / RATE)
    keep = (freq > 0.5) & (freq < 40)
    mag, freq = mag[keep], freq[keep]
    med = float(np.median(mag))
    order = np.argsort(mag)[::-1]
    tops = []
    for i in order:
        if any(abs(freq[i] - f) < 0.8 for f, _ in tops):
            continue
        tops.append((float(freq[i]), float(20 * np.log10(mag[i] / med))))
        if len(tops) >= 4:
            break
    print(f"  {label}: modulation peaks -> " +
          "  ".join(f"{f:.1f} Hz +{d:.0f} dB" for f, d in tops))


for label, pkts in (("raw mic  ", raw_pkts), ("processed", proc_pkts)):
    if not pkts:
        print(f"  {label}: no data")
        continue
    sizes = sorted({len(m) // 2 for _, m in pkts})
    dts = np.diff([t for t, _ in pkts])
    x = np.frombuffer(b"".join(m for _, m in pkts), dtype="<i2").astype(np.float64)
    rms = float(np.sqrt(np.mean(x * x)))
    print(f"\n{label}: {len(pkts)} pkts, sizes {sizes[:4]}, "
          f"{x.size / RATE:.1f}s audio in {SECS:.0f}s wall "
          f"({x.size / RATE / SECS * 100:.0f}% coverage), rms {rms:.0f}")
    if dts.size:
        print(f"  arrival gap: median {np.median(dts) * 1000:.0f} ms, "
              f"p90 {np.percentile(dts, 90) * 1000:.0f} ms")
    envelope_spectrum(x, label)
