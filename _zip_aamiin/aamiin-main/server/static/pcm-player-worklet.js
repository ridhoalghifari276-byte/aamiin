// Adaptive jitter buffer for the bodycam voice stream.
//
// The previous version held a fixed 20-40 ms and answered trouble by writing
// zeros (underrun) or discarding a whole chunk (overrun). On WiFi that fired
// several times a second, which is what the crackling actually was.
//
// Three mechanisms replace that, in order of how often they act:
//   1. Playback runs slightly fast or slow to track the arrival rate, so the
//      buffer is steered rather than allowed to empty or run away. This also
//      absorbs the ESP's sample clock being a fraction off 16 kHz.
//   2. If the buffer still runs dry it stretches hard (up to -15 %) instead of
//      starving. A brief pitch dip is far less noticeable than 80 ms of silence.
//   3. Only if that fails does it re-prime — and the target depth then grows,
//      so a bad link buys latency automatically instead of crackling. The
//      target decays back down once the link behaves, keeping a good link
//      near the floor.
class BodycamPcmPlayer extends AudioWorkletProcessor {
  constructor(options) {
    super();
    const o = (options && options.processorOptions) || {};
    this.srcRate = o.srcRate > 0 ? o.srcRate : 16000;
    this.base = this.srcRate / sampleRate;

    const ms = (v) => Math.round((v * this.srcRate) / 1000);
    this.targetMin = Math.max(160, ms(o.targetMs > 0 ? o.targetMs : 80));
    this.targetMax = Math.max(this.targetMin, ms(o.maxTargetMs > 0 ? o.maxTargetMs : 260));
    this.hardMax = Math.max(this.targetMax, ms(o.maxMs > 0 ? o.maxMs : 600));
    this.target = this.targetMin;
    this.makeup = o.makeup > 0 ? o.makeup : 1.5;

    this.size = 1 << 16;
    this.mask = this.size - 1;
    this.ring = new Float32Array(this.size);
    this.w = 0;
    this.r = 0;
    this.trim = 0;
    this.env = 0;
    this.envStep = 1 / Math.max(1, 0.008 * sampleRate);
    // Per-sample factor giving a ~25 ms coast to silence, whatever the device
    // rate. Fading faster turns a shortfall into a buzz; see _silence.
    this.coast = Math.exp(-1 / (0.025 * sampleRate));
    this.last = 0;
    this.priming = true;
    // Target decays back toward the floor with a ~12 s time constant.
    // One render quantum is 128 frames.
    this.decay = 128 / sampleRate / 12;
    // Both of these are per render quantum. 2 s of depth averaging and a 4 s
    // trim, so neither can follow the packet cadence.
    this.avgDepth = this.target;
    this.depthK = 128 / sampleRate / 2;
    this.trimK = 128 / sampleRate / 4;

    this.port.onmessage = (e) => {
      const msg = e.data;
      if (!msg || msg.type !== "push" || !msg.samples) return;
      const s = msg.samples;
      const k = this.makeup / 32768;
      const ring = this.ring;
      const mask = this.mask;
      for (let i = 0; i < s.length; i++) {
        let v = s[i] * k;
        // Soft knee from 0.72 up. A clamp at full scale is audible crackle.
        if (v > 0.72) v = 0.72 + 0.28 * Math.tanh((v - 0.72) / 0.28);
        else if (v < -0.72) v = -0.72 - 0.28 * Math.tanh((-v - 0.72) / 0.28);
        ring[this.w & mask] = v;
        this.w++;
      }
      const ceiling = Math.min(this.hardMax, this.target * 3);
      if (this.w - this.r > ceiling) {
        // The link stalled and then dumped a burst. Jump to the live edge and
        // fade back in, rather than playing a backlog that keeps growing.
        this.r = this.w - this.target;
        this.env = 0;
        // The averaged depth describes a buffer that no longer exists.
        this.avgDepth = this.target;
      }
    };
  }

  // Hard zeros. Coasting the last speech sample used to turn an underrun
  // after a sentence into a decaying tone whose loudness tracked how hard
  // they had just spoken — that was the post-speech hum.
  _silence(out, from) {
    for (let i = from; i < out.length; i++) out[i] = 0;
    this.last = 0;
    this.env = 0;
  }

  _starved() {
    this.priming = true;
    this.target = Math.min(this.targetMax, Math.round(this.target * 1.4));
  }

  process(_inputs, outputs) {
    const out = outputs[0] && outputs[0][0];
    if (!out) return true;
    const n = out.length;

    if (this.priming) {
      if (this.w - this.r < this.target) {
        this._silence(out, 0);
        return true;
      }
      this.priming = false;
    }

    const depth = this.w - this.r;
    // Depth jumps by a whole packet on every arrival and drains away in
    // between, so the instantaneous value says nothing about whether playback
    // is keeping up. Average over seconds before steering on it.
    this.avgDepth += (depth - this.avgDepth) * this.depthK;

    // The trim steers the *mean* depth to target, and depth swings by a whole
    // packet, so the trough sits about half a packet below target. At 0.5 the
    // rescue branch was tripping on that ordinary trough hundreds of times a
    // minute and yanking the pitch 15 % each time.
    const low = this.target * 0.3;
    if (depth < low) {
      // Stretch, and react this block: smoothing a rescue over 100 ms is too
      // slow to be a rescue. Instantaneous depth on purpose — this is the one
      // case where the current value is exactly what matters.
      const want = -0.15 * (1 - depth / low);
      this.trim += (want - this.trim) * 0.5;
    } else {
      // Trim exists to cancel the drift between the device's sample clock and
      // this sound card, which is a fraction of a percent over minutes. It has
      // no business reacting inside one packet period: at 250 ms packets the
      // old ±3 % with a 53 ms time constant chased the buffer's own sawtooth
      // and warbled the pitch four times a second, which over a quiet room is
      // heard as a pulsing whine.
      let want = (this.avgDepth - this.target) / (this.target * 12);
      if (want > 0.01) want = 0.01;
      else if (want < -0.01) want = -0.01;
      this.trim += (want - this.trim) * this.trimK;
      this.target += (this.targetMin - this.target) * this.decay;
      if (this.target < this.targetMin) this.target = this.targetMin;
    }
    const step = this.base * (1 + this.trim);

    const ring = this.ring;
    const mask = this.mask;
    for (let i = 0; i < n; i++) {
      if (this.w - this.r < 2) {
        this._silence(out, i);
        this._starved();
        return true;
      }
      const idx = Math.floor(this.r);
      const f = this.r - idx;
      const a = ring[idx & mask];
      const b = ring[(idx + 1) & mask];
      if (this.env < 1) {
        this.env += this.envStep;
        if (this.env > 1) this.env = 1;
      }
      out[i] = (a + (b - a) * f) * this.env;
      this.r += step;
    }
    this.last = out[n - 1];
    return true;
  }
}

registerProcessor("bodycam-pcm-player", BodycamPcmPlayer);
