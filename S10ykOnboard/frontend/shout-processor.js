/* 单向喊话 AudioWorklet（2026-11-17 v4 / D-036 + D-037）
 * 把麦克风 Float32 采样混单声道、线性插值重采样到 16k、转 16bit PCM 分块回传。
 * v4：输出固定 16k——上下文保持设备原生采样率（避免非默认采样率导致的采集静音），
 *     但不再直传原生速率：48k 直传需 96KB/s 带宽，弱上联网络会大量拥塞丢片
 *     （实测 ~35KB/s 上联丢 333 片），16k 只需 32KB/s。
 * v3：多声道下混（getUserMedia 不再强制单声道——channelCount:1 强约束在部分
 *     安卓 Chrome 上会让整条轨道输出静音）。
 */
class ShoutProcessor extends AudioWorkletProcessor {
  constructor() {
    super();
    this.outRate = 16000;
    this.ratio = sampleRate / this.outRate;             // 输入/输出采样率比（>1 下采样）
    this.chunkSamples = Math.round(this.outRate / 10);  // 100ms @16k = 1600 样本
    this.buf = new Int16Array(this.chunkSamples);
    this.filled = 0;
    this.pos = 0;      // 输出样本对应的输入流浮点位置（跨块连续，保证重采样无缝）
    this.count = 0;    // 已处理的输入样本总数
    this.tail = 0;     // 上一块最后一个样本（跨块插值左端点）
    this.mono = new Float32Array(128);
  }
  push(v) {
    if (v > 1) v = 1; else if (v < -1) v = -1;
    this.buf[this.filled++] = v < 0 ? (v * 0x8000) | 0 : (v * 0x7fff) | 0;
    if (this.filled >= this.chunkSamples) {
      this.port.postMessage(this.buf.buffer, [this.buf.buffer]);
      this.buf = new Int16Array(this.chunkSamples);
      this.filled = 0;
    }
  }
  process(inputs) {
    const input = inputs[0];
    if (input && input.length) {
      const nch = input.length, ch0 = input[0];
      if (ch0) {
        const n = ch0.length;
        if (n > this.mono.length) this.mono = new Float32Array(n);
        const mono = this.mono;
        for (let i = 0; i < n; i++) {                   // 多声道平均下混
          let s = 0;
          for (let c = 0; c < nch; c++){ const d = input[c]; if (d) s += d[i]; }
          mono[i] = s / nch;
        }
        if (this.ratio === 1) {
          for (let i = 0; i < n; i++) this.push(mono[i]);
        } else {
          // 线性插值重采样：pos 只推进到本块倒数第二个样本，保证插值右端点总在本块内
          const start = this.count;
          this.count += n;
          const lim = start + n - 1;
          while (this.pos < lim) {
            const base = Math.floor(this.pos);
            const i0 = base - start;
            const frac = this.pos - base;
            const a = i0 < 0 ? this.tail : mono[i0];
            const b = mono[i0 + 1];
            this.push(a + (b - a) * frac);
            this.pos += this.ratio;
          }
        }
        this.tail = mono[n - 1];
      }
    }
    return true;
  }
}
registerProcessor('shout-processor', ShoutProcessor);
