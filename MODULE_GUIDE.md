# FFT Pipeline Module Guide

A detailed walkthrough of each module: how it works, its ports, key arithmetic, and how to test it.

---

## Fixed-Point Convention: Q24.8

Every signal in the datapath is **signed 32-bit Q24.8**:
- Bit 31: sign
- Bits [30:8]: integer part (23 bits)
- Bits [7:0]: fractional part (8 bits)

| Float | Q24.8 integer | Hex |
|---|---|---|
| 1.0 | 256 | 0x00000100 |
| -1.0 | -256 | 0xFFFFFF00 |
| 0.5 | 128 | 0x00000080 |
| 3.14159 | 804 | 0x00000324 |

**Multiply rule:** `Q24.8 × Q24.8 = Q48.16`. Extract Q24.8 result as bits `[39:8]` of the 64-bit signed product:
```verilog
wire signed [63:0] prod = $signed(a) * $signed(b);
wire signed [31:0] result = prod[39:8];
```

---

## 1. `async_fifo.v` — Clock-Domain Crossing FIFO

### Purpose
Safely passes market data words from the 156.25 MHz ingress clock into the 300 MHz processing clock domain. Uses Gray-code pointers and two-flop synchronizer chains to avoid metastability.

### How Gray-Code FIFO Works
Binary counters (wr_ptr, rd_ptr) address the memory. Before crossing clock domains, each pointer is converted to Gray code (only 1 bit changes per increment), so even if a crossing flip-flop metastabilizes, the resulting value is at most one step away from the correct value.

```
Binary: 0→1→2→3→4 = 000→001→010→011→100
Gray:   0→1→3→2→6 = 000→001→011→010→110
                        ↑ only 1 bit changes each step
```

**Full condition:** the write Gray pointer matches the read Gray pointer with the top two bits inverted — meaning the write pointer has lapped the read pointer.

**Empty condition:** both Gray pointers are equal.

### Port Table
| Port | Dir | Width | Description |
|---|---|---|---|
| wr_clk | in | 1 | Write domain clock |
| wr_rst | in | 1 | Sync reset (wr domain) |
| wr_en | in | 1 | Write enable |
| wr_data | in | DATA_WIDTH | Data to write |
| wr_full | out | 1 | FIFO is full (do not write) |
| rd_clk | in | 1 | Read domain clock |
| rd_rst | in | 1 | Sync reset (rd domain) |
| rd_en | in | 1 | Read enable |
| rd_data | out | DATA_WIDTH | Read data (registered) |
| rd_empty | out | 1 | FIFO is empty (no valid data) |

### Common Pitfall
Always reset both domains for at least 4 `rd_clk` cycles simultaneously. Releasing `wr_rst` while `rd_rst` is still held can cause false empty/full readings.

---

## 2. `market_data_parser.v` — Packet Deserializer

### Purpose
Converts a raw 32-bit word stream (5 words per market data record) into structured fields: symbol_id, price, volume, timestamp.

### Packet Format
```
Word 0: {symbol_id[7:0], 24'b0}
Word 1: price[31:0]          ← Q24.8 normalized price
Word 2: {volume[15:0], 16'b0}
Word 3: timestamp[47:32]     ← microseconds since epoch, upper 32 bits
Word 4: {timestamp[15:0], 16'b0}
```

### State Machine
```
IDLE ──s_fire──► W1 ──s_fire──► W2 ──s_fire──► W3 ──s_fire──► W4
                                                                │
                                                            (latch fields)
                                                                │
                                                             HOLD ──m_ready──► IDLE
```
In `HOLD`, `m_valid=1` and `s_ready=0` until the downstream accepts the record.

### Worked Example
Sending AAPL price = $185.50, volume = 500, timestamp = 1704200000000000 µs, symbol_id = 0:
- price Q24.8 = round(185.50 × 256) = 47488 = 0x0000B980
- Word 0 = 0x00000000 (symbol_id=0)
- Word 1 = 0x0000B980
- Word 2 = 0x01F40000 (500 << 16)
- Word 3 = 0x0612360F
- Word 4 = 0x00000000

---

## 3. `preprocessing.v` — Normalization and Resampling

### Purpose
- Maintains an **Exponential Moving Average (EMA)** of the price per channel
- Subtracts mean to center signal around zero
- Outputs samples at a **uniform rate** (configurable, default ~146 kHz at 300 MHz)
- Fills gaps with **Zero-Order Hold (ZOH)** when no new price arrives

### EMA Mean Update
```
mean_new = mean + (price - mean) >> alpha_shift
```
With `alpha_shift=8`, this gives `alpha = 1/256`. The mean tracks slow trends; subtracting it isolates shorter-term oscillations for the FFT.

### ZOH Resampling
The module has a per-channel counter that ticks every `sample_rate` clock cycles. On each tick, it emits the most recent received price (or repeats the last one if no new price arrived). `m_gap=1` signals that the emitted sample is a hold.

### Channel Mapping
Symbol IDs 0–3 map to channels 0–3 via `symbol_id[1:0]`. Configure `sample_rate` per channel via the config bus at address `{4'h0, ch[1:0], 2'b00}`.

---

## 4. `windowing.v` — Overlap-Add Windowing

### Purpose
Multiplies each input sample by a precomputed window coefficient to reduce spectral leakage. Implements 50% overlap-add to preserve energy across frame boundaries.

### Window Functions (all Q24.8)

| Function | Formula | Purpose |
|---|---|---|
| Hamming | 0.54 - 0.46·cos(2πn/N) | General purpose, good sidelobe rejection |
| Hann | 0.5·(1 - cos(2πn/N)) | Better sidelobe rejection, slightly wider main lobe |
| Blackman | 0.42 - 0.5·cos(2πn/N) + 0.08·cos(4πn/N) | Best sidelobe rejection, wider main lobe |

**ROM sub-sampling:** ROMs store 4096 coefficients for the maximum window. For N=256, every 16th entry is read (`step = 4096/256 = 16`).

### Windowing Multiply
```verilog
// Q24.8 × Q24.8: take product[39:8] (i.e., >> 8 then keep 32 bits)
windowed = ($signed(sample) * $signed(coef)) >>> 8;
```

### Worked Example: Hamming Window on Impulse
- Input: sample[n] = 256 (= 1.0 in Q24.8) for all n
- Hamming coef[0] = round(0.54 × 256) = 138, coef[N/2] ≈ round(1.0 × 256) = 256
- windowed[0] = (256 × 138) >> 8 = 35328 >> 8 = 138 ✓ (= 0.54 in Q24.8)
- windowed[N/2] ≈ (256 × 256) >> 8 = 65536 >> 8 = 256 ✓ (= 1.0 in Q24.8)

---

## 5. `twiddle_rom.v` — Twiddle Factor ROM

### Purpose
Provides `W_N^k = cos(2πk/N) - j·sin(2πk/N)` for all butterfly operations.
Generated by `gen_twiddle.py`; stored as pairs `{sin_q24_8, cos_q24_8}` in a 2688-entry ROM.

### Address Mapping
For FFT stage `s` (0-indexed) and butterfly group index `k`:
```
twiddle_k = k * (N / 2^(s+1))   ← within-N index
ROM_addr  = twiddle_base[N_sel] + twiddle_k
```
Twiddle bases: N=256 → 0, N=1024 → 128, N=4096 → 640.

### Key Values to Verify
| N | k | Expected W_N^k | cos Q24.8 | sin Q24.8 |
|---|---|---|---|---|
| 256 | 0 | 1 + 0j | 256 (0x100) | 0 |
| 256 | 64 | 0 - j | 0 | -256 |
| 1024 | 0 | 1 + 0j | 256 | 0 |
| 1024 | 256 | 0 - j | 0 | -256 |

---

## 6. `butterfly.v` — Radix-2 Complex Butterfly

### Purpose
Computes one Radix-2 DIT butterfly:
```
p = (a + b·W) / 2     ← /2 is the block-FP scaling
q = (a - b·W) / 2
```

### 4-Cycle Pipeline
```
Cycle 1: Register a, b, W inputs
Cycle 2: Compute partial products t1=b_re×w_re, t2=b_im×w_im, t3=b_re×w_im, t4=b_im×w_re
Cycle 3: bw_re = t1[39:8] - t2[39:8];  bw_im = t3[39:8] + t4[39:8]
Cycle 4: p = (a + bw) >>> 1;  q = (a - bw) >>> 1
```

The `a` inputs are delayed 4 cycles in shift registers to align with the multiply result.

### Worked Example
- a = (256, 0) = 1+0j in Q24.8
- b = (256, 0) = 1+0j in Q24.8
- W = (0, -256) = -j in Q24.8 (i.e., w_re=0, w_im=-256)
- b·W = (0·0 - 0·(-256), 0·(-256) + 0·0) = but wait: bw_re = b_re×w_re - b_im×w_im = 256×0 - 0×(-256) = 0; bw_im = b_re×w_im + b_im×w_re = 256×(-256) + 0×0 = -65536; truncated: bw_im = -65536>>8 = ... 
  
  Actually b×W where b=1+0j, W=0-j: b·W = 1·(0-j) = 0-j = (0,-1) in float = (0,-256) in Q24.8.
  - bw_re = (256×0 - 0×(-256)) >> 8 = 0
  - bw_im = (256×(-256) + 0×0) >> 8 = -65536 >> 8 = -256 ✓
- p = (a_re + bw_re, a_im + bw_im) >>> 1 = (256+0, 0+(-256)) >>> 1 = (128, -128) = (0.5 - 0.5j)
- q = (256-0, 0-(-256)) >>> 1 = (128, 128) = (0.5 + 0.5j)

---

## 7. `fft_core.v` — FFT Controller

### Purpose
Orchestrates `log2(N)` stages of butterfly operations using a single butterfly unit and ping-pong BRAMs.

### Three Phases

**Phase 1 — LOAD:** Accept N samples from windowing. Each sample at natural index `n` is written to bit-reversed address `bit_rev(n, log2_N)` in BRAM bank A.

**Phase 2 — COMPUTE:** For each stage `s` from 0 to log2(N)-1:
- `stride = N >> (s+1)` — distance between butterfly pair elements
- For butterfly index `i` = 0 to N/2-1:
  - Read `a` at `i & ~(stride-1)`, `b` at same + stride
  - Twiddle index: `(i & (stride-1)) * (N >> (s+2))`
  - Write results `p` and `q` back to alternate BRAM after 4-cycle pipeline delay

**Phase 3 — OUTPUT:** Stream all N bins from the final BRAM in natural order with `m_bin=0..N-1`.

### Scale Exponent
`m_scale_exp` = `log2(N)` after computation (each stage adds 1). The true DFT bin value = `output × 2^scale_exp`. Downstream code uses this to normalize threshold comparisons.

### Bit-Reversal Example (N=8, 3 bits)
```
Natural: 0  1  2  3  4  5  6  7
Binary:  000 001 010 011 100 101 110 111
Bitrev:  000 100 010 110 001 101 011 111
Natural: 0   4   2   6   1   5   3   7
```
Sample originally at index 1 is loaded into BRAM address 4.

---

## 8. `cordic.v` — Magnitude and Phase

### Purpose
Computes magnitude = |Z| = √(Re²+Im²) and phase = ∠Z = atan2(Im,Re) from a complex Q24.8 number using the CORDIC vectoring algorithm.

### CORDIC Algorithm (Vectoring Mode)
Starting from (x₀, y₀) = the input complex number:
1. **Quadrant normalize:** if x < 0, negate both x and y, and set z = ±π
2. **16 iterations:** at each step rotate (x,y) toward the x-axis:
   - If y ≥ 0: rotate clockwise → x += y>>i, y -= x>>i, z += atan(2⁻ⁱ)
   - If y < 0: rotate counter-clockwise → x -= y>>i, y += x>>i, z -= atan(2⁻ⁱ)
3. After 16 steps, x ≈ K·|Z| where K = ∏√(1+2⁻²ⁱ) ≈ 1.6468
4. **Magnitude correction:** multiply by 1/K ≈ 0.6073 ≈ 155/256

### Atan LUT (Q24.8 radians)
```
i=0:  atan(1)     = 45°    = 0.7854 rad → 201
i=1:  atan(0.5)   = 26.6°  = 0.4636 rad → 119
i=2:  atan(0.25)  = 14.0°  = 0.2450 rad → 63
i=3:  atan(0.125) = 7.1°   = 0.1244 rad → 32
...
```

### Worked Example: (3, 4) → |Z| = 5, ∠Z = 53.13°
- Input: re = 768 (3.0 Q24.8), im = 1024 (4.0 Q24.8)
- Re > 0, no quadrant normalization needed
- After 16 iterations, x ≈ 1.6468 × 5 × 256 = 2107.9 ≈ 2108
- Magnitude: (2108 × 155) >> 8 = 327040 >> 8 = 1277 ≈ 1280 = 5.0 in Q24.8 (within ~0.5 LSB)
- Phase: z ≈ 0.9273 rad × 256 = 237 ≈ 237

---

## 9. `post_fft.v` — Spectral Analysis

### Purpose
Accumulates per-frame statistics across all N bins:
- **Dominant bin:** argmax of magnitude
- **Spectral centroid:** weighted mean bin = Σ(bin × mag) / Σ(mag)
- **Total power:** Σ(mag²)
- **Power in dB:** 10 × log₁₀(total_power), approximated via log2 LUT

### Processing Per Bin (1 cycle per bin)
```
if mag > max_mag:  max_mag = mag;  max_bin = bin
cent_num += bin × mag
cent_den += mag
total_pow += (mag × mag) >> 8
```
On `s_frame_last`: compute centroid (divide), convert power to dB, assert `m_valid`.

### Interpreting Output
- `m_dom_bin = 64` in a 256-point FFT at 1-minute bars means the dominant oscillation period ≈ 256×60s / 64 = 4 hours.
- `m_power_db` is approximate (log2 LUT with 8-bit index, ±2 dB accuracy).
- `m_scale_exp` from the FFT should be used for absolute magnitude recovery: true_mag = `m_dom_mag × 2^m_scale_exp`.

---

## 10. `signal_logic.v` — Trading Signal Decision Engine

### Purpose
Fires `m_valid` with a BUY/SELL/ALERT signal when per-channel threshold conditions are met simultaneously.

### Conditions (evaluated in parallel, cycle 1)
```
cond_mag   = dom_mag   > r_mag_thresh[ch]
cond_cent  = centroid  > r_centroid_lo[ch]  AND  centroid < r_centroid_hi[ch]
cond_power = power_db  > r_power_thresh[ch]
```
All three must be true (AND combination, default) to trigger a signal.

### Config Bus (module ID = 4'h4)
```
cfg_addr = {4'h4, ch[1:0], reg[1:0]}
reg 0: magnitude threshold     (default: 100 Q24.8 ≈ 0.39)
reg 1: centroid lower bound    (default: 0)
reg 2: centroid upper bound    (default: max)
reg 3: power threshold (dB)    (default: 0)
```

### Output Signals
| m_signal | Meaning |
|---|---|
| 2'b00 | No signal |
| 2'b01 | BUY |
| 2'b10 | SELL |
| 2'b11 | ALERT (anomaly) |

`m_confidence` = sum of how much each threshold was exceeded, scaled by `confidence_scale`. Range 0–255.

---

## 11. End-to-End Testing with Alpaca Markets

### Setup
```bash
pip install alpaca-py numpy
export ALPACA_API_KEY=your_paper_key
export ALPACA_SECRET_KEY=your_paper_secret
```

Free paper trading API keys at: https://app.alpaca.markets/

### Data Flow
```
Alpaca API (AAPL 1-min bars)
    │
    ▼ fetch_alpaca.py
Normalize prices to [-1, 1]  ← avoids Q24.8 overflow in FFT
    │
    ▼ packets.bin
5-word binary packets (symbol_id, price Q24.8, volume, timestamp)
    │
    ▼ sim_main.cpp (Verilator driver)
Inject into tb_top.s_data/s_valid/s_last at clk_slow rate
    │
    ▼ Full pipeline simulation
market_data_parser → preprocessing → windowing → fft_core → cordic → post_fft → signal_logic
    │
    ▼ m_valid + m_channel + m_signal + m_confidence
    │
    ▼ Compare m_dom_bin vs reference_fft.npy dominant bin
```

### Expected Results
For AAPL 1-minute bars over a typical trading week:
- Dominant frequency bin likely in range 50–200 (period of a few hours)
- `reference_summary.txt` (written by `fetch_alpaca.py`) lists the top 5 bins
- FPGA output should match Python reference dominant bin (same integer bin number)

### Troubleshooting
| Symptom | Likely Cause |
|---|---|
| `packets.bin not found` | fetch_alpaca.py not run, or API keys not set |
| No `m_valid` output | Threshold too high — lower `r_mag_thresh` via config bus |
| Wrong dominant bin | Check N_size config matches `N_SAMPLES` in fetch_alpaca.py |
| Simulation hangs | FFT_LOAD phase waiting for samples — verify packet injection |

---

## Build Reference

```bash
cd FFT

# Generate twiddle ROM and compile
make

# Run with real Alpaca data
make sim-real

# Open waveforms (requires GTKWave)
make waves

# Clean build artifacts
make clean
```

**Verilator version:** 5.046 (installed at `/opt/homebrew/Cellar/verilator/5.046`)

**Key waveform signals to inspect in GTKWave:**
- `tb_top.dut.u_fft.fsm_state` — FFT FSM: IDLE(0) LOAD(1) COMPUTE(2) OUTPUT(3)
- `tb_top.dut.u_fft.m_valid` / `m_bin` — FFT bin output
- `tb_top.dut.u_postfft.m_dom_bin` — dominant bin
- `tb_top.dut.m_valid` / `m_signal` / `m_confidence` — final decisions
