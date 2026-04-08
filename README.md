# FPGA FFT Pipeline

FPGA-based FFT pipeline for real-time market data signal analysis, written in Verilog.
Designed around a dual-clock architecture: 156.25 MHz ingress (slow domain) and 300 MHz processing (fast domain).

## Architecture

```
market data (slow) → async_fifo → market_data_parser → preprocessing
                                                            ↓
signal_logic ← post_fft ← fft_core ← windowing ← preprocessing
    ↓
output (channel, signal, confidence, timestamps)
```

## Modules

| Module | Description |
|--------|-------------|
| `top.v` | Top-level integration, clock domain wiring, config bus |
| `async_fifo.v` | Gray-code async FIFO for slow→fast domain crossing |
| `market_data_parser.v` | Parses raw 32-bit word stream into price/volume fields |
| `preprocessing.v` | Windowing + normalization before FFT |
| `windowing.v` | Hann window coefficients applied per sample |
| `fft_core.v` | Radix-2 DIT FFT, parameterized point count |
| `butterfly.v` | Single butterfly unit with twiddle factor multiply |
| `cordic.v` | CORDIC-based complex multiply for twiddle factors |
| `twiddle_rom.v` | Pre-computed twiddle factor ROM |
| `post_fft.v` | Magnitude estimation, bin selection, peak detection |
| `signal_logic.v` | Classifies FFT output into trading signals with confidence |

## Tools

- Simulation: Verilator + `sim_main.cpp`
- Target: Xilinx Artix-7 / Kintex-7

## Run Simulation

```bash
make
```
