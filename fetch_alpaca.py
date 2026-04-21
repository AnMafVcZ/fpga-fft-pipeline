#!/usr/bin/env python3
"""
Fetch historical stock bar data from Alpaca Markets Free API.
Converts close prices to Q24.8 packet format for Verilator replay.
Also computes NumPy FFT reference for output verification.

Setup:
    pip install alpaca-py numpy
    export ALPACA_API_KEY=your_key_here
    export ALPACA_SECRET_KEY=your_secret_here

Output files:
    packets.bin        — binary packet stream for sim_main.cpp
    reference_fft.npy  — NumPy magnitude spectrum for comparison
"""
import os
import sys
import struct
import math
import numpy as np
from datetime import datetime, timezone

try:
    from alpaca.data.historical import StockHistoricalDataClient
    from alpaca.data.requests import StockBarsRequest
    from alpaca.data.timeframe import TimeFrame
except ImportError:
    print("ERROR: alpaca-py not installed. Run: pip install alpaca-py numpy")
    sys.exit(1)

# ---- Configuration ----
SYMBOL    = os.environ.get("ALPACA_SYMBOL", "AAPL")
N_SAMPLES = int(os.environ.get("ALPACA_N", "1024"))  # must match FFT size
START_DATE = datetime(2024, 1, 2, tzinfo=timezone.utc)
SYMBOL_ID  = 0   # maps AAPL → channel 0

API_KEY    = os.environ.get("ALPACA_API_KEY", "")
SECRET_KEY = os.environ.get("ALPACA_SECRET_KEY", "")

if not API_KEY or not SECRET_KEY:
    print("ERROR: Set ALPACA_API_KEY and ALPACA_SECRET_KEY environment variables.")
    print("Free paper trading keys available at: https://app.alpaca.markets/")
    sys.exit(1)


def to_q24_8(f: float) -> int:
    """Convert float to Q24.8 signed 32-bit integer (unsigned packed form)."""
    v = int(round(f * 256.0))
    v = max(-0x80000000, min(0x7FFFFFFF, v))
    return v & 0xFFFFFFFF


def write_packet(f, symbol_id: int, price_q: int, volume: int, timestamp_us: int):
    """Write one 5-word (20-byte) big-endian packet to file f."""
    ts = timestamp_us & 0xFFFFFFFFFFFF
    words = [
        (symbol_id & 0xFF) << 24,
        price_q & 0xFFFFFFFF,
        (volume & 0xFFFF) << 16,
        (ts >> 16) & 0xFFFFFFFF,
        ((ts & 0xFFFF) << 16) & 0xFFFFFFFF,
    ]
    f.write(struct.pack(">5I", *words))


def main():
    print(f"Fetching {N_SAMPLES} bars for {SYMBOL} from Alpaca...")
    client = StockHistoricalDataClient(API_KEY, SECRET_KEY)

    req = StockBarsRequest(
        symbol_or_symbols=SYMBOL,
        timeframe=TimeFrame.Minute,
        start=START_DATE,
        limit=N_SAMPLES + 512,  # extra buffer for any missing bars
    )

    try:
        bar_data = client.get_stock_bars(req)
        bars = bar_data[SYMBOL]
    except Exception as e:
        print(f"ERROR fetching data: {e}")
        sys.exit(1)

    if len(bars) < N_SAMPLES:
        print(f"WARNING: only {len(bars)} bars available, need {N_SAMPLES}. "
              f"Repeating to fill.")
        while len(bars) < N_SAMPLES:
            bars = bars + bars
    bars = bars[:N_SAMPLES]

    prices     = np.array([b.close for b in bars], dtype=np.float64)
    volumes    = np.array([b.volume for b in bars], dtype=np.int64)
    timestamps = np.array([int(b.timestamp.timestamp() * 1e6)
                            for b in bars], dtype=np.int64)

    # Normalize prices to roughly [-1, 1] for Q24.8 pipeline
    mu    = prices.mean()
    sigma = prices.std() + 1e-9
    prices_norm = (prices - mu) / sigma

    print(f"Price stats: mean={mu:.4f}, std={sigma:.4f}, "
          f"norm range=[{prices_norm.min():.3f}, {prices_norm.max():.3f}]")

    # Write packets.bin
    with open("packets.bin", "wb") as f:
        for i in range(N_SAMPLES):
            price_q = to_q24_8(prices_norm[i])
            vol     = int(volumes[i]) & 0xFFFF
            ts_us   = int(timestamps[i])
            write_packet(f, SYMBOL_ID, price_q, vol, ts_us)

    print(f"Wrote packets.bin ({N_SAMPLES} packets × 20 bytes = {N_SAMPLES*20} bytes)")

    # Compute NumPy FFT reference
    fft_ref    = np.fft.fft(prices_norm)
    magnitudes = np.abs(fft_ref)

    # Dominant bin (exclude DC at bin 0)
    half = N_SAMPLES // 2
    dom_bin = int(np.argmax(magnitudes[1:half])) + 1
    dom_freq_hz = dom_bin / (N_SAMPLES * 60.0)  # 1-minute bars → 60s period

    print(f"NumPy FFT reference:")
    print(f"  Dominant bin : {dom_bin}")
    print(f"  Frequency    : {dom_freq_hz*1000:.4f} mHz  "
          f"(period ≈ {1/dom_freq_hz/3600:.1f} hours)")
    print(f"  Magnitude    : {magnitudes[dom_bin]:.4f}")

    np.save("reference_fft.npy", magnitudes)
    print("Wrote reference_fft.npy")

    # Write human-readable summary
    with open("reference_summary.txt", "w") as f:
        f.write(f"Symbol     : {SYMBOL}\n")
        f.write(f"N samples  : {N_SAMPLES}\n")
        f.write(f"Mean price : {mu:.4f}\n")
        f.write(f"Std price  : {sigma:.4f}\n")
        f.write(f"Dominant bin : {dom_bin}\n")
        f.write(f"Dominant freq: {dom_freq_hz*1000:.4f} mHz\n")
        f.write(f"Top 5 bins:\n")
        top5 = np.argsort(magnitudes[1:half])[-5:][::-1] + 1
        for b in top5:
            f.write(f"  bin {b:4d}  mag={magnitudes[b]:.4f}  "
                    f"freq={b/(N_SAMPLES*60)*1000:.4f} mHz\n")

    print("Wrote reference_summary.txt")
    print(f"\nTo run simulation with this data:")
    print(f"  make sim-real")
    print(f"\nExpected FPGA dominant bin output: {dom_bin}")


if __name__ == "__main__":
    main()

def fetch_bars(symbol: str, timeframe: str = "1Min", limit: int = 1000):
    """Fetch OHLCV bars from Alpaca and return as numpy array for FFT input."""
    import os, numpy as np
    key = os.environ.get("APCA_API_KEY_ID", "")
    secret = os.environ.get("APCA_API_SECRET_KEY", "")
    if not key or not secret:
        raise EnvironmentError("Set APCA_API_KEY_ID and APCA_API_SECRET_KEY")
    # returns shape (limit, 5): open, high, low, close, volume
    return np.zeros((limit, 5))  # placeholder until live feed connected
