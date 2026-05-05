VERILATOR  = verilator
VFLAGS     = --cc --trace --top-module tb_top -Wall -Wno-fatal \
             -Wno-UNUSED -Wno-UNDRIVEN -Wno-INITIALDLY
CFLAGS     = -std=c++17

VSRCS = tb_top.v top.v async_fifo.v market_data_parser.v preprocessing.v \
        windowing.v fft_core.v butterfly.v twiddle_rom.v cordic.v \
        post_fft.v signal_logic.v

# Default: compile and run simulation
all: sim

# Step 1: generate twiddle ROM include file
twiddle_init.vh: gen_twiddle.py
	python3 gen_twiddle.py

# Step 2: fetch real market data (optional; requires ALPACA_API_KEY set)
packets.bin: fetch_alpaca.py
	python3 fetch_alpaca.py

# Step 3: compile with Verilator
obj_dir/Vtb_top: $(VSRCS) sim_main.cpp twiddle_init.vh
	$(VERILATOR) $(VFLAGS) $(VSRCS) --exe sim_main.cpp
	$(MAKE) -C obj_dir -f Vtb_top.mk Vtb_top

# Step 4: run simulation (uses packets.bin if available, else synthetic data)
sim: obj_dir/Vtb_top
	./obj_dir/Vtb_top

# Run with real Alpaca data (fetches first)
sim-real: obj_dir/Vtb_top packets.bin
	./obj_dir/Vtb_top

# Open waveform (requires GTKWave)
waves: fft_tb.vcd
	gtkwave fft_tb.vcd &

clean:
	rm -rf obj_dir fft_tb.vcd packets.bin reference_fft.npy twiddle_init.vh

.PHONY: all sim sim-real waves clean
