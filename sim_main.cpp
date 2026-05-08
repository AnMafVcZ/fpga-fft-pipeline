// Verilator simulation driver for the FFT HFT pipeline.
// Matches the 4bitcounter/main.cpp pattern (VerilatedContext new-style API).
// Signals accessed via top->rootp->tb_top__DOT__<signal>.
//
// Clock domains:
//   clk_fast: half-period = 1 ps → symbolic 300 MHz
//   clk_slow: half-period = 2 ps → symbolic ~150 MHz (approximates 156.25/300 ratio)
//
// Reads packets.bin from fetch_alpaca.py; falls back to synthetic sine test data.

#include "Vtb_top.h"
#include "Vtb_top___024root.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include <cstdio>
#include <cstdint>
#include <cmath>

#define CLK_FAST_HALF  1ULL
#define CLK_SLOW_HALF  2ULL
#define MAX_SIM_TIME   50000000ULL   // 50 M sim units (enough for a full FFT frame)

#define PKT_WORDS 5

// Big-endian byte swap
static uint32_t bswap32(uint32_t v) {
    return ((v & 0xFF000000u) >> 24) | ((v & 0x00FF0000u) >>  8) |
           ((v & 0x0000FF00u) <<  8) | ((v & 0x000000FFu) << 24);
}

// Convenience macros for signal access
#define SIG(name)  (top->rootp->tb_top__DOT__##name)

int main(int argc, char** argv) {
    VerilatedContext* ctx = new VerilatedContext;
    ctx->commandArgs(argc, argv);

    Vtb_top* top = new Vtb_top(ctx);

    VerilatedVcdC* tfp = new VerilatedVcdC;
    ctx->traceEverOn(true);
    top->trace(tfp, 99);
    tfp->open("fft_tb.vcd");

    // Initialize signals via rootp
    SIG(clk_fast)  = 0;  SIG(clk_slow)  = 0;
    SIG(rst_fast)  = 1;  SIG(rst_slow)  = 1;
    SIG(s_valid)   = 0;  SIG(s_data)    = 0;  SIG(s_last) = 0;
    SIG(cfg_valid) = 0;  SIG(cfg_addr)  = 0;  SIG(cfg_data) = 0;

    top->eval();

    // Try to open real packet file
    FILE* pkt_file = fopen("packets.bin", "rb");
    if (!pkt_file)
        fprintf(stderr, "[sim] packets.bin not found — using synthetic sine (bin 64 of 256)\n");

    // Synthetic test data: single-tone sine at bin 64, N=256
    static const int SYNTH_N = 256;
    static uint32_t synth[SYNTH_N][PKT_WORDS];
    if (!pkt_file) {
        for (int n = 0; n < SYNTH_N; n++) {
            double angle = 2.0 * 3.14159265 * 64.0 * n / SYNTH_N;
            int32_t pq = (int32_t)round(sin(angle) * 128.0 * 256.0);
            uint64_t ts = (uint64_t)n * 60000000ULL;
            synth[n][0] = 0u;
            synth[n][1] = (uint32_t)pq;
            synth[n][2] = (100u << 16);
            synth[n][3] = (uint32_t)(ts >> 16);
            synth[n][4] = (uint32_t)((ts & 0xFFFF) << 16);
        }
    }

    int  pkt_word_idx = 0;
    int  pkt_count    = 0;
    int  synth_idx    = 0;
    int  total_pkts   = pkt_file ? 1024 : SYNTH_N;
    bool done_sending = false;

    uint64_t sim_time = 0;
    printf("[sim] Starting. Total packets to send: %d\n", total_pkts);

    while (!ctx->gotFinish() && sim_time < MAX_SIM_TIME) {

        // Release reset after 10 fast-clock periods = 20 ps
        if (sim_time == 20) {
            SIG(rst_fast) = 0;
            SIG(rst_slow) = 0;
            printf("[sim] Reset released at t=%llu\n", (unsigned long long)sim_time);
        }

        // Config writes: preprocessing sample_rate[ch0]=4 (fast), ch1-3=0xFFFF (disabled).
        // cfg_addr[7:4]=4'h0 (preprocessing), [3:2]=ch, [1:0]=reg0 (sample_rate).
        if (sim_time == 22) {
            SIG(cfg_valid) = 1;
            SIG(cfg_addr)  = 0x00;   // ch0 sample_rate
            SIG(cfg_data)  = 4;
        } else if (sim_time == 24) {
            SIG(cfg_addr)  = 0x04;   // ch1 sample_rate → max (disabled)
            SIG(cfg_data)  = 0xFFFF;
        } else if (sim_time == 26) {
            SIG(cfg_addr)  = 0x08;   // ch2 sample_rate → max
            SIG(cfg_data)  = 0xFFFF;
        } else if (sim_time == 28) {
            SIG(cfg_addr)  = 0x0C;   // ch3 sample_rate → max
            SIG(cfg_data)  = 0xFFFF;
        } else if (sim_time == 30) {
            SIG(cfg_valid) = 0;
        }

        // Toggle clocks
        if (sim_time % CLK_FAST_HALF == 0)
            SIG(clk_fast) = !SIG(clk_fast);
        if (sim_time % CLK_SLOW_HALF == 0)
            SIG(clk_slow) = !SIG(clk_slow);

        bool fast_posedge = (sim_time % CLK_FAST_HALF == 0) && SIG(clk_fast);
        bool slow_posedge = (sim_time % CLK_SLOW_HALF == 0) && SIG(clk_slow);

        // Inject packets on slow posedge
        if (slow_posedge && !SIG(rst_slow) && !done_sending) {
            uint32_t word = 0;
            bool have_word = false;

            if (pkt_file) {
                uint32_t raw;
                if (fread(&raw, 4, 1, pkt_file) == 1) {
                    word = bswap32(raw);
                    have_word = true;
                }
            } else if (pkt_count < total_pkts) {
                word = synth[synth_idx][pkt_word_idx];
                have_word = true;
            }

            if (have_word) {
                SIG(s_valid) = 1;
                SIG(s_data)  = word;
                SIG(s_last)  = (pkt_word_idx == PKT_WORDS - 1) ? 1 : 0;

                pkt_word_idx++;
                if (pkt_word_idx == PKT_WORDS) {
                    pkt_word_idx = 0;
                    pkt_count++;
                    synth_idx = (synth_idx + 1) % SYNTH_N;
                    if (pkt_count % 64 == 0)
                        printf("[sim] Sent %d packets\n", pkt_count);
                    if (pkt_count >= total_pkts) {
                        done_sending = true;
                        SIG(s_valid) = 0;
                        printf("[sim] All %d packets sent\n", total_pkts);
                    }
                }
            } else {
                SIG(s_valid) = 0;
                done_sending = true;
            }
        }

        // Monitor fast-domain outputs
        if (fast_posedge && SIG(m_valid)) {
            printf("[sim] SIGNAL ch=%d sig=%d conf=%d lat_ts=%llu src_ts=%llu\n",
                   (int)SIG(m_channel),
                   (int)SIG(m_signal),
                   (int)SIG(m_confidence),
                   (unsigned long long)SIG(m_latency_ts),
                   (unsigned long long)SIG(m_source_ts));
        }

        top->eval();
        tfp->dump((uint64_t)sim_time);
        ctx->timeInc(1);
        sim_time++;
    }

    printf("[sim] Done at t=%llu. Packets sent: %d\n",
           (unsigned long long)sim_time, pkt_count);

    if (pkt_file) fclose(pkt_file);
    tfp->close();
    delete top;
    delete tfp;
    delete ctx;
    return 0;
}
