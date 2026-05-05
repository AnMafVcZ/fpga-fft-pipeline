`timescale 1ps/1ps
// System testbench. Clock and stimulus driven from sim_main.cpp.
// VCD dump is initiated here.
module tb_top ();

    reg        clk_fast;
    reg        clk_slow;
    reg        rst_fast;
    reg        rst_slow;

    // Market data input (slow domain)
    reg        s_valid;
    wire       s_ready;
    reg [31:0] s_data;
    reg        s_last;

    // Config bus (fast domain)
    reg        cfg_valid;
    reg [7:0]  cfg_addr;
    reg [31:0] cfg_data;

    // Signal output
    wire        m_valid;
    wire [1:0]  m_channel;
    wire [1:0]  m_signal;
    wire [7:0]  m_confidence;
    wire [47:0] m_latency_ts;
    wire [47:0] m_source_ts;

    // DUT instantiation
    top dut (
        .clk_fast    (clk_fast),
        .rst_fast    (rst_fast),
        .clk_slow    (clk_slow),
        .rst_slow    (rst_slow),
        .s_valid     (s_valid),
        .s_ready     (s_ready),
        .s_data      (s_data),
        .s_last      (s_last),
        .cfg_valid   (cfg_valid),
        .cfg_addr    (cfg_addr),
        .cfg_data    (cfg_data),
        .m_valid     (m_valid),
        .m_channel   (m_channel),
        .m_signal    (m_signal),
        .m_confidence(m_confidence),
        .m_latency_ts(m_latency_ts),
        .m_source_ts (m_source_ts)
    );

    // VCD waveform dump
    initial begin
        $dumpfile("fft_tb.vcd");
        $dumpvars(0, tb_top);
    end

endmodule
