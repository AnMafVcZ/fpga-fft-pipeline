`timescale 1ps/1ps
// Top-level wrapper. Re-exports FFT/top.v as the fft module.
// Provides the same port interface as top.v.
module fft (
    input  wire        clk_fast,
    input  wire        rst_fast,
    input  wire        clk_slow,
    input  wire        rst_slow,
    input  wire        s_valid,
    output wire        s_ready,
    input  wire [31:0] s_data,
    input  wire        s_last,
    input  wire        cfg_valid,
    input  wire [7:0]  cfg_addr,
    input  wire [31:0] cfg_data,
    output wire        m_valid,
    output wire [1:0]  m_channel,
    output wire [1:0]  m_signal,
    output wire [7:0]  m_confidence,
    output wire [47:0] m_latency_ts,
    output wire [47:0] m_source_ts
);

    top u_top (
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

endmodule
