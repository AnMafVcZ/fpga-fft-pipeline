`timescale 1ps/1ps
// Top-level system integration. Wires all pipeline blocks.
// Two clock domains: clk_slow (156.25 MHz ingress) and clk_fast (300 MHz processing).
// Config bus wired in parallel; each module decodes cfg_addr[7:4] as module ID.
module top (
    input  wire        clk_fast,
    input  wire        rst_fast,
    input  wire        clk_slow,
    input  wire        rst_slow,
    // Market data input (slow domain) — raw 32-bit word stream
    input  wire        s_valid,
    output wire        s_ready,
    input  wire [31:0] s_data,
    input  wire        s_last,
    // Config bus (fast domain)
    input  wire        cfg_valid,
    input  wire [7:0]  cfg_addr,
    input  wire [31:0] cfg_data,
    // Signal output (fast domain)
    output wire        m_valid,
    output wire [1:0]  m_channel,
    output wire [1:0]  m_signal,
    output wire [7:0]  m_confidence,
    output wire [47:0] m_latency_ts,
    output wire [47:0] m_source_ts
);

    // ---- async_fifo: slow → fast domain ----
    // Pack parser input fields into 32-bit FIFO word (reuse s_data + s_last directly)
    // We pass the raw word stream through the FIFO; parser runs in fast domain.
    wire        fifo_wr_full;
    wire [32:0] fifo_wr_data = {s_last, s_data};  // 33 bits: {last, data}
    wire        fifo_rd_empty;
    wire [32:0] fifo_rd_data;
    wire        fifo_rd_en;

    // FIFO write: accept whenever not full
    assign s_ready = !fifo_wr_full;

    async_fifo #(
        .DATA_WIDTH(33),
        .ADDR_WIDTH(5)
    ) u_fifo (
        .wr_clk  (clk_slow),
        .wr_rst  (rst_slow),
        .wr_en   (s_valid && s_ready),
        .wr_data (fifo_wr_data),
        .wr_full (fifo_wr_full),
        .rd_clk  (clk_fast),
        .rd_rst  (rst_fast),
        .rd_en   (fifo_rd_en),
        .rd_data (fifo_rd_data),
        .rd_empty(fifo_rd_empty)
    );

    // ---- market_data_parser (fast domain) ----
    wire        parser_m_valid;
    wire        parser_m_ready;
    wire [7:0]  parser_symbol_id;
    wire [31:0] parser_price;
    wire [15:0] parser_volume;
    wire [47:0] parser_timestamp;
    wire        parser_s_ready;

    assign fifo_rd_en = parser_s_ready && !fifo_rd_empty;

    market_data_parser u_parser (
        .clk         (clk_fast),
        .rst         (rst_fast),
        .s_valid     (!fifo_rd_empty),
        .s_ready     (parser_s_ready),
        .s_data      (fifo_rd_data[31:0]),
        .s_last      (fifo_rd_data[32]),
        .m_valid     (parser_m_valid),
        .m_ready     (parser_m_ready),
        .m_symbol_id (parser_symbol_id),
        .m_price     (parser_price),
        .m_volume    (parser_volume),
        .m_timestamp (parser_timestamp)
    );

    // ---- preprocessing (fast domain) ----
    wire        preproc_m_valid;
    wire        preproc_m_ready;
    wire [1:0]  preproc_channel;
    wire signed [31:0] preproc_sample;
    wire [47:0] preproc_timestamp;
    wire        preproc_gap;
    wire        preproc_s_ready;
    assign parser_m_ready = preproc_s_ready;

    preprocessing u_preproc (
        .clk         (clk_fast),
        .rst         (rst_fast),
        .cfg_valid   (cfg_valid),
        .cfg_addr    (cfg_addr),
        .cfg_data    (cfg_data),
        .s_valid     (parser_m_valid),
        .s_ready     (preproc_s_ready),
        .s_symbol_id (parser_symbol_id),
        .s_price     (parser_price),
        .s_volume    (parser_volume),
        .s_timestamp (parser_timestamp),
        .m_valid     (preproc_m_valid),
        .m_ready     (preproc_m_ready),
        .m_channel   (preproc_channel),
        .m_sample    (preproc_sample),
        .m_timestamp (preproc_timestamp),
        .m_gap       (preproc_gap)
    );

    // ---- windowing (fast domain) ----
    wire        window_m_valid;
    wire        window_m_ready;
    wire [1:0]  window_channel;
    wire signed [31:0] window_re, window_im;
    wire [11:0] window_bin_idx;
    wire        window_frame_last;
    wire [47:0] window_timestamp;
    wire        window_s_ready;
    assign preproc_m_ready = window_s_ready;

    windowing u_window (
        .clk          (clk_fast),
        .rst          (rst_fast),
        .cfg_valid    (cfg_valid),
        .cfg_addr     (cfg_addr),
        .cfg_data     (cfg_data),
        .s_valid      (preproc_m_valid),
        .s_ready      (window_s_ready),
        .s_channel    (preproc_channel),
        .s_sample     (preproc_sample),
        .s_timestamp  (preproc_timestamp),
        .s_gap        (preproc_gap),
        .m_valid      (window_m_valid),
        .m_ready      (window_m_ready),
        .m_channel    (window_channel),
        .m_re         (window_re),
        .m_im         (window_im),
        .m_bin_idx    (window_bin_idx),
        .m_frame_last (window_frame_last),
        .m_timestamp  (window_timestamp)
    );

    // ---- Shared butterfly and twiddle ROM ----
    wire        bfly_in_valid;
    wire signed [31:0] bfly_a_re, bfly_a_im, bfly_b_re, bfly_b_im;
    wire signed [31:0] bfly_w_re, bfly_w_im;
    wire [11:0] bfly_tag;
    wire        bfly_out_valid;
    wire signed [31:0] bfly_p_re, bfly_p_im, bfly_q_re, bfly_q_im;
    wire [11:0] bfly_tag_out;

    butterfly u_butterfly (
        .clk       (clk_fast),
        .rst       (rst_fast),
        .in_valid  (bfly_in_valid),
        .a_re      (bfly_a_re),  .a_im (bfly_a_im),
        .b_re      (bfly_b_re),  .b_im (bfly_b_im),
        .w_re      (bfly_w_re),  .w_im (bfly_w_im),
        .tag       (bfly_tag),
        .out_valid (bfly_out_valid),
        .p_re      (bfly_p_re),  .p_im (bfly_p_im),
        .q_re      (bfly_q_re),  .q_im (bfly_q_im),
        .tag_out   (bfly_tag_out)
    );

    wire [10:0] trom_addr;
    wire        trom_rd_en;
    wire signed [31:0] trom_cos, trom_sin;

    twiddle_rom u_trom (
        .clk     (clk_fast),
        .addr    (trom_addr),
        .rd_en   (trom_rd_en),
        .cos_out (trom_cos),
        .sin_out (trom_sin)
    );

    // ---- fft_core (fast domain) ----
    wire        fft_m_valid;
    wire        fft_m_ready;
    wire [1:0]  fft_channel;
    wire signed [31:0] fft_re, fft_im;
    wire [11:0] fft_bin;
    wire        fft_frame_last;
    wire [47:0] fft_timestamp;
    wire [3:0]  fft_scale_exp;
    wire        fft_s_ready;
    assign window_m_ready = fft_s_ready;

    fft_core u_fft (
        .clk           (clk_fast),
        .rst           (rst_fast),
        .cfg_valid     (cfg_valid),
        .cfg_addr      (cfg_addr),
        .cfg_data      (cfg_data),
        .s_valid       (window_m_valid),
        .s_ready       (fft_s_ready),
        .s_channel     (window_channel),
        .s_re          (window_re),
        .s_im          (window_im),
        .s_bin_idx     (window_bin_idx),
        .s_frame_last  (window_frame_last),
        .s_timestamp   (window_timestamp),
        .bfly_in_valid (bfly_in_valid),
        .bfly_a_re     (bfly_a_re),  .bfly_a_im (bfly_a_im),
        .bfly_b_re     (bfly_b_re),  .bfly_b_im (bfly_b_im),
        .bfly_w_re     (bfly_w_re),  .bfly_w_im (bfly_w_im),
        .bfly_tag      (bfly_tag),
        .bfly_out_valid(bfly_out_valid),
        .bfly_p_re     (bfly_p_re),  .bfly_p_im (bfly_p_im),
        .bfly_q_re     (bfly_q_re),  .bfly_q_im (bfly_q_im),
        .bfly_tag_out  (bfly_tag_out),
        .trom_addr     (trom_addr),
        .trom_rd_en    (trom_rd_en),
        .trom_cos      (trom_cos),
        .trom_sin      (trom_sin),
        .m_valid       (fft_m_valid),
        .m_ready       (fft_m_ready),
        .m_channel     (fft_channel),
        .m_re          (fft_re),
        .m_im          (fft_im),
        .m_bin         (fft_bin),
        .m_frame_last  (fft_frame_last),
        .m_timestamp   (fft_timestamp),
        .m_scale_exp   (fft_scale_exp)
    );

    // ---- cordic (fast domain) ----
    wire        cordic_out_valid;
    wire signed [31:0] cordic_mag, cordic_phase;
    wire [11:0] cordic_bin_tag;
    wire        cordic_m_ready;

    // Pack bin + channel + frame_last into tag
    wire [11:0] cordic_in_tag = fft_bin;

    cordic u_cordic (
        .clk       (clk_fast),
        .rst       (rst_fast),
        .in_valid  (fft_m_valid),
        .in_re     (fft_re),
        .in_im     (fft_im),
        .in_tag    (cordic_in_tag),
        .out_valid (cordic_out_valid),
        .out_mag   (cordic_mag),
        .out_phase (cordic_phase),
        .out_tag   (cordic_bin_tag)
    );
    assign fft_m_ready = 1'b1;  // CORDIC is always ready (pipelined, no backpressure)

    // Delay fft metadata by 18 cycles to align with cordic output.
    // CORDIC latency = 18 cycles: stage0 (1) + iterations 1-16 (16) + output reg (1).
    reg [1:0]  cordic_delay_ch    [0:17];
    reg        cordic_delay_fl    [0:17];
    reg [47:0] cordic_delay_ts    [0:17];
    reg [3:0]  cordic_delay_scale [0:17];
    // Gate frame_last with fft_m_valid so stale high-after-last-bin doesn't propagate.
    wire fft_frame_last_gated = fft_frame_last && fft_m_valid;
    integer d;

    always @(posedge clk_fast) begin
        cordic_delay_ch   [0] <= fft_channel;
        cordic_delay_fl   [0] <= fft_frame_last_gated;
        cordic_delay_ts   [0] <= fft_timestamp;
        cordic_delay_scale[0] <= fft_scale_exp;
        for (d = 1; d <= 17; d = d + 1) begin
            cordic_delay_ch   [d] <= cordic_delay_ch   [d-1];
            cordic_delay_fl   [d] <= cordic_delay_fl   [d-1];
            cordic_delay_ts   [d] <= cordic_delay_ts   [d-1];
            cordic_delay_scale[d] <= cordic_delay_scale[d-1];
        end
    end

    // ---- post_fft (fast domain) ----
    wire        postfft_m_valid;
    wire        postfft_m_ready;
    wire [1:0]  postfft_channel;
    wire [11:0] postfft_dom_bin;
    wire signed [31:0] postfft_dom_mag, postfft_dom_phase, postfft_centroid, postfft_power_db;
    wire [47:0] postfft_timestamp;

    post_fft u_postfft (
        .clk          (clk_fast),
        .rst          (rst_fast),
        .cfg_valid    (cfg_valid),
        .cfg_addr     (cfg_addr),
        .cfg_data     (cfg_data),
        .s_valid      (cordic_out_valid),
        .s_channel    (cordic_delay_ch   [17]),
        .s_mag        (cordic_mag),
        .s_phase      (cordic_phase),
        .s_bin        (cordic_bin_tag),
        .s_frame_last (cordic_delay_fl   [17]),
        .s_timestamp  (cordic_delay_ts   [17]),
        .s_scale_exp  (cordic_delay_scale[17]),
        .m_valid      (postfft_m_valid),
        .m_ready      (postfft_m_ready),
        .m_channel    (postfft_channel),
        .m_dom_bin    (postfft_dom_bin),
        .m_dom_mag    (postfft_dom_mag),
        .m_dom_phase  (postfft_dom_phase),
        .m_centroid   (postfft_centroid),
        .m_power_db   (postfft_power_db),
        .m_timestamp  (postfft_timestamp)
    );

    // ---- signal_logic (fast domain) ----
    wire sigl_m_ready = 1'b1;
    assign postfft_m_ready = sigl_m_ready;

    signal_logic u_siglogic (
        .clk          (clk_fast),
        .rst          (rst_fast),
        .cfg_valid    (cfg_valid),
        .cfg_addr     (cfg_addr),
        .cfg_data     (cfg_data),
        .s_valid      (postfft_m_valid),
        .s_channel    (postfft_channel),
        .s_dom_bin    (postfft_dom_bin),
        .s_dom_mag    (postfft_dom_mag),
        .s_dom_phase  (postfft_dom_phase),
        .s_centroid   (postfft_centroid),
        .s_power_db   (postfft_power_db),
        .s_timestamp  (postfft_timestamp),
        .m_valid      (m_valid),
        .m_channel    (m_channel),
        .m_signal     (m_signal),
        .m_confidence (m_confidence),
        .m_latency_ts (m_latency_ts),
        .m_source_ts  (m_source_ts)
    );

endmodule
