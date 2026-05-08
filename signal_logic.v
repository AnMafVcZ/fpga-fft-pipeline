`timescale 1ps/1ps
// Trading signal decision engine. Evaluates spectral analysis results
// against per-channel configurable thresholds and fires a decision output.
module signal_logic (
    input  wire        clk,
    input  wire        rst,
    // Config (module id 4'h4)
    input  wire        cfg_valid,
    input  wire [7:0]  cfg_addr,
    input  wire [31:0] cfg_data,
    // From post_fft
    input  wire        s_valid,
    input  wire [1:0]  s_channel,
    input  wire [11:0] s_dom_bin,
    input  wire signed [31:0] s_dom_mag,
    input  wire signed [31:0] s_dom_phase,
    input  wire signed [31:0] s_centroid,
    input  wire signed [31:0] s_power_db,
    input  wire [47:0] s_timestamp,
    // Decision output
    output reg         m_valid,
    output reg  [1:0]  m_channel,
    output reg  [1:0]  m_signal,      // 00=none, 01=buy, 10=sell, 11=alert
    output reg  [7:0]  m_confidence,
    output reg  [47:0] m_latency_ts,
    output reg  [47:0] m_source_ts
);

    // Per-channel threshold registers: 8 registers per channel
    // cfg_addr[7:4] == 4'h4 (module id), cfg_addr[3:2] = channel, cfg_addr[1:0] = reg index
    reg signed [31:0] r_mag_thresh   [0:3];
    reg signed [31:0] r_centroid_lo  [0:3];
    reg signed [31:0] r_centroid_hi  [0:3];
    reg signed [31:0] r_power_thresh [0:3];
    reg signed [31:0] r_phase_thresh [0:3];
    reg        [7:0]  r_conf_scale   [0:3];

    // Free-running clock counter for latency timestamp
    reg [47:0] clk_cnt;

    integer ch;
    initial begin
        for (ch = 0; ch < 4; ch = ch + 1) begin
            r_mag_thresh  [ch] = 32'sd100;  // default: fire if dom_mag > 100 Q24.8
            r_centroid_lo [ch] = 32'sd0;
            r_centroid_hi [ch] = 32'h7FFFFFFF;
            r_power_thresh[ch] = -32'sd1;  // -1: any non-negative power_db passes
            r_phase_thresh[ch] = 32'sd0;
            r_conf_scale  [ch] = 8'd1;
        end
    end

    // Config decode
    always @(posedge clk) begin
        if (cfg_valid && (cfg_addr[7:4] == 4'h4)) begin
            case (cfg_addr[1:0])
                2'd0: r_mag_thresh  [cfg_addr[3:2]] <= $signed(cfg_data);
                2'd1: r_centroid_lo [cfg_addr[3:2]] <= $signed(cfg_data);
                2'd2: r_centroid_hi [cfg_addr[3:2]] <= $signed(cfg_data);
                2'd3: r_power_thresh[cfg_addr[3:2]] <= $signed(cfg_data);
                default: ;
            endcase
        end
    end

    // Clock counter
    always @(posedge clk) begin
        if (rst) clk_cnt <= 48'd0;
        else     clk_cnt <= clk_cnt + 1;
    end

    // ---- Stage 1: compute conditions ----
    reg        p1_valid;
    reg [1:0]  p1_ch;
    reg        p1_cond_mag, p1_cond_cent, p1_cond_power;
    reg signed [31:0] p1_mag_margin, p1_cent_margin, p1_pow_margin;
    reg [47:0] p1_ts;

    always @(posedge clk) begin
        if (rst) begin
            p1_valid <= 1'b0;
        end else begin
            p1_valid <= s_valid;
            p1_ch    <= s_channel;
            p1_ts    <= s_timestamp;
            p1_cond_mag   <= ($signed(s_dom_mag)  > $signed(r_mag_thresh  [s_channel]));
            p1_cond_cent  <= ($signed(s_centroid)  > $signed(r_centroid_lo[s_channel])) &&
                             ($signed(s_centroid)  < $signed(r_centroid_hi[s_channel]));
            p1_cond_power <= ($signed(s_power_db) > $signed(r_power_thresh[s_channel]));
            p1_mag_margin  <= $signed(s_dom_mag)  - $signed(r_mag_thresh  [s_channel]);
            p1_cent_margin <= $signed(s_centroid) - $signed(r_centroid_lo [s_channel]);
            p1_pow_margin  <= $signed(s_power_db) - $signed(r_power_thresh[s_channel]);
        end
    end

    // ---- Stage 2: combine and encode decision ----
    always @(posedge clk) begin
        if (rst) begin
            m_valid <= 1'b0;
        end else begin
            m_valid <= p1_valid && (p1_cond_mag && p1_cond_cent && p1_cond_power);
            m_channel    <= p1_ch;
            m_latency_ts <= clk_cnt;
            m_source_ts  <= p1_ts;
            // Signal direction: phase-based heuristic (positive phase → buy, negative → sell)
            m_signal     <= (p1_cond_mag && p1_cond_cent && p1_cond_power) ?
                            2'b01 : 2'b00;  // simplified: always buy when triggered
            // Confidence: sum of normalized margins, clamped to 8 bits
            begin
                reg [31:0] conf_raw;
                conf_raw    = ((p1_mag_margin  >>> 8) +
                               (p1_cent_margin >>> 8) +
                               (p1_pow_margin  >>> 8)) * r_conf_scale[p1_ch];
                m_confidence <= conf_raw[7:0];
            end
        end
    end

endmodule
