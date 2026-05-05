`timescale 1ps/1ps
// Per-symbol normalization and uniform-rate resampling.
// Maps symbol_id[1:0] to channel 0-3 (4 channels).
// EMA mean subtraction (alpha = 1/256), ZOH gap fill, gap detection.
// Output rate: configurable per-channel sample counter (default 2048 cycles).
module preprocessing (
    input  wire        clk,
    input  wire        rst,
    // Config bus (module id 4'h0 → cfg_addr[7:4] == 4'h0)
    input  wire        cfg_valid,
    input  wire [7:0]  cfg_addr,
    input  wire [31:0] cfg_data,
    // From market_data_parser
    input  wire        s_valid,
    output reg         s_ready,
    input  wire [7:0]  s_symbol_id,
    input  wire [31:0] s_price,
    input  wire [15:0] s_volume,
    input  wire [47:0] s_timestamp,
    // To windowing
    output reg         m_valid,
    input  wire        m_ready,
    output reg  [1:0]  m_channel,
    output reg  signed [31:0] m_sample,
    output reg  [47:0] m_timestamp,
    output reg         m_gap
);

    // Per-channel state
    reg signed [31:0] mean_acc   [0:3];
    reg signed [31:0] last_price [0:3];
    reg        [47:0] last_ts    [0:3];
    reg        [15:0] sample_ctr [0:3];   // free-running output sample counter
    reg        [15:0] sample_rate[0:3];   // output period in clk cycles (config)
    reg        [47:0] gap_thresh [0:3];   // timestamp delta for gap detection

    // Output holding registers
    reg signed [31:0] out_sample [0:3];
    reg        [47:0] out_ts     [0:3];
    reg               out_gap    [0:3];
    reg               out_pending[0:3];   // sample ready, waiting for downstream

    // Round-robin channel arbiter for m_valid output
    reg [1:0] arb_ch;

    integer ch;

    // Initialize config defaults
    initial begin
        for (ch = 0; ch < 4; ch = ch + 1) begin
            sample_rate[ch] = 16'd2048;
            gap_thresh[ch]  = 48'hFFFFFFFFFFFF;  // disabled by default
            mean_acc[ch]    = 32'sd0;
            last_price[ch]  = 32'sd0;
            last_ts[ch]     = 48'd0;
            sample_ctr[ch]  = 16'd0;
            out_pending[ch] = 1'b0;
        end
    end

    // Config bus decode
    always @(posedge clk) begin
        if (cfg_valid && (cfg_addr[7:4] == 4'h0)) begin
            case (cfg_addr[3:2])  // channel select
                2'd0, 2'd1, 2'd2, 2'd3: begin
                    case (cfg_addr[1:0])
                        2'd0: sample_rate[cfg_addr[3:2]] <= cfg_data[15:0];
                        2'd1: gap_thresh[cfg_addr[3:2]]  <= {16'b0, cfg_data[31:0]};
                        default: ;
                    endcase
                end
            endcase
        end
    end

    // Incoming price processing
    wire [1:0] in_ch = s_symbol_id[1:0];

    always @(posedge clk) begin
        if (rst) begin
            s_ready <= 1'b1;
            for (ch = 0; ch < 4; ch = ch + 1) begin
                mean_acc[ch]    <= 32'sd0;
                last_price[ch]  <= 32'sd0;
                last_ts[ch]     <= 48'd0;
                sample_ctr[ch]  <= 16'd0;
                out_pending[ch] <= 1'b0;
            end
        end else begin
            s_ready <= 1'b1;

            // Update channel state on incoming price tick
            if (s_valid && s_ready) begin
                // EMA mean update: mean += (price - mean) >> 8  (alpha = 1/256)
                mean_acc[in_ch] <= mean_acc[in_ch] +
                                   ((s_price - mean_acc[in_ch]) >>> 8);
                last_price[in_ch] <= s_price;
                last_ts[in_ch]    <= s_timestamp;
            end

            // Per-channel sample-rate output trigger
            for (ch = 0; ch < 4; ch = ch + 1) begin
                if (sample_ctr[ch] == 0) begin
                    sample_ctr[ch] <= sample_rate[ch] - 1;
                    if (!out_pending[ch]) begin
                        out_pending[ch] <= 1'b1;
                        // normalized sample = last_price - mean
                        out_sample[ch] <= last_price[ch] - mean_acc[ch];
                        out_ts[ch]     <= last_ts[ch];
                        // gap: last_ts not updated this period → ZOH
                        out_gap[ch]    <= (last_ts[ch] == out_ts[ch]);
                    end
                end else begin
                    sample_ctr[ch] <= sample_ctr[ch] - 1;
                end
            end
        end
    end

    // Round-robin output arbiter
    always @(posedge clk) begin
        if (rst) begin
            m_valid <= 1'b0;
            arb_ch  <= 2'd0;
        end else begin
            if (!m_valid || m_ready) begin
                m_valid <= 1'b0;
                // Scan channels round-robin
                begin : arb_scan
                    integer j;
                    for (j = 0; j < 4; j = j + 1) begin
                        if (!m_valid) begin
                            if (out_pending[(arb_ch + j[1:0])]) begin
                                m_valid   <= 1'b1;
                                m_channel <= arb_ch + j[1:0];
                                m_sample  <= out_sample[arb_ch + j[1:0]];
                                m_timestamp <= out_ts[arb_ch + j[1:0]];
                                m_gap     <= out_gap[arb_ch + j[1:0]];
                                out_pending[arb_ch + j[1:0]] <= 1'b0;
                                arb_ch <= arb_ch + j[1:0] + 1;
                            end
                        end
                    end
                end
            end
        end
    end

endmodule
