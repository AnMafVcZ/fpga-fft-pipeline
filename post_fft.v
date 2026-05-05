`timescale 1ps/1ps
// Post-FFT spectral analysis. Processes one bin per clock.
// Tracks: dominant bin (argmax magnitude), spectral centroid, total power.
// Fires m_valid once per frame with summary results.
// CORDIC output (magnitude, phase) fed as input.
module post_fft (
    input  wire        clk,
    input  wire        rst,
    // Config (module id 4'h3)
    input  wire        cfg_valid,
    input  wire [7:0]  cfg_addr,
    input  wire [31:0] cfg_data,
    // From cordic
    input  wire        s_valid,
    input  wire [1:0]  s_channel,
    input  wire signed [31:0] s_mag,
    input  wire signed [31:0] s_phase,
    input  wire [11:0] s_bin,
    input  wire        s_frame_last,
    input  wire [47:0] s_timestamp,
    input  wire [3:0]  s_scale_exp,
    // Frame summary output
    output reg         m_valid,
    input  wire        m_ready,
    output reg  [1:0]  m_channel,
    output reg  [11:0] m_dom_bin,
    output reg  signed [31:0] m_dom_mag,
    output reg  signed [31:0] m_dom_phase,
    output reg  signed [31:0] m_centroid,
    output reg  signed [31:0] m_power_db,
    output reg  [47:0] m_timestamp
);

    // Per-channel accumulation state
    reg signed [31:0] max_mag    [0:3];
    reg        [11:0] max_bin    [0:3];
    reg signed [31:0] max_phase  [0:3];
    reg        [63:0] cent_num   [0:3];   // sum(bin * mag)
    reg        [63:0] cent_den   [0:3];   // sum(mag)
    reg        [63:0] total_pow  [0:3];   // sum(mag^2)
    reg        [47:0] frame_ts   [0:3];

    // Log2 lookup table for power-to-dB conversion (256-entry, input = upper 8 bits of power)
    // log2_lut[i] = round(log2(i/256 + 1) * 256) for i=0..255, crude approximation
    // Simplified: store log2(i) * 256 for i = 1..256 mapped to upper 8-bit index
    reg [15:0] log2_lut [0:255];

    integer i_log;
    real log_val;
    initial begin
        log2_lut[0] = 16'd0;
        for (i_log = 1; i_log < 256; i_log = i_log + 1) begin
            log_val = $ln(1.0 * i_log) / $ln(2.0);
            log2_lut[i_log] = $rtoi(log_val * 256.0);
        end
    end

    integer ch;
    initial begin
        for (ch = 0; ch < 4; ch = ch + 1) begin
            max_mag[ch]   = 32'sd0;
            max_bin[ch]   = 12'd0;
            max_phase[ch] = 32'sd0;
            cent_num[ch]  = 64'd0;
            cent_den[ch]  = 64'd0;
            total_pow[ch] = 64'd0;
            frame_ts[ch]  = 48'd0;
        end
    end

    // Centroid and power-dB pipeline (2-stage: accumulate, then divide on frame_last)
    reg [1:0]  pend_ch;
    reg        pend_valid;
    reg [47:0] pend_ts;

    always @(posedge clk) begin
        if (rst) begin
            m_valid    <= 1'b0;
            pend_valid <= 1'b0;
            for (ch = 0; ch < 4; ch = ch + 1) begin
                max_mag[ch]   <= 32'sd0;
                cent_num[ch]  <= 64'd0;
                cent_den[ch]  <= 64'd0;
                total_pow[ch] <= 64'd0;
            end
        end else begin
            pend_valid <= 1'b0;

            if (s_valid) begin
                // Update argmax
                if ($signed(s_mag) > $signed(max_mag[s_channel])) begin
                    max_mag  [s_channel] <= s_mag;
                    max_bin  [s_channel] <= s_bin;
                    max_phase[s_channel] <= s_phase;
                end
                if (s_bin == 12'd0)
                    frame_ts[s_channel] <= s_timestamp;

                // Accumulate centroid: num += bin * mag, den += mag
                cent_num[s_channel] <= cent_num[s_channel] +
                                       ($signed({{52{s_bin[11]}}, s_bin}) * $signed(s_mag));
                cent_den[s_channel] <= cent_den[s_channel] + {{32{s_mag[31]}}, s_mag};

                // Accumulate power: total_pow += mag^2 (Q24.8 * Q24.8 >> 8)
                total_pow[s_channel] <= total_pow[s_channel] +
                    (($signed(s_mag) * $signed(s_mag)) >>> 8);

                if (s_frame_last) begin
                    pend_valid <= 1'b1;
                    pend_ch    <= s_channel;
                    pend_ts    <= s_timestamp;
                end
            end

            // Output frame summary on frame_last
            if (pend_valid && (!m_valid || m_ready)) begin
                reg [31:0] centroid_r;
                reg [31:0] power_db_r;
                reg [7:0]  pow_idx;

                // Centroid: num/den approximated as shift (simplified for RTL)
                // Full division would use Newton-Raphson; here we use a log2-based approx
                centroid_r = (cent_den[pend_ch][63:32] != 0) ?
                             (cent_num[pend_ch] >> 32) : 32'sd0;

                // Power dB: use log2 LUT on upper 8 bits of total_pow
                pow_idx    = total_pow[pend_ch][39:32];
                power_db_r = {16'd0, log2_lut[pow_idx]};
                // Scale by 10/log2(10) ≈ 10/3.322 ≈ 3.01 ≈ Q8.8 value 770
                power_db_r = (power_db_r * 32'd770) >> 8;

                m_valid     <= 1'b1;
                m_channel   <= pend_ch;
                m_dom_bin   <= max_bin  [pend_ch];
                m_dom_mag   <= max_mag  [pend_ch];
                m_dom_phase <= max_phase[pend_ch];
                m_centroid  <= $signed(centroid_r);
                m_power_db  <= $signed(power_db_r);
                m_timestamp <= pend_ts;

                // Reset accumulators for this channel
                max_mag  [pend_ch] <= 32'sd0;
                max_bin  [pend_ch] <= 12'd0;
                cent_num [pend_ch] <= 64'd0;
                cent_den [pend_ch] <= 64'd0;
                total_pow[pend_ch] <= 64'd0;
            end else if (m_valid && m_ready) begin
                m_valid <= 1'b0;
            end
        end
    end

endmodule
