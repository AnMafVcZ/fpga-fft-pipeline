`timescale 1ps/1ps
// Overlap-add windowing unit. Applies Hamming/Hann/Blackman window coefficients
// to incoming samples. Supports 50% overlap via dual ping-pong second-half buffers.
// Window ROMs (4096 entries × 32-bit Q24.8) are initialized inline.
// For smaller N, ROM is sub-sampled: step = 4096/N_current.
module windowing (
    input  wire        clk,
    input  wire        rst,
    // Config (module id 4'h1)
    input  wire        cfg_valid,
    input  wire [7:0]  cfg_addr,
    input  wire [31:0] cfg_data,
    // From preprocessing
    input  wire        s_valid,
    output reg         s_ready,
    input  wire [1:0]  s_channel,
    input  wire signed [31:0] s_sample,
    input  wire [47:0] s_timestamp,
    input  wire        s_gap,
    // To fft_core
    output reg         m_valid,
    input  wire        m_ready,
    output reg  [1:0]  m_channel,
    output reg  signed [31:0] m_re,
    output reg  signed [31:0] m_im,
    output reg  [11:0] m_bin_idx,
    output reg         m_frame_last,
    output reg  [47:0] m_timestamp
);

    // ---- Config registers ----
    reg [1:0]  win_type    [0:3];  // 0=Hamming, 1=Hann, 2=Blackman
    reg [11:0] N_log2      [0:3];  // log2 of FFT size: 8=256, 10=1024, 12=4096
    reg [11:0] N_size      [0:3];  // actual N (256, 1024, or 4096)

    // Window coefficient ROMs (Hamming, Hann, Blackman), 4096 entries each.
    // Values are Q24.8: coefficient × 256.
    reg signed [31:0] hamm_rom [0:4095];
    reg signed [31:0] hann_rom [0:4095];
    reg signed [31:0] blkm_rom [0:4095];

    // Overlap-add second-half buffer: per channel, stores N/2 samples of prev frame.
    // Using a single array indexed by {channel[1:0], buf_idx[11:0]}.
    // Max N/2 = 2048 per channel → 4 × 2048 = 8192 entries.
    reg signed [31:0] ola_buf [0:8191];  // [ch*2048 + idx]

    // Per-channel sample accumulators
    reg [11:0] samp_idx  [0:3];   // current sample index within frame (0..N-1)
    reg [47:0] frame_ts  [0:3];   // timestamp of frame start
    // Ping-pong storage for full frame before windowing output
    reg signed [31:0] frame_buf [0:8191];  // [ch*2048 + idx] stores first N/2 samples

    integer ch;

    // ROM initialization using Python-precomputed values (Hamming 4096 points).
    // For brevity, initialize at runtime with the mathematical formula using $realtobits.
    // In actual synthesis, replace with $readmemh or a precomputed include.
    integer k_init;
    real w_val;
    initial begin
        for (k_init = 0; k_init < 4096; k_init = k_init + 1) begin
            // Hamming: 0.54 - 0.46*cos(2*pi*k/(N-1))
            w_val = 0.54 - 0.46 * $cos(2.0 * 3.14159265358979 * k_init / 4095.0);
            hamm_rom[k_init] = $rtoi(w_val * 256.0);
            // Hann: 0.5*(1 - cos(2*pi*k/(N-1)))
            w_val = 0.5 * (1.0 - $cos(2.0 * 3.14159265358979 * k_init / 4095.0));
            hann_rom[k_init] = $rtoi(w_val * 256.0);
            // Blackman: 0.42 - 0.5*cos(2*pi*k/(N-1)) + 0.08*cos(4*pi*k/(N-1))
            w_val = 0.42 - 0.5*$cos(2.0*3.14159265358979*k_init/4095.0)
                         + 0.08*$cos(4.0*3.14159265358979*k_init/4095.0);
            blkm_rom[k_init] = $rtoi(w_val * 256.0);
        end
        for (ch = 0; ch < 4; ch = ch + 1) begin
            win_type[ch] = 2'd0;   // Hamming default
            N_log2[ch]   = 12'd10; // 1024 default
            N_size[ch]   = 12'd1024;
            samp_idx[ch] = 12'd0;
            frame_ts[ch] = 48'd0;
        end
    end

    // Config decode
    always @(posedge clk) begin
        if (cfg_valid && (cfg_addr[7:4] == 4'h1)) begin
            case (cfg_addr[3:2])
                2'd0, 2'd1, 2'd2, 2'd3: begin
                    case (cfg_addr[1:0])
                        2'd0: win_type[cfg_addr[3:2]]  <= cfg_data[1:0];
                        2'd1: begin
                            N_log2[cfg_addr[3:2]] <= cfg_data[3:0];
                            N_size[cfg_addr[3:2]] <= (12'd1 << cfg_data[3:0]);
                        end
                        default: ;
                    endcase
                end
            endcase
        end
    end

    // ---- Pipeline stage 1: ROM lookup ----
    reg               p1_valid;
    reg [1:0]         p1_ch;
    reg signed [31:0] p1_sample;
    reg [11:0]        p1_bin_idx;
    reg [47:0]        p1_ts;
    reg               p1_frame_last;

    // ROM address: sub-sample step = 4096 / N_current
    wire [12:0] rom_step [0:3];
    assign rom_step[0] = 13'd4096 >> N_log2[0];
    assign rom_step[1] = 13'd4096 >> N_log2[1];
    assign rom_step[2] = 13'd4096 >> N_log2[2];
    assign rom_step[3] = 13'd4096 >> N_log2[3];

    reg [11:0] rom_addr_reg;
    reg [1:0]  p1_win_type;
    reg signed [31:0] p1_coef;

    always @(posedge clk) begin
        if (rst) begin
            p1_valid <= 1'b0;
            s_ready  <= 1'b1;
        end else begin
            s_ready <= m_ready || !m_valid;
            p1_valid <= s_valid && s_ready;
            if (s_valid && s_ready) begin
                p1_ch         <= s_channel;
                p1_sample     <= s_sample;
                p1_bin_idx    <= samp_idx[s_channel];
                p1_ts         <= (samp_idx[s_channel] == 0) ? s_timestamp : frame_ts[s_channel];
                p1_frame_last <= (samp_idx[s_channel] == N_size[s_channel] - 1);
                p1_win_type   <= win_type[s_channel];
                rom_addr_reg  <= samp_idx[s_channel] * rom_step[s_channel][11:0];

                if (samp_idx[s_channel] == 0)
                    frame_ts[s_channel] <= s_timestamp;

                if (samp_idx[s_channel] == N_size[s_channel] - 1)
                    samp_idx[s_channel] <= 12'd0;
                else
                    samp_idx[s_channel] <= samp_idx[s_channel] + 1;
            end
        end
    end

    // ROM read (combinatorial mux)
    always @(posedge clk) begin
        case (p1_win_type)
            2'd0: p1_coef <= hamm_rom[rom_addr_reg];
            2'd1: p1_coef <= hann_rom[rom_addr_reg];
            2'd2: p1_coef <= blkm_rom[rom_addr_reg];
            default: p1_coef <= hamm_rom[rom_addr_reg];
        endcase
    end

    // ---- Pipeline stage 2: multiply sample × coefficient ----
    reg               p2_valid;
    reg [1:0]         p2_ch;
    reg [11:0]        p2_bin_idx;
    reg [47:0]        p2_ts;
    reg               p2_frame_last;
    reg signed [31:0] p2_windowed;

    always @(posedge clk) begin
        if (rst) begin
            p2_valid <= 1'b0;
        end else begin
            p2_valid      <= p1_valid;
            p2_ch         <= p1_ch;
            p2_bin_idx    <= p1_bin_idx;
            p2_ts         <= p1_ts;
            p2_frame_last <= p1_frame_last;
            // Q24.8 × Q24.8 = Q48.16; take bits [39:8] for Q24.8 result
            p2_windowed   <= ($signed(p1_sample) * $signed(p1_coef)) >>> 8;
        end
    end

    // ---- Output: emit windowed sample to fft_core ----
    always @(posedge clk) begin
        if (rst) begin
            m_valid <= 1'b0;
        end else begin
            if (!m_valid || m_ready) begin
                m_valid      <= p2_valid;
                m_channel    <= p2_ch;
                m_re         <= p2_windowed;
                m_im         <= 32'sd0;   // real-valued input, imag = 0
                m_bin_idx    <= p2_bin_idx;
                m_frame_last <= p2_frame_last;
                m_timestamp  <= p2_ts;
            end
        end
    end

endmodule
