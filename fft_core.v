`timescale 1ps/1ps
// Radix-2 DIT FFT controller. Single butterfly unit, ping-pong BRAM.
//
// FSM:
//   IDLE    — s_ready=0; snoops for s_bin_idx==0 (start of new frame) → LOAD
//   LOAD    — s_ready=1; writes N samples at bit-reversed addresses; on s_frame_last → COMPUTE
//   COMPUTE — sequences log2(N) butterfly stages using ping-pong BRAMs
//   OUTPUT  — streams N bins; on completion → IDLE
//
// Ping-pong: LOAD always writes to bram_a (bank=0).
//   COMPUTE reads from bank, writes both p and q to !bank, toggles bank at end of each stage.
//   OUTPUT reads from bank after last toggle.
//
// Twiddle index for stage s, offset k: tw_idx = (k << s) + base[N_sel]
// a_addr = ((bfly_cnt & ~(stride-1)) << 1) | (bfly_cnt & (stride-1))
module fft_core (
    input  wire        clk,
    input  wire        rst,
    // Config (module id 4'h2)
    input  wire        cfg_valid,
    input  wire [7:0]  cfg_addr,
    input  wire [31:0] cfg_data,
    // From windowing (AXI4-Stream)
    input  wire        s_valid,
    output reg         s_ready,
    input  wire [1:0]  s_channel,
    input  wire signed [31:0] s_re,
    input  wire signed [31:0] s_im,
    input  wire [11:0] s_bin_idx,
    input  wire        s_frame_last,
    input  wire [47:0] s_timestamp,
    // Butterfly interface
    output reg         bfly_in_valid,
    output reg  signed [31:0] bfly_a_re, bfly_a_im,
    output reg  signed [31:0] bfly_b_re, bfly_b_im,
    output reg  signed [31:0] bfly_w_re, bfly_w_im,
    output reg  [11:0] bfly_tag,
    input  wire        bfly_out_valid,
    input  wire signed [31:0] bfly_p_re, bfly_p_im,
    input  wire signed [31:0] bfly_q_re, bfly_q_im,
    input  wire [11:0] bfly_tag_out,
    // Twiddle ROM interface
    output reg  [10:0] trom_addr,
    output reg         trom_rd_en,
    input  wire signed [31:0] trom_cos,
    input  wire signed [31:0] trom_sin,
    // Output bins (AXI4-Stream)
    output reg         m_valid,
    input  wire        m_ready,
    output reg  [1:0]  m_channel,
    output reg  signed [31:0] m_re,
    output reg  signed [31:0] m_im,
    output reg  [11:0] m_bin,
    output reg         m_frame_last,
    output reg  [47:0] m_timestamp,
    output reg  [3:0]  m_scale_exp
);

    // ---- Config registers ----
    reg [1:0]  cfg_n_sel [0:3];   // 00=256, 01=1024, 10=4096

    function [12:0] n_from_sel;
        input [1:0] sel;
        case (sel)
            2'd0: n_from_sel = 13'd256;
            2'd1: n_from_sel = 13'd1024;
            2'd2: n_from_sel = 13'd4096;
            default: n_from_sel = 13'd1024;
        endcase
    endfunction

    function [3:0] log2_from_sel;
        input [1:0] sel;
        case (sel)
            2'd0: log2_from_sel = 4'd8;
            2'd1: log2_from_sel = 4'd10;
            2'd2: log2_from_sel = 4'd12;
            default: log2_from_sel = 4'd10;
        endcase
    endfunction

    function [10:0] twiddle_base;
        input [1:0] n_sel;
        case (n_sel)
            2'd0: twiddle_base = 11'd0;
            2'd1: twiddle_base = 11'd128;
            2'd2: twiddle_base = 11'd640;
            default: twiddle_base = 11'd128;
        endcase
    endfunction

    // Bit-reversal: reverse the lower 'nbits' bits of in_val
    function [11:0] bit_rev;
        input [11:0] in_val;
        input [3:0]  nbits;
        integer j;
        begin
            bit_rev = 12'd0;
            for (j = 0; j < 12; j = j + 1) begin
                if (j < nbits)
                    bit_rev[nbits-1-j] = in_val[j];
            end
        end
    endfunction

    integer i_init;
    initial begin
        for (i_init = 0; i_init < 4; i_init = i_init + 1)
            cfg_n_sel[i_init] = 2'd1;
    end

    always @(posedge clk) begin
        if (cfg_valid && (cfg_addr[7:4] == 4'h2))
            cfg_n_sel[cfg_addr[1:0]] <= cfg_data[1:0];
    end

    // ---- Ping-pong BRAMs ----
    reg signed [63:0] bram_a [0:4095];
    reg signed [63:0] bram_b [0:4095];

    reg bank;  // 0 → bram_a is the current READ bank; 1 → bram_b is current

    task bram_write;
        input        b;
        input [11:0] addr;
        input signed [31:0] re, im;
        begin
            if (b == 0) bram_a[addr] <= {im, re};
            else        bram_b[addr] <= {im, re};
        end
    endtask

    // ---- FSM state ----
    localparam FSM_IDLE    = 2'd0,
               FSM_LOAD    = 2'd1,
               FSM_COMPUTE = 2'd2,
               FSM_OUTPUT  = 2'd3;

    reg [1:0]  fsm_state;
    reg [1:0]  active_ch;
    reg [1:0]  active_n_sel;
    reg [12:0] active_n;
    reg [3:0]  active_log2n;
    reg [47:0] active_ts;
    reg [3:0]  scale_exp_r;
    reg [11:0] load_cnt;
    reg [3:0]  stage;
    reg [11:0] bfly_cnt;
    reg        wr_pending;
    reg [11:0] wr_p_addr, wr_q_addr;
    reg [11:0] out_cnt;

    always @(posedge clk) begin
        if (rst) begin
            fsm_state     <= FSM_IDLE;
            s_ready       <= 1'b0;
            m_valid       <= 1'b0;
            bfly_in_valid <= 1'b0;
            trom_rd_en    <= 1'b0;
            wr_pending    <= 1'b0;
            bank          <= 1'b0;
        end else begin

            // ---- Butterfly result writeback (asynchronous to FSM state) ----
            if (bfly_out_valid && wr_pending) begin
                bram_write(!bank, wr_p_addr, bfly_p_re, bfly_p_im);
                bram_write(!bank, wr_q_addr, bfly_q_re, bfly_q_im);
                wr_pending <= 1'b0;
            end

            case (fsm_state)

                // ---- IDLE: snoop for start of frame (s_bin_idx==0), don't consume ----
                FSM_IDLE: begin
                    s_ready <= 1'b0;   // stall windowing until we're ready to load
                    if (s_valid && (s_bin_idx == 12'd0)) begin
                        active_ch    <= s_channel;
                        active_n_sel <= cfg_n_sel[s_channel];
                        active_n     <= n_from_sel(cfg_n_sel[s_channel]);
                        active_log2n <= log2_from_sel(cfg_n_sel[s_channel]);
                        active_ts    <= s_timestamp;
                        scale_exp_r  <= 4'd0;
                        load_cnt     <= 12'd0;
                        bank         <= 1'b0;
                        fsm_state    <= FSM_LOAD;
                        // s_ready becomes 1 NEXT cycle so bin 0 is consumed in FSM_LOAD
                    end
                end

                // ---- LOAD: accept N samples, write to bram_a at bit-reversed addresses ----
                FSM_LOAD: begin
                    s_ready <= 1'b1;
                    if (s_valid && s_ready && (s_channel == active_ch)) begin
                        bram_a[bit_rev(load_cnt[11:0], active_log2n)] <= {s_im, s_re};
                        load_cnt <= load_cnt + 1;
                        if (s_frame_last) begin
                            fsm_state     <= FSM_COMPUTE;
                            s_ready       <= 1'b0;
                            stage         <= 4'd0;
                            bfly_cnt      <= 12'd0;
                            bfly_in_valid <= 1'b0;
                        end
                    end
                end

                // ---- COMPUTE: sequence butterfly stages ----
                FSM_COMPUTE: begin
                    if (!wr_pending) begin
                        if (bfly_cnt < active_n[12:1]) begin   // bfly_cnt < N/2
                            begin
                                reg [11:0] stride_r, offset_r, a_addr_r, b_addr_r;
                                reg [12:0] a_addr13;
                                reg [10:0] tw_idx_r;

                                // stride = N / 2^(stage+1)
                                stride_r = active_n[12:1] >> stage;

                                // offset within butterfly group
                                offset_r = bfly_cnt[11:0] & (stride_r - 12'd1);

                                // a_addr = (group * 2*stride) + offset
                                a_addr13 = ({1'b0, bfly_cnt[11:0] & ~(stride_r - 12'd1)} << 1)
                                           | {1'b0, offset_r};
                                a_addr_r = a_addr13[11:0];
                                b_addr_r = a_addr_r + stride_r;

                                // twiddle: k<<stage + base
                                tw_idx_r = (offset_r[9:0] << stage) + twiddle_base(active_n_sel);

                                // Read from current read bank
                                bfly_a_re <= $signed(bank ? bram_b[a_addr_r][31:0]
                                                          : bram_a[a_addr_r][31:0]);
                                bfly_a_im <= $signed(bank ? bram_b[a_addr_r][63:32]
                                                          : bram_a[a_addr_r][63:32]);
                                bfly_b_re <= $signed(bank ? bram_b[b_addr_r][31:0]
                                                          : bram_a[b_addr_r][31:0]);
                                bfly_b_im <= $signed(bank ? bram_b[b_addr_r][63:32]
                                                          : bram_a[b_addr_r][63:32]);

                                // Twiddle ROM: issue read; use previous cycle's result for bfly
                                trom_addr  <= tw_idx_r;
                                trom_rd_en <= 1'b1;
                                bfly_w_re  <= trom_cos;
                                bfly_w_im  <= trom_sin;

                                bfly_tag      <= a_addr_r;
                                bfly_in_valid <= 1'b1;
                                wr_p_addr     <= a_addr_r;
                                wr_q_addr     <= b_addr_r;
                                wr_pending    <= 1'b1;
                                bfly_cnt      <= bfly_cnt + 1;
                            end
                        end else begin
                            // Stage complete: toggle bank, advance or finish
                            bfly_in_valid <= 1'b0;
                            trom_rd_en    <= 1'b0;
                            scale_exp_r   <= scale_exp_r + 1;
                            bank          <= !bank;
                            if (stage == active_log2n - 1) begin
                                fsm_state <= FSM_OUTPUT;
                                out_cnt   <= 12'd0;
                            end else begin
                                stage    <= stage + 1;
                                bfly_cnt <= 12'd0;
                            end
                        end
                    end
                end

                // ---- OUTPUT: stream N bins to cordic/post_fft ----
                FSM_OUTPUT: begin
                    if (!m_valid || m_ready) begin
                        if (out_cnt < active_n[11:0]) begin
                            m_valid      <= 1'b1;
                            m_channel    <= active_ch;
                            m_re         <= $signed(bank ? bram_b[out_cnt][31:0]
                                                        : bram_a[out_cnt][31:0]);
                            m_im         <= $signed(bank ? bram_b[out_cnt][63:32]
                                                        : bram_a[out_cnt][63:32]);
                            m_bin        <= out_cnt;
                            m_frame_last <= (out_cnt == active_n[11:0] - 12'd1);
                            m_timestamp  <= active_ts;
                            m_scale_exp  <= scale_exp_r;
                            out_cnt      <= out_cnt + 1;
                        end else begin
                            m_valid   <= 1'b0;
                            fsm_state <= FSM_IDLE;
                        end
                    end
                end

                default: fsm_state <= FSM_IDLE;
            endcase
        end
    end

endmodule
