`timescale 1ps/1ps
// Radix-2 DIT butterfly unit. Fully pipelined, 4-cycle latency.
// Computes: p = (a + b*W) >> 1,  q = (a - b*W) >> 1
// All values Q24.8 signed (32-bit). The >>1 is the per-stage block-FP scaling.
// Q24.8 × Q24.8 multiply: take product[39:8] of the 64-bit result.
module butterfly (
    input  wire        clk,
    input  wire        rst,
    input  wire        in_valid,
    input  wire signed [31:0] a_re,
    input  wire signed [31:0] a_im,
    input  wire signed [31:0] b_re,
    input  wire signed [31:0] b_im,
    input  wire signed [31:0] w_re,   // Q24.8 cos(2*pi*k/N)
    input  wire signed [31:0] w_im,   // Q24.8 −sin(2*pi*k/N)
    input  wire        [11:0] tag,
    output reg         out_valid,
    output reg  signed [31:0] p_re,
    output reg  signed [31:0] p_im,
    output reg  signed [31:0] q_re,
    output reg  signed [31:0] q_im,
    output reg         [11:0] tag_out
);

    // ---- Stage 1: register inputs, start DSP multiply stage 1 ----
    reg signed [31:0] s1_a_re, s1_a_im;
    reg signed [63:0] s1_t1, s1_t2, s1_t3, s1_t4;
    reg        [11:0] s1_tag;
    reg               s1_valid;

    always @(posedge clk) begin
        if (rst) begin
            s1_valid <= 0;
        end else begin
            s1_valid <= in_valid;
            s1_a_re  <= a_re;
            s1_a_im  <= a_im;
            s1_tag   <= tag;
            s1_t1    <= a_re * w_re;   // b_re*w_re (a_re reused as pipe reg below)
            s1_t2    <= a_im * w_im;   // b_im*w_im
            s1_t3    <= a_re * w_im;   // b_re*w_im
            s1_t4    <= a_im * w_re;   // b_im*w_re
        end
    end

    // We need b_re/b_im in stage 1 but used a_re/a_im names above — fix:
    // Actually the multiply inputs are b and w, not a. Use separate regs.
    // Rewrite to be correct:
    reg signed [31:0] s0_b_re, s0_b_im, s0_w_re, s0_w_im;
    reg signed [31:0] s0_a_re, s0_a_im;
    reg        [11:0] s0_tag;
    reg               s0_valid;

    // Stage 0 capture (combinatorial latch into stage 1)
    // Replace the above stage 1 with correct two-stage:

    // ---- Correct 4-stage pipeline ----
    // Cycle 0 → 1: capture inputs
    reg signed [31:0] p1_a_re, p1_a_im, p1_b_re, p1_b_im, p1_w_re, p1_w_im;
    reg        [11:0] p1_tag;
    reg               p1_valid;

    always @(posedge clk) begin
        if (rst) p1_valid <= 0;
        else begin
            p1_valid <= in_valid;
            p1_a_re  <= a_re;  p1_a_im <= a_im;
            p1_b_re  <= b_re;  p1_b_im <= b_im;
            p1_w_re  <= w_re;  p1_w_im <= w_im;
            p1_tag   <= tag;
        end
    end

    // Cycle 1 → 2: first multiply stage (DSP pipeline reg 1)
    reg signed [63:0] p2_t1, p2_t2, p2_t3, p2_t4;
    reg signed [31:0] p2_a_re, p2_a_im;
    reg        [11:0] p2_tag;
    reg               p2_valid;

    always @(posedge clk) begin
        if (rst) p2_valid <= 0;
        else begin
            p2_valid <= p1_valid;
            p2_a_re  <= p1_a_re;
            p2_a_im  <= p1_a_im;
            p2_tag   <= p1_tag;
            p2_t1    <= p1_b_re * p1_w_re;
            p2_t2    <= p1_b_im * p1_w_im;
            p2_t3    <= p1_b_re * p1_w_im;
            p2_t4    <= p1_b_im * p1_w_re;
        end
    end

    // Cycle 2 → 3: compute b*W from partial products, Q24.8 truncate product[39:8]
    reg signed [31:0] p3_bw_re, p3_bw_im, p3_a_re, p3_a_im;
    reg        [11:0] p3_tag;
    reg               p3_valid;

    always @(posedge clk) begin
        if (rst) p3_valid <= 0;
        else begin
            p3_valid  <= p2_valid;
            p3_a_re   <= p2_a_re;
            p3_a_im   <= p2_a_im;
            p3_tag    <= p2_tag;
            // Q24.8 × Q24.8 = Q48.16; keep bits [39:8] → Q24.8
            p3_bw_re  <= $signed(p2_t1[39:8]) - $signed(p2_t2[39:8]);
            p3_bw_im  <= $signed(p2_t3[39:8]) + $signed(p2_t4[39:8]);
        end
    end

    // Cycle 3 → 4: add/sub and apply block-FP >>1
    always @(posedge clk) begin
        if (rst) begin
            out_valid <= 0;
        end else begin
            out_valid <= p3_valid;
            tag_out   <= p3_tag;
            // arithmetic right-shift by 1 for per-stage block-FP scaling
            p_re <= ($signed(p3_a_re) + $signed(p3_bw_re)) >>> 1;
            p_im <= ($signed(p3_a_im) + $signed(p3_bw_im)) >>> 1;
            q_re <= ($signed(p3_a_re) - $signed(p3_bw_re)) >>> 1;
            q_im <= ($signed(p3_a_im) - $signed(p3_bw_im)) >>> 1;
        end
    end

endmodule
