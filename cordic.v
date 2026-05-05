`timescale 1ps/1ps
// 16-stage unrolled CORDIC pipeline, vectoring mode.
// Computes magnitude = sqrt(re^2 + im^2) and phase = atan2(im, re).
// Input/output: Q24.8 signed 32-bit.
// CORDIC gain K ≈ 1.6468; correction: multiply x_out by 155 >> 8 (≈ 0.605).
// Throughput: 1 result per clock. Latency: 17 cycles (1 normalize + 16 iterations).
module cordic (
    input  wire        clk,
    input  wire        rst,
    input  wire        in_valid,
    input  wire signed [31:0] in_re,
    input  wire signed [31:0] in_im,
    input  wire        [11:0] in_tag,
    output reg         out_valid,
    output reg  signed [31:0] out_mag,
    output reg  signed [31:0] out_phase,
    output reg         [11:0] out_tag
);

    // Atan LUT: atan(2^-i) in Q24.8 radians, i=0..15
    // atan(2^0)=pi/4=0.7854→201, atan(2^-1)=0.4636→119, atan(2^-2)=0.2450→63, ...
    integer atan_lut [0:15];
    initial begin
        atan_lut[0]  = 201;
        atan_lut[1]  = 119;
        atan_lut[2]  = 63;
        atan_lut[3]  = 32;
        atan_lut[4]  = 16;
        atan_lut[5]  = 8;
        atan_lut[6]  = 4;
        atan_lut[7]  = 2;
        atan_lut[8]  = 1;
        atan_lut[9]  = 1;
        atan_lut[10] = 0;
        atan_lut[11] = 0;
        atan_lut[12] = 0;
        atan_lut[13] = 0;
        atan_lut[14] = 0;
        atan_lut[15] = 0;
    end

    // Pipeline stage storage: 17 stages (stage 0 = input/normalize, stages 1-16 = CORDIC)
    reg signed [31:0] px [0:16];
    reg signed [31:0] py [0:16];
    reg signed [31:0] pz [0:16];
    reg        [11:0] ptag [0:16];
    reg               pvalid [0:16];

    // Q24.8 representation of pi: 3.14159... * 256 = 804
    localparam signed [31:0] PI_Q24_8 = 32'sd804;

    integer i;

    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i <= 16; i = i + 1) pvalid[i] <= 0;
        end else begin
            // Stage 0: Quadrant normalization — rotate into right half-plane
            pvalid[0] <= in_valid;
            ptag[0]   <= in_tag;
            if (in_re[31]) begin
                // Re < 0: negate both, adjust phase by +pi or -pi
                px[0] <= -in_re;
                py[0] <= -in_im;
                pz[0] <= in_im[31] ? PI_Q24_8 : -PI_Q24_8;
            end else begin
                px[0] <= in_re;
                py[0] <= in_im;
                pz[0] <= 32'sd0;
            end

            // Stages 1-16: CORDIC micro-rotations
            for (i = 1; i <= 16; i = i + 1) begin
                pvalid[i] <= pvalid[i-1];
                ptag[i]   <= ptag[i-1];
                if (!py[i-1][31]) begin
                    // y >= 0: rotate clockwise
                    px[i] <= px[i-1] + (py[i-1] >>> (i-1));
                    py[i] <= py[i-1] - (px[i-1] >>> (i-1));
                    pz[i] <= pz[i-1] + $signed(atan_lut[i-1]);
                end else begin
                    // y < 0: rotate counter-clockwise
                    px[i] <= px[i-1] - (py[i-1] >>> (i-1));
                    py[i] <= py[i-1] + (px[i-1] >>> (i-1));
                    pz[i] <= pz[i-1] - $signed(atan_lut[i-1]);
                end
            end

            // Output stage: apply CORDIC gain correction (multiply by 155/256 ≈ 1/K)
            out_valid <= pvalid[16];
            out_tag   <= ptag[16];
            // magnitude: px[16] * 155 >> 8 (Q24.8 * const → keep top bits)
            out_mag   <= $signed(px[16] * 32'sd155) >>> 8;
            out_phase <= pz[16];
        end
    end

endmodule
