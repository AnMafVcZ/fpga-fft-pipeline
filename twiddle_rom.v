`timescale 1ps/1ps
// Twiddle factor ROM for all three FFT sizes.
// Address map: N=256 → 0-127, N=1024 → 128-639, N=4096 → 640-2687.
// Each entry: {sin_q24_8[63:32], cos_q24_8[31:0]}
// W_N^k = cos(2*pi*k/N) - j*sin(2*pi*k/N).
// 1-cycle registered output latency.
module twiddle_rom (
    input  wire        clk,
    input  wire [10:0] addr,   // 0–2687
    input  wire        rd_en,
    output reg  signed [31:0] cos_out,
    output reg  signed [31:0] sin_out
);

    reg [63:0] mem [0:2687];

    initial begin
        `include "twiddle_init.vh"
    end

    always @(posedge clk) begin
        if (rd_en) begin
            cos_out <= $signed(mem[addr][31:0]);
            sin_out <= $signed(mem[addr][63:32]);
        end
    end

endmodule
