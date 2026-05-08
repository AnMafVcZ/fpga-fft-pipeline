`timescale 1ps/1ps
// Gray-code async FIFO for clock-domain crossing.
// Write domain: 156.25 MHz ingress. Read domain: 300 MHz processing.
module async_fifo #(
    parameter DATA_WIDTH = 104,
    parameter ADDR_WIDTH = 5    // depth = 2^ADDR_WIDTH = 32
) (
    // Write domain
    input  wire                  wr_clk,
    input  wire                  wr_rst,
    input  wire                  wr_en,
    input  wire [DATA_WIDTH-1:0] wr_data,
    output wire                  wr_full,
    // Read domain
    input  wire                  rd_clk,
    input  wire                  rd_rst,
    input  wire                  rd_en,
    output wire [DATA_WIDTH-1:0] rd_data,
    output wire                  rd_empty
);

    localparam DEPTH = 1 << ADDR_WIDTH;

    // Dual-port memory
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // Write-domain pointers (binary and gray)
    reg [ADDR_WIDTH:0] wr_ptr_bin;
    wire [ADDR_WIDTH:0] wr_ptr_gray = wr_ptr_bin ^ (wr_ptr_bin >> 1);

    // Read-domain pointers (binary and gray)
    reg [ADDR_WIDTH:0] rd_ptr_bin;
    wire [ADDR_WIDTH:0] rd_ptr_gray = rd_ptr_bin ^ (rd_ptr_bin >> 1);

    // Synchronizers: rd gray ptr → wr domain (2-flop)
    reg [ADDR_WIDTH:0] rd_gray_sync1_wr, rd_gray_sync2_wr;
    // Synchronizers: wr gray ptr → rd domain (2-flop)
    reg [ADDR_WIDTH:0] wr_gray_sync1_rd, wr_gray_sync2_rd;

    // ---- Write domain ----
    always @(posedge wr_clk) begin
        if (wr_rst) begin
            wr_ptr_bin       <= 0;
            rd_gray_sync1_wr <= 0;
            rd_gray_sync2_wr <= 0;
        end else begin
            rd_gray_sync1_wr <= rd_ptr_gray;
            rd_gray_sync2_wr <= rd_gray_sync1_wr;
            if (wr_en && !wr_full) begin
                mem[wr_ptr_bin[ADDR_WIDTH-1:0]] <= wr_data;
                wr_ptr_bin <= wr_ptr_bin + 1;
            end
        end
    end

    // Full: MSB and next-MSB of gray ptrs differ, rest equal
    assign wr_full = (wr_ptr_gray == {~rd_gray_sync2_wr[ADDR_WIDTH:ADDR_WIDTH-1],
                                       rd_gray_sync2_wr[ADDR_WIDTH-2:0]});

    // ---- Read domain ----
    // Combinatorial (show-ahead) output: rd_data always reflects current head.
    // rd_empty goes low as soon as data is available; rd_en just advances the pointer.

    always @(posedge rd_clk) begin
        if (rd_rst) begin
            rd_ptr_bin       <= 0;
            wr_gray_sync1_rd <= 0;
            wr_gray_sync2_rd <= 0;
        end else begin
            wr_gray_sync1_rd <= wr_ptr_gray;
            wr_gray_sync2_rd <= wr_gray_sync1_rd;
            if (rd_en && !rd_empty)
                rd_ptr_bin <= rd_ptr_bin + 1;
        end
    end

    assign rd_empty = (rd_ptr_gray == wr_gray_sync2_rd);
    assign rd_data  = mem[rd_ptr_bin[ADDR_WIDTH-1:0]];

endmodule
