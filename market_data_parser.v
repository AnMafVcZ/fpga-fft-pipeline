`timescale 1ps/1ps
// Market data packet deserializer.
// Accepts a 32-bit AXI4-Stream word stream (s_last marks end of packet).
// Packet layout (5 words, big-endian field order):
//   Word 0: {symbol_id[7:0], 24'b0}
//   Word 1: price[31:0]  (Q24.8)
//   Word 2: {volume[15:0], 16'b0}
//   Word 3: timestamp[47:32]
//   Word 4: {timestamp[15:0], 16'b0}
// Outputs one structured record per packet on AXI4-Stream master port.
module market_data_parser (
    input  wire        clk,
    input  wire        rst,
    // AXI4-Stream slave
    input  wire        s_valid,
    output reg         s_ready,
    input  wire [31:0] s_data,
    input  wire        s_last,
    // AXI4-Stream master — structured record
    output reg         m_valid,
    input  wire        m_ready,
    output reg  [7:0]  m_symbol_id,
    output reg  [31:0] m_price,
    output reg  [15:0] m_volume,
    output reg  [47:0] m_timestamp
);

    localparam S_IDLE = 3'd0,
               S_W0   = 3'd1,
               S_W1   = 3'd2,
               S_W2   = 3'd3,
               S_W3   = 3'd4,
               S_W4   = 3'd5,
               S_HOLD = 3'd6;

    reg [2:0] state;
    reg [7:0]  r_symbol_id;
    reg [31:0] r_price;
    reg [15:0] r_volume;
    reg [47:0] r_timestamp;

    wire s_fire = s_valid && s_ready;

    always @(posedge clk) begin
        if (rst) begin
            state    <= S_IDLE;
            s_ready  <= 1'b1;
            m_valid  <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    m_valid <= 1'b0;
                    s_ready <= 1'b1;
                    if (s_fire) begin
                        r_symbol_id <= s_data[31:24];
                        state <= S_W1;
                    end
                end
                S_W1: begin
                    if (s_fire) begin
                        r_price <= s_data;
                        state   <= S_W2;
                    end
                end
                S_W2: begin
                    if (s_fire) begin
                        r_volume <= s_data[31:16];
                        state    <= S_W3;
                    end
                end
                S_W3: begin
                    if (s_fire) begin
                        r_timestamp[47:16] <= s_data;
                        state              <= S_W4;
                    end
                end
                S_W4: begin
                    if (s_fire) begin
                        r_timestamp[15:0] <= s_data[31:16];
                        s_ready  <= 1'b0;
                        m_valid  <= 1'b1;
                        m_symbol_id <= r_symbol_id;
                        m_price     <= r_price;
                        m_volume    <= r_volume;
                        m_timestamp <= {r_timestamp[47:16], s_data[31:16]};
                        state <= S_HOLD;
                    end
                end
                S_HOLD: begin
                    // Hold m_valid until downstream accepts
                    if (m_valid && m_ready) begin
                        m_valid <= 1'b0;
                        s_ready <= 1'b1;
                        state   <= S_IDLE;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
