// UART 8N1，valid && ready 的时钟沿接受字节；发送期间保持数据快照。
`timescale 1ns/1ps
`default_nettype none
module uart_tx_byte #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 115200
) (
    input wire clk,
    input wire rst_n,
    input wire valid,
    input wire [7:0] data,
    output wire ready,
    output wire tx
);
    function integer counter_width;
        input integer value;
        integer v;
        begin
            counter_width = 1;
            for (v = value - 1; v > 1; v = v >> 1)
                counter_width = counter_width + 1;
        end
    endfunction
    localparam integer DIVISOR = (CLK_HZ + BAUD / 2) / BAUD;
    localparam integer BIT_CYCLES = DIVISOR < 2 ? 2 : DIVISOR;
    localparam integer CW = counter_width(BIT_CYCLES);
    reg [CW-1:0] bit_timer;
    reg [3:0] bit_index;
    reg [9:0] shift_data;
    reg busy;
    assign ready = !busy;
    assign tx = busy ? shift_data[0] : 1'b1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_timer <= {CW{1'b0}};
            bit_index <= 4'd0;
            shift_data <= 10'b1111111111;
            busy <= 1'b0;
        end else if (!busy) begin
            if (valid) begin
                shift_data <= {1'b1, data, 1'b0};
                bit_index <= 4'd0;
                bit_timer <= BIT_CYCLES - 1;
                busy <= 1'b1;
            end
        end else if (bit_timer == 0) begin
            bit_timer <= BIT_CYCLES - 1;
            if (bit_index == 4'd9) busy <= 1'b0;
            else begin
                shift_data <= {1'b1, shift_data[9:1]};
                bit_index <= bit_index + 1'b1;
            end
        end else bit_timer <= bit_timer - 1'b1;
    end
endmodule
`default_nettype wire
