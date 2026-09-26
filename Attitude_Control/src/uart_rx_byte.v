// UART 8N1；双级同步、起始位中心确认、数据/停止位中心采样。
// framing_error 与 valid 都是单周期脉冲；停止位错误的字节不会 valid。
`timescale 1ns/1ps
`default_nettype none
module uart_rx_byte #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 115200
) (
    input wire clk,
    input wire rst_n,
    input wire rx,
    output reg [7:0] data,
    output reg valid,
    output reg framing_error
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
    localparam [1:0] IDLE=2'd0, START=2'd1, DATA=2'd2, STOP=2'd3;
    (* syn_preserve = 1, async_reg = "true" *) reg [1:0] rx_sync;
    reg rx_previous;
    reg [1:0] state;
    reg [CW-1:0] bit_timer;
    reg [2:0] bit_index;
    reg [7:0] shift_data;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync <= 2'b11;
            rx_previous <= 1'b1;
            state <= IDLE;
            bit_timer <= {CW{1'b0}};
            bit_index <= 3'd0;
            shift_data <= 8'd0;
            data <= 8'd0;
            valid <= 1'b0;
            framing_error <= 1'b0;
        end else begin
            rx_sync <= {rx_sync[0], rx};
            rx_previous <= rx_sync[1];
            valid <= 1'b0;
            framing_error <= 1'b0;
            case (state)
                IDLE: if (rx_previous && !rx_sync[1]) begin
                    state <= START;
                    bit_timer <= BIT_CYCLES / 2 - 1;
                end
                START: if (bit_timer != 0) bit_timer <= bit_timer - 1'b1;
                    else if (!rx_sync[1]) begin
                        state <= DATA;
                        bit_index <= 3'd0;
                        bit_timer <= BIT_CYCLES - 1;
                    end else state <= IDLE;
                DATA: if (bit_timer != 0) bit_timer <= bit_timer - 1'b1;
                    else begin
                        shift_data[bit_index] <= rx_sync[1];
                        bit_timer <= BIT_CYCLES - 1;
                        if (bit_index == 3'd7) state <= STOP;
                        else bit_index <= bit_index + 1'b1;
                    end
                STOP: if (bit_timer != 0) bit_timer <= bit_timer - 1'b1;
                    else begin
                        state <= IDLE;
                        if (rx_sync[1]) begin
                            data <= shift_data;
                            valid <= 1'b1;
                        end else framing_error <= 1'b1;
                    end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
