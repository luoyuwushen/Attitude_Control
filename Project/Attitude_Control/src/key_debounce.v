// 双级同步后对按下和松开均消抖；仅确认按下时产生单周期 press。
`timescale 1ns/1ps
`default_nettype none
module key_debounce #(
    parameter integer STABLE_CYCLES = 1000000
) (
    input wire clk,
    input wire rst_n,
    input wire key_n,
    output reg level_n,
    output reg press
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
    localparam integer CW = counter_width(STABLE_CYCLES);
    (* syn_preserve = 1, async_reg = "true" *) reg [1:0] key_sync;
    reg [CW-1:0] stable_count;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_sync <= 2'b11;
            stable_count <= {CW{1'b0}};
            level_n <= 1'b1;
            press <= 1'b0;
        end else begin
            key_sync <= {key_sync[0], key_n};
            press <= 1'b0;
            if (key_sync[1] == level_n) stable_count <= {CW{1'b0}};
            else if (stable_count == STABLE_CYCLES - 1) begin
                level_n <= key_sync[1];
                press <= !key_sync[1];
                stable_count <= {CW{1'b0}};
            end else stable_count <= stable_count + 1'b1;
        end
    end
endmodule
`default_nettype wire
