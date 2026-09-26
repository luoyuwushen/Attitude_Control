// TB6612FNG：停止为 IN1=IN2=0、PWM=1（高阻），见数据手册第 4 页。
// 默认 50MHz/2500=20kHz；指令单位千分比，周期边界更新，换向先停。
`timescale 1ns/1ps
`default_nettype none
module motor_pwm #(
    parameter integer PERIOD_CYCLES = 2500,
    parameter integer DEAD_CYCLES = 100
) (
    input wire clk,
    input wire rst_n,
    input wire enable,
    input wire signed [15:0] command,
    output wire in1,
    output wire in2,
    output wire pwm
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
    localparam integer PW = counter_width(PERIOD_CYCLES + 1);
    localparam integer DW = counter_width(DEAD_CYCLES + 1);
    function integer gcd;
        input integer a, b;
        integer x, y, remainder;
        begin
            x = a; y = b;
            while (y != 0) begin remainder = x % y; x = y; y = remainder; end
            gcd = x;
        end
    endfunction
    localparam integer SCALE_GCD = gcd(PERIOD_CYCLES,1000);
    reg [PW-1:0] phase_count, active_duty;
    reg [DW-1:0] dead_remaining;
    reg [1:0] active_direction, last_direction;
    wire [1:0] requested_direction = command[15] ? 2'b10 : 2'b01;
    wire [15:0] command_abs = command[15] ? -command : command;
    wire [10:0] magnitude = (command > 16'sd1000 || command < -16'sd1000) ?
        11'd1000 : command_abs[10:0];
    // Reduce the constant fraction before synthesis: 2500/1000 is exactly 5/2.
    wire [31:0] requested_duty = (magnitude * (PERIOD_CYCLES/SCALE_GCD)) /
                               (1000/SCALE_GCD);
    // enable/reset are asynchronous safety gates. The synchronous zero command
    // clears active_direction on the next clock, keeping a 16-bit comparator
    // out of the pad's combinational safety path.
    wire stopped = !rst_n || !enable || (active_direction == 2'b00);
    assign in1 = !stopped && (active_direction == 2'b01);
    assign in2 = !stopped && (active_direction == 2'b10);
    assign pwm = stopped ? 1'b1 : (phase_count < active_duty);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase_count <= {PW{1'b0}};
            active_duty <= {PW{1'b0}};
            dead_remaining <= {DW{1'b0}};
            active_direction <= 2'b00;
            last_direction <= 2'b00;
        end else begin
            if (phase_count == PERIOD_CYCLES - 1) phase_count <= {PW{1'b0}};
            else phase_count <= phase_count + 1'b1;
            if (dead_remaining != 0) dead_remaining <= dead_remaining - 1'b1;
            if (!enable || command == 16'sd0) begin
                active_direction <= 2'b00;
                active_duty <= {PW{1'b0}};
                if (active_direction != 0) dead_remaining <= DEAD_CYCLES;
            end else if (phase_count == PERIOD_CYCLES - 1 && dead_remaining == 0) begin
                if (last_direction != 0 && last_direction != requested_direction &&
                    DEAD_CYCLES != 0) begin
                    active_direction <= 2'b00;
                    active_duty <= {PW{1'b0}};
                    dead_remaining <= DEAD_CYCLES;
                    last_direction <= requested_direction;
                end else begin
                    active_direction <= requested_direction;
                    last_direction <= requested_direction;
                    active_duty <= requested_duty[PW-1:0];
                end
            end
        end
    end
endmodule
`default_nettype wire
