`timescale 1ns/1ps
`default_nettype none
// 1kHz energy swing-up + discrete full-state feedback. Q10 state, permille PWM.
// Gains are derived from the documented nominal model, not a tuned real board.
module attitude_controller #(
    parameter signed [31:0] K_THETA = 32'sd2362613,
    parameter signed [31:0] K_OMEGA = 32'sd299007,
    parameter signed [31:0] K_ARM = -32'sd109597,
    parameter signed [31:0] K_SPEED = -32'sd118252,
    parameter integer SWING_GAIN = 667,
    parameter integer SWING_LIMIT = 800,
    parameter integer KICK_COMMAND = 667,
    parameter integer KICK_MS = 40,
    parameter integer SWING_TIMEOUT_MS = 15000,
    parameter integer WATCHDOG_CYCLES = 100000
)(
    input wire clk, rst_n, sample_valid, calibrated, sensor_fault,
    input wire start, stop, clear,
    input wire signed [15:0] theta, omega, arm, arm_speed,
    output reg [2:0] state,
    output reg [7:0] fault,
    output reg signed [15:0] command,
    output wire enable
);
    localparam [2:0] IDLE=0, SWING=1, BALANCE=2, FAULT=3;
    reg [31:0] watchdog;
    reg [15:0] swing_ms, catch_ms;
    reg signed [15:0] capture_arm;
    reg [2:0] stage;
    reg signed [63:0] p_theta, p_omega, p_arm, p_speed, balance_sum;
    reg signed [31:0] omega_square, energy_deficit, swing_command;
    reg signed [15:0] cos_theta;
    reg pump_negative;
    wire [15:0] abs_theta = theta[15] ? -theta : theta;
    wire [15:0] abs_omega = omega[15] ? -omega : omega;
    wire [15:0] abs_arm = arm[15] ? -arm : arm;
    wire [15:0] abs_speed = arm_speed[15] ? -arm_speed : arm_speed;
    reg [31:0] index_product;
    reg [20:0] angle_times32;
    reg [5:0] lut_index;
    reg omega_negative;
    wire [5:0] index_candidate = index_product[25:20];
    // ceil(32*2^20/3217), then one correction yields exact floor division.
    wire [20:0] candidate_times3217 = index_candidate * 21'd3217;
    wire signed [31:0] arm_error = $signed(arm)-$signed(capture_arm);
    wire signed [15:0] lookup_cos = cosine(lut_index);
    wire signed [31:0] swing_effort = (energy_deficit < 0) ^ pump_negative ?
                                    -SWING_GAIN : SWING_GAIN;
    wire signed [63:0] kinetic_term = ($signed({1'b0,omega_square})*64'sd167) >>> 25;
    localparam signed [15:0] KICK_VALUE = KICK_COMMAND;
    assign enable = (state == SWING || state == BALANCE) && calibrated &&
                    !sensor_fault && !stop && fault == 0;
    function signed [15:0] cosine;
        input [5:0] index;
        begin case (index)
            6'd0: cosine = 16'sd1024;
            6'd1: cosine = 16'sd1019;
            6'd2: cosine = 16'sd1004;
            6'd3: cosine = 16'sd980;
            6'd4: cosine = 16'sd946;
            6'd5: cosine = 16'sd903;
            6'd6: cosine = 16'sd851;
            6'd7: cosine = 16'sd792;
            6'd8: cosine = 16'sd724;
            6'd9: cosine = 16'sd650;
            6'd10: cosine = 16'sd569;
            6'd11: cosine = 16'sd483;
            6'd12: cosine = 16'sd392;
            6'd13: cosine = 16'sd297;
            6'd14: cosine = 16'sd200;
            6'd15: cosine = 16'sd100;
            6'd16: cosine = 16'sd0;
            6'd17: cosine = -16'sd100;
            6'd18: cosine = -16'sd200;
            6'd19: cosine = -16'sd297;
            6'd20: cosine = -16'sd392;
            6'd21: cosine = -16'sd483;
            6'd22: cosine = -16'sd569;
            6'd23: cosine = -16'sd650;
            6'd24: cosine = -16'sd724;
            6'd25: cosine = -16'sd792;
            6'd26: cosine = -16'sd851;
            6'd27: cosine = -16'sd903;
            6'd28: cosine = -16'sd946;
            6'd29: cosine = -16'sd980;
            6'd30: cosine = -16'sd1004;
            6'd31: cosine = -16'sd1019;
            6'd32: cosine = -16'sd1024;
            default: cosine = -16'sd1024;
        endcase end
    endfunction
    function signed [15:0] limit_command;
        input signed [63:0] value;
        input integer maximum;
        begin
            if (value > maximum) limit_command = maximum;
            else if (value < -maximum) limit_command = -maximum;
            else limit_command = value[15:0];
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; fault <= 0; command <= 0; watchdog <= 0;
            swing_ms <= 0; catch_ms <= 0; capture_arm <= 0; stage <= 0;
            p_theta <= 0; p_omega <= 0; p_arm <= 0; p_speed <= 0;
            balance_sum <= 0; omega_square <= 0; energy_deficit <= 0;
            swing_command <= 0; cos_theta <= 0; pump_negative <= 0;
            index_product <= 0; angle_times32 <= 0; lut_index <= 0; omega_negative <= 0;
        end else begin
            if (sample_valid) watchdog <= 0;
            else if (watchdog < WATCHDOG_CYCLES) watchdog <= watchdog+1'b1;
            if (stop) begin
                state <= IDLE; fault <= 0; command <= 0; stage <= 0;
            end else if (clear && state != SWING && state != BALANCE) begin
                state <= IDLE; fault <= 0; command <= 0; stage <= 0;
            end else if ((state == SWING || state == BALANCE) &&
                         (sensor_fault || !calibrated || watchdog >= WATCHDOG_CYCLES)) begin
                state <= FAULT; command <= 0; stage <= 0;
                fault <= sensor_fault ? 8'h01 : (!calibrated ? 8'h02 : 8'h04);
            end else if (start && state == IDLE && calibrated && !sensor_fault) begin
                state <= SWING; swing_ms <= 0; catch_ms <= 0;
                command <= 0; fault <= 0; stage <= 0; watchdog <= 0;
            end else begin
                if (sample_valid && (state == SWING || state == BALANCE)) begin
                    if (abs_arm > 6144 || abs_speed > 20480 || abs_omega > 30720) begin
                        state <= FAULT; fault <= 8'h08; command <= 0; stage <= 0;
                    end else if (state == SWING && swing_ms >= SWING_TIMEOUT_MS) begin
                        state <= FAULT; fault <= 8'h10; command <= 0; stage <= 0;
                    end else if (state == BALANCE && abs_theta > 715) begin
                        state <= FAULT; fault <= 8'h20; command <= 0; stage <= 0;
                    end else begin
                        if (state == SWING) begin
                            swing_ms <= swing_ms+1'b1;
                            if (abs_theta < 268 && abs_omega < 3584) begin
                                state <= BALANCE; capture_arm <= arm; catch_ms <= 0;
                            end
                        end else if (catch_ms < 512) catch_ms <= catch_ms+1'b1;
                        // Each product has a separate register; sum on the next cycle.
                        p_theta <= K_THETA * theta;
                        p_omega <= K_OMEGA * omega;
                        p_arm <= K_ARM * (state == SWING ? 32'sd0 : arm_error);
                        p_speed <= K_SPEED * arm_speed;
                        omega_square <= omega * omega;
                        index_product <= abs_theta * 32'd10431;
                        angle_times32 <= {abs_theta,5'b0};
                        omega_negative <= omega[15];
                        stage <= 1;
                    end
                end else case (stage)
                    1: begin
                        lut_index <= candidate_times3217 > angle_times32 ? index_candidate-6'd1 : index_candidate;
                        // Smooth arm stiffness during capture; damping stays fully active.
                        if (catch_ms < 128) balance_sum <= p_theta+p_omega+p_speed+(p_arm >>> 2);
                        else if (catch_ms < 256) balance_sum <= p_theta+p_omega+p_speed+(p_arm >>> 1);
                        else if (catch_ms < 384) balance_sum <= p_theta+p_omega+p_speed+(p_arm >>> 1)+(p_arm >>> 2);
                        else balance_sum <= p_theta+p_omega+p_speed+p_arm;
                        stage <= 2;
                    end
                    2: begin
                        cos_theta <= lookup_cos;
                        pump_negative <= omega_negative ^ lookup_cos[15];
                        stage <= 3;
                    end
                    3: begin
                        // d = 1-cos(theta)-omega^2/(2*m*g*l/J).
                        energy_deficit <= 1024-$signed(cos_theta)-$signed(kinetic_term[31:0]);
                        stage <= 4;
                    end
                    4: begin
                        swing_command <= swing_effort -
                            (($signed(arm_speed)*25) >>> 10) -
                            (($signed(arm)*42) >>> 10);
                        stage <= 5;
                    end
                    5: begin
                        if (state == BALANCE)
                            command <= limit_command(-(balance_sum >>> 20),1000);
                        else if (state == SWING)
                            command <= swing_ms <= KICK_MS ? KICK_VALUE :
                                       limit_command(swing_command,SWING_LIMIT);
                        else command <= 0;
                        stage <= 0;
                    end
                    default: stage <= 0;
                endcase
            end
        end
    end
endmodule
`default_nettype wire
