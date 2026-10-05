`timescale 1ns/1ps
`default_nettype none
// 1kHz energy swing-up + discrete full-state feedback. Q10 state, permille PWM.
// Gains are derived from the documented nominal model, not a tuned real board.
module attitude_controller #(
    parameter signed [31:0] K_THETA = 32'sd2362613,
    parameter signed [31:0] K_OMEGA = 32'sd299007,
    parameter signed [31:0] K_ARM = -32'sd109597,
    parameter signed [31:0] K_SPEED = -32'sd118252,
    // H-only speed-gain comparison derived from the R=32/Qi=1 LQI baseline.
    // Other H gains/integral and the independent G baseline remain unchanged.
    parameter signed [31:0] H_K_THETA = 32'sd904758,
    parameter signed [31:0] H_K_OMEGA = 32'sd77909,
    parameter signed [31:0] H_K_ARM = -32'sd34213,
    parameter signed [31:0] H_K_SPEED = -32'sd61768,
    parameter integer SWING_GAIN = 667,
    parameter integer SWING_LIMIT = 800,
    parameter integer KICK_COMMAND = 667,
    parameter integer KICK_MS = 40,
    parameter integer SWING_TIMEOUT_MS = 15000,
    parameter integer WATCHDOG_CYCLES = 100000
)(
    input wire clk, rst_n, sample_valid, calibrated, sensor_fault,
    input wire start, stop, clear,
    input wire start_balance, measurement_ready,
    input wire signed [15:0] theta, omega, arm, arm_speed,
    output reg [2:0] state,
    output reg [7:0] fault,
    output reg signed [15:0] command,
    output wire enable,
    output reg [55:0] handover_detail,
    // Passive observer ports. Pulses reflect actual accepted/committed work;
    // they never participate in controller admission or motor permission.
    output wire trace_h_active,
    output wire signed [15:0] trace_capture_arm,
    output reg trace_sample_accept, trace_command_commit,
    output reg [63:0] trace_states,
    output reg signed [31:0] trace_integral_q24
);
    localparam [2:0] IDLE=0, SWING=1, BALANCE=2, FAULT=3;
    reg [31:0] watchdog;
    reg [15:0] swing_ms, catch_ms;
    reg signed [15:0] capture_arm;
    reg handover_profile;
    assign trace_h_active = state == BALANCE && handover_profile;
    assign trace_capture_arm = capture_arm;
    reg signed [16:0] arm_error_sample;
    reg signed [49:0] arm_magnitude_product;
    reg [2:0] stage;
    reg signed [63:0] p_theta, p_omega, p_arm, p_speed, balance_sum;
    reg signed [63:0] balance_request;
    // Q24 permille; 237 per Q10 error/sample is 14.46533203125 permille/rad/s.
    // 100 permille and the +/-0.02 permille/sample rate fit signed 32 bits.
    localparam signed [31:0] INTEGRAL_LIMIT = 32'sd1677721600;
    localparam signed [31:0] INTEGRAL_RATE = 32'sd335544;
    reg signed [31:0] integral_accumulator, integral_delta;
    reg signed [32:0] integral_candidate;
    reg signed [24:0] integral_times240, integral_times3, integral_raw_delta;
    reg integral_window_sample, integral_allowed_sample;
    reg signed [31:0] omega_square, energy_deficit, swing_command;
    reg signed [15:0] cos_theta;
    reg pump_negative;
    wire [15:0] abs_theta = theta[15] ? -theta : theta;
    wire [15:0] abs_omega = omega[15] ? -omega : omega;
    wire [15:0] abs_arm = arm[15] ? -arm : arm;
    wire [15:0] abs_speed = arm_speed[15] ? -arm_speed : arm_speed;
    // Unsigned magnitudes retain abs(-32768)=32768. Admission and running
    // protection share the same strict limits; equality remains permitted.
    wire motion_limit_exceeded = abs_arm > 16'd6144 ||
                                abs_speed > 16'd20480 || abs_omega > 16'd30720;
    reg [31:0] index_product;
    reg [20:0] angle_times32;
    reg [5:0] lut_index;
    reg omega_negative;
    wire [5:0] index_candidate = index_product[25:20];
    // ceil(32*2^20/3217), then one correction yields exact floor division.
    wire [20:0] candidate_times3217 = index_candidate * 21'd3217;
    // Two signed 16-bit coordinates have an exact signed 17-bit difference.
    wire signed [16:0] arm_error = $signed({arm[15],arm})-$signed({capture_arm[15],capture_arm});
    // 25 bits retain the largest 240*65535 intermediate without overflow.
    wire signed [24:0] integral_error_extended = {{8{arm_error_sample[16]}},arm_error_sample};
    wire integral_saturation_freeze = integral_allowed_sample &&
        ((balance_request >= 64'sd1000 && integral_delta > 0) ||
         (balance_request <= -64'sd1000 && integral_delta < 0));
    wire integral_update_allowed = integral_allowed_sample && !integral_saturation_freeze;
    wire integral_at_limit = integral_accumulator >= INTEGRAL_LIMIT ||
                            integral_accumulator <= -INTEGRAL_LIMIT;
    wire signed [15:0] integral_rounded = round_integral(integral_accumulator);
    wire signed [63:0] integral_rounded_extended = {{48{integral_rounded[15]}},integral_rounded};
    // Separate a negative coefficient's sign restoration from multiplication.
    // The extra gain bit also handles K_ARM=-2^31 without absolute overflow.
    localparam signed [32:0] ARM_GAIN_EXT = {K_ARM[31],K_ARM};
    localparam signed [32:0] ARM_GAIN_MAG = K_ARM < 0 ? -ARM_GAIN_EXT : ARM_GAIN_EXT;
    localparam signed [32:0] H_ARM_GAIN_EXT = {H_K_ARM[31],H_K_ARM};
    localparam signed [32:0] H_ARM_GAIN_MAG = H_K_ARM < 0 ? -H_ARM_GAIN_EXT : H_ARM_GAIN_EXT;
    wire signed [31:0] selected_theta_gain = handover_profile ? H_K_THETA : K_THETA;
    wire signed [31:0] selected_omega_gain = handover_profile ? H_K_OMEGA : K_OMEGA;
    wire signed [31:0] selected_speed_gain = handover_profile ? H_K_SPEED : K_SPEED;
    wire signed [32:0] selected_arm_magnitude = handover_profile ? H_ARM_GAIN_MAG : ARM_GAIN_MAG;
    wire selected_arm_negative = handover_profile ? (H_K_ARM < 0) : (K_ARM < 0);
    wire signed [49:0] arm_mult /* synthesis syn_dspstyle = "dsp" */;
    assign arm_mult = selected_arm_magnitude * arm_error_sample;
    wire signed [63:0] arm_product_extended = {{14{arm_magnitude_product[49]}},arm_magnitude_product};
    wire signed [15:0] lookup_cos = cosine(lut_index);
    wire signed [31:0] swing_effort = (energy_deficit < 0) ^ pump_negative ?
                                    -SWING_GAIN : SWING_GAIN;
    wire signed [63:0] kinetic_term = ($signed({1'b0,omega_square})*64'sd167) >>> 25;
    localparam signed [15:0] KICK_VALUE = KICK_COMMAND;
    assign enable = (state == SWING || state == BALANCE) && calibrated &&
                    measurement_ready && !sensor_fault && !stop && fault == 0;
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
    // Symmetric nearest-integer rounding, with exact half values away from zero.
    function signed [15:0] round_integral;
        input signed [31:0] value;
        reg signed [32:0] magnitude, rounded;
        begin
            magnitude = value < 0 ? -$signed({value[31],value}) : $signed({value[31],value});
            rounded = (magnitude+33'sd8388608) >>> 24;
            round_integral = value < 0 ? -rounded : rounded;
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; fault <= 0; command <= 0; watchdog <= 0;
            swing_ms <= 0; catch_ms <= 0; capture_arm <= 0; arm_error_sample <= 0;
            handover_profile <= 0;
            arm_magnitude_product <= 0; stage <= 0;
            p_theta <= 0; p_omega <= 0; p_arm <= 0; p_speed <= 0;
            balance_sum <= 0; omega_square <= 0; energy_deficit <= 0;
            balance_request <= 0; integral_accumulator <= 0; integral_delta <= 0;
            integral_candidate <= 0; integral_times240 <= 0; integral_times3 <= 0;
            integral_raw_delta <= 0; integral_window_sample <= 0;
            integral_allowed_sample <= 0; handover_detail <= 0;
            swing_command <= 0; cos_theta <= 0; pump_negative <= 0;
            index_product <= 0; angle_times32 <= 0; lut_index <= 0; omega_negative <= 0;
            trace_sample_accept <= 0; trace_command_commit <= 0;
            trace_states <= 0; trace_integral_q24 <= 0;
        end else begin
            trace_sample_accept <= 0; trace_command_commit <= 0;
            // G and idle never retain a hidden H compensation. Running R keeps
            // its existing ignored semantics; only the stopped clear below acts.
            if (!handover_profile) integral_accumulator <= 0;
            if (sample_valid) watchdog <= 0;
            else if (watchdog < WATCHDOG_CYCLES) watchdog <= watchdog+1'b1;
            if (stop) begin
                state <= IDLE; fault <= 0; command <= 0; stage <= 0;
                handover_profile <= 0;
                integral_accumulator <= 0; handover_detail <= 0;
            end else if (clear && state != SWING && state != BALANCE) begin
                state <= IDLE; fault <= 0; command <= 0; stage <= 0;
                handover_profile <= 0;
                integral_accumulator <= 0; handover_detail <= 0;
            end else if ((state == SWING || state == BALANCE) &&
                         (sensor_fault || !calibrated || !measurement_ready || watchdog >= WATCHDOG_CYCLES)) begin
                state <= FAULT; command <= 0; stage <= 0;
                integral_accumulator <= 0; handover_detail <= 0;
                fault <= sensor_fault ? 8'h01 : (!calibrated ? 8'h02 :
                         (watchdog >= WATCHDOG_CYCLES ? 8'h04 : 8'h01));
            end else if ((start || start_balance) && state == IDLE && calibrated &&
                         !sensor_fault && measurement_ready && !motion_limit_exceeded) begin
                // H is an independent upright handover. Never fall back to a
                // swing kick if its angle/speed admission fails, even with G.
                if (!start_balance || (abs_theta < 268 && abs_omega < 3584)) begin
                    state <= start_balance ? BALANCE : SWING;
                    handover_profile <= start_balance;
                    capture_arm <= arm; swing_ms <= 0; catch_ms <= 0;
                    command <= 0; fault <= 0; stage <= 0; watchdog <= 0;
                    integral_accumulator <= 0; handover_detail <= 0;
                end
            end else begin
                if (sample_valid && (state == SWING || state == BALANCE)) begin
                    if (motion_limit_exceeded) begin
                        state <= FAULT; fault <= 8'h08; command <= 0; stage <= 0;
                        integral_accumulator <= 0; handover_detail <= 0;
                    end else if (state == SWING && swing_ms >= SWING_TIMEOUT_MS) begin
                        state <= FAULT; fault <= 8'h10; command <= 0; stage <= 0;
                        integral_accumulator <= 0; handover_detail <= 0;
                    end else if (state == BALANCE && abs_theta > 715) begin
                        state <= FAULT; fault <= 8'h20; command <= 0; stage <= 0;
                        integral_accumulator <= 0; handover_detail <= 0;
                    end else begin
                        if (handover_profile) begin
                            trace_sample_accept <= 1;
                            trace_states <= {arm_speed,arm,omega,theta};
                        end
                        if (state == SWING) begin
                            swing_ms <= swing_ms+1'b1;
                            if (abs_theta < 268 && abs_omega < 3584) begin
                                state <= BALANCE; capture_arm <= arm; catch_ms <= 0;
                            end
                        end else if (catch_ms < 512) catch_ms <= catch_ms+1'b1;
                        // Each product has a separate register; sum on the next cycle.
                        p_theta <= selected_theta_gain * theta;
                        p_omega <= selected_omega_gain * omega;
                        arm_error_sample <= state == SWING ? 17'sd0 : arm_error;
                        integral_window_sample <= abs_theta < 268 && abs_omega < 3584 && abs_speed < 10240;
                        p_speed <= selected_speed_gain * arm_speed;
                        omega_square <= omega * omega;
                        index_product <= abs_theta * 32'd10431;
                        angle_times32 <= {abs_theta,5'b0};
                        omega_negative <= omega[15];
                        stage <= 1;
                    end
                end else case (stage)
                    1: begin
                        lut_index <= candidate_times3217 > angle_times32 ? index_candidate-6'd1 : index_candidate;
                        // Keep subtraction and multiplication in separate clocks.
                        arm_magnitude_product <= arm_mult;
                        // 237 = 240-3, split across registers instead of a DSP.
                        integral_times240 <= (integral_error_extended <<< 8)-(integral_error_extended <<< 4);
                        integral_times3 <= (integral_error_extended <<< 1)+integral_error_extended;
                        stage <= 2;
                    end
                    2: begin
                        // Restore the sign BEFORE arithmetic shifting so negative
                        // rounding remains bit-exact during the stiffness ramp.
                        p_arm <= selected_arm_negative ? -arm_product_extended : arm_product_extended;
                        integral_raw_delta <= integral_times240-integral_times3;
                        cos_theta <= lookup_cos;
                        pump_negative <= omega_negative ^ lookup_cos[15];
                        stage <= 3;
                    end
                    3: begin
                        // Smooth arm stiffness during capture; damping stays fully active.
                        if (catch_ms < 128) balance_sum <= p_theta+p_omega+p_speed+(p_arm >>> 2);
                        else if (catch_ms < 256) balance_sum <= p_theta+p_omega+p_speed+(p_arm >>> 1);
                        else if (catch_ms < 384) balance_sum <= p_theta+p_omega+p_speed+(p_arm >>> 1)+(p_arm >>> 2);
                        else balance_sum <= p_theta+p_omega+p_speed+p_arm;
                        if (integral_raw_delta > INTEGRAL_RATE) integral_delta <= INTEGRAL_RATE;
                        else if (integral_raw_delta < -INTEGRAL_RATE) integral_delta <= -INTEGRAL_RATE;
                        else integral_delta <= {{7{integral_raw_delta[24]}},integral_raw_delta};
                        // d = 1-cos(theta)-omega^2/(2*m*g*l/J).
                        energy_deficit <= 1024-$signed(cos_theta)-$signed(kinetic_term[31:0]);
                        stage <= 4;
                    end
                    4: begin
                        // This request uses the OLD accumulator. Next d is
                        // committed only with this command at stage 5.
                        balance_request <= -(balance_sum >>> 20)+
                            (handover_profile ? integral_rounded_extended : 64'sd0);
                        integral_candidate <= $signed({integral_accumulator[31],integral_accumulator})+
                                              $signed({integral_delta[31],integral_delta});
                        integral_allowed_sample <= handover_profile && integral_window_sample && catch_ms >= 384;
                        swing_command <= swing_effort -
                            (($signed(arm_speed)*25) >>> 10) -
                            (($signed(arm)*42) >>> 10);
                        stage <= 5;
                    end
                    5: begin
                        if (state == BALANCE) begin
                            command <= limit_command(balance_request,1000);
                            if (handover_profile) begin
                                trace_command_commit <= 1;
                                trace_integral_q24 <= integral_accumulator;
                                handover_detail <= {{4'd0,integral_at_limit,integral_saturation_freeze,integral_update_allowed,1'b1},
                                                    catch_ms,capture_arm,integral_accumulator[31:16]};
                                if (integral_update_allowed) begin
                                    if (integral_candidate > $signed({INTEGRAL_LIMIT[31],INTEGRAL_LIMIT}))
                                        integral_accumulator <= INTEGRAL_LIMIT;
                                    else if (integral_candidate < -$signed({INTEGRAL_LIMIT[31],INTEGRAL_LIMIT}))
                                        integral_accumulator <= -INTEGRAL_LIMIT;
                                    else integral_accumulator <= integral_candidate[31:0];
                                end
                            end else begin integral_accumulator <= 0; handover_detail <= 0; end
                        end else if (state == SWING) begin
                            command <= swing_ms <= KICK_MS ? KICK_VALUE :
                                       limit_command(swing_command,SWING_LIMIT);
                            integral_accumulator <= 0; handover_detail <= 0;
                        end else begin command <= 0; integral_accumulator <= 0; handover_detail <= 0; end
                        stage <= 0;
                    end
                    default: stage <= 0;
                endcase
            end
        end
    end
endmodule
`default_nettype wire
