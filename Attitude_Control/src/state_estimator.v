`timescale 1ns/1ps
`default_nettype none
// State units: signed Q10 radians / radians per second, updated at 1 kHz.
module state_estimator #(
    parameter integer THETA_SIGN = 1,
    parameter integer ENCODER_SIGN = 1,
    parameter integer COUNTS_PER_REV = 1040
)(
    input wire clk, rst_n, sample_valid,
    input wire [13:0] sample_q4,
    input wire signed [31:0] position,
    input wire cal_down, cal_up, stopped,
    output reg calibrated, sensor_fault, valid,
    output reg signed [15:0] theta, omega, arm, arm_speed
);
    localparam integer ENC_COEFF = 6588397 / COUNTS_PER_REV;
    reg [13:0] down_code, up_code, divisor;
    reg have_down, div_start, slope_negative, primed;
    reg [15:0] slope;
    wire [31:0] quotient;
    wire div_busy, div_done;
    reg signed [31:0] origin, previous_position;
    reg signed [15:0] previous_theta;
    reg [13:0] previous_adc;
    reg signed [31:0] velocity_filter, arm_velocity_filter;
    reg [2:0] stage;
    reg signed [14:0] adc_difference;
    reg signed [31:0] count_difference, count_step;
    reg signed [63:0] theta_product, arm_product, arm_step_product;
    reg signed [31:0] theta_work;
    reg signed [31:0] theta_step, speed_work, arm_speed_work;
    reg bad_sample;
    wire signed [14:0] span = $signed({1'b0,sample_q4})-$signed({1'b0,down_code});
    wire [14:0] span_abs = span < 0 ? -span : span;
    wire [13:0] adc_step = sample_q4 > previous_adc ? sample_q4-previous_adc : previous_adc-sample_q4;
    calibration_divider u_div(clk,rst_n,div_start,divisor,quotient,div_busy,div_done);
    function signed [15:0] sat16;
        input signed [63:0] value;
        begin
            if (value > 32767) sat16 = 16'sh7fff;
            else if (value < -32768) sat16 = 16'sh8000;
            else sat16 = value[15:0];
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            calibrated <= 0; sensor_fault <= 0; valid <= 0; down_code <= 0;
            up_code <= 0; divisor <= 0; have_down <= 0; div_start <= 0;
            slope_negative <= 0; slope <= 0; origin <= 0; previous_position <= 0;
            previous_theta <= 0; previous_adc <= 0; primed <= 0;
            velocity_filter <= 0; arm_velocity_filter <= 0; stage <= 0;
            adc_difference <= 0; count_difference <= 0; count_step <= 0;
            theta_product <= 0; arm_product <= 0; arm_step_product <= 0;
            theta_work <= 0; theta_step <= 0; speed_work <= 0; arm_speed_work <= 0;
            bad_sample <= 0; theta <= 0; omega <= 0; arm <= 0; arm_speed <= 0;
        end else begin
            valid <= 0; div_start <= 0;
            if (stopped && cal_down && !div_busy) begin
                down_code <= sample_q4; have_down <= 1; calibrated <= 0;
                sensor_fault <= 0; primed <= 0;
            end
            if (stopped && cal_up && !cal_down && have_down && !div_busy) begin
                calibrated <= 0; primed <= 0;
                // Reject too close points and a zero near either ADC rail.
                if (span_abs >= 512 && span_abs <= 14400 && sample_q4 > 160 && sample_q4 < 16208) begin
                    up_code <= sample_q4; divisor <= span_abs[13:0];
                    slope_negative <= (span > 0); div_start <= 1; origin <= position;
                    previous_position <= position; velocity_filter <= 0; arm_velocity_filter <= 0;
                    sensor_fault <= 0;
                end else sensor_fault <= 1;
            end
            if (div_done && !cal_down && !cal_up) begin slope <= quotient[15:0]; calibrated <= 1; end
            // Calibration cancels an in-flight state sample; old work must never
            // overwrite calibration's derivative initialization.
            if (stopped && (cal_down || cal_up)) begin
                stage <= 0; primed <= 0; valid <= 0;
            end else case (stage)
                0: if (sample_valid && calibrated && !cal_down && !cal_up) begin
                    adc_difference <= $signed({1'b0,sample_q4})-$signed({1'b0,up_code});
                    count_difference <= (position-origin)*ENCODER_SIGN;
                    count_step <= (position-previous_position)*ENCODER_SIGN;
                    previous_position <= position; previous_adc <= sample_q4;
                    bad_sample <= (sample_q4 < 16 || sample_q4 > 16352 ||
                                   (primed && !stopped && adc_step > 512));
                    stage <= 1;
                end
                1: begin
                    theta_product <= adc_difference * $signed({1'b0,slope}) *
                                     (slope_negative ? -THETA_SIGN : THETA_SIGN);
                    arm_product <= count_difference * ENC_COEFF;
                    arm_step_product <= count_step * ENC_COEFF * 1000;
                    stage <= 2;
                end
                2: begin
                    theta_work <= $signed(theta_product[41:10]);
                    if ((theta_product >>> 10) > 9651 || (theta_product >>> 10) < -9651)
                        bad_sample <= 1;
                    stage <= 3;
                end
                3: begin
                    // Map the two calibrated half turns to [-pi, pi].
                    if (theta_work > 3217) theta_work <= theta_work-6434;
                    else if (theta_work < -3217) theta_work <= theta_work+6434;
                    stage <= 4;
                end
                4: begin
                    theta_step <= theta_work-$signed(previous_theta);
                    arm_speed_work <= $signed(arm_step_product[41:10]);
                    stage <= 5;
                end
                5: begin
                    if (theta_step > 3217) speed_work <= (theta_step-6434)*1000;
                    else if (theta_step < -3217) speed_work <= (theta_step+6434)*1000;
                    else speed_work <= theta_step*1000;
                    stage <= 6;
                end
                6: begin
                    if (bad_sample) begin
                        sensor_fault <= 1; primed <= 0;
                        velocity_filter <= 0; arm_velocity_filter <= 0; omega <= 0; arm_speed <= 0;
                    end else begin
                        theta <= sat16(theta_work); arm <= sat16(arm_product >>> 10);
                        previous_theta <= sat16(theta_work); primed <= 1;
                        if (!primed || stopped) begin
                            velocity_filter <= 0; arm_velocity_filter <= 0; omega <= 0; arm_speed <= 0;
                        end else begin
                            // Retain fractional filter residues in 32 bits to avoid a dead zone.
                            velocity_filter <= velocity_filter+((speed_work-velocity_filter) >>> 3);
                            arm_velocity_filter <= arm_velocity_filter+((arm_speed_work-arm_velocity_filter) >>> 3);
                            omega <= sat16(velocity_filter+((speed_work-velocity_filter) >>> 3));
                            arm_speed <= sat16(arm_velocity_filter+((arm_speed_work-arm_velocity_filter) >>> 3));
                        end
                    end
                    valid <= 1; stage <= 0;
                end
                default: stage <= 0;
            endcase
        end
    end
endmodule
`default_nettype wire
