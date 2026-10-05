`timescale 1ns/1ps
`default_nettype none
// State units: signed Q10 radians / radians per second, updated at 1 kHz.
module state_estimator #(
    parameter integer THETA_SIGN = 1,
    parameter integer ENCODER_SIGN = 1,
    parameter integer COUNTS_PER_REV = 1040,
    parameter integer SAMPLE_FRESH_CYCLES = 100000,
    parameter integer BAD_WINDOW_LIMIT = 8
)(
    input wire clk, rst_n, sample_valid,
    input wire [13:0] sample_q4,
    input wire sample_bad,
    input wire sample_blind, sample_fault,
    input wire [7:0] sample_quality_reason,
    input wire signed [31:0] position,
    input wire cal_down, cal_up, stopped, clear_fault,
    output reg calibrated, sensor_fault, valid,
    output reg [15:0] sensor_fault_reason,
    output wire fault_clear_ready,
    output reg signed [15:0] theta, omega, arm, arm_speed,
    output wire [13:0] adc_down_q4, adc_up_q4,
    output reg measurement_ready, measurement_valid
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
    reg bad_sample, reprime_sample, blind_sample, hard_sample, step_primed;
    reg sample_seen;
    reg [31:0] sample_age;
    reg [4:0] trusted_samples;
    reg [4:0] good_windows;
    reg [7:0] bad_windows;
    reg [15:0] bad_reason;
    wire signed [14:0] span = $signed({1'b0,sample_q4})-$signed({1'b0,down_code});
    wire [14:0] span_abs = span < 0 ? -span : span;
    wire [13:0] adc_step = sample_q4 > previous_adc ? sample_q4-previous_adc : previous_adc-sample_q4;
    wire sample_fresh = sample_valid || (sample_seen && sample_age < SAMPLE_FRESH_CYCLES);
    // ADC rails belong to the potentiometer's unobservable sector. They are
    // unavailable measurements, not proof of an electrical sensor failure.
    wire blind_window = sample_blind || sample_q4 < 16 || sample_q4 > 16352;
    wire window_good = !sample_bad && !sample_fault && !blind_window;
    wire discontinuity = calibrated && step_primed && sample_age < SAMPLE_FRESH_CYCLES && adc_step > 512;
    wire unusable_window = !window_good || discontinuity;
    wire calibration_request = stopped && (cal_down || cal_up);
    wire [15:0] window_reason = {8'd0,sample_quality_reason} |
                               ((sample_q4 < 16 || sample_q4 > 16352) ? 16'h0002 : 16'd0) |
                               ((sample_bad && (sample_quality_reason & 8'h5f) == 0) ? 16'h1000 : 16'd0) |
                               (sample_fault ? 16'h0001 : 16'd0);
    // An explicit R may acknowledge a recovered sensor while stopped. It must
    // never convert stale/untrusted data into health or start the controller.
    assign fault_clear_ready = stopped && sample_fresh && window_good &&
                               good_windows >= 16 &&
                               (!calibrated || (measurement_valid && measurement_ready));
    assign adc_down_q4 = down_code;
    assign adc_up_q4 = up_code;
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
            bad_sample <= 0; reprime_sample <= 0; blind_sample <= 0; hard_sample <= 0; step_primed <= 0;
            theta <= 0; omega <= 0; arm <= 0; arm_speed <= 0;
            sample_seen <= 0; sample_age <= 0; trusted_samples <= 0;
            measurement_ready <= 0; measurement_valid <= 0;
            good_windows <= 0; bad_windows <= 0; bad_reason <= 0; sensor_fault_reason <= 0;
        end else begin
            valid <= 0; div_start <= 0;
            if (sample_valid) begin sample_seen <= 1; sample_age <= 0; end
            else if (sample_age < SAMPLE_FRESH_CYCLES) sample_age <= sample_age+1'b1;
            if (!sample_fresh) good_windows <= 0;
            if (sample_valid) begin
                if (unusable_window) good_windows <= 0;
                else if (sample_seen && sample_age >= SAMPLE_FRESH_CYCLES) good_windows <= 1;
                else if (good_windows < 16) good_windows <= good_windows+1'b1;
                if (blind_window || !unusable_window) bad_windows <= 0;
                else if (sample_seen && sample_age >= SAMPLE_FRESH_CYCLES) bad_windows <= 1;
                else if (bad_windows < BAD_WINDOW_LIMIT) bad_windows <= bad_windows+1'b1;
            end
            if (clear_fault && fault_clear_ready) begin
                sensor_fault <= 0; sensor_fault_reason <= 0;
            end
            if (stopped && cal_down && !div_busy && sample_fresh && window_good) begin
                down_code <= sample_q4; have_down <= 1; calibrated <= 0;
                sensor_fault <= 0; sensor_fault_reason <= 0; primed <= 0;
            end
            if (stopped && cal_up && !cal_down && have_down && !div_busy && sample_fresh && window_good) begin
                calibrated <= 0; primed <= 0;
                // Reject too close points and a zero near either ADC rail.
                if (span_abs >= 512 && span_abs <= 14400 && sample_q4 > 160 && sample_q4 < 16208) begin
                    up_code <= sample_q4; divisor <= span_abs[13:0];
                    slope_negative <= (span > 0); div_start <= 1; origin <= position;
                    previous_position <= position; velocity_filter <= 0; arm_velocity_filter <= 0;
                    sensor_fault <= 0; sensor_fault_reason <= 0;
                end else begin
                    sensor_fault <= 1;
                    if (!sensor_fault) sensor_fault_reason <= 16'h0400;
                end
            end
            if (div_done && !cal_down && !cal_up) begin slope <= quotient[15:0]; calibrated <= 1; end
            // Calibration cancels an in-flight state sample; old work must never
            // overwrite calibration's derivative initialization.
            if (calibration_request) begin
                stage <= 0; primed <= 0; step_primed <= 0; valid <= 0; bad_windows <= 0;
                trusted_samples <= 0; measurement_ready <= 0; measurement_valid <= 0;
                velocity_filter <= 0; arm_velocity_filter <= 0; omega <= 0; arm_speed <= 0;
                // A calibration key must never turn a stale or contaminated
                // ADC window into a trusted endpoint or clear its fault.
                if (!sample_fresh || (!window_good && !blind_window)) begin
                    sensor_fault <= 1;
                    if (!sensor_fault) sensor_fault_reason <= (!sample_fresh ? 16'h0800 : 16'd0) |
                                                             (!window_good ? window_reason : 16'd0);
                end
            end else if (!calibrated) begin
                stage <= 0; primed <= 0; step_primed <= 0; trusted_samples <= 0;
                measurement_ready <= 0; measurement_valid <= 0;
                if (sample_valid && !blind_window &&
                    (sample_fault || (!window_good && bad_windows >= BAD_WINDOW_LIMIT-1))) begin
                    sensor_fault <= 1;
                    if (!sensor_fault) sensor_fault_reason <= window_reason;
                end
            end else case (stage)
                0: if (sample_valid && calibrated && !cal_down && !cal_up) begin
                    adc_difference <= $signed({1'b0,sample_q4})-$signed({1'b0,up_code});
                    count_difference <= (position-origin)*ENCODER_SIGN;
                    count_step <= (position-previous_position)*ENCODER_SIGN;
                    previous_position <= position; previous_adc <= sample_q4;
                    step_primed <= !blind_window;
                    reprime_sample <= sample_seen && sample_age >= SAMPLE_FRESH_CYCLES;
                    bad_sample <= unusable_window;
                    blind_sample <= blind_window;
                    hard_sample <= !blind_window && (sample_fault ||
                        (unusable_window && bad_windows >= BAD_WINDOW_LIMIT-1));
                    bad_reason <= (!window_good ? window_reason : 16'd0) |
                                  (discontinuity ? 16'h0100 : 16'd0);
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
                    if ((theta_product >>> 10) > 9651 || (theta_product >>> 10) < -9651) begin
                        bad_sample <= 1;
                        bad_reason <= bad_reason | 16'h0200;
                        // This is a calibration/model domain failure, not an
                        // impulse threshold or a blind-sector observation.
                        if (!blind_sample && !bad_sample) hard_sample <= 1;
                    end
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
                        primed <= 0;
                        if (hard_sample) begin
                            sensor_fault <= 1;
                            if (!sensor_fault) sensor_fault_reason <= bad_reason;
                        end
                        trusted_samples <= 0; measurement_ready <= 0; measurement_valid <= 0;
                        velocity_filter <= 0; arm_velocity_filter <= 0; omega <= 0; arm_speed <= 0;
                    end else begin
                        theta <= sat16(theta_work); arm <= sat16(arm_product >>> 10);
                        previous_theta <= sat16(theta_work); primed <= 1;
                        // These flags describe completed measurement quality,
                        // not its current age. Top-level freshness gating owns
                        // timeout classification and masks both flags when stale.
                        measurement_valid <= 1;
                        if (reprime_sample) begin
                            trusted_samples <= 1; measurement_ready <= 0;
                        end else begin
                            if (trusted_samples < 16) trusted_samples <= trusted_samples+1'b1;
                            if (trusted_samples >= 15) measurement_ready <= 1;
                        end
                        // Estimate motion in IDLE/FAULT as well. Controller stop
                        // does not imply the physical mechanism has stopped.
                        if (!primed || reprime_sample) begin
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
