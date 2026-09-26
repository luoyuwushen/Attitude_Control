`timescale 1ns/1ps
`default_nettype none
module top #(
    parameter integer THETA_SIGN = 1,
    parameter integer ENCODER_SIGN = 1,
    parameter integer MOTOR_SIGN = 1,
    parameter integer COUNTS_PER_REV = 1040,
    parameter integer KEY_CYCLES = 1000000,
    parameter integer ADC_SAMPLE_CYCLES = 3125,
    parameter integer JOG_CYCLES = 7500000
)(
    input wire clk_50m, rst_n,
    input wire [3:0] key_sw,
    input wire uart_rx,
    output wire uart_tx,
    input wire [9:0] adc_data_in,
    input wire adc_otr,
    output wire adc_clk, adc_oe_n,
    input wire enc1_a, enc1_b,
    output wire AN1, AN2, PWMA,
    output wire BN1, BN2, PWMB
);
    wire reset_n;
    reset_sync u_reset(clk_50m,rst_n,reset_n);
    wire [3:0] press, key_level;
    genvar i;
    generate for (i=0;i<4;i=i+1) begin: keys
        key_debounce #(.STABLE_CYCLES(KEY_CYCLES)) u_key(
            clk_50m,reset_n,key_sw[i],key_level[i],press[i]);
    end endgenerate
    wire [7:0] rx_data;
    wire rx_valid, rx_error;
    uart_rx_byte u_rx(clk_50m,reset_n,uart_rx,rx_data,rx_valid,rx_error);
    wire request_down = press[0] || (rx_valid && rx_data == 8'h44); // D
    wire request_up = press[3] || (rx_valid && rx_data == 8'h55); // U
    wire request_start = press[1] || (rx_valid && rx_data == 8'h47); // G
    wire request_stop = press[2] || (rx_valid && rx_data == 8'h53); // S
    (* syn_preserve = 1, async_reg = "true" *) reg [1:0] stop_sync;
    always @(posedge clk_50m or negedge reset_n) begin
        if (!reset_n) stop_sync <= 2'b11;
        else stop_sync <= {stop_sync[0],key_sw[2]};
    end
    wire request_clear = rx_valid && rx_data == 8'h52; // R
    wire request_forward = rx_valid && rx_data == 8'h46; // F
    wire request_backward = rx_valid && rx_data == 8'h42; // B
    wire [2:0] state;
    reg [31:0] jog_remaining;
    reg signed [15:0] jog_command;
    wire jogging = jog_remaining != 0;
    wire stopped = (state == 0 || state == 3) && !jogging;
    wire [13:0] sample_q4;
    wire adc_valid, over_range;
    adc_sampler #(.SAMPLE_CYCLES(ADC_SAMPLE_CYCLES)) u_adc(
        clk_50m,reset_n,adc_data_in,adc_otr,adc_clk,adc_oe_n,
        sample_q4,adc_valid,over_range);
    wire signed [31:0] position;
    wire encoder_illegal;
    quadrature_encoder u_encoder(clk_50m,reset_n,enc1_a,enc1_b,1'b0,position,encoder_illegal);
    wire calibrated, sensor_fault, state_valid;
    wire signed [15:0] theta, omega, arm, arm_speed;
    state_estimator #(.THETA_SIGN(THETA_SIGN),.ENCODER_SIGN(ENCODER_SIGN),
                      .COUNTS_PER_REV(COUNTS_PER_REV)) u_estimator(
        clk_50m,reset_n,adc_valid,sample_q4,position,
        request_down && stopped,request_up && stopped,stopped,
        calibrated,sensor_fault,state_valid,theta,omega,arm,arm_speed);
    reg input_fault;
    always @(posedge clk_50m or negedge reset_n) begin
        if (!reset_n) input_fault <= 0;
        else if (stopped && request_down) input_fault <= 0;
        else if (!stopped && (over_range || encoder_illegal)) input_fault <= 1;
    end
    wire [7:0] fault;
    wire signed [15:0] command;
    wire motor_enable;
    wire board_fault = sensor_fault || input_fault || over_range;
    // Limited diagnostic movement for verifying the installed motor direction.
    // It is available only after calibration, while the controller is idle.
    always @(posedge clk_50m or negedge reset_n) begin
        if (!reset_n) begin jog_remaining <= 0; jog_command <= 0; end
        else if (request_stop || !stop_sync[1] || request_down || request_up ||
                 board_fault || state != 0 || !calibrated ||
                 arm > 6144 || arm < -6144 || arm_speed > 20480 || arm_speed < -20480) begin
            jog_remaining <= 0; jog_command <= 0;
        end else if ((request_forward || request_backward) && !jogging) begin
            jog_remaining <= JOG_CYCLES;
            jog_command <= request_backward ? -16'sd100 : 16'sd100;
        end else if (jogging) jog_remaining <= jog_remaining-1'b1;
    end
    // A press on a calibration key while running first stops the mechanism.
    attitude_controller u_controller(
        clk_50m,reset_n,state_valid,calibrated,board_fault,
        request_start && !jogging,request_stop || !stop_sync[1] || request_down || request_up,request_clear,
        theta,omega,arm,arm_speed,state,fault,command,motor_enable);
    wire signed [15:0] requested_command = jogging ? jog_command : command;
    reg signed [15:0] motor_command;
    reg motor_permission;
    always @(posedge clk_50m or negedge reset_n) begin
        if (!reset_n) begin motor_permission <= 0; motor_command <= 0; end
        else begin
            motor_permission <= motor_enable || (jogging && !board_fault);
            motor_command <= MOTOR_SIGN < 0 ? -requested_command : requested_command;
        end
    end
    // SW3 raw assertion is an immediate hardware gate; release requires a fresh G/SW2.
    motor_pwm u_motor(clk_50m,reset_n,motor_permission && key_sw[2] && rst_n,
                      motor_command,AN1,AN2,PWMA);
    // Only motor A belongs to this single-arm mechanism. Keep B in high impedance.
    assign BN1=1'b0; assign BN2=1'b0; assign PWMB=1'b1;
    reg [4:0] telemetry_count;
    reg telemetry_valid;
    reg telemetry_due;
    always @(posedge clk_50m or negedge reset_n) begin
        if (!reset_n) begin telemetry_count <= 0; telemetry_valid <= 0; telemetry_due <= 0; end
        else begin
            telemetry_valid <= 0;
            if (adc_valid) begin
                if (telemetry_count == 19) begin
                    telemetry_count <= 0; telemetry_due <= 1;
                end else telemetry_count <= telemetry_count+1'b1;
            end
            // Publish angle, speed and ADC from the same acquisition frame.
            if (telemetry_due && (state_valid || !calibrated)) begin
                telemetry_valid <= 1; telemetry_due <= 0;
            end
        end
    end
    telemetry u_telemetry(clk_50m,reset_n,telemetry_valid,sample_q4[13:4],
                          theta,omega,arm,arm_speed,requested_command,jogging ? 3'd4 : state,fault,
                          calibrated,uart_tx);
endmodule
`default_nettype wire
