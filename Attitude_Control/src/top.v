`timescale 1ns/1ps
`default_nettype none
module top #(
    parameter integer THETA_SIGN = 1,
    parameter integer ENCODER_SIGN = 1,
    // Near-stationary downward tests on 2026-09-28: positive hardware command
    // decreased raw encoder counts while theta increased. Keep both measured
    // coordinates and invert the actuator mapping to match the model's input.
    // This sign evidence does not establish physical closed-loop stability.
    parameter integer MOTOR_SIGN = -1,
    parameter integer COUNTS_PER_REV = 1040,
    parameter integer KEY_CYCLES = 1000000,
    parameter integer ADC_SAMPLE_CYCLES = 3125,
    parameter integer JOG_CYCLES = 7500000,
    parameter integer MOTOR_TEST_CYCLES = 25000000,
    parameter integer MOTOR_TEST_NO_MOTION_CYCLES = 15000000,
    parameter integer MOTOR_TEST_MAX_TRAVEL = 128,
    parameter integer SAMPLE_WATCHDOG_CYCLES = 100000,
    parameter integer TELEMETRY_EXTENDED = 1,
    parameter integer TRACE_ADDRESS_BITS = 12,
    parameter integer UART_BAUD = 115200
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
    uart_rx_byte #(.BAUD(UART_BAUD)) u_rx(clk_50m,reset_n,uart_rx,rx_data,rx_valid,rx_error);
    wire request_down = press[0] || (rx_valid && rx_data == 8'h44); // D
    wire request_up = press[3] || (rx_valid && rx_data == 8'h55); // U
    wire request_start = press[1] || (rx_valid && rx_data == 8'h47); // G
    wire request_balance = rx_valid && rx_data == 8'h48; // H: upright handover
    wire request_motion = request_start || request_balance;
    wire request_stop = press[2] || (rx_valid && rx_data == 8'h53); // S
    (* syn_preserve = 1, async_reg = "true" *) reg [1:0] stop_sync;
    always @(posedge clk_50m or negedge reset_n) begin
        if (!reset_n) stop_sync <= 2'b11;
        else stop_sync <= {stop_sync[0],key_sw[2]};
    end
    wire request_clear = rx_valid && rx_data == 8'h52; // R
    wire request_forward = rx_valid && rx_data == 8'h46; // F
    wire request_backward = rx_valid && rx_data == 8'h42; // B
    // Independent fixed-duty tests: 15% J/K, 22% L/M, forward/reverse.
    wire request_motor_test = rx_valid && (rx_data == "J" || rx_data == "K" ||
                                          rx_data == "L" || rx_data == "M");
    wire signed [15:0] test_requested_command = rx_data == "K" ? -16'sd150 :
        rx_data == "M" ? -16'sd220 : rx_data == "L" ? 16'sd220 : 16'sd150;
    wire test_active;
    wire signed [15:0] test_command;
    wire [7:0] motor_test_status;
    wire signed [31:0] motor_test_delta;
    wire [2:0] state;
    reg [31:0] jog_remaining;
    reg signed [15:0] jog_command;
    wire jogging = jog_remaining != 0;
    wire stopped = (state == 0 || state == 3) && !jogging && !test_active;
    wire [13:0] sample_q4, control_q4;
    wire adc_valid, over_range;
    wire [9:0] adc_raw, adc_window_min, adc_window_max;
    wire sample_bad, sample_otr, sample_blind, sample_fault, spike_rejected;
    wire [7:0] adc_quality_reason;
    wire [3:0] motor_observe;
    wire [271:0] adc_window_detail;
    adc_sampler #(.SAMPLE_CYCLES(ADC_SAMPLE_CYCLES)) u_adc(
        .clk(clk_50m),.rst_n(reset_n),.data_in(adc_data_in),.otr(adc_otr),
        .adc_clk(adc_clk),.adc_oe_n(adc_oe_n),.sample_q4(sample_q4),
        .valid(adc_valid),.over_range(over_range),.raw_code(adc_raw),
        .window_min(adc_window_min),.window_max(adc_window_max),
        .sample_bad(sample_bad),.sample_otr(sample_otr),.quality_reason(adc_quality_reason),
        .motor_observe(motor_observe),.window_detail(adc_window_detail),
        .control_q4(control_q4),.sample_blind(sample_blind),
        .sample_fault(sample_fault),.spike_rejected(spike_rejected));
    wire signed [31:0] position;
    wire encoder_illegal;
    quadrature_encoder u_encoder(clk_50m,reset_n,enc1_a,enc1_b,1'b0,position,encoder_illegal);
    wire calibrated, sensor_fault, state_valid;
    wire measurement_ready, measurement_valid;
    wire fault_clear_ready;
    wire [15:0] sensor_fault_reason;
    wire [13:0] adc_down_q4, adc_up_q4;
    wire signed [15:0] theta, omega, arm, arm_speed;
    reg [31:0] sample_age;
    reg sample_timeout;
    // Link heartbeats are not proof of fresh measurements. Apply the same
    // 2 ms sample deadline in every mode, including idle and manual jog.
    always @(posedge clk_50m or negedge reset_n) begin
        if (!reset_n) begin sample_age <= 0; sample_timeout <= 0; end
        else if (calibrated ? state_valid : adc_valid) begin
            sample_age <= 0; sample_timeout <= 0;
        end else if (!sample_timeout) begin
            sample_age <= sample_age+1'b1;
            // Same expiry edge as age>=deadline, with a single-bit pad gate.
            if (sample_age >= SAMPLE_WATCHDOG_CYCLES-1) sample_timeout <= 1;
        end
    end
    wire accept_down = request_down && stopped && !sample_timeout && !sample_bad && !sample_blind && !sample_fault;
    wire accept_up = request_up && stopped && !sample_timeout && !sample_bad && !sample_blind && !sample_fault;
    state_estimator #(.THETA_SIGN(THETA_SIGN),.ENCODER_SIGN(ENCODER_SIGN),
                      .COUNTS_PER_REV(COUNTS_PER_REV),
                      .SAMPLE_FRESH_CYCLES(SAMPLE_WATCHDOG_CYCLES)) u_estimator(
        .clk(clk_50m),.rst_n(reset_n),.sample_valid(adc_valid),.sample_q4(control_q4),
        .sample_blind(sample_blind),.sample_fault(sample_fault),
        .sample_bad(sample_bad),.position(position),.cal_down(accept_down),
        .sample_quality_reason(adc_quality_reason),.clear_fault(request_clear),
        .sensor_fault_reason(sensor_fault_reason),.fault_clear_ready(fault_clear_ready),
        .cal_up(accept_up),.stopped(stopped),.calibrated(calibrated),
        .sensor_fault(sensor_fault),.valid(state_valid),.theta(theta),.omega(omega),
        .arm(arm),.arm_speed(arm_speed),.adc_down_q4(adc_down_q4),.adc_up_q4(adc_up_q4),
        .measurement_ready(measurement_ready),.measurement_valid(measurement_valid));
    reg input_fault;
    always @(posedge clk_50m or negedge reset_n) begin
        if (!reset_n) input_fault <= 0;
        else if (accept_down) input_fault <= 0;
        else if (request_clear && fault_clear_ready && !sample_fault && !sample_timeout && !encoder_illegal)
            input_fault <= 0;
        else if (!stopped && (sample_fault || encoder_illegal)) input_fault <= 1;
    end
    wire [7:0] fault;
    wire signed [15:0] command;
    wire motor_enable;
    // Raw OTR/rail codes remain telemetry evidence. Only qualified acquisition
    // faults enter the sticky fault path; a horizontal blind zone is not 0x01.
    wire board_fault = sensor_fault || input_fault || sample_fault;
    // Manual bounded motion uses speed protection, not the closed-loop origin.
    // It must remain usable after accumulated travel; no command resets origin.
    wire manual_speed_limit = arm_speed > 20480 || arm_speed < -20480;
    wire controller_motion_limit = arm > 6144 || arm < -6144 ||
                                   manual_speed_limit || omega > 30720 || omega < -30720;
    // Leaving the observable domain ends this run without inventing a sensor
    // fault or allowing automatic restart on recovery. A new H/G is required.
    wire measurement_interrupted = calibrated && !measurement_ready &&
                                   !board_fault && !sample_timeout;
    wire measurement_stop = (sample_blind || measurement_interrupted) &&
                            (state == 1 || state == 2) && fault == 0 && !board_fault && !sample_timeout;
    wire controller_stop = request_stop || !stop_sync[1] || request_down || request_up ||
                           measurement_stop;
    // Keep the last admission result: a key/UART pulse is much shorter than
    // the 20 ms telemetry interval. This register never enables the motor.
    reg [2:0] start_result;
    always @(posedge clk_50m or negedge reset_n) begin
        if (!reset_n) start_result <= 3'd0;
        else if (request_motion) begin
            // Match the controller's stop/clear/start priority using the
            // signals before this edge, including its old calibration flag.
            if (controller_stop) start_result <= 3'd5;
            else if (request_clear && state != 1 && state != 2) start_result <= 3'd7;
            else if (jogging || test_active) start_result <= 3'd6;
            else if (state != 0) start_result <= 3'd7;
            else if (!calibrated) start_result <= 3'd2;
            else if (sensor_fault) start_result <= 3'd3;
            else if (input_fault || sample_fault || sample_blind || sample_timeout || !measurement_ready) start_result <= 3'd4;
            else if (controller_motion_limit) start_result <= 3'd7;
            else if (request_balance && (theta <= -268 || theta >= 268 ||
                                         omega <= -3584 || omega >= 3584)) start_result <= 3'd7;
            else start_result <= 3'd1;
        end else if (stopped && (request_down || request_up)) start_result <= 3'd0;
    end
    // v1 byte 21: bit 7 identifies this diagnostic layout, bits 6:4 retain
    // admission, bits 3:0 are live SW3/sampler/input/estimator flags.
    wire [7:0] diagnostic_status = {1'b1,start_result,!stop_sync[1],
                                    over_range,input_fault,sensor_fault};
    // Idle board faults must be visible to existing host-v0.4.1 displays.
    // This is status aggregation; the controller's sticky fault is unchanged.
    wire [7:0] telemetry_fault = fault | (board_fault ? 8'h01 : 8'h00) |
                                (sample_timeout ? 8'h04 : 8'h00);
    wire motor_test_fault = board_fault || sample_timeout || !calibrated || manual_speed_limit;
    motor_test #(.RUN_CYCLES(MOTOR_TEST_CYCLES),
                 .NO_MOTION_CYCLES(MOTOR_TEST_NO_MOTION_CYCLES),
                 .MAX_TRAVEL(MOTOR_TEST_MAX_TRAVEL)) u_motor_test(
        .clk(clk_50m),.rst_n(reset_n),.request(request_motor_test),
        .requested_command(test_requested_command),
        .permit(state == 0 && !jogging && calibrated && measurement_ready &&
                measurement_valid && !request_motion && !request_forward && !request_backward),
        .stop(controller_stop || sample_blind || measurement_interrupted),.fault(motor_test_fault),.position(position),
        .active(test_active),.command(test_command),.status(motor_test_status),.delta(motor_test_delta));
    // Limited diagnostic movement for verifying the installed motor direction.
    // It is available only after calibration, while the controller is idle.
    always @(posedge clk_50m or negedge reset_n) begin
        if (!reset_n) begin jog_remaining <= 0; jog_command <= 0; end
        else if (request_stop || !stop_sync[1] || request_down || request_up ||
                 board_fault || sample_blind || sample_timeout || state != 0 || !calibrated ||
                 !measurement_ready || !measurement_valid || test_active ||
                 manual_speed_limit) begin
            jog_remaining <= 0; jog_command <= 0;
        end else if ((request_forward || request_backward) && !jogging && !request_motion && !request_motor_test) begin
            jog_remaining <= JOG_CYCLES;
            jog_command <= request_backward ? -16'sd100 : 16'sd100;
        end else if (jogging) jog_remaining <= jog_remaining-1'b1;
    end
    wire [55:0] handover_detail;
    wire trace_h_active, trace_sample_accept, trace_command_commit;
    wire [63:0] trace_states;
    wire signed [31:0] trace_integral_q24;
    wire signed [15:0] trace_capture_arm;
    // A press on a calibration key while running first stops the mechanism.
    attitude_controller #(.WATCHDOG_CYCLES(SAMPLE_WATCHDOG_CYCLES)) u_controller(
        .clk(clk_50m),.rst_n(reset_n),.sample_valid(state_valid),.calibrated(calibrated),
        .sensor_fault(board_fault),.start(request_start && !jogging && !test_active && !sample_timeout),
        .start_balance(request_balance && !jogging && !test_active && !sample_timeout),
        .measurement_ready(measurement_ready),.stop(controller_stop),.clear(request_clear),
        .theta(theta),.omega(omega),.arm(arm),.arm_speed(arm_speed),
        .state(state),.fault(fault),.command(command),.enable(motor_enable),
        .handover_detail(handover_detail),
        .trace_h_active(trace_h_active),.trace_sample_accept(trace_sample_accept),
        .trace_command_commit(trace_command_commit),.trace_states(trace_states),
        .trace_integral_q24(trace_integral_q24),.trace_capture_arm(trace_capture_arm));
    wire signed [15:0] requested_command = test_active ? test_command : jogging ? jog_command : command;
    reg signed [15:0] motor_command;
    reg motor_permission;
    always @(posedge clk_50m or negedge reset_n) begin
        if (!reset_n) begin motor_permission <= 0; motor_command <= 0; end
        else begin
            motor_permission <= !sample_timeout && !sample_blind &&
                                (motor_enable || ((jogging || test_active) && !board_fault));
            motor_command <= MOTOR_SIGN < 0 ? -requested_command : requested_command;
        end
    end
    // SW3 raw assertion is an immediate hardware gate; release requires a fresh G/SW2.
    // Balance_Ctrl_20260618 p1: XH1 carries encoder 1 and O1/O2, which are
    // TB6612 BO1/BO2. Drive B so the actuator and encoder share XH1.
    // Board faults enter the registered permission path; raw SW3/reset and
    // sample expiry retain their direct gates without a long fault-to-pad path.
    wire drive_enabled = motor_permission && !sample_timeout && key_sw[2] && rst_n;
    // Observed digital drive signals, not measured pad voltage/current.
    // The sampler propagates conversion-time tags through its ADC pipeline.
    assign motor_observe = {drive_enabled,BN2,BN1,PWMB};
    motor_pwm u_motor(clk_50m,reset_n,drive_enabled,
                      motor_command,BN1,BN2,PWMB);
    // The unused XH2 connector carries TB6612 A; keep it in high impedance.
    assign AN1=1'b0; assign AN2=1'b0; assign PWMA=1'b1;
    reg telemetry_valid;
    // A stalled acquisition must not suppress its own watchdog fault report.
    // Defaults to 20 ms at 50 MHz; scaled ADC parameters retain test cadence.
    localparam integer HEARTBEAT_CYCLES = ADC_SAMPLE_CYCLES * 16 * 20;
    // Only while a stopped download is active, leave room for whole trace
    // packets between 10 Hz current-status frames. Motor/control clocks do not change.
    localparam integer DUMP_HEARTBEAT_CYCLES = HEARTBEAT_CYCLES * 5;
    localparam integer UART_DIVISOR = (50000000 + UART_BAUD/2) / UART_BAUD;
    localparam integer UART_BIT_CYCLES = UART_DIVISOR < 2 ? 2 : UART_DIVISOR;
    localparam integer TRACE_PACKET_CYCLES = 240 * (UART_BIT_CYCLES * 10 + 1) + 64;
    wire trace_export_active, trace_packet_busy, trace_tx, telemetry_tx, telemetry_busy;
    wire [31:0] heartbeat_period = trace_export_active && stopped ?
        DUMP_HEARTBEAT_CYCLES : HEARTBEAT_CYCLES;
    wire trace_packet_grant = !telemetry_busy && !telemetry_valid &&
        DUMP_HEARTBEAT_CYCLES > TRACE_PACKET_CYCLES &&
        heartbeat_count < DUMP_HEARTBEAT_CYCLES-TRACE_PACKET_CYCLES;
    reg [31:0] heartbeat_count;
    reg [9:0] telemetry_adc;
    reg signed [15:0] telemetry_theta, telemetry_omega;
    reg signed [15:0] telemetry_arm, telemetry_arm_speed;
    always @(posedge clk_50m or negedge reset_n) begin
        if (!reset_n) begin
            telemetry_valid <= 0;
            heartbeat_count <= 0; telemetry_adc <= 0;
            telemetry_theta <= 0; telemetry_omega <= 0;
            telemetry_arm <= 0; telemetry_arm_speed <= 0;
        end
        else begin
            // Pending heartbeat is consumed only when the normal transmitter
            // actually accepts it. A running trace packet cannot discard it.
            if (telemetry_valid && !telemetry_busy && !trace_packet_busy)
                telemetry_valid <= 0;
            // Retain the ADC that belongs to the completed state estimate.
            // Cache all four state values together: if the heartbeat falls
            // on a pipeline completion edge, the UART still gets one coherent
            // old or new sample. A fault keeps the last trusted state visible.
            if (state_valid) begin
                telemetry_adc <= sample_q4[13:4];
                telemetry_theta <= theta; telemetry_omega <= omega;
                telemetry_arm <= arm; telemetry_arm_speed <= arm_speed;
            end else if (!calibrated) begin
                telemetry_adc <= sample_q4[13:4];
            end
            if (heartbeat_count >= heartbeat_period-1) begin
                telemetry_valid <= 1;
                heartbeat_count <= 0;
            end else heartbeat_count <= heartbeat_count+1'b1;
        end
    end
    reg [15:0] millisecond_cycles;
    reg [31:0] device_time_ms, sample_counter;
    reg [7:0] first_fault;
    reg previous_sensor_fault;
    reg [9:0] adc_fault_window_min, adc_fault_window_max;
    reg [13:0] adc_fault_mean_q4;
    reg [271:0] adc_fault_detail;
    reg [31:0] adc_fault_time_ms, adc_fault_sample_counter;
    always @(posedge clk_50m or negedge reset_n) begin
        if (!reset_n) begin
            millisecond_cycles <= 0; device_time_ms <= 0; sample_counter <= 0; first_fault <= 0;
            previous_sensor_fault <= 0;
            adc_fault_window_min <= 0; adc_fault_window_max <= 0; adc_fault_mean_q4 <= 0;
            adc_fault_detail <= 0; adc_fault_time_ms <= 0; adc_fault_sample_counter <= 0;
        end else begin
            previous_sensor_fault <= sensor_fault;
            // Fault publication is slower than acquisition: retain the window
            // at the sensor-fault edge rather than reporting a later good one.
            if (!sensor_fault) begin
                adc_fault_window_min <= 0; adc_fault_window_max <= 0; adc_fault_mean_q4 <= 0;
                adc_fault_detail <= 0; adc_fault_time_ms <= 0; adc_fault_sample_counter <= 0;
            end else if (!previous_sensor_fault) begin
                adc_fault_window_min <= adc_window_min; adc_fault_window_max <= adc_window_max;
                adc_fault_mean_q4 <= sample_q4;
                adc_fault_detail <= adc_window_detail;
                adc_fault_time_ms <= device_time_ms;
                adc_fault_sample_counter <= sample_counter;
            end
            if (millisecond_cycles == 49999) begin
                millisecond_cycles <= 0; device_time_ms <= device_time_ms+1'b1;
            end else millisecond_cycles <= millisecond_cycles+1'b1;
            if (adc_valid) sample_counter <= sample_counter+1'b1;
            // Preserve the initial cause across secondary failures and S.
            if (stopped && (accept_down || accept_up || request_clear)) first_fault <= 0;
            else if (first_fault == 0 && telemetry_fault != 0) first_fault <= telemetry_fault;
        end
    end
    wire [15:0] sensor_flags = {9'd0,spike_rejected,sample_blind,over_range,measurement_ready && !sample_timeout,
                              measurement_valid && !sample_timeout,sample_otr,sample_bad};
    wire trace_capturing, trace_frozen, trace_overwritten, trace_read_request, trace_read_valid;
    wire [15:0] trace_capture_id;
    wire [TRACE_ADDRESS_BITS:0] trace_row_count, trace_read_index;
    wire [255:0] trace_read_data;
    wire [31:0] trace_total_committed, trace_freeze_ms;
    wire [7:0] trace_freeze_reason;
    wire [13:0] trace_down_q4, trace_up_q4;
    wire signed [15:0] trace_saved_capture;
    control_trace_probe #(.ADDRESS_BITS(TRACE_ADDRESS_BITS)) u_trace_probe(
        .clk(clk_50m),.rst_n(reset_n),.adc_valid(adc_valid),.adc_mean_q4(control_q4),
        .encoder_count(position),.sample_counter(sample_counter),.device_time_ms(device_time_ms),
        .adc_quality(adc_quality_reason),.sensor_flags(sensor_flags[7:0]),.fault(telemetry_fault),
        .down_q4(adc_down_q4),.up_q4(adc_up_q4),.h_active(trace_h_active),
        .sample_accept(trace_sample_accept),.command_commit(trace_command_commit),.exit_fault(state==3),
        .control_states(trace_states),.integral_q24(trace_integral_q24),
        .capture_arm(trace_capture_arm),.command(command),
        .gated_motor_command(drive_enabled ? motor_command : 16'sd0),.handover_detail(handover_detail),
        .read_request(trace_read_request),.read_index(trace_read_index),
        .read_valid(trace_read_valid),.read_data(trace_read_data),
        .capturing(trace_capturing),.frozen(trace_frozen),.capture_id(trace_capture_id),
        .row_count(trace_row_count),.total_committed(trace_total_committed),.overwritten(trace_overwritten),
        .freeze_time_ms(trace_freeze_ms),.freeze_reason(trace_freeze_reason),
        .saved_down_q4(trace_down_q4),.saved_up_q4(trace_up_q4),.saved_capture_arm_q10(trace_saved_capture));
    wire trace_permit = stopped && !request_motion && !request_forward &&
                        !request_backward && !request_motor_test;
    control_trace_export #(.ADDRESS_BITS(TRACE_ADDRESS_BITS),.BAUD(UART_BAUD),
                          .SAMPLE_PERIOD_CYCLES(ADC_SAMPLE_CYCLES*16)) u_trace_export(
        .clk(clk_50m),.rst_n(reset_n),.request(rx_valid && rx_data==8'h54),
        .permit(trace_permit),.frozen(trace_frozen),.packet_grant(trace_packet_grant),
        .capture_id(trace_capture_id),.row_count(trace_row_count),
        .total_committed(trace_total_committed),.freeze_time_ms(trace_freeze_ms),
        .overwritten(trace_overwritten),.freeze_reason(trace_freeze_reason),
        .down_q4(trace_down_q4),.up_q4(trace_up_q4),.capture_arm_q10(trace_saved_capture),
        .read_request(trace_read_request),.read_index(trace_read_index),
        .read_valid(trace_read_valid),.read_data(trace_read_data),
        .active(trace_export_active),.packet_busy(trace_packet_busy),.tx(trace_tx));
    assign uart_tx = trace_packet_busy ? trace_tx : telemetry_tx;
    telemetry #(.EXTENDED(TELEMETRY_EXTENDED),.TRACE_STATUS(1),.CONTROL_ADC(1),.HOLD_ENABLE(1),.BAUD(UART_BAUD)) u_telemetry(
        .clk(clk_50m),.rst_n(reset_n),.sample_valid(telemetry_valid),.adc(telemetry_adc),
        .theta(telemetry_theta),.omega(telemetry_omega),.arm(telemetry_arm),
        .arm_speed(telemetry_arm_speed),.command(requested_command),
        .state(test_active ? 3'd5 : jogging ? 3'd4 : state),.fault(telemetry_fault),.calibrated(calibrated),
        .diagnostic_status(diagnostic_status),.tx(telemetry_tx),.busy(telemetry_busy),
        .hold_frame(trace_packet_busy),
        .trace_flags({4'd0,trace_overwritten,trace_export_active,trace_frozen,trace_capturing}),
        .trace_row_count({{(15-TRACE_ADDRESS_BITS){1'b0}},trace_row_count}),.trace_capture_id(trace_capture_id),
        .device_time_ms(device_time_ms),.encoder_count(position),.sensor_flags(sensor_flags),
        .adc_down(adc_down_q4[13:4]),.adc_up(adc_up_q4[13:4]),.adc_raw(adc_raw),
        .adc_window_min(adc_window_min),.adc_window_max(adc_window_max),.adc_mean_q4(sample_q4),
        .adc_control_q4(control_q4),
        .motor_command(drive_enabled ? motor_command : 16'sd0),.first_fault(first_fault),
        .adc_quality_reason(adc_quality_reason),.sensor_fault_reason(sensor_fault_reason),
        .adc_fault_window_min(adc_fault_window_min),.adc_fault_window_max(adc_fault_window_max),
        .adc_fault_mean_q4(adc_fault_mean_q4),
        .adc_window_detail(adc_window_detail),.adc_fault_detail(adc_fault_detail),
        .adc_fault_time_ms(adc_fault_time_ms),.adc_fault_sample_counter(adc_fault_sample_counter),
        .motor_test_status(motor_test_status),.motor_test_delta(motor_test_delta),
        .handover_detail(handover_detail),
        .sample_counter(sample_counter));
endmodule
`default_nettype wire
