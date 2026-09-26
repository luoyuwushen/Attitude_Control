`timescale 1ns/1ps
module tb_interfaces;
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg arst_n = 1'b0;
    wire rst_n;
    reset_sync reset_dut (.clk(clk), .arst_n(arst_n), .rst_n(rst_n));

    reg key_n = 1'b1;
    wire key_level, key_press;
    key_debounce #(.STABLE_CYCLES(4)) key_dut (
        .clk(clk), .rst_n(rst_n), .key_n(key_n), .level_n(key_level), .press(key_press));
    reg enc_a = 1'b1, enc_b = 1'b1, enc_zero = 1'b0;
    wire signed [31:0] enc_position;
    wire enc_illegal;
    quadrature_encoder #(.FILTER_CYCLES(3)) encoder_dut (
        .clk(clk), .rst_n(rst_n), .a(enc_a), .b(enc_b), .zero(enc_zero),
        .position(enc_position), .illegal(enc_illegal));

    reg motor_enable = 1'b0;
    reg signed [15:0] motor_command = 16'sd0;
    wire motor_in1, motor_in2, motor_pwm_out;
    motor_pwm #(.PERIOD_CYCLES(20), .DEAD_CYCLES(5)) motor_dut (
        .clk(clk), .rst_n(rst_n), .enable(motor_enable), .command(motor_command),
        .in1(motor_in1), .in2(motor_in2), .pwm(motor_pwm_out));

    reg tx_valid = 1'b0;
    reg [7:0] tx_data = 8'd0;
    wire tx_ready, tx_line, loop_valid, loop_error;
    wire [7:0] loop_data;
    uart_tx_byte #(.CLK_HZ(1000000), .BAUD(100000)) tx_dut (
        .clk(clk), .rst_n(rst_n), .valid(tx_valid), .data(tx_data),
        .ready(tx_ready), .tx(tx_line));
    uart_rx_byte #(.CLK_HZ(1000000), .BAUD(100000)) loop_rx_dut (
        .clk(clk), .rst_n(rst_n), .rx(tx_line), .data(loop_data),
        .valid(loop_valid), .framing_error(loop_error));
    reg manual_rx = 1'b1;
    wire [7:0] manual_data;
    wire manual_valid, manual_error;
    uart_rx_byte #(.CLK_HZ(1000000), .BAUD(100000)) manual_rx_dut (
        .clk(clk), .rst_n(rst_n), .rx(manual_rx), .data(manual_data),
        .valid(manual_valid), .framing_error(manual_error));

    reg sample_valid = 1'b0;
    reg [9:0] adc = 10'h2A5;
    reg signed [15:0] theta = -16'sd1234, omega = 16'sd567;
    reg signed [15:0] arm = -16'sd77, arm_speed = 16'sd88, command = -16'sd500;
    reg [2:0] state = 3'd3;
    reg [7:0] fault = 8'h02;
    reg calibrated = 1'b1;
    wire telemetry_tx, telemetry_valid, telemetry_error;
    wire [7:0] telemetry_data;
    telemetry #(.CLK_HZ(1000000), .BAUD(100000)) telemetry_dut (
        .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid), .adc(adc),
        .theta(theta), .omega(omega), .arm(arm), .arm_speed(arm_speed),
        .command(command), .state(state), .fault(fault), .calibrated(calibrated),
        .tx(telemetry_tx));
    uart_rx_byte #(.CLK_HZ(1000000), .BAUD(100000)) telemetry_rx_dut (
        .clk(clk), .rst_n(rst_n), .rx(telemetry_tx), .data(telemetry_data),
        .valid(telemetry_valid), .framing_error(telemetry_error));

    integer key_events = 0, illegal_events = 0;
    integer loop_count = 0, manual_count = 0, manual_errors = 0, telemetry_count = 0;
    reg [7:0] loop_bytes [0:7];
    reg [7:0] telemetry_bytes [0:63];
    reg key_press_previous = 0, illegal_previous = 0;
    always @(posedge clk) begin
        #1;
        if (rst_n) begin
            if (key_press) key_events = key_events + 1;
            if (enc_illegal) illegal_events = illegal_events + 1;
            if (key_press && key_press_previous) $fatal(1, "key press wider than one cycle");
            if (enc_illegal && illegal_previous) $fatal(1, "illegal wider than one cycle");
            if (motor_in1 && motor_in2) $fatal(1, "motor direction unsafe");
            if (loop_error || telemetry_error) $fatal(1, "unexpected UART framing error");
            if (loop_valid) begin
                if (loop_count >= 8) $fatal(1, "too many loopback bytes");
                loop_bytes[loop_count] = loop_data;
                loop_count = loop_count + 1;
            end
            if (manual_valid) begin
                if (manual_data !== 8'h96) $fatal(1, "manual RX mismatch");
                manual_count = manual_count + 1;
            end
            if (manual_error) manual_errors = manual_errors + 1;
            if (telemetry_valid) begin
                if (telemetry_count >= 64) $fatal(1, "too many telemetry bytes");
                telemetry_bytes[telemetry_count] = telemetry_data;
                telemetry_count = telemetry_count + 1;
            end
        end
        key_press_previous = key_press;
        illegal_previous = enc_illegal;
    end

    task tick(input integer cycles);
        begin repeat (cycles) begin @(posedge clk); #2; end end
    endtask
    task enc_state(input [1:0] ab);
        begin @(negedge clk); {enc_a, enc_b} = ab; tick(9); end
    endtask
    task send_byte(input [7:0] value);
        begin
            @(negedge clk);
            while (!tx_ready) @(negedge clk);
            tx_data = value;
            tx_valid = 1;
            @(negedge clk); tx_valid = 0;
        end
    endtask
    task rx_bit(input bit value);
        begin manual_rx = value; repeat (10) @(negedge clk); end
    endtask
    task rx_byte(input [7:0] value, input bit good_stop);
        integer k;
        begin
            @(negedge clk); rx_bit(0);
            for (k = 0; k < 8; k = k + 1) rx_bit(value[k]);
            rx_bit(good_stop);
            rx_bit(1);
        end
    endtask
    task count_pwm(input integer expected_high);
        integer k, high_count;
        begin
            high_count = 0;
            for (k = 0; k < 20; k = k + 1) begin
                tick(1);
                if (motor_pwm_out) high_count = high_count + 1;
            end
            if (high_count != expected_high)
                $fatal(1, "PWM expected %0d high clocks, got %0d", expected_high, high_count);
        end
    endtask
    // 独立按位 CRC 校验，不读取被测模块内部寄存器。
    function automatic [15:0] packet_crc(input integer offset);
        integer k, b;
        reg [15:0] value;
        reg feedback;
        begin
            value = 16'hFFFF;
            for (k = 0; k < 22; k = k + 1)
                for (b = 7; b >= 0; b = b - 1) begin
                    feedback = value[15] ^ telemetry_bytes[offset+k][b];
                    value = value << 1;
                    if (feedback) value = value ^ 16'h1021;
                end
            packet_crc = value;
        end
    endfunction
    task check_packet(input integer offset, input [15:0] seq, input [15:0] expected_theta);
        reg [15:0] crc_check;
        begin
            if ({telemetry_bytes[offset],telemetry_bytes[offset+1],
                 telemetry_bytes[offset+2],telemetry_bytes[offset+3]} !== 32'hAA550118)
                $fatal(1, "telemetry header");
            if ({telemetry_bytes[offset+5],telemetry_bytes[offset+4]} !== seq)
                $fatal(1, "telemetry sequence");
            if ({telemetry_bytes[offset+7],telemetry_bytes[offset+6]} !== 16'h02A5)
                $fatal(1, "telemetry ADC");
            if ({telemetry_bytes[offset+9],telemetry_bytes[offset+8]} !== expected_theta)
                $fatal(1, "telemetry snapshot theta");
            if ({telemetry_bytes[offset+11],telemetry_bytes[offset+10]} !== 16'd567 ||
                {telemetry_bytes[offset+13],telemetry_bytes[offset+12]} !== -16'sd77 ||
                {telemetry_bytes[offset+15],telemetry_bytes[offset+14]} !== 16'd88 ||
                {telemetry_bytes[offset+17],telemetry_bytes[offset+16]} !== -16'sd500)
                $fatal(1, "telemetry signed fields");
            if (telemetry_bytes[offset+18] !== 8'd3 || telemetry_bytes[offset+19] !== 8'd1 ||
                telemetry_bytes[offset+20] !== 8'h02 || telemetry_bytes[offset+21] !== 8'd0)
                $fatal(1, "telemetry status fields");
            crc_check = packet_crc(offset);
            if ({telemetry_bytes[offset+23],telemetry_bytes[offset+22]} !== crc_check)
                $fatal(1, "telemetry CRC");
        end
    endtask

    integer stop_clocks, deadline, i;
    initial begin
        // 总超时独立于各任务，任何挂死都以失败结束。
        #2000000; $fatal(1, "tb_interfaces timeout");
    end
    initial begin
        tick(2);
        if (rst_n !== 0 || motor_in1 !== 0 || motor_in2 !== 0 || motor_pwm_out !== 1)
            $fatal(1, "reset outputs");
        @(negedge clk); arst_n = 1;
        tick(1); if (rst_n !== 0) $fatal(1, "reset release too early");
        tick(1); if (rst_n !== 1) $fatal(1, "reset release missing");
        tick(20);
        if (enc_position !== 0 || illegal_events != 0) $fatal(1, "encoder startup phantom");

        // 短脉冲、稳定按下、长按、带抖动松开、第二次按下。
        @(negedge clk); key_n = 0; tick(2);
        @(negedge clk); key_n = 1; tick(10);
        if (key_events != 0 || key_level !== 1) $fatal(1, "key glitch accepted");
        @(negedge clk); key_n = 0; tick(12);
        if (key_events != 1 || key_level !== 0) $fatal(1, "key press missing");
        tick(20); if (key_events != 1) $fatal(1, "key held repeated");
        @(negedge clk); key_n = 1; tick(2);
        @(negedge clk); key_n = 0; tick(8);
        if (key_level !== 0 || key_events != 1) $fatal(1, "key release glitch accepted");
        @(negedge clk); key_n = 1; tick(12);
        @(negedge clk); key_n = 0; tick(12);
        if (key_events != 2) $fatal(1, "second key press missing");

        enc_state(2'b01); enc_state(2'b00); enc_state(2'b10); enc_state(2'b11);
        if (enc_position !== 4) $fatal(1, "encoder forward");
        enc_state(2'b10); enc_state(2'b00); enc_state(2'b01); enc_state(2'b11);
        if (enc_position !== 0) $fatal(1, "encoder reverse");
        @(negedge clk); {enc_a, enc_b} = 2'b00; tick(1);
        @(negedge clk); {enc_a, enc_b} = 2'b11; tick(10);
        if (enc_position !== 0 || illegal_events != 0) $fatal(1, "encoder glitch accepted");
        enc_state(2'b00);
        if (enc_position !== 0 || illegal_events != 1) $fatal(1, "encoder illegal transition");
        enc_state(2'b10);
        if (enc_position !== 1) $fatal(1, "encoder recovery");
        @(negedge clk); enc_zero = 1; tick(1);
        @(negedge clk); enc_zero = 0; tick(2);
        if (enc_position !== 0) $fatal(1, "encoder zero");

        @(negedge clk); motor_enable = 1; motor_command = 500; tick(45);
        if ({motor_in1,motor_in2} !== 2'b10) $fatal(1, "motor forward direction");
        count_pwm(10);
        // 正向PWM周期中途置零，下一系统拍必须高阻，不能等周期边界。
        @(negedge motor_pwm_out); @(negedge clk); motor_command = 0;
        tick(1);
        if ({motor_in1,motor_in2,motor_pwm_out} !== 3'b001)
            $fatal(1, "active forward zero command did not stop next clock");
        @(negedge clk); motor_command = 500; tick(40);
        if ({motor_in1,motor_in2} !== 2'b10) $fatal(1, "forward restart after zero");
        count_pwm(10);
        @(negedge clk); motor_command = 1000; tick(40); count_pwm(20);
        @(negedge clk); motor_command = 32767; tick(40); count_pwm(20);
        @(negedge clk); motor_command = 500; tick(40);
        // 在 PWM 下降沿后更改幅值，本周期低电平不能被中途抬高。
        @(negedge motor_pwm_out); @(negedge clk); motor_command = 250;
        tick(3); if (motor_pwm_out !== 0) $fatal(1, "PWM changed within period");
        tick(40); count_pwm(5);
        @(negedge clk); motor_command = -500;
        stop_clocks = 0; deadline = 0;
        while (!motor_in2 && deadline < 100) begin
            tick(1); deadline = deadline + 1;
            if (!motor_in1 && !motor_in2) begin
                stop_clocks = stop_clocks + 1;
                if (motor_pwm_out !== 1) $fatal(1, "motor blank not stop mode");
            end
        end
        if (!motor_in2 || stop_clocks < 5) $fatal(1, "motor reversal dead time");
        tick(40); count_pwm(10);
        // 反向同样检查零指令的一拍响应。
        @(negedge motor_pwm_out); @(negedge clk); motor_command = 0;
        tick(1);
        if ({motor_in1,motor_in2,motor_pwm_out} !== 3'b001)
            $fatal(1, "active reverse zero command did not stop next clock");
        @(negedge clk); motor_command = -500; tick(40);
        if ({motor_in1,motor_in2} !== 2'b01) $fatal(1, "reverse restart after zero");
        count_pwm(10);
        @(negedge clk); motor_command = -32768; tick(40); count_pwm(20);
        @(negedge clk); motor_enable = 0; #1;
        if ({motor_in1,motor_in2,motor_pwm_out} !== 3'b001) $fatal(1, "motor disable delayed");
        tick(20);
        @(negedge clk); motor_enable = 1; motor_command = 0; #1;
        if ({motor_in1,motor_in2,motor_pwm_out} !== 3'b001) $fatal(1, "zero command not stop mode");

        send_byte(8'h00); send_byte(8'hFF); send_byte(8'hA5); send_byte(8'h5A);
        tick(120);
        if (loop_count != 4 || loop_bytes[0] !== 8'h00 || loop_bytes[1] !== 8'hFF ||
            loop_bytes[2] !== 8'hA5 || loop_bytes[3] !== 8'h5A)
            $fatal(1, "UART loopback/handshake");
        @(negedge clk); manual_rx = 0; repeat(2) @(negedge clk); manual_rx = 1;
        tick(30);
        if (manual_count != 0 || manual_errors != 0) $fatal(1, "false UART start accepted");
        rx_byte(8'h96, 1); tick(20);
        if (manual_count != 1 || manual_errors != 0) $fatal(1, "manual UART RX");
        rx_byte(8'h96, 0); tick(20);
        if (manual_count != 1 || manual_errors != 1) $fatal(1, "UART framing error handling");

        @(negedge clk); sample_valid = 1;
        @(negedge clk); sample_valid = 0;
        tick(50);
        // 忙时输入变化及第二个采样脉冲不能修改正在发送的快照。
        @(negedge clk); theta = 16'sd4321; sample_valid = 1;
        @(negedge clk); sample_valid = 0;
        tick(2500);
        if (telemetry_count != 24) $fatal(1, "telemetry busy sample not dropped");
        check_packet(0, 16'd0, -16'sd1234);
        @(negedge clk); sample_valid = 1;
        @(negedge clk); sample_valid = 0;
        tick(2500);
        if (telemetry_count != 48) $fatal(1, "telemetry second packet");
        check_packet(24, 16'd1, 16'd4321);

        // 非时钟边沿拉低复位须立即清除外部控制输出。
        @(negedge clk); #2; arst_n = 0; #1;
        if (rst_n !== 0 || {motor_in1,motor_in2,motor_pwm_out} !== 3'b001 || tx_line !== 1)
            $fatal(1, "asynchronous reset assertion");
        $display("PASS tb_interfaces: reset/key/encoder/motor/UART/telemetry");
        $finish;
    end
endmodule
