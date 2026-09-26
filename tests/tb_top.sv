`timescale 1ns/1ps
module tb_top;
    reg clk = 0;
    always #10 clk = ~clk;
    reg rst_n = 0;
    reg [3:0] key_sw = 4'b1111;
    reg uart_rx = 1;
    wire uart_tx;
    reg [9:0] adc_data = 624;
    reg adc_otr = 0;
    wire adc_clk, adc_oe_n;
    reg enc_a = 0, enc_b = 0;
    wire an1, an2, pwma, bn1, bn2, pwmb;
    localparam integer JOG_TEST_CYCLES = 20000;
    top #(.KEY_CYCLES(4), .ADC_SAMPLE_CYCLES(10), .JOG_CYCLES(JOG_TEST_CYCLES)) dut (
        .clk_50m(clk), .rst_n(rst_n), .key_sw(key_sw), .uart_rx(uart_rx), .uart_tx(uart_tx),
        .adc_data_in(adc_data), .adc_otr(adc_otr), .adc_clk(adc_clk), .adc_oe_n(adc_oe_n),
        .enc1_a(enc_a), .enc1_b(enc_b), .AN1(an1), .AN2(an2), .PWMA(pwma),
        .BN1(bn1), .BN2(bn2), .PWMB(pwmb));
    integer jog_observed_cycles = 0, completed_jog_cycles = 0;
    reg jogging_previous = 0;
    always @(posedge clk) begin
        #1;
        if ({bn1,bn2,pwmb} !== 3'b001) $fatal(1, "unused motor B not high impedance");
        if (an1 && an2) $fatal(1, "motor A invalid direction");
        if (rst_n) begin
            if (dut.jogging) begin
                jog_observed_cycles = jogging_previous ? jog_observed_cycles+1 : 1;
                if (jog_observed_cycles > JOG_TEST_CYCLES) $fatal(1, "jog exceeded bounded duration");
            end else if (jogging_previous) completed_jog_cycles = jog_observed_cycles;
        end
        jogging_previous = dut.jogging;
    end
    task tick(input integer cycles);
        begin repeat(cycles) begin @(posedge clk); #2; end end
    endtask
    task uart_bit(input bit value);
        begin uart_rx = value; repeat(434) @(negedge clk); end
    endtask
    task uart_command(input [7:0] value);
        integer k;
        begin
            @(negedge clk); uart_bit(0);
            for(k=0;k<8;k=k+1) uart_bit(value[k]);
            uart_bit(1);
            tick(20);
        end
    endtask
    task press_key(input integer index);
        begin
            @(negedge clk); key_sw[index] = 0; tick(10);
            @(negedge clk); key_sw[index] = 1; tick(10);
        end
    endtask
    task expect_stop;
        begin
            if ({an1,an2,pwma} !== 3'b001 || dut.state !== 0 || dut.motor_enable !== 0)
                $fatal(1, "top stop outputs/state");
        end
    endtask
    task count_jog_pwm;
        integer high_cycles, k;
        begin
            high_cycles = 0;
            for(k=0;k<2500;k=k+1) begin
                tick(1);
                if (pwma) high_cycles = high_cycles+1;
            end
            if (high_cycles != 250) $fatal(1, "100permille jog PWM expected250 got%0d",high_cycles);
        end
    endtask
    task wait_jog_end;
        integer timeout;
        begin
            timeout = 0;
            while (dut.jogging && timeout <= JOG_TEST_CYCLES+10) begin
                tick(1); timeout = timeout+1;
            end
            if (dut.jogging) $fatal(1, "jog failed to stop on timer");
            tick(3); expect_stop;
        end
    endtask
    initial begin #5000000; $fatal(1, "tb_top timeout"); end
    initial begin
        tick(3);
        if ({an1,an2,pwma} !== 3'b001) $fatal(1, "top reset motor output");
        @(negedge clk); rst_n = 1; tick(1000);
        if (adc_oe_n !== 0 || dut.u_estimator.calibrated !== 0) $fatal(1, "top startup");
        uart_command("G");
        if (dut.state !== 0 || dut.motor_enable) $fatal(1, "top uncalibrated UART G accepted");
        uart_command("F"); uart_command("B");
        if (dut.jogging || dut.requested_command !== 0) $fatal(1, "uncalibrated jog accepted");

        uart_command("D");
        if (dut.u_estimator.down_code !== 624*16 || !dut.u_estimator.have_down)
            $fatal(1, "UART D/down calibration mapping");
        @(negedge clk); adc_data = 800; tick(350);
        uart_command("U"); tick(80);
        if (!dut.calibrated || dut.u_estimator.up_code !== 800*16)
            $fatal(1, "UART U/up calibration mapping");
        @(negedge clk); adc_data = 624; tick(350);
        // 已经超量程的停止态不能接受G后短暂驱动。
        @(negedge clk); adc_otr = 1; tick(30);
        uart_command("G");
        if (dut.state !== 0 || dut.motor_enable || {an1,an2,pwma} !== 3'b001)
            $fatal(1, "OTR active before start was not rejected");
        @(negedge clk); adc_otr = 0; tick(30);

        // 已校准停止态F/B为限时100permille点动，G在点动期间不接受。
        uart_command("F"); tick(3000);
        if (!dut.jogging || dut.requested_command !== 100 || {an1,an2} !== 2'b10 || dut.state !== 0)
            $fatal(1, "forward jog mapping/direction");
        count_jog_pwm;
        uart_command("G");
        if (!dut.jogging || dut.state !== 0) $fatal(1, "G accepted while jogging");
        wait_jog_end;
        if (completed_jog_cycles != JOG_TEST_CYCLES || dut.requested_command !== 0)
            $fatal(1, "jog duration or command cleanup");
        uart_command("B"); tick(5500);
        if (!dut.jogging || dut.requested_command !== -100 || {an1,an2} !== 2'b01)
            $fatal(1, "backward jog mapping/direction");
        count_jog_pwm;
        uart_command("S"); tick(3); expect_stop;
        if (dut.jogging) $fatal(1, "S did not cancel jog");
        // 校准按键中断点动，但不会在运动过程中记录新标定点。
        uart_command("F"); tick(5500);
        press_key(0); tick(3); expect_stop;
        if (dut.jogging) $fatal(1, "calibration request did not cancel jog");
        uart_command("B"); tick(5500);
        @(negedge clk); key_sw[2] = 0; #1;
        if ({an1,an2,pwma} !== 3'b001) $fatal(1, "raw SW3 did not immediately stop jog");
        tick(3); @(negedge clk); key_sw[2] = 1; tick(20); expect_stop;
        if (dut.jogging) $fatal(1, "raw SW3 release restarted jog");

        // OTR采样后最多两个系统拍关闭点动，故障锁存阻止后续F。
        uart_command("F"); tick(5500);
        @(negedge clk); adc_otr = 1;
        @(posedge dut.over_range); tick(2);
        if (dut.jogging || {an1,an2,pwma} !== 3'b001)
            $fatal(1, "OTR did not cancel jog within two sampled clocks");
        @(negedge clk); adc_otr = 0; tick(30);
        uart_command("F");
        if (dut.jogging) $fatal(1, "latched jog fault bypassed by F");
        uart_command("D");
        @(negedge clk); adc_data = 800; tick(350);
        uart_command("U"); tick(80);
        @(negedge clk); adc_data = 624; tick(350);

        uart_command("G"); tick(5500);
        if (dut.state !== 1 || !dut.motor_enable || !(an1 || an2))
            $fatal(1, "UART G/start motor mapping state=%0d calibrated=%0d sensor=%0d input=%0d enable=%0d direction=%b%b",
                dut.state,dut.calibrated,dut.sensor_fault,dut.input_fault,dut.motor_enable,an1,an2);
        uart_command("F"); tick(3000);
        if (dut.jogging || dut.state !== 1) $fatal(1, "jog accepted while controller running");

        // SW3短脉冲长于同步深度但短于消抖门槛：立即停输出，释放后仍须重新启动。
        @(negedge clk); key_sw[2] = 0; #1;
        if ({an1,an2,pwma} !== 3'b001) $fatal(1, "raw SW3 did not immediately stop motor");
        tick(3);
        @(negedge clk); key_sw[2] = 1;
        tick(20); expect_stop;
        tick(5000); expect_stop;

        uart_command("G"); tick(4500);
        if (dut.state !== 1 || !dut.motor_enable) $fatal(1, "new G failed after raw stop");
        uart_command("S"); tick(20); expect_stop;

        // 按键 SW1=下点，SW4=上点，SW2=启动，SW3=停止。
        press_key(0);
        if (dut.u_estimator.down_code !== 624*16 || dut.calibrated)
            $fatal(1, "SW1 down calibration mapping");
        @(negedge clk); adc_data = 800; tick(350);
        press_key(3); tick(80);
        if (!dut.calibrated || dut.u_estimator.up_code !== 800*16)
            $fatal(1, "SW4 up calibration mapping");
        @(negedge clk); adc_data = 624; tick(350);
        press_key(1); tick(4500);
        if (dut.state !== 1 || !dut.motor_enable) $fatal(1, "SW2 start mapping");
        @(negedge clk); adc_otr = 1; tick(30);
        if (dut.state !== 3 || dut.fault !== 8'h01 || {an1,an2,pwma} !== 3'b001)
            $fatal(1, "OTR fault did not stop motor");
        @(negedge clk); adc_otr = 0; tick(30);
        uart_command("G");
        if (dut.state !== 3 || dut.motor_enable) $fatal(1, "OTR fault bypassed by G");
        press_key(2); expect_stop;
        $display("PASS tb_top: UART/key mapping/jog limit/PWM/raw stop/OTR/motor B isolation");
        $finish;
    end
endmodule
