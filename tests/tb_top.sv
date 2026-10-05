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
    wire motor_in1, motor_in2, motor_pwm, unused_in1, unused_in2, unused_pwm;
    localparam integer JOG_TEST_CYCLES = 20000;
    top #(.TELEMETRY_EXTENDED(0),.KEY_CYCLES(4), .ADC_SAMPLE_CYCLES(10), .JOG_CYCLES(JOG_TEST_CYCLES)) dut (
        .clk_50m(clk), .rst_n(rst_n), .key_sw(key_sw), .uart_rx(uart_rx), .uart_tx(uart_tx),
        .adc_data_in(adc_data), .adc_otr(adc_otr), .adc_clk(adc_clk), .adc_oe_n(adc_oe_n),
        .enc1_a(enc_a), .enc1_b(enc_b), .AN1(unused_in1), .AN2(unused_in2), .PWMA(unused_pwm),
        .BN1(motor_in1), .BN2(motor_in2), .PWMB(motor_pwm));
    integer jog_observed_cycles = 0, completed_jog_cycles = 0;
    reg jogging_previous = 0;
    // Raw OTR alone is an observation. Bound the pad gate from a qualified
    // acquisition fault, after its sustained-conversion confirmation.
    always @(posedge dut.sample_fault or posedge dut.sensor_fault) begin
        if (rst_n) begin
            @(posedge clk); #1;
            if (dut.motor_permission !== 0 ||
                {motor_in1,motor_in2,motor_pwm} !== 3'b001)
                $fatal(1, "qualified sensor/OTR fault did not disable drive by the next 50 MHz clock");
        end
    end
    always @(posedge clk) begin
        #1;
        if ({unused_in1,unused_in2,unused_pwm} !== 3'b001) $fatal(1, "unused XH2 / motor A not high impedance");
        if (motor_in1 && motor_in2) $fatal(1, "XH1 / motor B invalid direction");
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
    // Keep manual movement within each ADC window and wait for trustworthy
    // velocity history; instantaneous jumps are reserved for fault injection.
    task move_adc(input integer target);
        integer remaining;
        begin
            while (adc_data != target) begin
                @(negedge clk);
                remaining=target-$signed({1'b0,adc_data});
                if (remaining>16) adc_data=adc_data+16;
                else if (remaining < -16) adc_data=adc_data-16;
                else adc_data=target;
                tick(160);
            end
            tick(5120);
        end
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
    task quadrature(input bit positive);
        begin
            @(negedge clk); {enc_a,enc_b}=positive?2'b10:2'b01; tick(20);
            @(negedge clk); {enc_a,enc_b}=2'b11; tick(20);
            @(negedge clk); {enc_a,enc_b}=positive?2'b01:2'b10; tick(20);
            @(negedge clk); {enc_a,enc_b}=2'b00; tick(20);
        end
    endtask
    task expect_limit_rejection(input [7:0] command_byte);
        begin
            uart_command(command_byte); tick(320);
            expect_stop;
            if(dut.start_result!==7 || dut.fault!==0 || dut.command!==0)
                $fatal(1,"outside-position request must reject in IDLE without transient running fault");
        end
    endtask
    task expect_stop;
        begin
            if ({motor_in1,motor_in2,motor_pwm} !== 3'b001 || dut.state !== 0 || dut.motor_enable !== 0)
                $fatal(1, "top stop outputs/state");
        end
    endtask
    task count_jog_pwm;
        integer high_cycles, k;
        begin
            high_cycles = 0;
            for(k=0;k<2500;k=k+1) begin
                tick(1);
                if (motor_pwm) high_cycles = high_cycles+1;
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
    initial begin #15000000; $fatal(1, "tb_top timeout"); end
    initial begin
        tick(3);
        if ({motor_in1,motor_in2,motor_pwm} !== 3'b001) $fatal(1, "top reset motor output");
        @(negedge clk); rst_n = 1; tick(1000);
        if (adc_oe_n !== 0 || dut.u_estimator.calibrated !== 0) $fatal(1, "top startup");
        uart_command("G");
        if (dut.state !== 0 || dut.motor_enable) $fatal(1, "top uncalibrated UART G accepted");
        uart_command("F"); uart_command("B");
        if (dut.jogging || dut.requested_command !== 0) $fatal(1, "uncalibrated jog accepted");

        uart_command("D");
        if (dut.u_estimator.down_code !== 624*16 || !dut.u_estimator.have_down)
            $fatal(1, "UART D/down calibration mapping");
        move_adc(800);
        uart_command("U"); tick(80);
        if (!dut.calibrated || dut.u_estimator.up_code !== 800*16)
            $fatal(1, "UART U/up calibration mapping");
        // UART F arrives shortly after physical SW4 refreshes U. This creates
        // the real post-calibration maturity gap without forcing estimator flags.
        fork
            uart_command("F");
            begin tick(3500); press_key(3); end
        join
        if(dut.measurement_ready || dut.measurement_valid!==1 || dut.jogging || dut.requested_command!==0)
            $fatal(1,"F during the sixteen-sample maturity gap was not rejected");
        move_adc(624);
        // 已经超量程的停止态不能接受G后短暂驱动。
        @(negedge clk); adc_otr = 1; tick(30);
        uart_command("G");
        if (dut.state !== 0 || dut.motor_enable || {motor_in1,motor_in2,motor_pwm} !== 3'b001)
            $fatal(1, "OTR active before start was not rejected");
        @(negedge clk); adc_otr = 0; tick(350);
        if (!dut.sensor_fault) $fatal(1,"OTR window did not preserve idle sensor fault");
        uart_command("D"); move_adc(800); uart_command("U"); move_adc(624);
        if (!dut.measurement_ready || dut.sensor_fault) $fatal(1,"OTR recovery calibration not ready");

        // 已校准停止态F/B为限时100permille点动，G在点动期间不接受。
        uart_command("F"); tick(3000);
        if (!dut.jogging || dut.requested_command !== 100 || dut.motor_command !== -100 ||
            {motor_in1,motor_in2} !== 2'b01 || dut.state !== 0)
            $fatal(1, "forward jog mapping/direction");
        // Model only the measured installed sign: negative hardware command
        // yields positive raw quadrature. It is not a physical stability test.
        quadrature(1); tick(320);
        if(dut.position!==4 || dut.arm<=0) $fatal(1,"positive request/raw-positive coordinate contract");
        count_jog_pwm;
        uart_command("G");
        if (!dut.jogging || dut.state !== 0) $fatal(1, "G accepted while jogging");
        wait_jog_end;
        if (completed_jog_cycles != JOG_TEST_CYCLES || dut.requested_command !== 0)
            $fatal(1, "jog duration or command cleanup");
        uart_command("B"); tick(5500);
        if (!dut.jogging || dut.requested_command !== -100 || dut.motor_command !== 100 ||
            {motor_in1,motor_in2} !== 2'b10)
            $fatal(1, "backward jog mapping/direction");
        quadrature(0); tick(320);
        if(dut.position!==0 || dut.arm!==0) $fatal(1,"negative request/raw-negative coordinate contract");
        count_jog_pwm;
        uart_command("S"); tick(3); expect_stop;
        if (dut.jogging) $fatal(1, "S did not cancel jog");
        // 校准按键中断点动，但不会在运动过程中记录新标定点。
        uart_command("F"); tick(5500);
        press_key(0); tick(3); expect_stop;
        if (dut.jogging) $fatal(1, "calibration request did not cancel jog");
        uart_command("B"); tick(5500);
        @(negedge clk); key_sw[2] = 0; #1;
        if ({motor_in1,motor_in2,motor_pwm} !== 3'b001) $fatal(1, "raw SW3 did not immediately stop jog");
        tick(3); @(negedge clk); key_sw[2] = 1; tick(20); expect_stop;
        if (dut.jogging) $fatal(1, "raw SW3 release restarted jog");

        // 持续非轨端OTR通过资格判定后下一系统拍关闭点动，锁存阻止后续F。
        uart_command("F"); tick(5500);
        @(negedge clk); adc_otr = 1;
        @(posedge dut.over_range); tick(1);
        if (!dut.jogging || dut.sample_fault)
            $fatal(1, "unqualified raw OTR prematurely ended jog");
        @(posedge dut.sample_fault); tick(1);
        if (dut.jogging || dut.motor_permission || {motor_in1,motor_in2,motor_pwm} !== 3'b001)
            $fatal(1, "qualified OTR did not cancel jog by the next system clock");
        @(negedge clk); adc_otr = 0; tick(30);
        uart_command("F");
        if (dut.jogging) $fatal(1, "latched jog fault bypassed by F");
        uart_command("D");
        move_adc(800);
        uart_command("U"); tick(80);
        move_adc(624);

        uart_command("G"); tick(5500);
        if (dut.state !== 1 || !dut.motor_enable || !(motor_in1 || motor_in2))
            $fatal(1, "UART G/start motor mapping state=%0d calibrated=%0d sensor=%0d input=%0d enable=%0d direction=%b%b",
                dut.state,dut.calibrated,dut.sensor_fault,dut.input_fault,dut.motor_enable,motor_in1,motor_in2);
        uart_command("F"); tick(3000);
        if (dut.jogging || dut.state !== 1) $fatal(1, "jog accepted while controller running");

        // SW3短脉冲长于同步深度但短于消抖门槛：立即停输出，释放后仍须重新启动。
        @(negedge clk); key_sw[2] = 0; #1;
        if ({motor_in1,motor_in2,motor_pwm} !== 3'b001) $fatal(1, "raw SW3 did not immediately stop motor");
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
        move_adc(800);
        press_key(3); tick(80);
        if (!dut.calibrated || dut.u_estimator.up_code !== 800*16)
            $fatal(1, "SW4 up calibration mapping");
        move_adc(624);
        press_key(1); tick(4500);
        if (dut.state !== 1 || !dut.motor_enable) $fatal(1, "SW2 start mapping");
        @(negedge clk); adc_otr = 1;
        @(posedge dut.sample_fault); tick(10);
        if (dut.state !== 3 || dut.fault !== 8'h01 || {motor_in1,motor_in2,motor_pwm} !== 3'b001)
            $fatal(1, "OTR fault did not stop motor");
        @(negedge clk); adc_otr = 0; tick(30);
        uart_command("G");
        if (dut.state !== 3 || dut.motor_enable) $fatal(1, "OTR fault bypassed by G");
        press_key(2); expect_stop;

        // Real encoder transitions reproduce accumulated travel beyond the U
        // origin. Neither R nor S nor rejected commands may reset that origin.
        @(negedge clk); rst_n=0; enc_a=0; enc_b=0; adc_data=624; tick(5);
        @(negedge clk); rst_n=1; tick(1000);
        uart_command("D"); move_adc(800); uart_command("U"); tick(5120);
        repeat(250) quadrature(1); tick(5120);
        if(dut.position!==1000 || dut.arm<=6144 || !dut.measurement_ready)
            $fatal(1,"positive outside-position setup");
        expect_limit_rejection("G"); expect_limit_rejection("H");
        uart_command("R"); tick(320); expect_stop;
        if(dut.position!==1000 || dut.u_estimator.origin!==0 || dut.arm<=6144)
            $fatal(1,"R silently recentered physical travel");
        expect_limit_rejection("G");
        // Manual commands have independent bounded travel/time and remain
        // usable in both directions outside the controller's absolute origin.
        uart_command("F"); tick(3000);
        if(!dut.jogging || dut.motor_command!==-100) $fatal(1,"outside-position F blocked");
        quadrature(1); uart_command("S"); tick(320); expect_stop;
        if(dut.position!==1004 || dut.u_estimator.origin!==0) $fatal(1,"F reset accumulated travel");
        uart_command("B"); tick(3000);
        if(!dut.jogging || dut.motor_command!==100) $fatal(1,"outside-position B blocked");
        quadrature(0); uart_command("S"); tick(320); expect_stop;
        uart_command("J"); tick(3000);
        if(!dut.test_active || dut.motor_command!==-150) $fatal(1,"outside-position J blocked");
        quadrature(1); uart_command("S"); tick(320); expect_stop;
        if(dut.motor_test_status!==4 || dut.motor_test_delta!==4)
            $fatal(1,"outside-position J stop/relative displacement");
        uart_command("K"); tick(3000);
        if(!dut.test_active || dut.motor_command!==150) $fatal(1,"outside-position K blocked");
        quadrature(0); uart_command("S"); tick(320); expect_stop;
        if(dut.position!==1000 || dut.motor_test_delta!==-4 || dut.u_estimator.origin!==0)
            $fatal(1,"manual reverse reset origin or lost signed travel");

        // Actual fast AB transitions must still abort either manual mode.
        uart_command("F"); tick(3000);
        repeat(12) quadrature(1); tick(20);
        if(dut.arm_speed<=20480 || dut.jogging) $fatal(1,"manual jog speed guard removed");
        tick(5120); repeat(12) quadrature(0); tick(5120);
        uart_command("J"); tick(3000);
        repeat(12) quadrature(1); tick(20);
        if(dut.arm_speed<=20480 || dut.test_active || dut.motor_test_status!==5)
            $fatal(1,"manual test speed guard removed");
        tick(5120); repeat(12) quadrature(0); tick(5120);
        if(dut.position!==1000 || dut.u_estimator.origin!==0) $fatal(1,"manual speed stop reset origin");
        repeat(250) quadrature(0); tick(5120);
        if(dut.arm!==0) $fatal(1,"return-to-origin coordinate setup");
        expect_stop;
        uart_command("H"); tick(320);
        if(dut.state!==2 || dut.start_result!==1) $fatal(1,"H after actual return inside was rejected");
        uart_command("S"); tick(3); expect_stop;
        repeat(250) quadrature(0); tick(5120);
        if(dut.position!==-1000 || dut.arm>=-6144) $fatal(1,"negative outside-position setup");
        expect_limit_rejection("G"); expect_limit_rejection("H");
        uart_command("S"); tick(320);
        if(dut.position!==-1000 || dut.u_estimator.origin!==0 || dut.arm>=-6144)
            $fatal(1,"S silently recentered physical travel");
        expect_limit_rejection("G");
        repeat(250) quadrature(1); tick(5120); move_adc(624);
        expect_stop;
        uart_command("G"); tick(320);
        if(dut.state!==1 || dut.start_result!==1 || dut.motor_command!==-667)
            $fatal(1,"G after actual return inside or corrected motor polarity failed");
        uart_command("S"); tick(3); expect_stop;
        $display("PASS tb_top: UART/key mapping/corrected actuator and raw encoder signs/shared admission limits/R-S no recenter/PWM/raw stop/OTR/XH2 isolation/XH1 drive");
        $finish;
    end
endmodule
