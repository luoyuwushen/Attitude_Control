`timescale 1ns/1ps
// A live telemetry heartbeat never authorizes motion with an expired sample.
// Outage injection only blocks acquisition completion; physical control inputs
// and every protection remain active.
module tb_sample_freshness;
    reg clk=0;
    always #10 clk=~clk;
    reg rst_n=0, uart_rx=1;
    reg [3:0] key_sw=4'b1111;
    reg [9:0] adc=624;
    wire uart_tx, adc_clk, adc_oe_n, an1, an2, pwma, bn1, bn2, pwmb;
    top #(.TELEMETRY_EXTENDED(0),.KEY_CYCLES(4),.ADC_SAMPLE_CYCLES(10),.JOG_CYCLES(20000),
          .SAMPLE_WATCHDOG_CYCLES(1000)) dut (
        .clk_50m(clk),.rst_n(rst_n),.key_sw(key_sw),.uart_rx(uart_rx),.uart_tx(uart_tx),
        .adc_data_in(adc),.adc_otr(1'b0),.adc_clk(adc_clk),.adc_oe_n(adc_oe_n),
        .enc1_a(1'b0),.enc1_b(1'b0),.AN1(an1),.AN2(an2),.PWMA(pwma),
        .BN1(bn1),.BN2(bn2),.PWMB(pwmb));
    task ticks(input integer count);
        begin repeat(count) begin @(posedge clk); #2; end end
    endtask
    task key(input integer index);
        begin
            @(negedge clk); key_sw[index]=0; ticks(10);
            @(negedge clk); key_sw[index]=1; ticks(10);
        end
    endtask
    // Keep manual movement within each ADC window and wait for trustworthy
    // velocity history; instantaneous jumps are reserved for fault injection.
    task move_adc(input integer target);
        integer remaining;
        begin
            while (adc != target) begin
                @(negedge clk);
                remaining=target-$signed({1'b0,adc});
                if (remaining>16) adc=adc+16;
                else if (remaining < -16) adc=adc-16;
                else adc=target;
                ticks(160);
            end
            ticks(5120);
        end
    endtask
    task uart_bit(input bit value);
        begin uart_rx=value; repeat(434) @(negedge clk); end
    endtask
    task send(input [7:0] value);
        integer n;
        begin
            @(negedge clk); uart_bit(0);
            for(n=0;n<8;n=n+1) uart_bit(value[n]);
            uart_bit(1); ticks(20);
        end
    endtask
    task coast;
        begin
            if({an1,an2,pwma,bn1,bn2,pwmb}!==6'b001001 || dut.motor_permission)
                $fatal(1,"stale measurement left an enabled motor output");
        end
    endtask
    task outage;
        begin
            @(negedge clk); force dut.adc_valid=1'b0; ticks(1020);
            if(!dut.sample_timeout || dut.telemetry_fault!==8'h04)
                $fatal(1,"sample outage must be reported as 04 in every mode");
            coast;
        end
    endtask
    task restore;
        begin
            @(negedge clk); release dut.adc_valid; ticks(3000);
            if(dut.sample_timeout) $fatal(1,"new completed measurement did not restore freshness");
            coast;
        end
    endtask
    initial begin #5000000; $fatal(1,"tb_sample_freshness timeout"); end
    initial begin
        ticks(5); @(negedge clk); rst_n=1; ticks(1000); key(0);
        move_adc(800); key(3); ticks(3000);
        move_adc(624);
        if(!dut.calibrated || dut.sample_timeout || dut.sensor_fault)
            $fatal(1,"freshness setup calibration");

        outage;
        if(dut.state!==0) $fatal(1,"idle outage changed controller mode");
        key(1);
        if(dut.start_result!==4 || dut.state!==0) $fatal(1,"SW2 accepted stale idle sample");
        send("G"); send("F"); send("B");
        if(dut.state!==0 || dut.jogging || dut.start_result!==4)
            $fatal(1,"UART motion accepted stale idle sample");
        key(0); key(3);
        if(!dut.calibrated || dut.u_estimator.down_code!==624*16 || dut.u_estimator.up_code!==800*16)
            $fatal(1,"D/U replaced calibration using a stale ADC");
        send("R");
        if(dut.telemetry_fault!==8'h04) $fatal(1,"R hid an ongoing idle sample outage");
        restore;
        if(dut.state!==0 || dut.jogging) $fatal(1,"idle recovery restarted an old request");

        send("F"); ticks(3000);
        if(!dut.jogging || dut.motor_command!==-100 || {bn1,bn2}!==2'b01) $fatal(1,"fresh F did not drive XH1");
        outage;
        if(dut.jogging || dut.state!==0) $fatal(1,"outage did not cancel jog");
        restore; ticks(3000);
        if(dut.jogging) $fatal(1,"sample recovery resumed an interrupted jog");

        send("G"); ticks(3000);
        if(dut.state!==1 || !dut.motor_enable) $fatal(1,"fresh G not accepted");
        outage;
        if(dut.state!==3 || dut.fault!==8'h04)
            $fatal(1,"SWING watchdog must retain 04, not become a board 01 fault");
        send("R");
        if(dut.state!==0 || dut.telemetry_fault!==8'h04)
            $fatal(1,"R cleared live timeout or did not clear controller latch");
        restore;
        if(dut.state!==0) $fatal(1,"measurement recovery restarted swing");
        key(1); ticks(3000);
        if(dut.state!==1 || {bn1,bn2}!==2'b01) $fatal(1,"fresh SW2 failed after recovery");
        key(2); coast;
        // ADC activity alone must not hide a stalled calibrated estimator.
        @(negedge clk); force dut.state_valid=1'b0; ticks(1020);
        if(!dut.sample_timeout || dut.telemetry_fault!==8'h04)
            $fatal(1,"raw ADC updates hid missing estimator completions");
        coast; key(1);
        if(dut.state!==0 || dut.start_result!==4) $fatal(1,"estimator outage admitted SW2");
        @(negedge clk); release dut.state_valid; ticks(350);
        if(dut.sample_timeout || dut.state!==0) $fatal(1,"estimator recovery restarted motion");
        coast;
        $display("PASS tb_sample_freshness: IDLE/JOG/SWING sample timeout/04/D-U rejection/no automatic restart");
        $finish;
    end
endmodule
