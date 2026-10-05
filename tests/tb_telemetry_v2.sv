`timescale 1ns/1ps
module tb_telemetry_v2;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0, trigger=0;
    reg [31:0] ms=32'h12345678;
    reg signed [31:0] encoder=-32'sd65537;
    reg [15:0] flags=16'h0013;
    reg [9:0] down=219, up=767, raw=1023, minimum=0, maximum=1023;
    reg [13:0] mean=8160, control_mean=12000;
    reg signed [15:0] motor=-16'sd200;
    reg [7:0] first=8'h20, quality=8'h29;
    reg [31:0] samples=32'hfedcba98;
    reg [15:0] sensor_reason=16'h1329;
    reg [9:0] fault_min=17, fault_max=1003;
    reg [13:0] fault_mean=14321;
    localparam [271:0] LIVE_FIRST = 272'h89abcdef80fffffe002013880da2011600c310e103e856781234ffff800001000001;
    localparam [271:0] FAULT_FIRST = 272'h76543210fedc03e80384032002bc025801f40190012c00c8006401f4001003ffabcd;
    reg [271:0] live_detail=LIVE_FIRST, fault_detail=FAULT_FIRST;
    reg [31:0] fault_ms=32'hfedc1234, fault_samples=32'h8765abcd;
    reg [7:0] test_status=5;
    reg signed [31:0] test_delta=-123456789;
    localparam [55:0] H_FIRST = {8'h03,16'd512,-16'sd316,-16'sd12345};
    localparam [55:0] H_SECOND = {8'h08,16'd384,16'sd32767,16'sd25600};
    reg [55:0] h_detail=H_FIRST;
    wire tx, received, framing_error;
    wire [7:0] data;
    telemetry #(.CLK_HZ(1000000),.BAUD(100000),.EXTENDED(1),.CONTROL_ADC(1)) dut(
        .clk(clk),.rst_n(rst_n),.sample_valid(trigger),.adc(10'd765),
        .theta(-16'sd759),.omega(16'sd12),.arm(-16'sd390),.arm_speed(16'sd96),
        .command(16'sd1000),.state(3'd3),.fault(8'h21),.calibrated(1'b1),
        .diagnostic_status(8'h95),.tx(tx),.device_time_ms(ms),
        .encoder_count(encoder),.sensor_flags(flags),
        .adc_down(down),.adc_up(up),.adc_raw(raw),
        .adc_window_min(minimum),.adc_window_max(maximum),.adc_mean_q4(mean),
        .adc_control_q4(control_mean),
        .motor_command(motor),.first_fault(first),.sample_counter(samples),
        .adc_quality_reason(quality),.sensor_fault_reason(sensor_reason),
        .adc_fault_window_min(fault_min),.adc_fault_window_max(fault_max),
        .adc_fault_mean_q4(fault_mean),
        .adc_window_detail(live_detail),.adc_fault_detail(fault_detail),
        .adc_fault_time_ms(fault_ms),.adc_fault_sample_counter(fault_samples),
        .motor_test_status(test_status),.motor_test_delta(test_delta),.handover_detail(h_detail));
    uart_rx_byte #(.CLK_HZ(1000000),.BAUD(100000)) rx(
        .clk(clk),.rst_n(rst_n),.rx(tx),.data(data),.valid(received),.framing_error(framing_error));
    reg [7:0] bytes[0:423];
    // Fixed wire oracle, independently packed and CRC checked with binascii.crc_hqx.
    localparam [1695:0] GOLDEN = 1696'haa5502d40000fd0209fd0c007afe6000e803030121950104785634120204fffffeff0902e8030a0213000b02db000c02ff020d02ff030e0200000f02ff031002e01f110238ff120120130400000300140498badcfe15012916022913170211001802eb031902f1371a22010000010080ffff34127856e803e110c3001601a20d88132000feffff80efcdab891b22cdabff031000f4016400c8002c019001f4015802bc0220038403e803dcfe103254761c043412dcfe1d04cdab65871e01051f04eb32a4f82007c7cfc4fe0002032202e02eb79a;
    integer count=0,k,cursor,base;
    reg [15:0] crc;
    reg [31:0] value;
    function [15:0] crcbyte(input[15:0] old,input[7:0] b);
        reg[15:0] x; integer j;
        begin x=old^{b,8'd0}; for(j=0;j<8;j=j+1) x=x[15]?(x<<1)^16'h1021:x<<1; crcbyte=x; end
    endfunction
    always @(posedge clk) if(rst_n) begin
        if(framing_error) $fatal(1,"UART framing");
        if(received) begin
            if(count>=424) $fatal(1,"unexpected extra bytes");
            bytes[count]=data;count=count+1;
        end
    end
    task ticks(input integer n); begin repeat(n) begin @(posedge clk);#2;end end endtask
    task pulse; begin @(negedge clk);trigger=1;@(negedge clk);trigger=0;end endtask
    task field(input integer kind,input integer length,input [31:0] expected);
        integer j;
        begin
            if(bytes[cursor]!=kind || bytes[cursor+1]!=length) $fatal(1,"TLV schema at %0d",cursor);
            value=0;
            for(j=0;j<length;j=j+1) value=value|({24'd0,bytes[cursor+2+j]}<<(8*j));
            if(value!==expected) $fatal(1,"TLV %0d expected %h got %h",kind,expected,value);
            cursor=cursor+length+2;
        end
    endtask
    task bundle(input integer kind,input [271:0] expected);
        integer j;
        begin
            if(bytes[cursor]!=kind || bytes[cursor+1]!=34) $fatal(1,"bundle TLV schema at %0d",cursor);
            for(j=0;j<34;j=j+1)
                if(bytes[cursor+2+j]!==expected[j*8 +: 8])
                    $fatal(1,"bundle TLV %0d byte %0d was truncated or mixed across snapshots",kind,j);
            cursor=cursor+36;
        end
    endtask
    task handover_bundle(input [55:0] expected);
        integer j;
        begin
            if(bytes[cursor]!=32 || bytes[cursor+1]!=7) $fatal(1,"H TLV schema");
            for(j=0;j<7;j=j+1)
                if(bytes[cursor+2+j]!==expected[j*8 +: 8])
                    $fatal(1,"H TLV mixed snapshot or byte order at %0d",j);
            cursor=cursor+9;
        end
    endtask
    initial begin #5000000;$fatal(1,"v2 timeout");end
    initial begin
        ticks(4);@(negedge clk);rst_n=1;ticks(4);pulse;
        // Change every externally supplied TLV while the first packet is busy.
        // Every field in that packet must retain its original snapshot.
        ticks(120); @(negedge clk);
        ms=32'h87654321;encoder=1234567;flags=16'h000c;
        down=231;up=803;raw=747;minimum=732;maximum=755;mean=11968;control_mean=3210;
        motor=400;first=1;samples=32'h01020304;
        quality=8'h20;sensor_reason=16'h0004;fault_min=211;fault_max=252;fault_mean=3588;
        live_detail=~LIVE_FIRST;fault_detail=~FAULT_FIRST;
        fault_ms=32'h0123cdef;fault_samples=32'hfedcba98;
        test_status=3;test_delta=123456789;
        h_detail=H_SECOND;
        pulse;
        wait(count==212);
        // The RX publishes the last byte before the transmitter's stop bit
        // finishes. A trigger in this drain interval must also be dropped.
        if(!dut.draining || dut.byte_ready) $fatal(1,"drain test missed the final stop bit");
        pulse; ticks(30);
        if(count!=212) $fatal(1,"busy frame leaked");
        pulse;wait(count==424);ticks(30);
        for(k=0;k<212;k=k+1)
            if(bytes[k] !== GOLDEN[1695-k*8 -: 8]) $fatal(1,"RTL golden mismatch at byte %0d",k);
        for(base=0;base<424;base=base+212) begin
            if({bytes[base],bytes[base+1],bytes[base+2],bytes[base+3]}!==32'haa5502d4)
                $fatal(1,"v2 header");
            if({bytes[base+5],bytes[base+4]}!==(base/212)) $fatal(1,"sequence");
            if($signed({bytes[base+9],bytes[base+8]})!==-16'sd759) $fatal(1,"signed theta");
            crc=16'hffff;for(k=base;k<base+210;k=k+1)crc=crcbyte(crc,bytes[k]);
            if({bytes[base+211],bytes[base+210]}!==crc) $fatal(1,"v2 CRC");
            cursor=base+22;
            field(1,4,base==0?32'h12345678:32'h87654321);
            field(2,4,base==0?32'hfffeffff:1234567);field(9,2,1000);
            field(10,2,base==0?19:12);
            field(11,2,base==0?219:231);field(12,2,base==0?767:803);
            field(13,2,base==0?1023:747);field(14,2,base==0?0:732);
            field(15,2,base==0?1023:755);field(16,2,base==0?8160:11968);
            field(17,2,base==0?16'hff38:400);field(18,1,base==0?32:1);
            field(19,4,32'h00030000);field(20,4,base==0?32'hfedcba98:32'h01020304);
            field(21,1,base==0?8'h29:8'h20);field(22,2,base==0?16'h1329:16'h0004);
            field(23,2,base==0?17:211);field(24,2,base==0?1003:252);
            field(25,2,base==0?14321:3588);
            bundle(26,base==0?LIVE_FIRST:~LIVE_FIRST);
            bundle(27,base==0?FAULT_FIRST:~FAULT_FIRST);
            field(28,4,base==0?32'hfedc1234:32'h0123cdef);
            field(29,4,base==0?32'h8765abcd:32'hfedcba98);
            field(30,1,base==0?5:3);field(31,4,base==0?32'hf8a432eb:123456789);
            handover_bundle(base==0?H_FIRST:H_SECOND);
            field(34,2,base==0?12000:3210);
            if(cursor!=base+210) $fatal(1,"TLV final offset");
        end
        $write("V2_WIRE_HEX=");for(k=0;k<212;k=k+1)$write("%02x",bytes[k]);$display("");
        $display("PASS tb_telemetry_v2: UART/CRC/all TLV/snapshot/sequence/busy/drain drop/full-width diagnostic bundles/control ADC TLV34");$finish;
    end
endmodule
