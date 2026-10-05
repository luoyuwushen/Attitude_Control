`timescale 1ns/1ps
// Real command UART -> production top/probe/buffer/export -> shared UART.
// ADC and encoder pins are synthetic bench stimuli, not a physical plant.
// No force/release, backdoor command or state/safety overrides are used.
module tb_control_trace_top;
    localparam integer ADC_CYCLES=80, WINDOW_CYCLES=1280, UART_BIT=10;
    reg clk=0;always #10 clk=~clk;
    reg rst_n=0, uart_rx=1, enc_a=0, enc_b=0;
    reg [9:0] adc=240;
    reg [3:0] keys=4'b1111;
    wire tx,adc_clk,oe,an1,an2,pwma,bn1,bn2,pwmb;
    top #(.KEY_CYCLES(4),.ADC_SAMPLE_CYCLES(ADC_CYCLES),
          .TRACE_ADDRESS_BITS(8),.UART_BAUD(5000000)) dut(
        .clk_50m(clk),.rst_n(rst_n),.key_sw(keys),.uart_rx(uart_rx),.uart_tx(tx),
        .adc_data_in(adc),.adc_otr(1'b0),.adc_clk(adc_clk),.adc_oe_n(oe),
        .enc1_a(enc_a),.enc1_b(enc_b),.AN1(an1),.AN2(an2),.PWMA(pwma),
        .BN1(bn1),.BN2(bn2),.PWMB(pwmb));
    wire [7:0] received_byte;
    wire received,framing_error;
    uart_rx_byte #(.BAUD(5000000)) receiver(
        .clk(clk),.rst_n(rst_n),.rx(tx),.data(received_byte),.valid(received),.framing_error(framing_error));

    integer checks=0, cycles=0, packet_index=0, length=0, wire_packets=0;
    integer v2_count=0, v3_count=0, last_v2_cycle=0, max_v2_gap=0;
    integer case_id=0, case_packets=0, case_rows=0, case_ends=0, case_v2_start=0;
    integer capture_commits=0, expected_count=0, expected_total=0, expected_id=0;
    integer stop_checks=0, stop_pending=-1, max_stop_clocks=0, stopped_clocks;
    integer j,k,n,wire_fd=0;
    reg [2047:0] wire_path;
    reg [15:0] last_v2_sequence=0,crc,rows_crc=16'hffff;
    reg [7:0] packet[0:239];
    reg [255:0] committed_ring[0:255],expected_rows[0:255];
    reg [191:0] expected_metadata;
    reg [31:0] last_v2_time=0,last_v2_sample=0;
    function [15:0] crcbyte;
        input [15:0] previous;input [7:0] value;
        reg [15:0] c;integer bitnum;
        begin
            c=previous^{value,8'd0};
            for(bitnum=0;bitnum<8;bitnum=bitnum+1)c=c[15]?(c<<1)^16'h1021:c<<1;
            crcbyte=c;
        end
    endfunction
    function [15:0] word16;
        input integer offset;
        begin word16={packet[offset+1],packet[offset]};end
    endfunction
    task check;
        input condition;input [1023:0] message;
        begin
            checks=checks+1;
            if(condition!==1'b1)$fatal(1,"case%0d cycle%0d: %0s",case_id,cycles,message);
        end
    endtask
    // This scoreboard watches completed row writes, not buffer read results.
    // The later UART must reproduce the chronological retained suffix exactly.
    always @(posedge clk) if(rst_n) begin
        cycles=cycles+1;
        if(dut.u_trace_probe.journal.start_capture)capture_commits=0;
        else if(dut.trace_capturing && dut.u_trace_probe.row_valid)begin
            committed_ring[capture_commits%256]=dut.u_trace_probe.row;
            capture_commits=capture_commits+1;
        end
        if(dut.request_stop)stop_pending=cycles;
        if(stop_pending>=0)begin
            #2;
            if(!dut.drive_enabled && dut.requested_command==0 && dut.stopped)begin
                stopped_clocks=cycles-stop_pending;
                check(stopped_clocks<=4,"S priority: stop independent of exporter readiness");
                if(stopped_clocks>max_stop_clocks)max_stop_clocks=stopped_clocks;
                stop_pending=-1;stop_checks=stop_checks+1;
            end else check(cycles-stop_pending<4,"S did not promptly remove drive");
        end
        check(!(dut.trace_packet_busy && dut.telemetry_busy),"exclusive UART packet ownership");
        check({an1,an2,pwma}===3'b001 && !(bn1 && bn2),"motor pad safety/routing");
        check(dut.state!=1,"H/T path must never enter SWING");
    end
    always @(posedge clk) if(rst_n)begin
        if(framing_error)$fatal(1,"shared UART framing error");
        if(received)begin
            packet[packet_index]=received_byte;
            if(packet_index==0)check(received_byte==8'haa,"every packet starts at actual AA byte");
            if(packet_index==1)check(received_byte==8'h55,"shared UART frame alignment");
            if(packet_index==2)check(received_byte==2 || received_byte==3,"v2/v3 only");
            if(packet_index==3)begin length=received_byte;check(length>=16 && length<=240,"bounded frame length");end
            packet_index=packet_index+1;
            if(length && packet_index==length)begin
                crc=16'hffff;
                for(j=0;j<length-2;j=j+1)crc=crcbyte(crc,packet[j]);
                check(word16(length-2)==crc,"current and historical packet CRC");
                if(packet[2]==2)begin
                    check(length==219 && packet[206]==33 && packet[207]==5 && packet[213]==34 && packet[214]==2,"current v2 trace-status TLV");
                    check({packet[78],packet[77],packet[76],packet[75]}==32'h00030000,"current device version");
                    if(v2_count)begin
                        check(word16(4)==last_v2_sequence+16'd1,"no current telemetry sequence loss/duplication");
                        check({packet[27],packet[26],packet[25],packet[24]}>=last_v2_time,"current device time cannot go backwards");
                        check({packet[84],packet[83],packet[82],packet[81]}>last_v2_sample,"current acquisition advances during download");
                        if(cycles-last_v2_cycle>max_v2_gap)max_v2_gap=cycles-last_v2_cycle;
                        check(cycles-last_v2_cycle<180000,"current heartbeat is not starved by dump");
                    end
                    last_v2_sequence=word16(4);last_v2_time={packet[27],packet[26],packet[25],packet[24]};
                    last_v2_sample={packet[84],packet[83],packet[82],packet[81]};
                    last_v2_cycle=cycles;v2_count=v2_count+1;
                end else begin
                    check(case_id>0 && case_ends==0,"trace belongs to explicitly requested capture");
                    check(packet[5]==1 && packet[13]==0 && word16(6)==expected_id && word16(10)==expected_count,"trace schema/id/total snapshot");
                    case(packet[4])
                        1:begin
                            check(case_packets==0 && length==40 && word16(8)==0 && packet[12]==0,"fresh opening metadata");
                            for(j=0;j<24;j=j+1)check(packet[14+j]===expected_metadata[j*8+:8],"metadata from accepted H/freeze");
                        end
                        2:begin
                            n=packet[12];
                            check(case_packets>0 && n>=1 && n<=7 && length==16+n*32,"data after META with exact rows");
                            check(word16(8)==case_rows && case_rows+n<=expected_count,"contiguous bounded rows");
                            for(k=0;k<n;k=k+1)for(j=0;j<32;j=j+1)begin
                                check(packet[14+k*32+j]===expected_rows[case_rows+k][j*8+:8],"actual committed row/full epoch bytes preserved");
                                rows_crc=crcbyte(rows_crc,packet[14+k*32+j]);
                            end
                            case_rows=case_rows+n;
                        end
                        3:begin
                            check(case_rows==expected_count && length==18 && word16(8)==expected_count && packet[12]==0,"complete END only");
                            check(word16(14)==rows_crc,"whole record CRC");case_ends=case_ends+1;
                        end
                        default:$fatal(1,"unknown trace packet kind");
                    endcase
                    case_packets=case_packets+1;v3_count=v3_count+1;
                end
                if(wire_fd)begin
                    $fwrite(wire_fd,"WIRE %0d ",case_id);
                    for(j=0;j<length;j=j+1)$fwrite(wire_fd,"%02x",packet[j]);
                    $fwrite(wire_fd,"\n");
                end
                wire_packets=wire_packets+1;packet_index=0;length=0;
            end
        end
    end
    task ticks;
        input integer count;
        begin repeat(count)begin @(posedge clk);#3;end end
    endtask
    task uart_bit;
        input value;
        begin uart_rx=value;repeat(UART_BIT)@(negedge clk);end
    endtask
    task send;
        input [7:0] value;
        integer bitnum;
        begin
            @(negedge clk);uart_bit(0);
            for(bitnum=0;bitnum<8;bitnum=bitnum+1)uart_bit(value[bitnum]);
            uart_bit(1);ticks(20);
        end
    endtask
    task snapshot;
        input integer new_case;
        integer i,first;
        begin
            check(dut.trace_frozen && !dut.trace_export_active,"snapshot only real stopped frozen capture");
            case_id=new_case;case_packets=0;case_rows=0;case_ends=0;rows_crc=16'hffff;case_v2_start=v2_count;
            expected_count=dut.trace_row_count;expected_total=dut.trace_total_committed;expected_id=dut.trace_capture_id;
            check(expected_total==capture_commits && expected_count==((capture_commits>256)?256:capture_commits),"ring count matches completed write scoreboard");
            first=capture_commits-expected_count;
            expected_metadata={7'd0,dut.trace_overwritten,dut.trace_freeze_reason,dut.trace_total_committed,
                dut.trace_freeze_ms,32'd1280,dut.trace_saved_capture,2'd0,dut.trace_up_q4,2'd0,dut.trace_down_q4,32'h00030000};
            if(wire_fd)$fwrite(wire_fd,"CASE %0d %0d %0d %0d %048x\n",case_id,expected_count,expected_total,expected_id,expected_metadata);
            for(i=0;i<expected_count;i=i+1)begin
                expected_rows[i]=committed_ring[(first+i)%256];
                if(wire_fd)$fwrite(wire_fd,"EXPECTED %0d %0d %064x\n",case_id,i,expected_rows[i]);
            end
        end
    endtask
    task finish_dump;
        input integer complete;
        integer before_count;
        begin
            wait(!dut.trace_export_active);ticks(100);
            before_count=v3_count;ticks(2000);
            check(v3_count==before_count && case_ends==complete,"no late packet/false END after abort");
            if(complete)check(case_rows==expected_count,"all frozen rows exported");
            else check(case_packets==2 && case_rows==7,"abort finishes current prefetched data packet only");
            if(wire_fd)$fwrite(wire_fd,"RESULT %0d %0d %0d %0d %0d\n",case_id,case_packets,case_rows,case_ends,v2_count-case_v2_start);
            $display("TOP CASE %0d PASS packets=%0d rows=%0d END=%0d live_frames=%0d",case_id,case_packets,case_rows,case_ends,v2_count-case_v2_start);
        end
    endtask
    integer first_v3,stops_before;
    initial begin
        if($value$plusargs("WIRE_LOG=%s",wire_path))begin
            wire_fd=$fopen(wire_path,"w");if(!wire_fd)$fatal(1,"wire file open");
        end
        ticks(5);@(negedge clk);rst_n=1;ticks(28000);
        send("D");check(dut.adc_down_q4==240*16 && !dut.calibrated,"UART D accepted");
        @(negedge clk);adc=800;ticks(28000);send("U");ticks(28000);
        check(dut.calibrated && dut.measurement_ready && !dut.sensor_fault && dut.adc_up_q4==800*16,"UART U calibrates and matures");
        send("H");check(dut.state==2 && dut.trace_capturing,"UART H starts real capture");
        send("T");ticks(100);check(!dut.trace_export_active && dut.state==2,"T refused while H moves");
        @(negedge clk);adc=792;ticks(WINDOW_CYCLES*25);
        // Four genuine quadrature transitions create a small arm error and
        // nonzero late integral without bypassing the production estimator.
        @(negedge clk);{enc_a,enc_b}=2'b10;ticks(WINDOW_CYCLES*2);
        @(negedge clk);{enc_a,enc_b}=2'b11;ticks(WINDOW_CYCLES*2);
        @(negedge clk);{enc_a,enc_b}=2'b01;ticks(WINDOW_CYCLES*2);
        @(negedge clk);{enc_a,enc_b}=2'b00;
        wait(capture_commits>=405);
        check(dut.requested_command!=0 && dut.drive_enabled,"H produces nonzero feedback before stop");
        send("S");wait(dut.trace_frozen);ticks(10);
        check(dut.trace_row_count==256 && dut.trace_overwritten && dut.trace_freeze_reason==1,"real stop freezes covered ring");

        snapshot(1);send("T");wait(dut.trace_export_active);
        wait(case_packets==1 && packet_index==20 && packet[2]==3);
        stops_before=stop_checks;send("S");check(stop_checks==stops_before+1 && dut.trace_packet_busy,"S remains immediate while stopped export continues");
        finish_dump(1);check(v2_count-case_v2_start>=3,"normal heartbeats continue during complete download");

        snapshot(2);send("T");wait(case_packets==1 && packet_index==20 && packet[2]==3);
        check(dut.trace_packet_busy,"H abort is inside actual UART packet");
        send("H");check(dut.state==2 && dut.trace_packet_busy,"H accepted without waiting for exporter drain");
        stops_before=stop_checks;send("S");
        check(stop_checks==stops_before+1 && dut.stopped && dut.trace_packet_busy,"T then H then S stops before old packet finishes");
        finish_dump(0);wait(dut.trace_frozen);

        send("H");wait(capture_commits>=20);send("S");wait(dut.trace_frozen);ticks(10);
        snapshot(3);send("T");wait(case_packets==1 && packet_index==20 && packet[2]==3);
        send("F");check(dut.jogging && dut.drive_enabled && dut.trace_packet_busy,"manual jog accepted during trace packet");
        stops_before=stop_checks;send("S");
        check(stop_checks==stops_before+1 && dut.stopped && dut.trace_packet_busy,"S aborts manual drive before trace packet drain");
        finish_dump(0);

        snapshot(4);send("T");wait(dut.trace_export_active);finish_dump(1);
        first_v3=v3_count;wait(v2_count>case_v2_start+2);ticks(100);
        check(v3_count==first_v3 && !dut.trace_export_active && !dut.drive_enabled,"retry completes and live status resumes without motor action");
        check(stop_checks>=5 && max_stop_clocks<=4,"all S requests checked at production RX decode");
        if(wire_fd)$fclose(wire_fd);
        $display("PASS control trace top: checks=%0d v2=%0d v3=%0d stop_checks=%0d stop_max_clocks=%0d max_v2_gap_clocks=%0d",checks,v2_count,v3_count,stop_checks,max_stop_clocks,max_v2_gap);
        $finish;
    end
    initial begin #100000000;$fatal(1,"bounded top trace timeout");end
endmodule
