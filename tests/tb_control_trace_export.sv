`timescale 1ns/1ps
// Actual buffer/export/UART path. No hierarchical force or synthetic TX bytes.
// Four clocks per bit accelerate wire time only; memory remains synchronous.
module tb_control_trace_export;
    localparam integer BIT_CYCLES=4;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0, start_capture=0, freeze_capture=0, row_valid=0;
    reg [255:0] row_data=0;
    reg [7:0] freeze_reason_in=1;
    reg [31:0] device_ms=0;
    reg [13:0] down=3557, up=12535;
    reg signed [15:0] capture=-316;
    wire capturing, frozen, overwritten, read_valid;
    wire [15:0] capture_id;
    wire [12:0] row_count;
    wire [31:0] total_committed, freeze_time;
    wire [7:0] freeze_reason;
    wire [13:0] saved_down, saved_up;
    wire signed [15:0] saved_capture;
    wire read_request;
    wire [12:0] read_index;
    wire [255:0] read_data;
    reg request=0, permit=1, packet_grant=1;
    reg suppress_read_valid=0, reject_read_address=0;
    wire active, packet_busy, tx;
    wire [12:0] buffer_read_index=reject_read_address ? row_count : read_index;
    control_trace_buffer buffer_dut(
        .clk(clk),.rst_n(rst_n),.start_capture(start_capture),
        .freeze_capture(freeze_capture),.freeze_reason_in(freeze_reason_in),
        .device_time_ms(device_ms),.down_q4(down),.up_q4(up),
        .capture_arm_q10(capture),.row_valid(row_valid),.row_data(row_data),
        .capturing(capturing),.frozen(frozen),.capture_id(capture_id),
        .row_count(row_count),.total_committed(total_committed),.overwritten(overwritten),
        .freeze_time_ms(freeze_time),.freeze_reason(freeze_reason),
        .saved_down_q4(saved_down),.saved_up_q4(saved_up),
        .saved_capture_arm_q10(saved_capture),.read_request(read_request),
        .read_index(buffer_read_index),.read_valid(read_valid),.read_data(read_data));
    control_trace_export #(.CLK_HZ(1000000),.BAUD(250000),
        .FIRMWARE_ID(32'h12345678),.SAMPLE_PERIOD_CYCLES(32'd50000)) dut(
        .clk(clk),.rst_n(rst_n),.request(request),.permit(permit),.frozen(frozen),
        .packet_grant(packet_grant),.capture_id(capture_id),.row_count(row_count),
        .total_committed(total_committed),.freeze_time_ms(freeze_time),
        .overwritten(overwritten),.freeze_reason(freeze_reason),
        .down_q4(saved_down),.up_q4(saved_up),.capture_arm_q10(saved_capture),
        .read_request(read_request),.read_index(read_index),
        .read_valid(read_valid && !suppress_read_valid),.read_data(read_data),
        .active(active),.packet_busy(packet_busy),.tx(tx));

    integer checks=0, case_id=0, packets=0, rows_seen=0, metas=0, ends=0;
    integer byte_pos=0, packet_length=0, total_bytes=0, read_requests=0;
    integer expected_rows=0, expected_first=0, expected_total=0, expected_id=0;
    reg [191:0] expected_metadata;
    reg [15:0] record_crc=16'hffff;
    reg [7:0] wire_packet[0:239];
    reg [7:0] rx_byte;
    integer bit_number, wire_fd=0;
    reg [2047:0] wire_path;

    function [255:0] pattern;
        input integer n;
        reg [255:0] result;
        integer command_value;
        begin
            result=0;
            result[31:0]=32'hfffff000+n;
            result[63:32]=-123456789+n*131071;
            result[79:64]=n%16384;
            result[95:80]=n*71-32768;
            result[111:96]=32767-n*43;
            result[127:112]=n*97-12345;
            result[143:128]=-32768+n*113;
            result[175:144]=32'h80000001^(n*32'h00010203);
            command_value=n%2001-1000;
            result[191:176]=command_value;
            result[207:192]=-command_value;
            result[223:208]=n%513;
            result[231:224]=n%16;
            result[239:232]=(n%2)*32;
            result[247:240]=12+(n%4);
            result[255:248]=n%4;
            pattern=result;
        end
    endfunction
    function [15:0] crc_byte;
        input [15:0] previous;
        input [7:0] value;
        integer j;
        reg [15:0] c;
        begin
            c=previous^{value,8'd0};
            for(j=0;j<8;j=j+1) c=c[15] ? (c<<1)^16'h1021 : c<<1;
            crc_byte=c;
        end
    endfunction
    function [15:0] word16;
        input integer offset;
        begin word16={wire_packet[offset+1],wire_packet[offset]}; end
    endfunction
    task check;
        input condition;
        input [1023:0] message;
        begin
            checks=checks+1;
            if(condition!==1'b1) $fatal(1,"case=%0d packet=%0d: %0s",case_id,packets,message);
        end
    endtask
    task ticks;
        input integer n;
        begin repeat(n) begin @(posedge clk);#2;end end
    endtask
    task check_packet;
        integer j, k, count;
        reg [15:0] crc;
        reg [255:0] expected;
        begin
            check(wire_packet[0]==8'haa && wire_packet[1]==8'h55 && wire_packet[2]==3,"wire header");
            check(wire_packet[3]==packet_length && wire_packet[5]==1 && wire_packet[13]==0,"length/schema/flags");
            check(word16(6)==expected_id && word16(10)==expected_rows,"capture and total snapshot");
            crc=16'hffff;
            for(j=0;j<packet_length-2;j=j+1) crc=crc_byte(crc,wire_packet[j]);
            check(word16(packet_length-2)==crc,"packet CRC from actual UART bytes");
            check(ends==0,"no data after END");
            case(wire_packet[4])
                1: begin
                    check(packets==0 && metas==0 && packet_length==40,"exactly one opening META");
                    check(word16(8)==0 && wire_packet[12]==0,"META index/count");
                    for(j=0;j<24;j=j+1) check(wire_packet[14+j]===expected_metadata[j*8+:8],"metadata snapshot byte");
                    metas=metas+1;
                end
                2: begin
                    count=wire_packet[12];
                    check(metas==1 && count>=1 && count<=7,"DATA requires META and 1..7 rows");
                    check(word16(8)==rows_seen && rows_seen+count<=expected_rows,"strict contiguous bounded row index");
                    check(count==((expected_rows-rows_seen>=7)?7:expected_rows-rows_seen),"seven-row grouping including final remainder");
                    check(packet_length==16+count*32,"DATA exact byte length");
                    for(k=0;k<count;k=k+1) begin
                        expected=pattern(expected_first+rows_seen+k);
                        for(j=0;j<32;j=j+1) begin
                            check(wire_packet[14+k*32+j]===expected[j*8+:8],"row byte order/ring chronology/full width");
                            record_crc=crc_byte(record_crc,wire_packet[14+k*32+j]);
                        end
                    end
                    rows_seen=rows_seen+count;
                end
                3: begin
                    check(metas==1 && rows_seen==expected_rows,"END only after every row");
                    check(packet_length==18 && word16(8)==expected_rows && wire_packet[12]==0,"END index/count/length");
                    check(word16(14)==record_crc,"record CRC includes only all raw row bytes");
                    ends=ends+1;
                end
                default: $fatal(1,"unknown wire packet kind");
            endcase
            if(wire_fd) begin
                $fwrite(wire_fd,"PACKET %0d ",case_id);
                for(j=0;j<packet_length;j=j+1) $fwrite(wire_fd,"%02x",wire_packet[j]);
                $fwrite(wire_fd,"\n");
            end
            packets=packets+1;
        end
    endtask
    // Sample UART bit centers directly, independently of production RX logic.
    always begin
        @(negedge tx);
        if(rst_n) begin
            check(packet_busy && active,"packet ownership before start bit");
            repeat(BIT_CYCLES+BIT_CYCLES/2) @(posedge clk);
            #2;
            for(bit_number=0;bit_number<8;bit_number=bit_number+1) begin
                rx_byte[bit_number]=tx;
                repeat(BIT_CYCLES) @(posedge clk);
                #2;
            end
            check(tx===1'b1,"UART stop bit");
            check(packet_busy && active,"packet ownership retained through final stop bit");
            wire_packet[byte_pos]=rx_byte;
            total_bytes=total_bytes+1;
            if(byte_pos==3) begin
                packet_length=rx_byte;
                check(packet_length>=16 && packet_length<=240,"bounded packet length");
            end
            byte_pos=byte_pos+1;
            if(packet_length!=0 && byte_pos==packet_length) begin
                check_packet;
                byte_pos=0;packet_length=0;
            end
        end
    end
    always @(posedge clk) if(rst_n && read_request) begin
        check(active && !packet_busy,"RAM reads only outside wire packet");
        check(read_index<expected_rows && read_index<4096,"exporter never requests beyond frozen rows/depth");
        read_requests=read_requests+1;
    end
    always @(negedge clk) if(rst_n && reject_read_address && read_request) begin
        // Actual buffer rejects index==row_count; the following registered
        // invalid response is separately checked in the scenario below.
        check(buffer_read_index==row_count,"fault fixture addresses exact first invalid row");
    end
    task pulse_request;
        begin @(negedge clk);request=1;@(negedge clk);request=0;end
    endtask
    task make_capture;
        input integer count;
        input integer first;
        input integer reason;
        integer j;
        begin
            check(!active,"capture fixture starts after export stops");
            @(negedge clk); start_capture=1;freeze_capture=0;row_valid=0;
            down=3557;up=12535;capture=-316;
            ticks(1);@(negedge clk);start_capture=0;
            for(j=0;j<count;j=j+1) begin
                row_valid=1;row_data=pattern(first+j);
                @(negedge clk);
            end
            row_valid=0;device_ms=32'hfedc0000+count;freeze_reason_in=reason;freeze_capture=1;
            ticks(1);@(negedge clk);freeze_capture=0;ticks(2);
            check(frozen && !capturing,"frozen real buffer");
            check(row_count==((count>4096)?4096:count) && total_committed==count,"capture capacity/count");
            // Saved metadata must not follow later live calibration pins.
            down=0;up=0;capture=0;
        end
    endtask
    task begin_case;
        input integer number;
        input integer first;
        begin
            check(!active && byte_pos==0,"new test starts only after complete wire drain");
            case_id=number;packets=0;rows_seen=0;metas=0;ends=0;
            total_bytes=0;read_requests=0;record_crc=16'hffff;
            expected_rows=row_count;expected_first=first;expected_total=total_committed;expected_id=capture_id;
            expected_metadata={7'd0,overwritten,freeze_reason,total_committed,freeze_time,
                               32'd50000,saved_capture,2'd0,saved_up,2'd0,saved_down,32'h12345678};
            if(wire_fd) $fwrite(wire_fd,"CASE %0d %0d %0d %0d %0d %048x\n",case_id,expected_rows,expected_first,expected_total,expected_id,expected_metadata);
        end
    endtask
    task finish_case;
        input integer expect_packets;
        input integer expect_rows;
        input integer expect_end;
        integer saved_bytes;
        begin
            wait(!active);ticks(20);saved_bytes=total_bytes;ticks(120);
            check(byte_pos==0 && tx===1'b1 && !packet_busy,"whole packet and stop-bit drain");
            check(total_bytes==saved_bytes,"no delayed extra bytes");
            check(packets==expect_packets && rows_seen==expect_rows && ends==expect_end,"scenario packet/row/completion result");
            if(wire_fd) $fwrite(wire_fd,"RESULT %0d %0d %0d %0d %0d %0d\n",case_id,packets,rows_seen,ends,total_bytes,read_requests);
            $display("CASE %0d PASS packets=%0d rows=%0d end=%0d reads=%0d bytes=%0d",case_id,packets,rows_seen,ends,read_requests,total_bytes);
        end
    endtask
    task transient_abort;
        begin
            @(negedge clk);permit=0;ticks(3);
            @(negedge clk);permit=1;
        end
    endtask
    integer prior_bytes;
    initial begin
        if($value$plusargs("WIRE_LOG=%s",wire_path)) begin
            wire_fd=$fopen(wire_path,"w");
            if(!wire_fd) $fatal(1,"cannot open wire log");
        end
        ticks(4);@(negedge clk);rst_n=1;ticks(4);
        pulse_request;ticks(20);check(!active && total_bytes==0,"request without frozen capture ignored");

        // Empty capture: no RAM access, END record CRC is ffff.
        make_capture(0,0,3);begin_case(1,0);packet_grant=0;pulse_request;
        ticks(200);check(active && !packet_busy && tx && packets==0 && read_requests==0,"grant stall before META");
        packet_grant=1;wait(packet_busy);pulse_request;finish_case(2,0,1);
        check(read_requests==0,"empty capture never reads stale RAM");

        make_capture(1,100,1);begin_case(2,100);pulse_request;wait(active);finish_case(3,1,1);

        // Deassert grant while META is on wire: META must finish; DATA waits.
        make_capture(15,200,2);begin_case(3,200);pulse_request;
        wait(byte_pos==8);packet_grant=0;wait(packets==1);wait(!packet_busy);ticks(200);
        check(active && packets==1 && !packet_busy && rows_seen==0,"packet boundary grant stall");
        prior_bytes=total_bytes;ticks(100);check(total_bytes==prior_bytes && tx,"no bytes during grant stall");
        packet_grant=1;finish_case(5,15,1);check(read_requests==15,"one synchronous read per row");
        begin_case(4,200);pulse_request;wait(active);finish_case(5,15,1);

        // Full production depth after wrap: 585*7 + 1 rows, no 12-bit wrap.
        make_capture(4105,10000,1);begin_case(5,10009);pulse_request;wait(active);
        finish_case(588,4096,1);check(read_requests==4096,"full-depth exact read count");

        make_capture(20,300,1);begin_case(6,300);packet_grant=0;pulse_request;wait(active);
        transient_abort;finish_case(0,0,0);packet_grant=1;
        begin_case(7,300);pulse_request;wait(byte_pos==10);transient_abort;
        finish_case(1,0,0);check(read_requests==0,"mid-META abort performs no DATA reads");

        begin_case(8,300);pulse_request;wait(packets==1 && byte_pos==9);packet_grant=0;
        transient_abort;finish_case(2,7,0);packet_grant=1;
        check(read_requests==7,"mid-DATA abort has no future row reads");
        begin_case(9,300);pulse_request;wait(active);finish_case(5,20,1);

        // An END that has not started must be suppressed. An END already on
        // wire must finish (its capture was already complete), never truncate.
        make_capture(1,400,1);begin_case(10,400);pulse_request;
        wait(packets==1 && byte_pos==9);packet_grant=0;
        wait(packets==2);wait(!packet_busy);ticks(30);
        transient_abort;finish_case(2,1,0);packet_grant=1;
        begin_case(11,400);pulse_request;wait(packets==2 && byte_pos==9);
        transient_abort;finish_case(3,1,1);

        // Losing the RAM response or rejecting its address may emit META,
        // but cannot fabricate a row/END or keep exporter busy forever.
        begin_case(12,400);suppress_read_valid=1;pulse_request;wait(packets==1);
        ticks(400);check(!active,"missing read response has bounded timeout");
        finish_case(1,0,0);suppress_read_valid=0;
        begin_case(13,400);reject_read_address=1;pulse_request;
        wait(read_request);ticks(2);check(!read_valid && read_data==0,"buffer out-of-bounds response hides old data");
        ticks(400);check(!active,"rejected address has bounded timeout");
        finish_case(1,0,0);reject_read_address=0;

        // Replacing the generation during a prefetched DATA packet must finish
        // only that old packet, then allow a clean new META on a later request.
        make_capture(20,500,1);begin_case(14,500);pulse_request;
        wait(packets==1 && byte_pos==9);
        @(negedge clk);start_capture=1;ticks(1);@(negedge clk);start_capture=0;
        finish_case(2,7,0);check(capturing && !frozen,"new generation remains independent of old export");
        make_capture(8,600,2);begin_case(15,600);pulse_request;wait(active);finish_case(4,8,1);

        begin_case(16,600);permit=0;pulse_request;ticks(40);
        check(!active,"motion permit rejects initial export request");finish_case(0,0,0);permit=1;
        if(wire_fd) $fclose(wire_fd);
        $display("PASS control trace export: cases=16 checks=%0d actual_UART/packet_CRC/record_CRC/4096_ring/empty/retry/grant/abort/read_timeout",checks);
        $finish;
    end
    initial begin #300000000;$fatal(1,"bounded simulation timeout");end
endmodule
