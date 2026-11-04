`timescale 1ns/1ps
// Independent DVP stimulus: every camera frame contains its own number in the
// high byte and the row-major pixel index in the low byte. This exposes tears,
// old banks, wrong bank selection, byte order, and incomplete frame acceptance.
module tb_frame_stream;
    logic clk=0,pclk=0,resetn=0;
    always #5 clk=~clk;
    always #12 pclk=~pclk;
    logic [7:0] data=0;
    logic href=0,vsync=1;
    logic [19:0] awaddr=0,araddr=0;
    logic [31:0] wdata=0;
    logic [3:0] wstrb=4'hf;
    logic awvalid=0,wvalid=0,bready=1,arvalid=0,rready=0;
    wire awready,wready,bvalid,arready,rvalid;
    wire [1:0] bresp,rresp;
    wire [31:0] rdata;
    integer mode=0,frame_number=0;
    // 0=normal, 1=short image, 2=no frames but PCLK continues.
    frame_capture_axi #(.WIDTH(8),.HEIGHT(4),.TIMEOUT_CYCLES(100000)) dut(
        .cam_pclk(pclk),.cam_data(data),.cam_href(href),.cam_vsync(vsync),
        .s_axi_aclk(clk),.s_axi_aresetn(resetn),
        .s_axi_awaddr(awaddr),.s_axi_awprot(3'b0),.s_axi_awvalid(awvalid),.s_axi_awready(awready),
        .s_axi_wdata(wdata),.s_axi_wstrb(wstrb),.s_axi_wvalid(wvalid),.s_axi_wready(wready),
        .s_axi_bresp(bresp),.s_axi_bvalid(bvalid),.s_axi_bready(bready),
        .s_axi_araddr(araddr),.s_axi_arprot(3'b0),.s_axi_arvalid(arvalid),.s_axi_arready(arready),
        .s_axi_rdata(rdata),.s_axi_rresp(rresp),.s_axi_rvalid(rvalid),.s_axi_rready(rready));

    task byte_out(input [7:0] value);
        data=value;
        @(posedge pclk); @(negedge pclk);
    endtask

    initial begin : camera
        integer row,col,frame_mode;
        forever begin
            @(negedge pclk); vsync=1; href=0;
            repeat(18) @(negedge pclk);
            frame_mode=mode;
            if(frame_mode!=2) begin
                frame_number=frame_number+1;
                vsync=0;
                repeat(6) @(negedge pclk);
                for(row=0;row<(frame_mode==1?3:4);row=row+1) begin
                    @(negedge pclk); href=1;
                    for(col=0;col<8;col=col+1) begin
                        byte_out(frame_number[7:0]);
                        byte_out(row*8+col);
                    end
                    href=0;
                    repeat(8) @(negedge pclk);
                end
                repeat(8) @(negedge pclk);
            end
        end
    end

    // order0: AW precedes W; order1: W precedes AW; order2: simultaneous.
    task write_command(input [31:0] value,input integer order,input [1:0] response);
        fork
            begin
                if(order==1) repeat(4) @(negedge clk);
                @(negedge clk); awaddr=8; awvalid=1;
                do @(posedge clk); while(!awready);
                @(negedge clk); awvalid=0;
            end
            begin
                if(order==0) repeat(4) @(negedge clk);
                @(negedge clk); wdata=value; wvalid=1;
                do @(posedge clk); while(!wready);
                @(negedge clk); wvalid=0;
            end
        join
        wait(bvalid); #1;
        if(bresp!==response)
            $fatal(1,"Write command %h response %b expected %b",value,bresp,response);
        @(negedge clk);
    endtask

    task read_response(input [19:0] address,output [31:0] value,
                       input integer stall,input [1:0] response);
        logic [31:0] held;
        logic [1:0] held_resp;
        @(negedge clk); araddr=address; arvalid=1; rready=0;
        do @(posedge clk); while(!arready);
        @(negedge clk); arvalid=0;
        wait(rvalid); #1; held=rdata; held_resp=rresp;
        if(rresp!==response) $fatal(1,"Read %h response %b expected %b",address,rresp,response);
        repeat(stall) begin
            @(posedge clk); #1;
            if(!rvalid || rdata!==held || rresp!==held_resp)
                $fatal(1,"Read payload changed under backpressure at %h",address);
        end
        value=held;
        @(negedge clk); rready=1;
        @(posedge clk); @(negedge clk); rready=0;
    endtask

    task read_word(input [19:0] address,output [31:0] value,input integer stall);
        read_response(address,value,stall,0);
    endtask

    task wait_latest_after(input [31:0] prior_sequence,output [31:0] current);
        logic [31:0] status;
        integer tries;
        tries=0; current=prior_sequence;
        while(current<=prior_sequence && tries<50000) begin
            read_word(12,status,0);
            read_word(20'h3c,current,0);
            if(!status[3]) current=prior_sequence;
            tries=tries+1;
            repeat(5) @(posedge clk);
        end
        if(current<=prior_sequence) $fatal(1,"No new valid latest frame after %d",prior_sequence);
    endtask

    task wait_errors_after(input [31:0] prior_errors,output [31:0] current);
        integer tries;
        tries=0; current=prior_errors;
        while(current<=prior_errors && tries<50000) begin
            read_word(20'h18,current,0); tries=tries+1;
            repeat(5) @(posedge clk);
        end
        if(current<=prior_errors) $fatal(1,"Expected visible capture error was not reported");
    endtask

    task check_image(output [31:0] first_word);
        logic [31:0] word_value;
        integer j,number;
        read_word(20'h01000,first_word,7); number=first_word[15:8];
        for(j=0;j<16;j=j+1) begin
            read_word(20'h01000+j*4,word_value,j%4);
            if(word_value[15:0]!==((number<<8)|(j*2)) ||
               word_value[31:16]!==((number<<8)|(j*2+1)))
                $fatal(1,"Mixed/incorrect pixel pair %d: %h camera frame %d",j,word_value,number);
        end
    endtask

    task claim_latest(output [31:0] claimed_sequence,output [31:0] first_word);
        logic [31:0] value;
        write_command(8,1,0);
        read_word(12,value,0);
        if(!value[0] || !value[1]) $fatal(1,"Claim did not freeze a frame: status %h",value);
        read_word(20'h10,claimed_sequence,0);
        read_word(20'h38,value,0); if(value>2) $fatal(1,"Invalid bank number %h",value);
        read_word(20'h20,value,0); if(value!=32) $fatal(1,"Claim has wrong pixel count");
        check_image(first_word);
    endtask

    initial begin : test
        logic [31:0] value,status,seq,seq2,latest,first_word,saved_word,again;
        logic [31:0] errors_before,errors_after,overflow_before,overflow_after;
        integer edge_case,held_camera_number;
        repeat(100) @(posedge clk); @(negedge clk); resetn=1;
        read_word(0,value,0); if(value!==32'h46524d32) $fatal(1,"Wrong FRM2 ID %h",value);
        read_word(4,value,0); if(value!==32'h00020000) $fatal(1,"Wrong FRM2 version");
        read_word(20'h14,value,0); if(value!=64) $fatal(1,"Wrong frame byte count");
        read_word(20'h24,value,0); if(value!==32'h00040008) $fatal(1,"Wrong geometry");
        read_response(20'h01000,value,4,2);
        write_command(8,0,2); // Cannot claim an absent frame.
        write_command(2,1,0); // An empty release is a no-op.
        write_command(1,0,0);
        wait_latest_after(0,latest);
        write_command(1,1,2); // Duplicate start never resets/restarts the stream.
        claim_latest(seq,saved_word);
        write_command(8,0,2); // Cannot acquire two banks with one ownership token.

        // More than three camera frames pass while the claimed bank is frozen.
        held_camera_number=frame_number;
        wait(frame_number>=held_camera_number+6);
        read_word(20'h10,value,0); if(value!==seq) $fatal(1,"Claimed sequence changed");
        check_image(again); if(again!==saved_word) $fatal(1,"Claimed bank overwritten");
        read_word(20'h30,value,0); if(value<=seq+2) $fatal(1,"Capture stopped while PS held bank");
        read_word(20'h34,value,0); if(value==0) $fatal(1,"Discard/replacement counter never advanced");
        read_word(20'h18,value,0); if(value!=0) $fatal(1,"Valid continuous frames raised an error");
        read_word(20'h1c,value,0); if(value!=0) $fatal(1,"Valid continuous frames overflowed");

        // In-flight read must pin the bank, including commands on independent AW/W.
        fork
            begin read_word(20'h01000,again,80); end
            begin
                wait(rvalid);
                write_command(2,0,2);
                write_command(4,1,2);
            end
        join
        if(again!==saved_word) $fatal(1,"Concurrent release corrupted stalled read");
        read_word(12,status,0); if(!status[1]) $fatal(1,"Rejected release dropped ownership");
        write_command(2,2,0);
        read_response(20'h01000,value,1,2);
        read_word(20'h38,value,0); if(value!==32'hffffffff) $fatal(1,"Unclaimed bank exposed");
        wait_latest_after(seq+2,latest);
        claim_latest(seq2,first_word);
        if(seq2<=seq+2) $fatal(1,"Claim returned old queue instead of recent frame");

        // A malformed incoming frame cannot invalidate or modify an existing claim.
        mode=1;
        read_word(20'h18,errors_before,0);
        wait_errors_after(errors_before,errors_after);
        read_word(20'h28,value,0); if(value!=1 && value!=2) $fatal(1,"Wrong malformed-frame diagnostic %d",value);
        read_word(12,status,0); if(!status[1] || !status[2]) $fatal(1,"Geometry fault lost claim/error");
        check_image(again); if(again!==first_word) $fatal(1,"Malformed frame damaged claim");
        // Any in-progress normal frame has finished by the first short-frame
        // error. Additional short camera frames must not become successful frames.
        read_word(20'h30,latest,0);
        held_camera_number=frame_number;
        wait(frame_number>=held_camera_number+4);
        read_word(20'h30,value,0);
        if(value!==latest) $fatal(1,"Short image incorrectly advanced successful frame sequence");
        mode=0;
        read_word(20'h30,latest,0); wait_latest_after(latest,value);

        // FIFO loss is injected into a different bank; it must remain visible.
        read_word(20'h18,errors_before,0);
        read_word(20'h1c,overflow_before,0);
        wait(dut.source_i.capturing && href);
        force dut.fifo_full=1'b1;
        repeat(6) @(posedge pclk);
        release dut.fifo_full;
        wait_errors_after(errors_before,errors_after);
        read_word(20'h1c,overflow_after,0);
        if(overflow_after<=overflow_before) $fatal(1,"FIFO loss counter did not advance");
        check_image(again); if(again!==first_word) $fatal(1,"FIFO loss damaged claim");

        // Stop supplying SOF while PCLK keeps running. Recovery must be automatic.
        mode=2;
        @(posedge vsync); repeat(40) @(negedge pclk);
        read_word(20'h18,errors_before,0);
        wait_errors_after(errors_before,errors_after);
        read_word(20'h28,value,0); if(value!=3) $fatal(1,"Missing SOF did not report timeout");
        read_word(12,status,0); if(!status[0] || !status[1]) $fatal(1,"Timeout stopped stream or lost claim");
        check_image(again); if(again!==first_word) $fatal(1,"Timeout damaged claim");
        mode=0;
        read_word(20'h30,latest,0); wait_latest_after(latest,value);
        check_image(again); if(again!==first_word) $fatal(1,"Recovery reused claimed bank");
        write_command(2,1,0);

        // Exercise release/claim close to EOF repeatedly, with every AXI ordering.
        for(edge_case=0;edge_case<12;edge_case=edge_case+1) begin
            wait_latest_after(0,latest);
            @(posedge vsync);
            write_command(8,edge_case%3,0);
            read_word(20'h10,seq,0);
            check_image(first_word);
            @(posedge vsync);
            write_command(2,(edge_case+1)%3,0);
            read_response(20'h01000,value,0,2);
        end

        // Abort invalidates all exposed ownership; restart begins a clean stream.
        wait_latest_after(0,latest);
        claim_latest(seq,first_word);
        write_command(4,2,0);
        read_word(12,status,0); if(status[1:0]!=0 || status[3]) $fatal(1,"Abort left active/claimed/latest");
        read_response(20'h01000,value,0,2);
        write_command(8,1,2);
        write_command(1,1,0);
        wait_latest_after(0,latest);
        claim_latest(seq,first_word);
        write_command(2,0,0);
        write_command(4,0,0);
        $display("FRAME_STREAM_TEST_PASS: continuous triple-bank ownership, complete pixels, latest claim, AXI split channels/backpressure, geometry/FIFO/timeout fault preservation, recovery, boundary operations, abort/restart");
        $finish;
    end
    initial begin #30000000; $fatal(1,"Stream simulation watchdog timeout"); end
endmodule
