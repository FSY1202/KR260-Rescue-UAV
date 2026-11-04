`timescale 1ns/1ps
module tb_frame_capture;
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
    frame_capture_axi #(.WIDTH(8),.HEIGHT(4),.TIMEOUT_CYCLES(250000)) dut(
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
        integer row,col;
        forever begin
            @(negedge pclk); vsync=1; href=0;
            repeat(12) @(negedge pclk);
            if(mode!=2) begin
                frame_number=frame_number+1;
                vsync=0;
                repeat(4) @(negedge pclk);
                for(row=0;row<(mode==1?3:4);row=row+1) begin
                    @(negedge pclk); href=1;
                    for(col=0;col<8;col=col+1) begin
                        byte_out(frame_number[7:0]);
                        byte_out(row*8+col);
                    end
                    href=0;
                    repeat(6) @(negedge pclk);
                end
                repeat(5) @(negedge pclk);
            end
        end
    end

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
        if(bresp!==response) $fatal(1,"AXI write response %b expected %b",bresp,response);
        @(negedge clk);
    endtask

    task read_word(input [19:0] address,output [31:0] value,input integer stall);
        logic [31:0] held;
        @(negedge clk); araddr=address; arvalid=1; rready=0;
        do @(posedge clk); while(!arready);
        @(negedge clk); arvalid=0;
        wait(rvalid); #1; held=rdata;
        if(rresp!==0) $fatal(1,"AXI read SLVERR at %h",address);
        repeat(stall) begin
            @(posedge clk); #1;
            if(!rvalid || rdata!==held) $fatal(1,"RDATA not held under backpressure");
        end
        value=held;
        @(negedge clk); rready=1;
        @(posedge clk); @(negedge clk); rready=0;
    endtask

    task terminal(output [31:0] status);
        integer tries;
        tries=0; status=1;
        while(status[0] && tries<40000) begin
            read_word(12,status,0); tries=tries+1;
            repeat(5) @(posedge clk);
        end
        if(status[0]) $fatal(1,"Capture never completed");
    endtask

    task check_image;
        logic [31:0] word_value,first_word;
        integer j,number;
        read_word(20'h01000,first_word,7); number=first_word[15:8];
        for(j=0;j<16;j=j+1) begin
            read_word(20'h01000+j*4,word_value,j%3);
            if(word_value[15:0]!==((number<<8)|(j*2)) ||
               word_value[31:16]!==((number<<8)|(j*2+1)))
                $fatal(1,"Mixed/incorrect pixels at %d: %h frame %d",j,word_value,number);
        end
    endtask

    initial begin : test
        logic [31:0] value,status,saved,again;
        repeat(100) @(posedge clk); @(negedge clk); resetn=1;
        read_word(0,value,0); if(value!=32'h46524d31) $fatal(1,"Wrong frame ID");
        write_command(1,0,0); terminal(status);
        if(!status[1] || status[2]) $fatal(1,"Normal capture failed %h",status);
        check_image(); read_word(20'h01000,saved,0);
        // Frozen frame cannot be overwritten by a second start.
        write_command(1,1,2); repeat(2000) @(posedge clk);
        read_word(20'h01000,again,0); if(again!==saved) $fatal(1,"Frozen frame overwritten");
        write_command(2,1,0); mode=1;
        write_command(1,1,0); terminal(status);
        if(!status[2] || status[1]) $fatal(1,"Truncated frame was accepted");
        write_command(4,0,0); mode=0;
        write_command(1,0,0);
        wait(dut.source_i.capturing);
        force dut.fifo_full=1'b1; repeat(5) @(posedge pclk); release dut.fifo_full;
        terminal(status);
        if(!status[2] || status[1]) $fatal(1,"FIFO loss not rejected");
        write_command(4,1,0); mode=2;
        write_command(1,1,0); terminal(status);
        read_word(40,value,0);
        if(!status[2] || status[1] || value!=3) $fatal(1,"Missing frame did not time out");
        write_command(4,0,0); mode=0;
        write_command(1,1,0); terminal(status);
        if(!status[1] || status[2]) $fatal(1,"Recovery capture failed");
        check_image();
        $display("FRAME_CAPTURE_TEST_PASS: full pixels, frozen ownership, split AXI channels, read stalls, geometry, FIFO loss, timeout, recovery");
        $finish;
    end
    initial begin #10000000; $fatal(1,"Test watchdog timeout"); end
endmodule
