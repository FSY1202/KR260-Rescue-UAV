`timescale 1ns/1ps
module tb_skin_stream;
    reg clk=0; always #5 clk=~clk;
    reg rstn=0, iv=0, il=0, ready=0;
    reg [31:0] id=0;
    wire ir, ov, ol;
    wire [31:0] od;
    wire [3:0] ok;
    integer sent=0, received=0, cycles=0, seed=2026;
    reg [31:0] expected[0:1023];
    reg expected_last[0:1023];
    reg stalled=0; reg [36:0] held;
    skin_stream dut(.aclk(clk),.aresetn(rstn),.bounds(32'had857f4d),.y_min(8'd40),
        .s_axis_tdata(id),.s_axis_tkeep(4'hf),.s_axis_tlast(il),.s_axis_tvalid(iv),.s_axis_tready(ir),
        .m_axis_tdata(od),.m_axis_tkeep(ok),.m_axis_tlast(ol),.m_axis_tvalid(ov),.m_axis_tready(ready));
    function [31:0] reference(input [31:0] pixel);
        integer r,g,b,y,cb,cr,mask;
        begin
            r=pixel[23:16];g=pixel[15:8];b=pixel[7:0];
            y=(77*r+150*g+29*b)>>8;
            cb=(32768-43*r-85*g+128*b)>>8;
            cr=(32768+128*r-107*g-21*b)>>8;
            mask=(y>=40 && cb>=77 && cb<=127 && cr>=133 && cr<=173)?255:0;
            reference=(y<<8)|mask;
        end
    endfunction
    initial begin repeat(4) @(negedge clk); rstn=1; end
    always @(negedge clk) if(rstn) begin
        ready <= ($random(seed)&7)!=0;
        if(!iv || ir) begin
            iv <= sent<1024 && (($random(seed)&3)!=0);
            id <= $random(seed);
            il <= (sent%257==256 || sent==1023);
        end
    end
    always @(posedge clk) if(rstn) begin
        cycles=cycles+1;
        if(stalled && (!ov || {ol,ok,od}!==held)) $fatal(1,"Output changed while stalled");
        stalled=ov && !ready; held={ol,ok,od};
        if(iv && ir) begin
            expected[sent]=reference(id); expected_last[sent]=il; sent=sent+1;
        end
        if(ov && ready) begin
            if(received>=sent || od!==expected[received] || ol!==expected_last[received] || ok!==4'hf)
                $fatal(1,"Mismatch at pixel %d",received);
            received=received+1;
        end
        if(received==1024) begin $display("PASS: 1024 pixels, randomized stalls, frame TLAST"); $finish; end
        if(cycles>10000) $fatal(1,"Timeout");
    end
endmodule
