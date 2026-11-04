`timescale 1ns/1ps
module tb_gesture_v2;
 reg clk=0;always #5 clk=~clk;
 reg rstn=0,sv=0,sl=0,mr=0;reg [31:0] sd=0,shape;
 wire sr,mv,ml,mk;wire [7:0] md;wire [31:0] version;
 integer seed=7020,w,h,n,iw,ih,nin,scaled,sent,got,cycles,f,i,j,k,xx,yy,p,stage;
 reg a[0:147455],b[0:147455];reg mask[0:147455];reg result;
 reg held_valid=0,accepted=0;reg [8:0] held;
 gesture_pipeline_v2 dut(.aclk(clk),.aresetn(rstn),.bounds(32'had857f4d),.y_min(8'd40),.shape(shape),.ip_version(version),
  .s_axis_tdata(sd),.s_axis_tkeep(4'hf),.s_axis_tvalid(sv),.s_axis_tready(sr),.s_axis_tlast(sl),
  .m_axis_tdata(md),.m_axis_tkeep(mk),.m_axis_tvalid(mv),.m_axis_tready(mr),.m_axis_tlast(ml));
 function [31:0] pair(input integer idx);
  pair={8'd150,mask[idx+1]?8'd100:8'd0,8'd100,mask[idx]?8'd100:8'd0};
 endfunction
 always @(posedge clk) if(rstn) begin
  if(held_valid&&(!mv||{ml,md}!==held)) $fatal(1,"Stalled output changed");
  held_valid=mv&&!mr;held={ml,md};
 end
 initial begin
  shape=0;repeat(4) @(negedge clk);rstn=1;
  for(f=0;f<12;f=f+1) begin
   w=(f==11)?192:((f%3==0)?4:((f%3==1)?10:32));h=(f==11)?192:(f%4+3);
   scaled=(f>=10);iw=w*(scaled?2:1);ih=h*(scaled?2:1);nin=iw*ih;
   n=w*h;shape=(h<<16)|w|(scaled?32'h80000000:0);
   for(i=0;i<nin;i=i+1) begin
    if(f%4==0)mask[i]=1;
    else if(f%4==1)mask[i]=0;
    else if(f%4==2)mask[i]=((i%iw)>1&&(i%iw)<iw-2);
    else mask[i]=($random(seed)&7)!=0;
   end
   for(i=0;i<n;i=i+1)a[i]=mask[scaled?((i/w*2)*iw+i%w*2):i];
   for(stage=0;stage<6;stage=stage+1) begin
    for(i=0;i<n;i=i+1) begin
     result=(stage==0||stage==4||stage==5);
     for(j=-1;j<=1;j=j+1)for(k=-1;k<=1;k=k+1)begin
      xx=i%w+k;yy=i/w+j;
      if(xx>=0&&xx<w&&yy>=0&&yy<h)begin
       if(stage==0||stage==4||stage==5)result=result&a[yy*w+xx];else result=result|a[yy*w+xx];
      end
     end
     b[i]=result;
    end
    for(i=0;i<n;i=i+1)a[i]=b[i];
   end
   sent=0;got=0;cycles=0;sv=0;accepted=0;
   while(got<n)begin
    @(negedge clk);mr=($random(seed)&3)!=0;
    if(!sv||accepted)begin sv=sent<nin/2&&(($random(seed)&3)!=0);sd=pair(sent*2);sl=sent==nin/2-1;end
    @(posedge clk);
    accepted=sv&&sr;if(accepted)sent=sent+1;
    if(mv&&mr)begin
     if(md!==(a[got]?8'hff:8'h00)||ml!==(got==n-1)||mk!==1'b1)$fatal(1,"Frame %d pixel %d mismatch: got %h expected %h",f,got,md,a[got]?8'hff:8'h00);
     got=got+1;
    end
    cycles=cycles+1;if(cycles>n*50+20000)$fatal(1,"Frame timeout");
   end
   @(negedge clk);sv=0;mr=1;repeat(4)@(negedge clk);
  end
  $display("PASS V2: 12 frames, six operators, borders, stalls, TLAST, dimension changes");$finish;
 end
endmodule
