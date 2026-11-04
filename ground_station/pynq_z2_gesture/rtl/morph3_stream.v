`timescale 1ns/1ps
// A padded raster makes OpenCV's neutral constant border explicit.
module morph3_stream #(parameter ERODE=1, MAX_WIDTH=384)(
 input wire clk,rstn,input wire [31:0] shape,
 input wire [7:0] sd,input wire sv,sl,output wire sr,
 output reg [7:0] md,output reg mv,ml,input wire mr);
 reg row1[0:MAX_WIDTH+1],row2[0:MAX_WIDTH+1];
 reg a0,a1,b0,b1,c0,c1;
 reg running;reg [15:0] x,y,w,h;
 wire room=!mv||mr;
 wire real_pixel=x>0&&x<=w&&y>0&&y<=h;
 wire value=real_pixel ? (sd!=0) : (ERODE!=0);
 wire advance=running&&room&&(!real_pixel||sv);
 wire window_valid=x>=2&&y>=2;
 wire result=ERODE ? (a0&a1&row2[x]&b0&b1&row1[x]&c0&c1&value)
                          : (a0|a1|row2[x]|b0|b1|row1[x]|c0|c1|value);
 assign sr=running&&real_pixel&&room;
 always @(posedge clk) begin
  if(!rstn) begin
   running<=0;x<=0;y<=0;w<=0;h<=0;mv<=0;ml<=0;md<=0;
   a0<=0;a1<=0;b0<=0;b1<=0;c0<=0;c1<=0;
  end else begin
   if(room) mv<=0;
   if(!running && sv && room && shape[15:0]>0 && shape[15:0]<=MAX_WIDTH && shape[31:16]>0) begin
    running<=1;x<=0;y<=0;w<=shape[15:0];h<=shape[31:16];
   end
   if(advance) begin
    row2[x]<=row1[x];row1[x]<=value;
    a0<=a1;a1<=row2[x];b0<=b1;b1<=row1[x];c0<=c1;c1<=value;
    if(window_valid) begin
     md<=result?8'hff:8'h00;mv<=1;ml<=x==w+1&&y==h+1;
    end
    if(x==w+1) begin
     x<=0;
     if(y==h+1) begin y<=0;running<=0;end else y<=y+1;
    end else x<=x+1;
   end
  end
 end
endmodule
