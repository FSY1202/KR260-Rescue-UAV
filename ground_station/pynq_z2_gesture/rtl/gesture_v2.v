`timescale 1ns/1ps
module gesture_pipeline_v2(
 (* X_INTERFACE_PARAMETER="ASSOCIATED_BUSIF S_AXIS:M_AXIS, ASSOCIATED_RESET aresetn" *)
 (* X_INTERFACE_INFO="xilinx.com:signal:clock:1.0 aclk CLK" *) input wire aclk,
 (* X_INTERFACE_PARAMETER="POLARITY ACTIVE_LOW" *)
 (* X_INTERFACE_INFO="xilinx.com:signal:reset:1.0 aresetn RST" *) input wire aresetn,
 input wire [31:0] bounds,input wire [7:0] y_min,input wire [31:0] shape,output wire [31:0] ip_version,
 (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 S_AXIS TDATA" *) input wire [31:0] s_axis_tdata,
 (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 S_AXIS TKEEP" *) input wire [3:0] s_axis_tkeep,
 (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 S_AXIS TVALID" *) input wire s_axis_tvalid,
 (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 S_AXIS TREADY" *) output wire s_axis_tready,
 (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 S_AXIS TLAST" *) input wire s_axis_tlast,
 (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 M_AXIS TDATA" *) output wire [7:0] m_axis_tdata,
 (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 M_AXIS TKEEP" *) output wire m_axis_tkeep,
 (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 M_AXIS TVALID" *) output wire m_axis_tvalid,
 (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 M_AXIS TREADY" *) input wire m_axis_tready,
 (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 M_AXIS TLAST" *) output wire m_axis_tlast);
 assign ip_version=32'h47535432;assign m_axis_tkeep=1'b1;
 reg [31:0] word_q,bounds_q;reg [7:0] ymin_q;reg full,phase,last_q,scaled_q;
 reg [15:0] pair_index,input_row;
 wire scaled=shape[31];wire [31:0] output_shape={1'b0,shape[30:0]};
 wire [7:0] luminance=phase?word_q[23:16]:word_q[7:0];
 wire skin=luminance>=ymin_q&&word_q[15:8]>=bounds_q[7:0]&&word_q[15:8]<=bounds_q[15:8]&&
           word_q[31:24]>=bounds_q[23:16]&&word_q[31:24]<=bounds_q[31:24];
 wire [7:0] d[0:6];wire [6:0] v,l,r;
 assign d[0]=skin?8'hff:0;assign v[0]=full;assign l[0]=last_q&&(phase||scaled_q);
 assign s_axis_tready=aresetn&&(!full||((phase||scaled_q)&&r[0]));
 always @(posedge aclk) begin
  if(!aresetn) begin full<=0;phase<=0;last_q<=0;word_q<=0;bounds_q<=0;ymin_q<=0;scaled_q<=0;pair_index<=0;input_row<=0;end
  else begin
   if(full&&r[0]) begin if(phase||scaled_q) begin full<=0;phase<=0;end else phase<=1;end
   if(s_axis_tvalid&&s_axis_tready) begin
    word_q<=s_axis_tdata;full<=!scaled||!input_row[0];phase<=0;scaled_q<=scaled;
    last_q<=scaled?(pair_index==shape[15:0]-1&&input_row==({1'b0,shape[30:16]}*2-2)):s_axis_tlast;
    bounds_q<=bounds;ymin_q<=y_min;
    if(s_axis_tlast)begin pair_index<=0;input_row<=0;end
    else if(scaled)begin
     if(pair_index==shape[15:0]-1)begin pair_index<=0;input_row<=input_row+1;end
     else pair_index<=pair_index+1;
    end
   end
  end
 end
 genvar i;
 generate for(i=0;i<6;i=i+1) begin: morphology
  // Open: E,D. Close(iterations=2): D,D,E,E.
  morph3_stream #(.ERODE(i==0||i==4||i==5)) stage(
   .clk(aclk),.rstn(aresetn),.shape(output_shape),.sd(d[i]),.sv(v[i]),.sl(l[i]),.sr(r[i]),
   .md(d[i+1]),.mv(v[i+1]),.ml(l[i+1]),.mr(r[i+1]));
 end endgenerate
 assign m_axis_tdata=d[6];assign m_axis_tvalid=v[6];assign m_axis_tlast=l[6];assign r[6]=m_axis_tready;
endmodule
