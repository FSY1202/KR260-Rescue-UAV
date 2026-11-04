`timescale 1ns/1ps
// BGRX input; output bytes: mask, luminance, 0, 0. TLAST ends a DMA frame.
module skin_stream (
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXIS:M_AXIS, ASSOCIATED_RESET aresetn" *)
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *) input wire aclk,
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *) input wire aresetn,
    input wire [31:0] bounds, // CrHi, CrLo, CbHi, CbLo
    input wire [7:0] y_min,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TDATA" *) input wire [31:0] s_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TKEEP" *) input wire [3:0] s_axis_tkeep,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TLAST" *) input wire s_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TVALID" *) input wire s_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TREADY" *) output wire s_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TDATA" *) output reg [31:0] m_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TKEEP" *) output reg [3:0] m_axis_tkeep,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TLAST" *) output reg m_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TVALID" *) output reg m_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TREADY" *) input wire m_axis_tready
);
    wire [7:0] b = s_axis_tdata[7:0], g = s_axis_tdata[15:8], r = s_axis_tdata[23:16];
    reg [15:0] y_sum, cb_sum, cr_sum;
    reg [31:0] bounds_q;
    reg [7:0] y_min_q;
    reg v1, last1;
    reg [3:0] keep1;
    wire ready2 = !m_axis_tvalid || m_axis_tready;
    assign s_axis_tready = aresetn && (!v1 || ready2);
    wire skin = y_sum[15:8] >= y_min_q &&
                cb_sum[15:8] >= bounds_q[7:0] && cb_sum[15:8] <= bounds_q[15:8] &&
                cr_sum[15:8] >= bounds_q[23:16] && cr_sum[15:8] <= bounds_q[31:24];
    always @(posedge aclk) begin
        if (!aresetn) begin
            v1 <= 0; m_axis_tvalid <= 0; m_axis_tdata <= 0;
            m_axis_tkeep <= 0; m_axis_tlast <= 0;
            y_sum <= 0; cb_sum <= 0; cr_sum <= 0; keep1 <= 0; last1 <= 0;
            bounds_q <= 0; y_min_q <= 0;
        end else begin
            if (ready2) begin
                m_axis_tvalid <= v1;
                if (v1) begin
                    m_axis_tdata <= {16'b0, y_sum[15:8], skin ? 8'hff : 8'h00};
                    m_axis_tkeep <= keep1;
                    m_axis_tlast <= last1;
                end
            end
            if (s_axis_tready) begin
                v1 <= s_axis_tvalid;
                if (s_axis_tvalid) begin
                    y_sum <= 77*r + 150*g + 29*b;
                    cb_sum <= 32768 - 43*r - 85*g + 128*b;
                    cr_sum <= 32768 + 128*r - 107*g - 21*b;
                    bounds_q <= bounds; y_min_q <= y_min;
                    keep1 <= s_axis_tkeep; last1 <= s_axis_tlast;
                end
            end
        end
    end
endmodule
