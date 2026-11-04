// Fixed 800x600@60.317Hz HDMI raster, 100MHz AXIS input / 40MHz pixels.
// The framebuffer is XRGB8888; SOF is the AXI VDMA TUSER bit.
module hdmi_scanout (
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXIS, ASSOCIATED_RESET aresetn" *)
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *) input wire aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *) input wire aresetn,
    input wire pixel_clk,
    input wire pixel_resetn,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TDATA" *) input wire [31:0] s_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TVALID" *) input wire s_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TREADY" *) output wire s_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TUSER" *) input wire s_axis_tuser,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TLAST" *) input wire s_axis_tlast,
    output reg [23:0] rgb,
    output reg de,
    output reg hsync,
    output reg vsync
);
wire full, empty, wr_busy, rd_busy;
wire [32:0] head;
wire sof = head[32];
reg [10:0] x = 0;
reg [9:0] y = 0;
reg frame_valid = 0;
wire active = x < 800 && y < 600;
wire first = x == 0 && y == 0;
wire unavailable = empty || rd_busy;
wire available_frame = first ? (!unavailable && sof) : frame_valid;
wire unexpected_sof = !first && active && available_frame && sof;
wire take = active && available_frame && !unavailable && !unexpected_sof;
wire discard = !available_frame && !unavailable && !sof;
assign s_axis_tready = aresetn && !full && !wr_busy;
xpm_fifo_async #(
    .FIFO_MEMORY_TYPE("block"), .FIFO_WRITE_DEPTH(4096),
    .WRITE_DATA_WIDTH(33), .READ_DATA_WIDTH(33),
    .READ_MODE("fwft"), .FIFO_READ_LATENCY(0),
    .CDC_SYNC_STAGES(2), .DOUT_RESET_VALUE("0"),
    .FULL_RESET_VALUE(0), .ECC_MODE("no_ecc"), .RELATED_CLOCKS(0),
    .USE_ADV_FEATURES("0000"), .RD_DATA_COUNT_WIDTH(13), .WR_DATA_COUNT_WIDTH(13),
    .PROG_EMPTY_THRESH(10), .PROG_FULL_THRESH(4080), .WAKEUP_TIME(0)
) pixels (
    .rst(!aresetn), .wr_clk(aclk), .rd_clk(pixel_clk),
    .din({s_axis_tuser,s_axis_tdata}), .wr_en(s_axis_tvalid && s_axis_tready),
    .full(full), .wr_rst_busy(wr_busy), .dout(head), .empty(empty),
    .rd_en(take || discard), .rd_rst_busy(rd_busy), .sleep(1'b0),
    .injectsbiterr(1'b0), .injectdbiterr(1'b0)
);
always @(posedge pixel_clk) begin
    if (!pixel_resetn) begin
        x <= 0; y <= 0; frame_valid <= 0;
        rgb <= 0; de <= 0; hsync <= 0; vsync <= 0;
    end else begin
        de <= active;
        hsync <= x >= 840 && x < 968;
        vsync <= y >= 601 && y < 605;
        // Digilent's parallel RGB bus is R:B:G, while XRGB8888 is R:G:B.
        rgb <= take ? {head[23:16],head[7:0],head[15:8]} : 24'b0;
        if (first)
            frame_valid <= !unavailable && sof;
        else if (active && available_frame && (unavailable || unexpected_sof))
            frame_valid <= 0;
        if (x == 1055) begin
            x <= 0;
            y <= y == 627 ? 0 : y + 1'b1;
        end else x <= x + 1'b1;
    end
end
endmodule
