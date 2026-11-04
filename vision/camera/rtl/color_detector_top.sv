`timescale 1ns/1ps

module color_detector_top #(
    parameter integer C_S_AXI_ADDR_WIDTH = 8,
    parameter integer C_S_AXI_DATA_WIDTH = 32
) (
    input  wire cam_pclk,
    input  wire [7:0] cam_data,
    input  wire cam_href,
    input  wire cam_vsync,
    output wire cam_reset_n,
    output wire cam_pwdn,
    output wire irq,

    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 S_AXI_ACLK CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXI, ASSOCIATED_RESET S_AXI_ARESETN, FREQ_HZ 100000000" *)
    input  wire s_axi_aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 S_AXI_ARESETN RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire s_axi_aresetn,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWADDR" *) input  wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWPROT" *) input  wire [2:0] s_axi_awprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWVALID" *) input  wire s_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWREADY" *) output logic s_axi_awready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WDATA" *) input  wire [C_S_AXI_DATA_WIDTH-1:0] s_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WSTRB" *) input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0] s_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WVALID" *) input  wire s_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WREADY" *) output logic s_axi_wready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BRESP" *) output logic [1:0] s_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BVALID" *) output logic s_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BREADY" *) input  wire s_axi_bready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARADDR" *) input  wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARPROT" *) input  wire [2:0] s_axi_arprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARVALID" *) input  wire s_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARREADY" *) output logic s_axi_arready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RDATA" *) output logic [C_S_AXI_DATA_WIDTH-1:0] s_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RRESP" *) output logic [1:0] s_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RVALID" *) output logic s_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RREADY" *) input  wire s_axi_rready
);

    localparam [31:0] CORE_ID = 32'h434f4c31; // "COL1"
    localparam [31:0] VERSION = 32'h0001_0000;

    logic [3:0] control_reg;
    logic cfg_toggle;
    logic [23:0] red_cfg_reg, green_cfg_reg, blue_cfg_reg;
    logic [31:0] red_min_count_reg, green_min_count_reg, blue_min_count_reg;
    logic [15:0] sample_x_reg, sample_y_reg;
    logic irq_pending, snapshot_valid;
    logic [31:0] dropped_frames;

    wire detector_enable = control_reg[0];
    assign cam_reset_n = control_reg[1];
    assign cam_pwdn = control_reg[2];
    assign irq = irq_pending && control_reg[3];

    logic frame_toggle_pclk;
    logic frame_active_pclk;
    logic [31:0] frame_seq_pclk, sync_errors_pclk;
    logic [15:0] width_pclk, height_pclk, sample_rgb_pclk;
    logic [31:0] rc_pclk, rsx_pclk, rsy_pclk;
    logic [15:0] rxmin_pclk, rxmax_pclk, rymin_pclk, rymax_pclk;
    logic [31:0] gc_pclk, gsx_pclk, gsy_pclk;
    logic [15:0] gxmin_pclk, gxmax_pclk, gymin_pclk, gymax_pclk;
    logic [31:0] bc_pclk, bsx_pclk, bsy_pclk;
    logic [15:0] bxmin_pclk, bxmax_pclk, bymin_pclk, bymax_pclk;

    logic [31:0] frame_seq_reg, sync_errors_reg;
    logic [15:0] width_reg, height_reg, sample_rgb_reg;
    logic [31:0] rc_reg, rsx_reg, rsy_reg;
    logic [15:0] rxmin_reg, rxmax_reg, rymin_reg, rymax_reg;
    logic [31:0] gc_reg, gsx_reg, gsy_reg;
    logic [15:0] gxmin_reg, gxmax_reg, gymin_reg, gymax_reg;
    logic [31:0] bc_reg, bsx_reg, bsy_reg;
    logic [15:0] bxmin_reg, bxmax_reg, bymin_reg, bymax_reg;

    color_detector_core core_i (
        .pclk(cam_pclk), .resetn(s_axi_aresetn), .enable_async(detector_enable),
        .cam_data(cam_data), .cam_href(cam_href), .cam_vsync(cam_vsync),
        .cfg_toggle_async(cfg_toggle),
        .red_cfg_async(red_cfg_reg), .red_min_count_async(red_min_count_reg),
        .green_cfg_async(green_cfg_reg), .green_min_count_async(green_min_count_reg),
        .blue_cfg_async(blue_cfg_reg), .blue_min_count_async(blue_min_count_reg),
        .sample_x_async(sample_x_reg), .sample_y_async(sample_y_reg),
        .frame_toggle(frame_toggle_pclk), .frame_active(frame_active_pclk),
        .frame_seq_snap(frame_seq_pclk), .width_snap(width_pclk), .height_snap(height_pclk),
        .sync_errors_snap(sync_errors_pclk), .sample_rgb565_snap(sample_rgb_pclk),
        .red_count_snap(rc_pclk), .red_sum_x_snap(rsx_pclk), .red_sum_y_snap(rsy_pclk),
        .red_xmin_snap(rxmin_pclk), .red_xmax_snap(rxmax_pclk), .red_ymin_snap(rymin_pclk), .red_ymax_snap(rymax_pclk),
        .green_count_snap(gc_pclk), .green_sum_x_snap(gsx_pclk), .green_sum_y_snap(gsy_pclk),
        .green_xmin_snap(gxmin_pclk), .green_xmax_snap(gxmax_pclk), .green_ymin_snap(gymin_pclk), .green_ymax_snap(gymax_pclk),
        .blue_count_snap(bc_pclk), .blue_sum_x_snap(bsx_pclk), .blue_sum_y_snap(bsy_pclk),
        .blue_xmin_snap(bxmin_pclk), .blue_xmax_snap(bxmax_pclk), .blue_ymin_snap(bymin_pclk), .blue_ymax_snap(bymax_pclk)
    );

    (* ASYNC_REG = "TRUE" *) logic frame_meta, frame_sync;
    (* ASYNC_REG = "TRUE" *) logic active_meta, active_sync;
    logic frame_sync_d, capture_pending;

    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            frame_meta <= 0;
            frame_sync <= 0;
            frame_sync_d <= 0;
            capture_pending <= 0;
            active_meta <= 0;
            active_sync <= 0;
            irq_pending <= 0;
            snapshot_valid <= 0;
            dropped_frames <= 0;
            frame_seq_reg <= 0;
            width_reg <= 0;
            height_reg <= 0;
            sync_errors_reg <= 0;
            sample_rgb_reg <= 0;
            rc_reg <= 0; rsx_reg <= 0; rsy_reg <= 0;
            rxmin_reg <= 0; rxmax_reg <= 0; rymin_reg <= 0; rymax_reg <= 0;
            gc_reg <= 0; gsx_reg <= 0; gsy_reg <= 0;
            gxmin_reg <= 0; gxmax_reg <= 0; gymin_reg <= 0; gymax_reg <= 0;
            bc_reg <= 0; bsx_reg <= 0; bsy_reg <= 0;
            bxmin_reg <= 0; bxmax_reg <= 0; bymin_reg <= 0; bymax_reg <= 0;
        end else begin
            frame_meta <= frame_toggle_pclk;
            frame_sync <= frame_meta;
            frame_sync_d <= frame_sync;
            active_meta <= frame_active_pclk;
            active_sync <= active_meta;

            if (frame_sync != frame_sync_d)
                capture_pending <= 1;

            if (capture_pending) begin
                capture_pending <= 0;
                if (irq_pending)
                    dropped_frames <= dropped_frames + 1;
                irq_pending <= 1;
                snapshot_valid <= 1;
                frame_seq_reg <= frame_seq_pclk;
                width_reg <= width_pclk;
                height_reg <= height_pclk;
                sync_errors_reg <= sync_errors_pclk;
                sample_rgb_reg <= sample_rgb_pclk;
                rc_reg <= rc_pclk; rsx_reg <= rsx_pclk; rsy_reg <= rsy_pclk;
                rxmin_reg <= rxmin_pclk; rxmax_reg <= rxmax_pclk; rymin_reg <= rymin_pclk; rymax_reg <= rymax_pclk;
                gc_reg <= gc_pclk; gsx_reg <= gsx_pclk; gsy_reg <= gsy_pclk;
                gxmin_reg <= gxmin_pclk; gxmax_reg <= gxmax_pclk; gymin_reg <= gymin_pclk; gymax_reg <= gymax_pclk;
                bc_reg <= bc_pclk; bsx_reg <= bsx_pclk; bsy_reg <= bsy_pclk;
                bxmin_reg <= bxmin_pclk; bxmax_reg <= bxmax_pclk; bymin_reg <= bymin_pclk; bymax_reg <= bymax_pclk;
            end

            if (s_axi_awready && s_axi_awvalid && s_axi_wready && s_axi_wvalid && (s_axi_awaddr[7:0] == 8'h8c) && s_axi_wdata[0])
                irq_pending <= 0;
        end
    end

    function automatic [31:0] apply_strobes(
        input [31:0] old_value,
        input [31:0] new_value,
        input [3:0] strobes
    );
        integer i;
        begin
            apply_strobes = old_value;
            for (i = 0; i < 4; i = i + 1)
                if (strobes[i])
                    apply_strobes[i*8 +: 8] = new_value[i*8 +: 8];
        end
    endfunction

    always_ff @(posedge s_axi_aclk) begin : axi_write
        logic [31:0] merged;
        if (!s_axi_aresetn) begin
            s_axi_awready <= 0;
            s_axi_wready <= 0;
            s_axi_bvalid <= 0;
            s_axi_bresp <= 2'b00;
            control_reg <= 4'b0100;
            cfg_toggle <= 0;
            red_cfg_reg <= {8'd32, 8'd32, 8'd96};
            red_min_count_reg <= 100;
            green_cfg_reg <= {8'd24, 8'd24, 8'd80};
            green_min_count_reg <= 100;
            blue_cfg_reg <= {8'd24, 8'd24, 8'd80};
            blue_min_count_reg <= 100;
            sample_x_reg <= 0;
            sample_y_reg <= 0;
        end else begin
            s_axi_awready <= 0;
            s_axi_wready <= 0;

            if (!s_axi_bvalid && s_axi_awvalid && s_axi_wvalid) begin
                s_axi_awready <= 1;
                s_axi_wready <= 1;
                s_axi_bvalid <= 1;
                s_axi_bresp <= 2'b00;
                case (s_axi_awaddr[7:0])
                    8'h08: begin
                        merged = apply_strobes({28'd0, control_reg}, s_axi_wdata, s_axi_wstrb);
                        control_reg <= merged[3:0];
                        if (s_axi_wdata[4]) cfg_toggle <= ~cfg_toggle;
                    end
                    8'h20: begin
                        merged = apply_strobes({sample_y_reg, sample_x_reg}, s_axi_wdata, s_axi_wstrb);
                        sample_x_reg <= merged[15:0];
                        sample_y_reg <= merged[31:16];
                    end
                    8'h30: begin
                        merged = apply_strobes({8'd0, red_cfg_reg}, s_axi_wdata, s_axi_wstrb);
                        red_cfg_reg <= merged[23:0];
                    end
                    8'h34: red_min_count_reg <= apply_strobes(red_min_count_reg, s_axi_wdata, s_axi_wstrb);
                    8'h38: begin
                        merged = apply_strobes({8'd0, green_cfg_reg}, s_axi_wdata, s_axi_wstrb);
                        green_cfg_reg <= merged[23:0];
                    end
                    8'h3c: green_min_count_reg <= apply_strobes(green_min_count_reg, s_axi_wdata, s_axi_wstrb);
                    8'h40: begin
                        merged = apply_strobes({8'd0, blue_cfg_reg}, s_axi_wdata, s_axi_wstrb);
                        blue_cfg_reg <= merged[23:0];
                    end
                    8'h44: blue_min_count_reg <= apply_strobes(blue_min_count_reg, s_axi_wdata, s_axi_wstrb);
                    default: ;
                endcase
            end else if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 0;
            end
        end
    end

    function automatic [31:0] read_register(input [7:0] address);
        begin
            case (address)
                8'h00: read_register = CORE_ID;
                8'h04: read_register = VERSION;
                8'h08: read_register = {28'd0, control_reg};
                8'h0c: read_register = {28'd0, active_sync, irq_pending, snapshot_valid, 1'b0};
                8'h10: read_register = frame_seq_reg;
                8'h14: read_register = {height_reg, width_reg};
                8'h18: read_register = sync_errors_reg;
                8'h1c: read_register = dropped_frames;
                8'h20: read_register = {sample_y_reg, sample_x_reg};
                8'h24: read_register = {16'd0, sample_rgb_reg};
                8'h30: read_register = {8'd0, red_cfg_reg};
                8'h34: read_register = red_min_count_reg;
                8'h38: read_register = {8'd0, green_cfg_reg};
                8'h3c: read_register = green_min_count_reg;
                8'h40: read_register = {8'd0, blue_cfg_reg};
                8'h44: read_register = blue_min_count_reg;
                8'h50: read_register = rc_reg;
                8'h54: read_register = rsx_reg;
                8'h58: read_register = rsy_reg;
                8'h5c: read_register = {rxmax_reg, rxmin_reg};
                8'h60: read_register = {rymax_reg, rymin_reg};
                8'h64: read_register = gc_reg;
                8'h68: read_register = gsx_reg;
                8'h6c: read_register = gsy_reg;
                8'h70: read_register = {gxmax_reg, gxmin_reg};
                8'h74: read_register = {gymax_reg, gymin_reg};
                8'h78: read_register = bc_reg;
                8'h7c: read_register = bsx_reg;
                8'h80: read_register = bsy_reg;
                8'h84: read_register = {bxmax_reg, bxmin_reg};
                8'h88: read_register = {bymax_reg, bymin_reg};
                default: read_register = 32'd0;
            endcase
        end
    endfunction

    always_ff @(posedge s_axi_aclk) begin : axi_read
        if (!s_axi_aresetn) begin
            s_axi_arready <= 0;
            s_axi_rvalid <= 0;
            s_axi_rdata <= 0;
            s_axi_rresp <= 2'b00;
        end else begin
            s_axi_arready <= 0;
            if (!s_axi_rvalid && s_axi_arvalid) begin
                s_axi_arready <= 1;
                s_axi_rvalid <= 1;
                s_axi_rdata <= read_register(s_axi_araddr[7:0]);
                s_axi_rresp <= 2'b00;
            end else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 0;
            end
        end
    end

    wire unused = &{1'b0, s_axi_awprot, s_axi_arprot};

endmodule
