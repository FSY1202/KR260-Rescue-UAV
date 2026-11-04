`timescale 1ns/1ps

// Two channels share a frame counter. Reset and ENABLE=0 force outputs low.
// Configuration writes only change shadow registers. COMMIT validates and
// snapshots all three values, then updates active values at a frame boundary
// (or immediately when both channels are disabled). No automatic arming.
module servo_pwm_axi #(
    parameter integer CLK_FREQ_HZ = 100000000
) (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 S_AXI_ACLK CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXI, ASSOCIATED_RESET S_AXI_ARESETN" *)
    input wire s_axi_aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 S_AXI_ARESETN RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input wire s_axi_aresetn,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWADDR" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME S_AXI, PROTOCOL AXI4LITE, DATA_WIDTH 32, ADDR_WIDTH 8, ID_WIDTH 0, AWUSER_WIDTH 0, ARUSER_WIDTH 0, WUSER_WIDTH 0, RUSER_WIDTH 0, BUSER_WIDTH 0, READ_WRITE_MODE READ_WRITE, NUM_READ_OUTSTANDING 1, NUM_WRITE_OUTSTANDING 1" *)
    input wire [7:0] s_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWPROT" *) input wire [2:0] s_axi_awprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWVALID" *) input wire s_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWREADY" *) output wire s_axi_awready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WDATA" *) input wire [31:0] s_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WSTRB" *) input wire [3:0] s_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WVALID" *) input wire s_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WREADY" *) output wire s_axi_wready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BRESP" *) output reg [1:0] s_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BVALID" *) output reg s_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BREADY" *) input wire s_axi_bready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARADDR" *) input wire [7:0] s_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARPROT" *) input wire [2:0] s_axi_arprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARVALID" *) input wire s_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARREADY" *) output wire s_axi_arready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RDATA" *) output reg [31:0] s_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RRESP" *) output reg [1:0] s_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RVALID" *) output reg s_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RREADY" *) input wire s_axi_rready,
    output wire servo0_pwm,
    output wire servo1_pwm
);

    // CLK_FREQ_HZ must be an integer multiple of 1 MHz, and at least 1 MHz.
    localparam integer CLOCKS_PER_US = CLK_FREQ_HZ / 1000000;
    localparam [1:0] AXI_OKAY = 2'b00, AXI_SLVERR = 2'b10;

    reg aw_pending, w_pending;
    reg [7:0] awaddr_held;
    reg [31:0] wdata_held;
    reg [3:0] wstrb_held;
    reg [1:0] enable_mask, enable_effective;
    reg commit_pending, commit_error;
    reg [31:0] period_shadow, pulse0_shadow, pulse1_shadow;
    reg [31:0] period_active, pulse0_active, pulse1_active;
    reg [31:0] period_pending, pulse0_pending, pulse1_pending;
    reg [31:0] sub_us_count, phase_us;

    wire write_fire = aw_pending && w_pending && !s_axi_bvalid;
    wire us_tick = (sub_us_count == CLOCKS_PER_US - 1);
    wire frame_boundary = (enable_mask != 0) && us_tick &&
                          (phase_us == period_active - 1);

    // AW and W are accepted independently, in either order. One response is
    // outstanding at a time; backpressure cannot overwrite address or data.
    assign s_axi_awready = s_axi_aresetn && !aw_pending && !s_axi_bvalid;
    assign s_axi_wready = s_axi_aresetn && !w_pending && !s_axi_bvalid;
    assign s_axi_arready = s_axi_aresetn && !s_axi_rvalid;

    // Combinational reset/disable gating prevents any remaining high tail.
    assign servo0_pwm = s_axi_aresetn && enable_effective[0] &&
                        (phase_us < pulse0_active);
    assign servo1_pwm = s_axi_aresetn && enable_effective[1] &&
                        (phase_us < pulse1_active);

    function [31:0] merge_bytes;
        input [31:0] old_value, new_value;
        input [3:0] byte_enable;
        integer i;
        begin
            merge_bytes = old_value;
            for (i = 0; i < 4; i = i + 1)
                if (byte_enable[i])
                    merge_bytes[i*8 +: 8] = new_value[i*8 +: 8];
        end
    endfunction

    function valid_configuration;
        input [31:0] period_value, pulse0_value, pulse1_value;
        begin
            valid_configuration = (period_value >= 10000) &&
                (period_value <= 30000) &&
                (pulse0_value >= 500) && (pulse0_value <= 2500) &&
                (pulse1_value >= 500) && (pulse1_value <= 2500) &&
                (pulse0_value < period_value) && (pulse1_value < period_value);
        end
    endfunction

    // All state uses the AXI clock. Protocol responses and read data are held
    // until READY; reading a changing register cannot alter a stalled response.
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            aw_pending <= 0;
            w_pending <= 0;
            awaddr_held <= 0;
            wdata_held <= 0;
            wstrb_held <= 0;
            s_axi_bvalid <= 0;
            s_axi_bresp <= AXI_OKAY;
            s_axi_rvalid <= 0;
            s_axi_rdata <= 0;
            s_axi_rresp <= AXI_OKAY;
            enable_mask <= 0;
            enable_effective <= 0;
            commit_pending <= 0;
            commit_error <= 0;
            period_shadow <= 20000;
            pulse0_shadow <= 1500;
            pulse1_shadow <= 1500;
            period_active <= 20000;
            pulse0_active <= 1500;
            pulse1_active <= 1500;
            period_pending <= 20000;
            pulse0_pending <= 1500;
            pulse1_pending <= 1500;
            sub_us_count <= 0;
            phase_us <= 0;
        end else begin
            if (enable_mask == 0) begin
                sub_us_count <= 0;
                phase_us <= 0;
                enable_effective <= 0;
            end else if (us_tick) begin
                sub_us_count <= 0;
                if (frame_boundary) begin
                    phase_us <= 0;
                    enable_effective <= enable_mask;
                end else
                    phase_us <= phase_us + 1;
            end else begin
                sub_us_count <= sub_us_count + 1;
            end

            // Disabling both channels also safely drains an accepted commit.
            if (commit_pending && ((enable_mask == 0) || frame_boundary)) begin
                period_active <= period_pending;
                pulse0_active <= pulse0_pending;
                pulse1_active <= pulse1_pending;
                commit_pending <= 0;
            end

            if (s_axi_bvalid && s_axi_bready)
                s_axi_bvalid <= 0;
            if (s_axi_awready && s_axi_awvalid) begin
                aw_pending <= 1;
                awaddr_held <= s_axi_awaddr;
            end
            if (s_axi_wready && s_axi_wvalid) begin
                w_pending <= 1;
                wdata_held <= s_axi_wdata;
                wstrb_held <= s_axi_wstrb;
            end
            if (write_fire) begin
                aw_pending <= 0;
                w_pending <= 0;
                s_axi_bvalid <= 1;
                s_axi_bresp <= AXI_OKAY;
                case (awaddr_held)
                    8'h08: if (wstrb_held[0]) begin
                        enable_mask <= wdata_held[1:0];
                        if (enable_mask == 0) begin
                            // The first channel starts a complete new frame.
                            enable_effective <= wdata_held[1:0];
                            sub_us_count <= 0;
                            phase_us <= 0;
                        end else begin
                            // Disable is immediate. Newly requested channels
                            // wait for the next boundary, avoiding runt pulses.
                            enable_effective <= (frame_boundary ? enable_mask :
                                                 enable_effective) & wdata_held[1:0];
                        end
                    end
                    8'h10: period_shadow <= merge_bytes(period_shadow, wdata_held, wstrb_held);
                    8'h14: pulse0_shadow <= merge_bytes(pulse0_shadow, wdata_held, wstrb_held);
                    8'h18: pulse1_shadow <= merge_bytes(pulse1_shadow, wdata_held, wstrb_held);
                    8'h1c: if (wstrb_held[0] && wdata_held[0]) begin
                        if (valid_configuration(period_shadow, pulse0_shadow, pulse1_shadow)) begin
                            commit_error <= 0;
                            if (enable_mask == 0) begin
                                period_active <= period_shadow;
                                pulse0_active <= pulse0_shadow;
                                pulse1_active <= pulse1_shadow;
                                commit_pending <= 0;
                            end else begin
                                period_pending <= period_shadow;
                                pulse0_pending <= pulse0_shadow;
                                pulse1_pending <= pulse1_shadow;
                                commit_pending <= 1;
                            end
                        end else begin
                            commit_error <= 1;
                            s_axi_bresp <= AXI_SLVERR;
                        end
                    end
                    default: s_axi_bresp <= AXI_SLVERR;
                endcase
            end

            if (s_axi_rvalid && s_axi_rready)
                s_axi_rvalid <= 0;
            if (s_axi_arvalid && s_axi_arready) begin
                s_axi_rvalid <= 1;
                s_axi_rresp <= AXI_OKAY;
                case (s_axi_araddr)
                    8'h00: s_axi_rdata <= 32'h53525631;
                    8'h04: s_axi_rdata <= 32'h00010000;
                    8'h08: s_axi_rdata <= {30'b0, enable_mask};
                    8'h0c: s_axi_rdata <= {22'b0, enable_effective, 6'b0, commit_error, commit_pending};
                    8'h10: s_axi_rdata <= period_shadow;
                    8'h14: s_axi_rdata <= pulse0_shadow;
                    8'h18: s_axi_rdata <= pulse1_shadow;
                    8'h1c: s_axi_rdata <= 0;
                    8'h20: s_axi_rdata <= period_active;
                    8'h24: s_axi_rdata <= pulse0_active;
                    8'h28: s_axi_rdata <= pulse1_active;
                    default: begin
                        s_axi_rdata <= 0;
                        s_axi_rresp <= AXI_SLVERR;
                    end
                endcase
            end
        end
    end
endmodule
