// Synchronize the six asynchronous PYNQ-Z2 user controls to the AXI clock.
module board_inputs (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *) input wire aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *) input wire aresetn,
    input wire [3:0] buttons,
    input wire [1:0] switches,
    output wire [5:0] controls
);
(* ASYNC_REG = "TRUE" *) reg [5:0] meta = 0;
(* ASYNC_REG = "TRUE" *) reg [5:0] synced = 0;
always @(posedge aclk) begin
    if (!aresetn) begin
        meta <= 0;
        synced <= 0;
    end else begin
        meta <= {switches, buttons};
        synced <= meta;
    end
end
assign controls = synced;
endmodule
