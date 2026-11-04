`timescale 1ns/1ps
// One requested complete frame. 00=pixel, 01=SOF, 10=EOF, 11=bad EOF.
module dvp_frame_source #(
    parameter integer WIDTH = 640,
    parameter integer HEIGHT = 480
) (
    input wire pclk, resetn, fifo_reset,
    input wire request_toggle,
    input wire [7:0] data,
    input wire href, vsync,
    input wire fifo_full, fifo_busy,
    output logic [17:0] token,
    output logic token_valid
);
    (* ASYNC_REG="TRUE" *) logic req_meta, req_sync;
    logic req_seen, armed, capturing, phase, vsync_d, href_d;
    logic [7:0] high_byte;
    logic [15:0] line_pixels, lines;
    logic [31:0] pixels;
    logic geometry_bad, overflow_bad, end_pending;
    wire start_frame = vsync_d && !vsync;
    wire end_frame = !vsync_d && vsync;

    always_ff @(posedge pclk or negedge resetn) begin
        if (!resetn) begin
            req_meta<=0; req_sync<=0; req_seen<=0;
            armed<=0; capturing<=0; phase<=0; vsync_d<=1; href_d<=0;
            high_byte<=0; line_pixels<=0; lines<=0; pixels<=0;
            geometry_bad<=0; overflow_bad<=0; end_pending<=0;
            token<=0; token_valid<=0;
        end else begin
            req_meta<=request_toggle; req_sync<=req_meta;
            vsync_d<=vsync; href_d<=href;
            token_valid<=0;
            if (fifo_reset || fifo_busy) begin
                req_seen<=req_sync;
                armed<=0; capturing<=0; phase<=0;
                line_pixels<=0; lines<=0; pixels<=0;
                geometry_bad<=0; overflow_bad<=0; end_pending<=0;
            end else begin
                if (req_sync!=req_seen) begin
                    req_seen<=req_sync;
                    armed<=1;
                end
                // Includes a failed previous registered FIFO write.
                if (token_valid && fifo_full) overflow_bad<=1;
                if (armed && start_frame) begin
                    armed<=0; capturing<=1; phase<=0;
                    line_pixels<=0; lines<=0; pixels<=0;
                    geometry_bad<=href; overflow_bad<=fifo_full;
                    if (!fifo_full) begin token<={2'b01,16'd0}; token_valid<=1; end
                end else if (capturing) begin
                    if (href) begin
                        if (!phase) begin high_byte<=data; phase<=1; end
                        else begin
                            phase<=0;
                            line_pixels<=line_pixels+1;
                            pixels<=pixels+1;
                            if (pixels>=WIDTH*HEIGHT) geometry_bad<=1;
                            if (!fifo_full) begin
                                token<={2'b00,high_byte,data}; token_valid<=1;
                            end else overflow_bad<=1;
                        end
                    end
                    if (href_d && !href) begin
                        if (line_pixels!=WIDTH || phase) geometry_bad<=1;
                        line_pixels<=0; lines<=lines+1; phase<=0;
                    end
                    if (end_frame) begin
                        capturing<=0; end_pending<=1; phase<=0;
                        if (pixels!=WIDTH*HEIGHT || lines!=HEIGHT || href || phase)
                            geometry_bad<=1;
                    end
                end
                if (end_pending && !fifo_full) begin
                    token<={(geometry_bad || overflow_bad) ? 2'b11 : 2'b10,
                            14'd0,geometry_bad,overflow_bad};
                    token_valid<=1; end_pending<=0;
                end
            end
        end
    end
endmodule
