`timescale 1ns/1ps

module color_detector_core (
    input  logic        pclk,
    input  logic        resetn,
    input  logic        enable_async,
    input  logic [7:0]  cam_data,
    input  logic        cam_href,
    input  logic        cam_vsync,

    input  logic        cfg_toggle_async,
    input  logic [23:0] red_cfg_async,
    input  logic [31:0] red_min_count_async,
    input  logic [23:0] green_cfg_async,
    input  logic [31:0] green_min_count_async,
    input  logic [23:0] blue_cfg_async,
    input  logic [31:0] blue_min_count_async,
    input  logic [15:0] sample_x_async,
    input  logic [15:0] sample_y_async,

    output logic        frame_toggle,
    output logic        frame_active,
    output logic [31:0] frame_seq_snap,
    output logic [15:0] width_snap,
    output logic [15:0] height_snap,
    output logic [31:0] sync_errors_snap,
    output logic [15:0] sample_rgb565_snap,

    output logic [31:0] red_count_snap,
    output logic [31:0] red_sum_x_snap,
    output logic [31:0] red_sum_y_snap,
    output logic [15:0] red_xmin_snap,
    output logic [15:0] red_xmax_snap,
    output logic [15:0] red_ymin_snap,
    output logic [15:0] red_ymax_snap,

    output logic [31:0] green_count_snap,
    output logic [31:0] green_sum_x_snap,
    output logic [31:0] green_sum_y_snap,
    output logic [15:0] green_xmin_snap,
    output logic [15:0] green_xmax_snap,
    output logic [15:0] green_ymin_snap,
    output logic [15:0] green_ymax_snap,

    output logic [31:0] blue_count_snap,
    output logic [31:0] blue_sum_x_snap,
    output logic [31:0] blue_sum_y_snap,
    output logic [15:0] blue_xmin_snap,
    output logic [15:0] blue_xmax_snap,
    output logic [15:0] blue_ymin_snap,
    output logic [15:0] blue_ymax_snap
);

    (* ASYNC_REG = "TRUE" *) logic enable_meta, enable;
    (* ASYNC_REG = "TRUE" *) logic cfg_meta, cfg_sync;
    logic cfg_seen;
    logic vsync_d, href_d;
    logic byte_phase;
    logic [7:0] byte_hi;
    logic [15:0] x_pos, y_pos;
    logic [15:0] line_pixels, max_width, line_count;
    logic [31:0] frame_seq, sync_errors;

    logic [23:0] red_cfg, green_cfg, blue_cfg;
    logic [31:0] red_min_count, green_min_count, blue_min_count;
    logic [15:0] sample_x, sample_y, sample_rgb565;

    logic [31:0] red_count, red_sum_x, red_sum_y;
    logic [15:0] red_xmin, red_xmax, red_ymin, red_ymax;
    logic [31:0] green_count, green_sum_x, green_sum_y;
    logic [15:0] green_xmin, green_xmax, green_ymin, green_ymax;
    logic [31:0] blue_count, blue_sum_x, blue_sum_y;
    logic [15:0] blue_xmin, blue_xmax, blue_ymin, blue_ymax;

    wire frame_start = vsync_d && !cam_vsync;
    wire frame_end   = !vsync_d && cam_vsync;
    wire line_start  = !href_d && cam_href;
    wire line_end    = href_d && !cam_href;

    function automatic [7:0] expand5(input logic [4:0] value);
        expand5 = {value, value[4:2]};
    endfunction

    function automatic [7:0] expand6(input logic [5:0] value);
        expand6 = {value, value[5:4]};
    endfunction

    task automatic clear_accumulators;
        begin
            red_count   <= 0; red_sum_x   <= 0; red_sum_y   <= 0;
            red_xmin    <= 16'hffff; red_xmax <= 0; red_ymin <= 16'hffff; red_ymax <= 0;
            green_count <= 0; green_sum_x <= 0; green_sum_y <= 0;
            green_xmin  <= 16'hffff; green_xmax <= 0; green_ymin <= 16'hffff; green_ymax <= 0;
            blue_count  <= 0; blue_sum_x  <= 0; blue_sum_y  <= 0;
            blue_xmin   <= 16'hffff; blue_xmax <= 0; blue_ymin <= 16'hffff; blue_ymax <= 0;
        end
    endtask

    always_ff @(posedge pclk or negedge resetn) begin : capture_and_classify
        logic [15:0] rgb565;
        logic [5:0]  green6;
        logic [7:0]  red8, green8, blue8;
        logic        is_red, is_green, is_blue;
        logic [16:0] measured_height;
        logic [16:0] measured_width;

        if (!resetn) begin
            enable_meta <= 0;
            enable <= 0;
            cfg_meta <= 0;
            cfg_sync <= 0;
            cfg_seen <= 0;
            vsync_d <= 1;
            href_d <= 0;
            byte_phase <= 0;
            byte_hi <= 0;
            x_pos <= 0;
            y_pos <= 0;
            line_pixels <= 0;
            max_width <= 0;
            line_count <= 0;
            frame_seq <= 0;
            sync_errors <= 0;
            frame_toggle <= 0;
            frame_active <= 0;

            red_cfg <= {8'd32, 8'd32, 8'd96};
            green_cfg <= {8'd24, 8'd24, 8'd80};
            blue_cfg <= {8'd24, 8'd24, 8'd80};
            red_min_count <= 100;
            green_min_count <= 100;
            blue_min_count <= 100;
            sample_x <= 0;
            sample_y <= 0;
            sample_rgb565 <= 0;

            frame_seq_snap <= 0;
            width_snap <= 0;
            height_snap <= 0;
            sync_errors_snap <= 0;
            sample_rgb565_snap <= 0;
            red_count_snap <= 0; red_sum_x_snap <= 0; red_sum_y_snap <= 0;
            red_xmin_snap <= 0; red_xmax_snap <= 0; red_ymin_snap <= 0; red_ymax_snap <= 0;
            green_count_snap <= 0; green_sum_x_snap <= 0; green_sum_y_snap <= 0;
            green_xmin_snap <= 0; green_xmax_snap <= 0; green_ymin_snap <= 0; green_ymax_snap <= 0;
            blue_count_snap <= 0; blue_sum_x_snap <= 0; blue_sum_y_snap <= 0;
            blue_xmin_snap <= 0; blue_xmax_snap <= 0; blue_ymin_snap <= 0; blue_ymax_snap <= 0;
            clear_accumulators();
        end else begin
            enable_meta <= enable_async;
            enable <= enable_meta;
            cfg_meta <= cfg_toggle_async;
            cfg_sync <= cfg_meta;
            vsync_d <= cam_vsync;
            href_d <= cam_href;

            if (!enable) begin
                frame_active <= 0;
                byte_phase <= 0;
            end

            if (frame_start && enable) begin
                frame_active <= 1;
                byte_phase <= 0;
                x_pos <= 0;
                y_pos <= 0;
                line_pixels <= 0;
                max_width <= 0;
                line_count <= 0;
                sample_rgb565 <= 0;
                clear_accumulators();

                if (cfg_sync != cfg_seen) begin
                    cfg_seen <= cfg_sync;
                    red_cfg <= red_cfg_async;
                    red_min_count <= red_min_count_async;
                    green_cfg <= green_cfg_async;
                    green_min_count <= green_min_count_async;
                    blue_cfg <= blue_cfg_async;
                    blue_min_count <= blue_min_count_async;
                    sample_x <= sample_x_async;
                    sample_y <= sample_y_async;
                end
            end

            if (line_start && frame_active) begin
                byte_phase <= 0;
                x_pos <= 0;
                line_pixels <= 0;
                y_pos <= line_count;
            end

            if (cam_href && frame_active && enable) begin
                if (!byte_phase) begin
                    byte_hi <= cam_data;
                    byte_phase <= 1;
                end else begin
                    byte_phase <= 0;
                    rgb565 = {byte_hi, cam_data};
                    green6 = {byte_hi[2:0], cam_data[7:5]};
                    red8 = expand5(byte_hi[7:3]);
                    green8 = expand6(green6);
                    blue8 = expand5(cam_data[4:0]);

                    is_red = (red8 >= red_cfg[7:0]) &&
                             ({1'b0, red8} >= ({1'b0, green8} + red_cfg[15:8])) &&
                             ({1'b0, red8} >= ({1'b0, blue8} + red_cfg[23:16]));
                    is_green = !is_red && (green8 >= green_cfg[7:0]) &&
                               ({1'b0, green8} >= ({1'b0, red8} + green_cfg[15:8])) &&
                               ({1'b0, green8} >= ({1'b0, blue8} + green_cfg[23:16]));
                    is_blue = !is_red && !is_green && (blue8 >= blue_cfg[7:0]) &&
                              ({1'b0, blue8} >= ({1'b0, red8} + blue_cfg[15:8])) &&
                              ({1'b0, blue8} >= ({1'b0, green8} + blue_cfg[23:16]));

                    if ((x_pos == sample_x) && (y_pos == sample_y))
                        sample_rgb565 <= rgb565;

                    if (is_red) begin
                        red_count <= red_count + 1;
                        red_sum_x <= red_sum_x + x_pos;
                        red_sum_y <= red_sum_y + y_pos;
                        if (x_pos < red_xmin) red_xmin <= x_pos;
                        if (x_pos > red_xmax) red_xmax <= x_pos;
                        if (y_pos < red_ymin) red_ymin <= y_pos;
                        if (y_pos > red_ymax) red_ymax <= y_pos;
                    end else if (is_green) begin
                        green_count <= green_count + 1;
                        green_sum_x <= green_sum_x + x_pos;
                        green_sum_y <= green_sum_y + y_pos;
                        if (x_pos < green_xmin) green_xmin <= x_pos;
                        if (x_pos > green_xmax) green_xmax <= x_pos;
                        if (y_pos < green_ymin) green_ymin <= y_pos;
                        if (y_pos > green_ymax) green_ymax <= y_pos;
                    end else if (is_blue) begin
                        blue_count <= blue_count + 1;
                        blue_sum_x <= blue_sum_x + x_pos;
                        blue_sum_y <= blue_sum_y + y_pos;
                        if (x_pos < blue_xmin) blue_xmin <= x_pos;
                        if (x_pos > blue_xmax) blue_xmax <= x_pos;
                        if (y_pos < blue_ymin) blue_ymin <= y_pos;
                        if (y_pos > blue_ymax) blue_ymax <= y_pos;
                    end

                    x_pos <= x_pos + 1;
                    line_pixels <= line_pixels + 1;
                end
            end

            if (line_end && frame_active) begin
                byte_phase <= 0;
                if (line_pixels > max_width)
                    max_width <= line_pixels;
                if (line_pixels != 0)
                    line_count <= line_count + 1;
                line_pixels <= 0;
            end

            if (frame_end && frame_active) begin
                frame_active <= 0;
                byte_phase <= 0;
                measured_width = (line_pixels > max_width) ? line_pixels : max_width;
                measured_height = line_count + ((cam_href || line_pixels != 0) ? 1 : 0);

                frame_seq <= frame_seq + 1;
                frame_seq_snap <= frame_seq + 1;
                width_snap <= measured_width[15:0];
                height_snap <= measured_height[15:0];
                sample_rgb565_snap <= sample_rgb565;

                if ((measured_width != 640) || (measured_height != 480)) begin
                    sync_errors <= sync_errors + 1;
                    sync_errors_snap <= sync_errors + 1;
                end else begin
                    sync_errors_snap <= sync_errors;
                end

                red_count_snap <= red_count;
                red_sum_x_snap <= red_sum_x;
                red_sum_y_snap <= red_sum_y;
                red_xmin_snap <= (red_count >= red_min_count) ? red_xmin : 0;
                red_xmax_snap <= (red_count >= red_min_count) ? red_xmax : 0;
                red_ymin_snap <= (red_count >= red_min_count) ? red_ymin : 0;
                red_ymax_snap <= (red_count >= red_min_count) ? red_ymax : 0;

                green_count_snap <= green_count;
                green_sum_x_snap <= green_sum_x;
                green_sum_y_snap <= green_sum_y;
                green_xmin_snap <= (green_count >= green_min_count) ? green_xmin : 0;
                green_xmax_snap <= (green_count >= green_min_count) ? green_xmax : 0;
                green_ymin_snap <= (green_count >= green_min_count) ? green_ymin : 0;
                green_ymax_snap <= (green_count >= green_min_count) ? green_ymax : 0;

                blue_count_snap <= blue_count;
                blue_sum_x_snap <= blue_sum_x;
                blue_sum_y_snap <= blue_sum_y;
                blue_xmin_snap <= (blue_count >= blue_min_count) ? blue_xmin : 0;
                blue_xmax_snap <= (blue_count >= blue_min_count) ? blue_xmax : 0;
                blue_ymin_snap <= (blue_count >= blue_min_count) ? blue_ymin : 0;
                blue_ymax_snap <= (blue_count >= blue_min_count) ? blue_ymax : 0;

                frame_toggle <= ~frame_toggle;
            end
        end
    end

endmodule
