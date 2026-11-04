`timescale 1ns/1ps

module tb_servo_pwm_axi;
    reg clk = 0;
    always #5 clk = ~clk; // Test the real 100 MHz divider, not an accelerated one.
    reg resetn = 0;
    reg [7:0] awaddr = 0, araddr = 0;
    reg [31:0] wdata = 0;
    reg [3:0] wstrb = 0;
    reg awvalid = 0, wvalid = 0, bready = 0;
    reg arvalid = 0, rready = 0;
    wire awready, wready, bvalid, arready, rvalid;
    wire [1:0] bresp, rresp;
    wire [31:0] rdata;
    wire pwm0, pwm1;
    integer assertions = 0;
    integer rises0 = 0, falls0 = 0, rises1 = 0, falls1 = 0;
    time rise0 = 0, rise1 = 0, width0 = 0, width1 = 0, period0 = 0;
    reg forbid_outputs = 1;

    servo_pwm_axi dut (
        .s_axi_aclk(clk), .s_axi_aresetn(resetn),
        .s_axi_awaddr(awaddr), .s_axi_awprot(3'b0),
        .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(wstrb),
        .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arprot(3'b0),
        .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp),
        .s_axi_rvalid(rvalid), .s_axi_rready(rready),
        .servo0_pwm(pwm0), .servo1_pwm(pwm1)
    );

    task check(input bit condition, input string message);
        begin
            assertions = assertions + 1;
            if (!condition) $fatal(1, "FAIL at %0t: %s", $time, message);
        end
    endtask
    task clocks(input integer count);
        repeat (count) @(negedge clk);
    endtask
    task send_aw(input [7:0] address);
        begin
            @(negedge clk); awaddr = address; awvalid = 1;
            do @(posedge clk); while (!awready);
            @(negedge clk); awvalid = 0;
        end
    endtask
    task send_w(input [31:0] value, input [3:0] strobes);
        begin
            @(negedge clk); wdata = value; wstrb = strobes; wvalid = 1;
            do @(posedge clk); while (!wready);
            @(negedge clk); wvalid = 0;
        end
    endtask
    task axi_write(input [7:0] address, input [31:0] value,
                   input [3:0] strobes, input integer aw_delay,
                   input integer w_delay, input integer response_stall,
                   input [1:0] expected_response);
        reg [1:0] held_response;
        begin
            bready = 0;
            fork
                begin clocks(aw_delay); send_aw(address); end
                begin clocks(w_delay); send_w(value, strobes); end
            join
            wait (bvalid); #1;
            held_response = bresp;
            check(bresp == expected_response, "write response");
            repeat (response_stall) begin
                @(negedge clk);
                check(bvalid && bresp == held_response, "B response remains stable under backpressure");
                check(!awready && !wready, "write slots cannot be overwritten while B stalled");
            end
            @(negedge clk); bready = 1;
            @(posedge clk);
            @(negedge clk); bready = 0;
        end
    endtask
    task write_reg(input [7:0] address, input [31:0] value);
        axi_write(address, value, 4'hf, 0, 0, 0, 2'b00);
    endtask
    // Schedule the execution of a write on a frame boundary. Hierarchical
    // counter observation only selects this otherwise difficult corner case;
    // all configuration still enters through the public AXI interface.
    task write_on_boundary(input [7:0] address, input [31:0] value);
        begin
            @(negedge clk);
            while (!(dut.phase_us == dut.period_active - 1 &&
                     dut.sub_us_count == 98)) @(negedge clk);
            bready = 0;
            awaddr = address; awvalid = 1;
            wdata = value; wstrb = 4'hf; wvalid = 1;
            @(posedge clk);
            check(awready && wready, "boundary write accepts both channels");
            @(negedge clk); awvalid = 0; wvalid = 0;
            @(posedge clk); #1;
            check(bvalid && bresp == 0, "boundary write completes with OKAY");
            check(dut.phase_us == 0 && dut.sub_us_count == 0,
                  "write execution coincides with the frame boundary");
            @(negedge clk); bready = 1;
            @(posedge clk);
            @(negedge clk); bready = 0;
        end
    endtask
    task axi_read(input [7:0] address, input integer response_stall,
                  input [31:0] expected_value, input [1:0] expected_response);
        reg [31:0] held_data;
        reg [1:0] held_response;
        begin
            @(negedge clk); araddr = address; arvalid = 1; rready = 0;
            do @(posedge clk); while (!arready);
            @(negedge clk); arvalid = 0;
            wait (rvalid); #1;
            held_data = rdata; held_response = rresp;
            check(rdata == expected_value && rresp == expected_response, "read value/response");
            repeat (response_stall) begin
                @(negedge clk);
                check(rvalid && rdata == held_data && rresp == held_response,
                      "R response remains stable under backpressure");
                check(!arready, "AR cannot overwrite stalled R response");
            end
            @(negedge clk); rready = 1;
            @(posedge clk);
            @(negedge clk); rready = 0;
        end
    endtask
    task read_reg(input [7:0] address, input [31:0] value);
        axi_read(address, 0, value, 2'b00);
    endtask

    always @(posedge pwm0) begin
        if (rises0 != 0) period0 = $time - rise0;
        rise0 = $time; rises0 = rises0 + 1;
    end
    always @(negedge pwm0) begin
        if (resetn && rises0 != 0) begin
            width0 = $time - rise0; falls0 = falls0 + 1;
        end
    end
    always @(posedge pwm1) begin
        rise1 = $time; rises1 = rises1 + 1;
    end
    always @(negedge pwm1) begin
        if (resetn && rises1 != 0) begin
            width1 = $time - rise1; falls1 = falls1 + 1;
        end
    end
    always @(negedge clk) begin
        if (forbid_outputs && (pwm0 !== 0 || pwm1 !== 0))
            $fatal(1, "output toggled before explicit enable or after disable");
    end

    integer saved_rises0, saved_rises1, saved_falls0, saved_falls1;
    time saved_rise0;
    initial begin
        // Watchdog bounds protocol hangs and missed waveform events.
        #200000000;
        $fatal(1, "simulation watchdog expired");
    end

    initial begin
        clocks(8);
        check(!pwm0 && !pwm1 && !bvalid && !rvalid, "reset is disarmed");
        resetn = 1;
        clocks(2);
        axi_read(8'h00, 7, 32'h53525631, 2'b00);
        read_reg(8'h04, 32'h00010000);
        read_reg(8'h08, 0);
        read_reg(8'h0c, 0);
        read_reg(8'h20, 20000);

        // AW-first and W-first must not execute before both handshakes.
        axi_write(8'h10, 20000, 4'hf, 0, 11, 9, 2'b00);
        axi_write(8'h14, 1500, 4'hf, 13, 0, 4, 2'b00);
        read_reg(8'h10, 20000);
        read_reg(8'h14, 1500);
        // Byte writes merge, including zero strobes and upper-byte enable no-op.
        axi_write(8'h14, 32'h12345678, 4'b0101, 0, 0, 0, 2'b00);
        read_reg(8'h14, 32'h00340578);
        axi_write(8'h14, 32'hffffffff, 0, 0, 0, 0, 2'b00);
        read_reg(8'h14, 32'h00340578);
        write_reg(8'h14, 1500);
        axi_write(8'h08, 3, 4'b0010, 0, 0, 0, 2'b00);
        read_reg(8'h08, 0);
        // Read-only, unmapped and unaligned accesses fail without side effects.
        axi_write(8'h00, 0, 4'hf, 0, 0, 3, 2'b10);
        axi_write(8'h11, 0, 4'hf, 0, 0, 0, 2'b10);
        axi_read(8'hfc, 3, 0, 2'b10);
        axi_read(8'h09, 0, 0, 2'b10);
        write_reg(8'h1c, 1);
        clocks(1000);
        check(rises0 == 0 && rises1 == 0, "configuration and commit never arm outputs");

        // Invalid commits are rejected; no invalid values reach active registers.
        write_reg(8'h10, 9999);
        axi_write(8'h1c, 1, 4'hf, 0, 0, 5, 2'b10);
        read_reg(8'h0c, 2);
        read_reg(8'h20, 20000);
        check(rises0 == 0 && rises1 == 0, "invalid commit cannot start output");
        write_reg(8'h10, 20000);
        write_reg(8'h18, 2501);
        axi_write(8'h1c, 1, 4'hf, 0, 0, 0, 2'b10);
        read_reg(8'h28, 1500);
        write_reg(8'h18, 1500);
        // COMMIT byte0/bit0 qualification: no-op must leave sticky error set.
        axi_write(8'h1c, 1, 4'b0010, 0, 0, 0, 2'b00);
        write_reg(8'h1c, 0);
        read_reg(8'h0c, 2);
        write_reg(8'h1c, 1);
        read_reg(8'h0c, 0);

        $display("AXI/reset/config tests passed at %0t", $time);
        forbid_outputs = 0;
        write_reg(8'h08, 1);
        read_reg(8'h0c, 32'h100);
        wait (falls0 == 1); #1;
        check(width0 == 1500000, "first high pulse is exactly 1500 us at 100 MHz");
        wait (rises0 == 2); #1;
        check(period0 == 20000000, "frame is exactly 20000 us / 50 Hz");
        saved_rise0 = rise0;

        // Opening channel 1 within a high pulse waits for the next full frame.
        clocks(100);
        write_reg(8'h08, 3);
        read_reg(8'h08, 3);
        read_reg(8'h0c, 32'h100);
        check(!pwm1 && rises1 == 0, "new channel does not emit a runt first pulse");
        // Queue new configuration and then change shadow: pending is a snapshot.
        write_reg(8'h10, 10000);
        write_reg(8'h14, 1000);
        write_reg(8'h18, 2000);
        write_reg(8'h1c, 1);
        read_reg(8'h0c, 32'h101);
        read_reg(8'h20, 20000);
        read_reg(8'h24, 1500);
        write_reg(8'h14, 1200);
        write_reg(8'h18, 499);
        axi_write(8'h1c, 1, 4'hf, 0, 0, 0, 2'b10);
        read_reg(8'h0c, 32'h103);
        wait (falls0 == 2); #1;
        check(width0 == 1500000, "shadow and pending do not alter current high pulse");
        wait (rises0 == 3); #1;
        check(rise0 - saved_rise0 == 20000000, "commit waits for old frame boundary");
        check(rises1 == 1 && rise1 == rise0, "second channel starts on shared frame boundary");
        read_reg(8'h20, 10000);
        read_reg(8'h24, 1000);
        read_reg(8'h28, 2000);
        read_reg(8'h0c, 32'h302);
        wait (falls0 == 3); #1;
        check(width0 == 1000000, "committed channel 0 pulse is exactly 1000 us");
        wait (falls1 == 1); #1;
        check(width1 == 2000000, "committed channel 1 pulse is exactly 2000 us");
        wait (rises0 == 4); #1;
        check(period0 == 10000000, "committed period is exactly 10000 us");

        // Disabling one channel while high must not truncate the other channel.
        clocks(100);
        write_reg(8'h08, 2);
        check(!pwm0 && pwm1, "single-channel disable is immediate and independent");
        saved_falls0 = falls0;
        wait (falls1 == 2); #1;
        check(width1 == 2000000, "other channel retains its complete pulse");
        check(falls0 == saved_falls0, "disabled channel has no extra tail transition");

        // A pending configuration drains safely when all channels are disabled.
        write_reg(8'h10, 20000);
        write_reg(8'h14, 1500);
        write_reg(8'h18, 1500);
        write_reg(8'h1c, 1);
        read_reg(8'h0c, 32'h201);
        write_reg(8'h08, 0);
        forbid_outputs = 1;
        clocks(2);
        read_reg(8'h0c, 0);
        read_reg(8'h20, 20000);
        saved_rises0 = rises0; saved_rises1 = rises1;
        clocks(2100000);
        check(rises0 == saved_rises0 && rises1 == saved_rises1,
              "disable has no later pulses over more than one frame");

        // Both pulse endpoints and the maximum period are valid. Values just
        // outside either range are rejected without changing active values.
        write_reg(8'h10, 30000);
        write_reg(8'h14, 500);
        write_reg(8'h18, 2500);
        write_reg(8'h1c, 1);
        read_reg(8'h20, 30000);
        read_reg(8'h24, 500);
        read_reg(8'h28, 2500);
        write_reg(8'h10, 30001);
        axi_write(8'h1c, 1, 4'hf, 0, 0, 0, 2'b10);
        read_reg(8'h20, 30000);
        write_reg(8'h10, 10000);
        write_reg(8'h14, 499);
        axi_write(8'h1c, 1, 4'hf, 0, 0, 0, 2'b10);
        write_reg(8'h14, 2500);
        write_reg(8'h18, 499);
        axi_write(8'h1c, 1, 4'hf, 0, 0, 0, 2'b10);
        write_reg(8'h18, 500);
        write_reg(8'h1c, 1);
        read_reg(8'h24, 2500);
        read_reg(8'h28, 500);
        write_reg(8'h14, 500);
        write_reg(8'h18, 2500);
        write_reg(8'h1c, 1);
        read_reg(8'h0c, 0);
        check(rises0 == saved_rises0 && rises1 == saved_rises1,
              "range validation and disabled commits never arm outputs");

        // A changing register must retain its sampled value while R is
        // stalled, even when an independent write changes the register.
        forbid_outputs = 0;
        saved_falls0 = falls0;
        fork
            axi_read(8'h08, 80, 0, 2'b00);
            begin clocks(6); write_reg(8'h08, 1); end
        join
        read_reg(8'h08, 1);
        read_reg(8'h0c, 32'h100);
        wait (falls0 == saved_falls0 + 1); #1;
        check(width0 == 500000, "minimum pulse starts with a complete 500 us high");

        // A newly enabled channel on the exact boundary is armed for the
        // following boundary, with no partial pulse on the execution edge.
        saved_rises1 = rises1;
        saved_falls1 = falls1;
        write_on_boundary(8'h08, 3);
        check(pwm0 && !pwm1 && rises1 == saved_rises1,
              "new channel at the boundary waits for the following frame");
        read_reg(8'h08, 3);
        read_reg(8'h0c, 32'h100);
        wait (rises1 == saved_rises1 + 1); #1;
        check(rise0 == rise1, "boundary enable starts channels on the same edge");
        wait (falls1 == saved_falls1 + 1); #1;
        check(width1 == 2500000, "maximum pulse starts with a complete 2500 us high");
        saved_rises0 = rises0;
        write_on_boundary(8'h08, 2);
        check(!pwm0 && pwm1 && rises0 == saved_rises0,
              "boundary disable does not emit an extra channel 0 pulse");

        // An older pending snapshot applies at this boundary. A new COMMIT
        // executed on that edge remains pending until the following boundary.
        write_reg(8'h10, 12000);
        write_reg(8'h14, 1000);
        write_reg(8'h18, 2000);
        write_reg(8'h1c, 1);
        write_reg(8'h10, 15000);
        write_reg(8'h14, 1500);
        write_reg(8'h18, 1800);
        write_on_boundary(8'h1c, 1);
        read_reg(8'h20, 12000);
        read_reg(8'h24, 1000);
        read_reg(8'h28, 2000);
        read_reg(8'h0c, 32'h201);
        saved_rises1 = rises1;
        wait (rises1 == saved_rises1 + 1); #1;
        read_reg(8'h20, 15000);
        read_reg(8'h24, 1500);
        read_reg(8'h28, 1800);
        read_reg(8'h0c, 32'h200);
        write_reg(8'h08, 0);
        forbid_outputs = 1;
        clocks(2);
        read_reg(8'h0c, 0);

        // Reset clears an incomplete AW transaction and immediately gates high.
        send_aw(8'h08);
        resetn = 0; #1;
        check(!pwm0 && !pwm1, "reset immediately gates output");
        clocks(3); resetn = 1; clocks(3);
        check(!bvalid && !rvalid, "reset clears partial transaction");
        read_reg(8'h08, 0);
        read_reg(8'h20, 20000);
        forbid_outputs = 0;
        write_reg(8'h08, 3);
        check(pwm0 && pwm1, "explicit enable after reset starts both channels");
        #2; resetn = 0; #1;
        check(!pwm0 && !pwm1, "reset between clock edges forces both high outputs low");
        forbid_outputs = 1;
        clocks(4); resetn = 1; clocks(4);
        read_reg(8'h08, 0);
        $display("PASS: servo_pwm_axi %0d assertions, AXI backpressure/WSTRB, 50Hz timing, atomic commit and disarm", assertions);
        $finish;
    end
endmodule
