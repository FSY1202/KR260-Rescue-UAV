`timescale 1ns/1ps
// FRM2: three owned UltraRAM banks. Only claimed complete frames are readable.
module frame_capture_axi #(
    parameter integer C_S_AXI_ADDR_WIDTH=20,
    parameter integer C_S_AXI_DATA_WIDTH=32,
    parameter integer WIDTH=640,
    parameter integer HEIGHT=480,
    parameter integer TIMEOUT_CYCLES=200000000
) (
    input wire cam_pclk,
    input wire [7:0] cam_data,
    input wire cam_href,cam_vsync,
    input wire s_axi_aclk,s_axi_aresetn,
    input wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_awaddr,
    input wire [2:0] s_axi_awprot,
    input wire s_axi_awvalid,
    output wire s_axi_awready,
    input wire [31:0] s_axi_wdata,
    input wire [3:0] s_axi_wstrb,
    input wire s_axi_wvalid,
    output wire s_axi_wready,
    output logic [1:0] s_axi_bresp,
    output logic s_axi_bvalid,
    input wire s_axi_bready,
    input wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_araddr,
    input wire [2:0] s_axi_arprot,
    input wire s_axi_arvalid,
    output wire s_axi_arready,
    output logic [31:0] s_axi_rdata,
    output logic [1:0] s_axi_rresp,
    output logic s_axi_rvalid,
    input wire s_axi_rready
);
    localparam integer PIXELS=WIDTH*HEIGHT;
    localparam integer WORDS=PIXELS/4;
    localparam integer MEM_AW=$clog2(WORDS*3);
    localparam [3:0] IDLE=0, RESET_FIFO=1, WAIT_FIFO=2,
                     WAIT_SOF=3, RECEIVING=4, COMMIT=5;
    logic [3:0] state;
    logic busy,error,frame_bad;
    logic [31:0] frame_seq,errors,overflows,pixel_count,last_error,ticks,drops;
    logic [13:0] reset_count;
    logic [1:0] pack_phase,commit_wait,write_bank;
    logic [47:0] packed_pixels;
    logic request_toggle,fifo_reset_axi;
    logic claimed_valid,latest_valid;
    logic [1:0] claimed_bank,latest_bank;
    logic [31:0] claimed_seq,latest_seq;
    logic claimed_valid_next,latest_valid_next;
    logic [1:0] claimed_bank_next,latest_bank_next,next_write_bank;
    logic [31:0] claimed_seq_next,latest_seq_next;
    logic replacement;
    logic [63:0] cycle_counter,completed_cycle[0:2];
    wire [63:0] claimed_age=claimed_valid ? cycle_counter-completed_cycle[claimed_bank] : 64'd0;
    (* ASYNC_REG="TRUE" *) logic reset_meta_pclk,reset_pclk;
    always_ff @(posedge cam_pclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin reset_meta_pclk<=1; reset_pclk<=1; end
        else begin reset_meta_pclk<=fifo_reset_axi; reset_pclk<=reset_meta_pclk; end
    end
    wire [17:0] fifo_in,fifo_out;
    wire fifo_write,fifo_full,fifo_empty,wr_busy,rd_busy;
    (* ASYNC_REG="TRUE" *) logic wr_busy_meta,wr_busy_sync;
    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin wr_busy_meta<=1; wr_busy_sync<=1; end
        else begin wr_busy_meta<=wr_busy; wr_busy_sync<=wr_busy_meta; end
    end
    dvp_frame_source #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) source_i (
        .pclk(cam_pclk),.resetn(s_axi_aresetn),.fifo_reset(reset_pclk),
        .request_toggle(request_toggle),.data(cam_data),.href(cam_href),.vsync(cam_vsync),
        .fifo_full(fifo_full),.fifo_busy(wr_busy),.token(fifo_in),.token_valid(fifo_write));
    wire fifo_read=busy && (state==WAIT_SOF || state==RECEIVING) && !fifo_empty && !rd_busy;
    xpm_fifo_async #(
        .FIFO_MEMORY_TYPE("block"),.ECC_MODE("no_ecc"),.RELATED_CLOCKS(0),
        .FIFO_WRITE_DEPTH(1024),.WRITE_DATA_WIDTH(18),.READ_DATA_WIDTH(18),
        .READ_MODE("fwft"),.FIFO_READ_LATENCY(0),.CDC_SYNC_STAGES(2),
        .WR_DATA_COUNT_WIDTH(11),.RD_DATA_COUNT_WIDTH(11),
        .PROG_FULL_THRESH(1000),.PROG_EMPTY_THRESH(10),
        // Vendor2022.1 EMPTY_CHECK reset false positive; local checks below.
        .USE_ADV_FEATURES("0000"),.SIM_ASSERT_CHK(0)
    ) fifo_i (
        .sleep(1'b0),.rst(reset_pclk),.wr_clk(cam_pclk),.wr_en(fifo_write && !wr_busy),
        .din(fifo_in),.full(fifo_full),.prog_full(),.wr_data_count(),.overflow(),
        .wr_rst_busy(wr_busy),.almost_full(),.wr_ack(),
        .rd_clk(s_axi_aclk),.rd_en(fifo_read),.dout(fifo_out),.empty(fifo_empty),
        .prog_empty(),.rd_data_count(),.underflow(),.rd_rst_busy(rd_busy),
        .almost_empty(),.data_valid(),.injectsbiterr(1'b0),.injectdbiterr(1'b0),
        .sbiterr(),.dbiterr());

    function automatic [MEM_AW-1:0] bank_offset(input [1:0] bank);
        case(bank)
            0: bank_offset=0;
            1: bank_offset=WORDS;
            2: bank_offset=WORDS*2;
            default: bank_offset=0;
        endcase
    endfunction
    logic mem_write;
    logic [MEM_AW-1:0] mem_write_addr;
    logic [63:0] mem_write_data;
    wire [63:0] mem_read_data;
    wire read_accept=s_axi_arvalid && s_axi_arready;
    wire mem_read_enable=read_accept && claimed_valid &&
         s_axi_araddr>=20'h01000 && s_axi_araddr<20'h01000+PIXELS*2 && s_axi_araddr[1:0]==0;
    wire [C_S_AXI_ADDR_WIDTH-1:0] byte_offset=s_axi_araddr-20'h01000;
    wire [MEM_AW-1:0] mem_read_addr=bank_offset(claimed_bank)+(byte_offset>>3);
    xpm_memory_sdpram #(
        .MEMORY_SIZE(WORDS*3*64),.MEMORY_PRIMITIVE("ultra"),.CLOCKING_MODE("common_clock"),
        .ECC_MODE("no_ecc"),.USE_MEM_INIT(0),.MEMORY_INIT_FILE("none"),
        .WRITE_DATA_WIDTH_A(64),.BYTE_WRITE_WIDTH_A(64),.ADDR_WIDTH_A(MEM_AW),
        .READ_DATA_WIDTH_B(64),.ADDR_WIDTH_B(MEM_AW),.READ_LATENCY_B(2),
        .READ_RESET_VALUE_B("0"),.WRITE_MODE_B("read_first"),.SIM_ASSERT_CHK(1)
    ) image_i (
        .sleep(1'b0),.clka(s_axi_aclk),.ena(mem_write),.wea(mem_write),
        .addra(mem_write_addr),.dina(mem_write_data),
        .clkb(s_axi_aclk),.rstb(!s_axi_aresetn),.enb(mem_read_enable),.regceb(1'b1),
        .addrb(mem_read_addr),.doutb(mem_read_data),
        .injectsbiterra(1'b0),.injectdbiterra(1'b0),.sbiterrb(),.dbiterrb());

    logic aw_pending,w_pending;
    logic [C_S_AXI_ADDR_WIDTH-1:0] aw_hold;
    logic [31:0] w_hold;
    logic [3:0] strobe_hold;
    logic [1:0] read_wait;
    logic read_high_word;
    wire command=aw_pending && w_pending && !s_axi_bvalid;
    wire command_address=aw_hold==20'h00008 && strobe_hold[0];
    wire read_inflight=read_wait!=0 || s_axi_rvalid || read_accept;
    wire command_bad= !command_address ||
        ((w_hold[2] || w_hold[1]) && read_inflight) ||
        (!w_hold[2] && w_hold[0] && busy) ||
        (!w_hold[2] && w_hold[3] && ((!w_hold[1] && claimed_valid) || !latest_valid));
    wire accepted_command=command && !command_bad;
    wire start_command=accepted_command && w_hold[0] && !w_hold[2];
    wire abort_command=accepted_command && w_hold[2];
    wire release_command=accepted_command && w_hold[1] && !w_hold[2];
    wire claim_command=accepted_command && w_hold[3] && !w_hold[2];
    wire commit_frame=busy && state==COMMIT && commit_wait==0 && !abort_command && ticks<TIMEOUT_CYCLES-1;
    assign s_axi_awready=!aw_pending && !s_axi_bvalid;
    assign s_axi_wready=!w_pending && !s_axi_bvalid;
    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            aw_pending<=0; w_pending<=0; aw_hold<=0; w_hold<=0; strobe_hold<=0;
            s_axi_bvalid<=0; s_axi_bresp<=0;
        end else begin
            if (s_axi_awvalid && s_axi_awready) begin aw_hold<=s_axi_awaddr; aw_pending<=1; end
            if (s_axi_wvalid && s_axi_wready) begin w_hold<=s_axi_wdata; strobe_hold<=s_axi_wstrb; w_pending<=1; end
            if (s_axi_bvalid && s_axi_bready) s_axi_bvalid<=0;
            if (command) begin
                aw_pending<=0; w_pending<=0; s_axi_bvalid<=1;
                s_axi_bresp<=command_bad ? 2'b10 : 2'b00;
            end
        end
    end

    // Commands claim the previous complete frame; an EOF on the same cycle is
    // published afterward. Never reclaim a bank which is still AXI-readable.
    always_comb begin
        claimed_valid_next=claimed_valid; claimed_bank_next=claimed_bank; claimed_seq_next=claimed_seq;
        latest_valid_next=latest_valid; latest_bank_next=latest_bank; latest_seq_next=latest_seq;
        replacement=0;
        if (abort_command) begin claimed_valid_next=0; latest_valid_next=0; end
        else begin
            if (release_command) claimed_valid_next=0;
            if (claim_command) begin
                claimed_valid_next=1; claimed_bank_next=latest_bank; claimed_seq_next=latest_seq;
                latest_valid_next=0;
            end
            if (commit_frame) begin
                replacement=latest_valid_next;
                latest_valid_next=1; latest_bank_next=write_bank; latest_seq_next=frame_seq+1;
            end
        end
        // At most one READING and one READY always leaves a FREE third bank.
        if ((!claimed_valid_next || claimed_bank_next!=0) && (!latest_valid_next || latest_bank_next!=0))
            next_write_bank=0;
        else if ((!claimed_valid_next || claimed_bank_next!=1) && (!latest_valid_next || latest_bank_next!=1))
            next_write_bank=1;
        else next_write_bank=2;
    end
    integer bank;
    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            claimed_valid<=0; latest_valid<=0; claimed_bank<=0; latest_bank<=0; claimed_seq<=0; latest_seq<=0;
            cycle_counter<=0;
            for(bank=0;bank<3;bank=bank+1) completed_cycle[bank]<=0;
        end else begin
            cycle_counter<=cycle_counter+1;
            claimed_valid<=claimed_valid_next; claimed_bank<=claimed_bank_next; claimed_seq<=claimed_seq_next;
            latest_valid<=latest_valid_next; latest_bank<=latest_bank_next; latest_seq<=latest_seq_next;
            if (commit_frame) completed_cycle[write_bank]<=cycle_counter;
        end
    end

    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            state<=IDLE; busy<=0; error<=0; frame_bad<=0;
            frame_seq<=0; errors<=0; overflows<=0; pixel_count<=0; last_error<=0; ticks<=0; drops<=0;
            reset_count<=0; pack_phase<=0; commit_wait<=0; packed_pixels<=0; write_bank<=0;
            request_toggle<=0; fifo_reset_axi<=1;
            mem_write<=0; mem_write_addr<=0; mem_write_data<=0;
        end else begin
            mem_write<=0;
            if (replacement) drops<=drops+1;
            if (abort_command) begin
                state<=IDLE; busy<=0; error<=0; last_error<=0; fifo_reset_axi<=1; frame_bad<=0;
            end else if (start_command && !busy) begin
                busy<=1; error<=0; last_error<=0; ticks<=0; frame_bad<=0;
                reset_count<=0; pixel_count<=0; pack_phase<=0; write_bank<=next_write_bank;
                fifo_reset_axi<=1; state<=RESET_FIFO;
            end else if (busy) begin
                ticks<=ticks+1;
                if (ticks>=TIMEOUT_CYCLES-1) begin
                    error<=1; errors<=errors+1; last_error<=3;
                    state<=RESET_FIFO; fifo_reset_axi<=1; reset_count<=0; ticks<=0;
                    pack_phase<=0; pixel_count<=0; frame_bad<=0;
                end else case (state)
                    RESET_FIFO: begin
                        reset_count<=reset_count+1;
                        if (reset_count==10000) begin fifo_reset_axi<=0; state<=WAIT_FIFO; end
                    end
                    WAIT_FIFO: if (!rd_busy && !wr_busy_sync) begin
                        request_toggle<=~request_toggle; state<=WAIT_SOF;
                    end
                    WAIT_SOF: if (fifo_read && fifo_out[17:16]==2'b01) begin
                        state<=RECEIVING; pixel_count<=0; pack_phase<=0; frame_bad<=0;
                    end
                    RECEIVING: if (fifo_read) begin
                        if (fifo_out[17:16]==2'b00) begin
                            if (pixel_count>=PIXELS) begin frame_bad<=1; end
                            else begin
                                case (pack_phase)
                                    0: packed_pixels[15:0]<=fifo_out[15:0];
                                    1: packed_pixels[31:16]<=fifo_out[15:0];
                                    2: packed_pixels[47:32]<=fifo_out[15:0];
                                    3: begin
                                        mem_write<=1; mem_write_addr<=bank_offset(write_bank)+(pixel_count>>2);
                                        mem_write_data<={fifo_out[15:0],packed_pixels};
                                    end
                                endcase
                                pack_phase<=pack_phase+1; pixel_count<=pixel_count+1;
                            end
                        end else if (fifo_out[17]) begin
                            if (fifo_out[16] || pixel_count!=PIXELS || pack_phase!=0 || frame_bad) begin
                                error<=1; errors<=errors+1; last_error<=fifo_out[16] ? 1 : 2;
                                if (fifo_out[0]) overflows<=overflows+1;
                                state<=WAIT_SOF; request_toggle<=~request_toggle; ticks<=0;
                                pack_phase<=0; pixel_count<=0; frame_bad<=0;
                            end else begin state<=COMMIT; commit_wait<=2; end
                        end else begin
                            error<=1; errors<=errors+1; last_error<=2;
                            state<=RESET_FIFO; fifo_reset_axi<=1; reset_count<=0; ticks<=0;
                            pack_phase<=0; pixel_count<=0; frame_bad<=0;
                        end
                    end
                    COMMIT: if (commit_wait!=0) commit_wait<=commit_wait-1;
                        else begin
                            frame_seq<=frame_seq+1; ticks<=0; write_bank<=next_write_bank;
                            state<=WAIT_SOF; request_toggle<=~request_toggle;
                            pixel_count<=0; pack_phase<=0; frame_bad<=0;
                        end
                    default: ;
                endcase
            end
        end
    end

    assign s_axi_arready=!s_axi_rvalid && read_wait==0;
    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_rvalid<=0; s_axi_rdata<=0; s_axi_rresp<=0; read_wait<=0; read_high_word<=0;
        end else begin
            if (s_axi_rvalid && s_axi_rready) s_axi_rvalid<=0;
            if (read_wait!=0) begin
                read_wait<=read_wait-1;
                if (read_wait==1) begin
                    s_axi_rdata<=read_high_word ? mem_read_data[63:32] : mem_read_data[31:0];
                    s_axi_rresp<=0; s_axi_rvalid<=1;
                end
            end
            if (read_accept) begin
                s_axi_rresp<=0;
                if (s_axi_araddr>=20'h01000) begin
                    if (mem_read_enable) begin read_wait<=2; read_high_word<=s_axi_araddr[2]; end
                    else begin s_axi_rvalid<=1; s_axi_rdata<=0; s_axi_rresp<=2'b10; end
                end else begin
                    s_axi_rvalid<=1;
                    case (s_axi_araddr)
                        20'h00000: s_axi_rdata<=32'h46524d32;
                        20'h00004: s_axi_rdata<=32'h00020000;
                        20'h00008: s_axi_rdata<=0;
                        20'h0000c: s_axi_rdata<={24'd0,state,latest_valid,error,claimed_valid,busy};
                        20'h00010: s_axi_rdata<=claimed_valid ? claimed_seq : latest_seq;
                        20'h00014: s_axi_rdata<=PIXELS*2;
                        20'h00018: s_axi_rdata<=errors;
                        20'h0001c: s_axi_rdata<=overflows;
                        20'h00020: s_axi_rdata<=claimed_valid ? PIXELS : 0;
                        20'h00024: s_axi_rdata<=(HEIGHT<<16)|WIDTH;
                        20'h00028: s_axi_rdata<=last_error;
                        20'h0002c: s_axi_rdata<=ticks;
                        20'h00030: s_axi_rdata<=frame_seq;
                        20'h00034: s_axi_rdata<=drops;
                        20'h00038: s_axi_rdata<=claimed_valid ? {30'd0,claimed_bank} : 32'hffffffff;
                        20'h0003c: s_axi_rdata<=latest_seq;
                        20'h00040: s_axi_rdata<=claimed_age[31:0];
                        20'h00044: s_axi_rdata<=32'd100000000;
                        20'h00048: s_axi_rdata<=claimed_age[63:32];
                        default: begin s_axi_rdata<=0; s_axi_rresp<=2'b10; end
                    endcase
                end
            end
        end
    end
    wire unused=&{1'b0,s_axi_awprot,s_axi_arprot};
`ifndef SYNTHESIS
    initial begin
        if (PIXELS%4!=0 || WORDS<2) $fatal(1,"Frame must contain a multiple of four pixels");
    end
    always @(posedge s_axi_aclk) if (s_axi_aresetn) begin
        if (fifo_read && (fifo_empty || rd_busy)) $fatal(1,"Illegal FIFO read");
        if (claimed_valid && latest_valid && claimed_bank==latest_bank) $fatal(1,"READING also READY");
        if (busy && claimed_valid && write_bank==claimed_bank) $fatal(1,"Writer owns READING bank");
        if (busy && latest_valid && write_bank==latest_bank) $fatal(1,"Writer owns READY bank");
        if (mem_write && claimed_valid && mem_write_addr>=bank_offset(claimed_bank) &&
            mem_write_addr<bank_offset(claimed_bank)+WORDS) $fatal(1,"Writing a frozen bank");
        if (mem_write && mem_write_addr>=WORDS*3) $fatal(1,"Frame address out of range");
    end
    always @(posedge cam_pclk) if (s_axi_aresetn) begin
        if (fifo_i.wr_en && (wr_busy || reset_pclk)) $fatal(1,"Illegal FIFO write during reset");
    end
`endif
endmodule
