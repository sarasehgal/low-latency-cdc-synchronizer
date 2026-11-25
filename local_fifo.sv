
`timescale 1ns/1ps


module local_fifo #(
    parameter int DATA_WIDTH = 5,
    parameter int DEPTH = 16,
    parameter int ADDR_WIDTH = $clog2(DEPTH)
) (
    input  logic                  write_clk,
    input  logic                  read_clk,
    input  logic                  rst,

    input  logic [DATA_WIDTH-1:0] wdata,
    input  logic                  write_en,
    input  logic                  read_en,

    output logic [DATA_WIDTH-1:0] rdata,
    output logic                  full,
    output logic                  empty
);

    logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // Binary pointers for memory addressing.
    logic [ADDR_WIDTH:0] wptr, rptr;
    logic [ADDR_WIDTH:0] wptr_next, rptr_next;

    // Gray-coded pointers for clock-domain crossing.
    logic [ADDR_WIDTH:0] wgray, rgray;
    logic [ADDR_WIDTH:0] wgray_next, rgray_next;

    // synced ptrs (mpam, or old 3ff w NO_MPAM)
    logic [ADDR_WIDTH:0] wgray_sync3, rgray_sync3;

    logic push, pop;
    logic full_next, empty_next;

    // Accepted read and write operations.
    assign push = write_en && !full && !rst;
    assign pop  = read_en && !empty && !rst;

    // Calculate next binary pointer positions.
    assign wptr_next = wptr + push;
    assign rptr_next = rptr + pop;

    // Convert binary pointers to Gray code.
    assign wgray_next = (wptr_next >> 1) ^ wptr_next;
    assign rgray_next = (rptr_next >> 1) ^ rptr_next;

    // Full detection in write clock domain.
    assign full_next = (wgray_next == {~rgray_sync3[ADDR_WIDTH:ADDR_WIDTH-1], rgray_sync3[ADDR_WIDTH-2:0]});

    // Empty detection in read clock domain.
    assign empty_next = (rgray_next == wgray_sync3);

    // Read forward - expose the next packet before read_en is asserted.
    assign rdata = (empty || rst) ? '0 : mem[rptr[ADDR_WIDTH-1:0]];

    // FIFO memory write.
    always_ff @(posedge write_clk) begin
        if (push)
            mem[wptr[ADDR_WIDTH-1:0]] <= wdata;
    end

    // Write pointer and full flag.
    always_ff @(posedge write_clk) begin
        if (rst) begin
            wptr  <= '0;
            wgray <= '0;
            full  <= 1'b0;
        end else begin
            wptr  <= wptr_next;
            wgray <= wgray_next;
            full  <= full_next;
        end
    end

    // Read pointer and empty flag.
    always_ff @(posedge read_clk) begin
        if (rst) begin
            rptr  <= '0;
            rgray <= '0;
            empty <= 1'b1;
        end else begin
            rptr  <= rptr_next;
            rgray <= rgray_next;
            empty <= empty_next;
        end
    end

`ifdef NO_MPAM
    // old 3ff sync
    logic [ADDR_WIDTH:0] wgray_sync1, wgray_sync2;
    logic [ADDR_WIDTH:0] rgray_sync1, rgray_sync2;
    always_ff @(posedge read_clk) begin
        if (rst) begin
            wgray_sync1 <= '0; wgray_sync2 <= '0; wgray_sync3 <= '0;
        end else begin
            wgray_sync1 <= wgray; wgray_sync2 <= wgray_sync1; wgray_sync3 <= wgray_sync2;
        end
    end
    always_ff @(posedge write_clk) begin
        if (rst) begin
            rgray_sync1 <= '0; rgray_sync2 <= '0; rgray_sync3 <= '0;
        end else begin
            rgray_sync1 <= rgray; rgray_sync2 <= rgray_sync1; rgray_sync3 <= rgray_sync2;
        end
    end
`else
    // mpam1 wptr->rclk, mpam2 rptr->wclk
    logic rclk_int, rclk_lag, wclk_int, wclk_lag;
    mpam_delay_line u_dly_r (.clk(read_clk),  .clk_int(rclk_int), .clk_lag(rclk_lag));
    mpam_delay_line u_dly_w (.clk(write_clk), .clk_int(wclk_int), .clk_lag(wclk_lag));

    mpam_sync #(.W(ADDR_WIDTH+1)) u_mpam1 (
        .rst_n(~rst), .clk_lead(read_clk), .clk_int(rclk_int), .clk_lag(rclk_lag),
        .ptr_in(wgray), .ptr_fast(wgray_sync3), .ptr_shadow(), .phase_sel(),
        .risk(), .err());
    mpam_sync #(.W(ADDR_WIDTH+1)) u_mpam2 (
        .rst_n(~rst), .clk_lead(write_clk), .clk_int(wclk_int), .clk_lag(wclk_lag),
        .ptr_in(rgray), .ptr_fast(rgray_sync3), .ptr_shadow(), .phase_sel(),
        .risk(), .err());
`endif

endmodule
