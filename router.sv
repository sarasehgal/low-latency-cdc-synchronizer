// =============================================================================
// This module implements a router for a 2x2 NoC with router_logic 
// and one local_fifo on each input (read) port.
// It allows routers to connect to eachother directly at the top level with only wires.
//
// IDs correspond to the following positions in a 2x2 gird.
//     R0 (00) ---- R1 (01)
//       |            |
//     R2 (10) ---- R3 (11)
//
// The router will pop a FIFO whenever it is not empty and the router is 
// ready to transmit a new packet from that port.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module router #(
    parameter [1:0] MY_ID      = 2'b00,
    parameter int   DATA_W     = 3,
    parameter int   FIFO_DEPTH = 16
)(
    input  wire              clk,        // this router's clock domain (also the PE's clock domain)
    input  wire              rst_n,

    // ---------------- input ports (FIFO write sides) ----------------
    input  wire              l_in_valid,
    output wire              l_in_full,
    input  wire [DATA_W-1:0] l_in_wdata,
    input  wire [1:0]        l_in_dest_addr,

    input  wire              x_in_clock,
    input  wire              x_in_valid,
    output wire              x_in_full,
    input  wire [DATA_W-1:0] x_in_wdata,
    input  wire [1:0]        x_in_dest_addr,

    input  wire              y_in_clock,
    input  wire              y_in_valid,
    output wire              y_in_full,
    input  wire [DATA_W-1:0] y_in_wdata,
    input  wire [1:0]        y_in_dest_addr,

    // ---------------- output ports (router_logic write ports) ----------------
    output wire              l_out_valid,
    input  wire              l_out_full,
    output wire [DATA_W-1:0] l_out_wdata,

    output wire              x_out_clock,
    output wire              x_out_valid,
    input  wire              x_out_full,
    output wire [DATA_W-1:0] x_out_wdata,
    output wire [1:0]        x_out_dest_addr,

    output wire              y_out_clock,
    output wire              y_out_valid,
    input  wire              y_out_full,
    output wire [DATA_W-1:0] y_out_wdata,
    output wire [1:0]        y_out_dest_addr
);

    localparam int FLIT_W = DATA_W + 2;      // {dest_addr[1:0], wdata}
    localparam int NP     = 3; // number of router ports (local, X, Y)
    localparam int P_L    = 0; // Port index for local port
    localparam int P_X    = 1; // Port index for router in adjacent row 
    localparam int P_Y    = 2; // Port index for router in adjacent column

    wire rst = ~rst_n;

    // Input-port signals, indexed by port.
    wire              in_clock [0:NP-1];
    wire              in_valid [0:NP-1];
    wire              in_full  [0:NP-1];
    wire [FLIT_W-1:0] in_flit  [0:NP-1];

    assign in_clock[P_L] = clk;
    assign in_clock[P_X] = x_in_clock;
    assign in_clock[P_Y] = y_in_clock;
    assign in_valid[P_L] = l_in_valid;
    assign in_valid[P_X] = x_in_valid;
    assign in_valid[P_Y] = y_in_valid;
    assign in_flit[P_L]  = {l_in_dest_addr, l_in_wdata};
    assign in_flit[P_X]  = {x_in_dest_addr, x_in_wdata};
    assign in_flit[P_Y]  = {y_in_dest_addr, y_in_wdata};
    assign l_in_full     = in_full[P_L];
    assign x_in_full     = in_full[P_X];
    assign y_in_full     = in_full[P_Y];

    // FIFO read (pop) signals.
    wire [FLIT_W-1:0] q_rdata [0:NP-1];
    wire              q_empty [0:NP-1];
    wire              q_ready [0:NP-1];

    genvar p;
    generate
        for (p = 0; p < NP; p = p + 1) begin : g_in
            local_fifo #(
                .DATA_WIDTH (FLIT_W),
                .DEPTH      (FIFO_DEPTH)
            ) u_fifo (
                .write_clk (in_clock[p]),
                .read_clk  (clk),
                .rst       (rst),
                .wdata     (in_flit[p]),
                .write_en  (in_valid[p]),
                .read_en   (~q_empty[p] & q_ready[p]),
                .rdata     (q_rdata[p]),
                .full      (in_full[p]),
                .empty     (q_empty[p])
            );
        end
    endgenerate

    wire [FLIT_W-1:0] l_out_flit, x_out_flit, y_out_flit;

    router_logic #(
        .MY_ID  (MY_ID),
        .DATA_W (DATA_W)
    ) u_logic (
        .clk         (clk),
        .rst_n       (rst_n),

        .l_in_valid  (~q_empty[P_L]),
        .l_in_flit   (q_rdata[P_L]),
        .l_in_ready  (q_ready[P_L]),

        .x_in_valid  (~q_empty[P_X]),
        .x_in_flit   (q_rdata[P_X]),
        .x_in_ready  (q_ready[P_X]),

        .y_in_valid  (~q_empty[P_Y]),
        .y_in_flit   (q_rdata[P_Y]),
        .y_in_ready  (q_ready[P_Y]),

        .l_out_wr_en (l_out_valid),
        .l_out_wdata (l_out_flit),
        .l_out_full  (l_out_full),

        .x_out_wr_en (x_out_valid),
        .x_out_wdata (x_out_flit),
        .x_out_full  (x_out_full),

        .y_out_wr_en (y_out_valid),
        .y_out_wdata (y_out_flit),
        .y_out_full  (y_out_full)
    );

    // The PE does not need to know the destination address.
    // if l_out_wdata is valid then it was meant for the PE. Our router makes no mistakes.
    assign l_out_wdata = l_out_flit[DATA_W-1:0];
    assign {x_out_dest_addr, x_out_wdata} = x_out_flit;
    assign {y_out_dest_addr, y_out_wdata} = y_out_flit;

    assign x_out_clock = clk;
    assign y_out_clock = clk;

endmodule

`default_nettype wire
