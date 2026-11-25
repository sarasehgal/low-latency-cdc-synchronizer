// =============================================================================
// This module wires all the routers together. 
// Exposes ports for the PEs to connect to the network.
//
// IDs correspond to the following positions in a 2x2 gird.
//     R0 (00) ---- R1 (01)
//       |            |
//     R2 (10) ---- R3 (11)
//
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module noc_top #(
    parameter int DATA_W     = 3,              // payload bits
    parameter int FIFO_DEPTH = 16              // Buffer size. There are a total of 12 FIFOs in the NoC, 3 per router.
)(
    input  wire  [3:0]        clk,             // PE and corresponding router clock domains
    input  wire               rst_n,

    // PE -> router L-in FIFO (write side, clk[r])
    input  wire  [3:0]        pe_in_valid,
    output wire  [3:0]        pe_in_full,
    input  wire  [DATA_W-1:0] pe_in_wdata     [0:3],
    input  wire  [1:0]        pe_in_dest_addr [0:3],

    // router L-out -> PE (write port, clk[r])
    output wire  [3:0]        pe_out_valid,
    input  wire  [3:0]        pe_out_full,
    output wire  [DATA_W-1:0] pe_out_wdata     [0:3]
);

    // X/Y output-port signals, indexed by the source router.
    wire              x_clock [0:3], y_clock [0:3];
    wire              x_valid [0:3], y_valid [0:3];
    wire [DATA_W-1:0] x_wdata [0:3], y_wdata [0:3];
    wire [1:0]        x_dest  [0:3], y_dest  [0:3];
    // X/Y input-port full, indexed by the receiving router.
    wire              x_full  [0:3], y_full  [0:3];

    genvar r;
    generate
        for (r = 0; r < 4; r = r + 1) begin : g_router
            localparam [1:0] ID  = r;
            localparam int   XNB = r ^ 1;   // r's X neighbour (same row)
            localparam int   YNB = r ^ 2;   // r's Y neighbour (same column)


            router #(
                .MY_ID      (ID),
                .DATA_W     (DATA_W),
                .FIFO_DEPTH (FIFO_DEPTH)
            ) u_router (
                .clk             (clk[r]),
                .rst_n           (rst_n),

                // L-in from PE
                .l_in_valid      (pe_in_valid[r]),
                .l_in_full       (pe_in_full[r]),
                .l_in_wdata      (pe_in_wdata[r]),
                .l_in_dest_addr  (pe_in_dest_addr[r]),

                // X-in from r's X neighbour to r
                .x_in_clock      (x_clock[XNB]),
                .x_in_valid      (x_valid[XNB]),
                .x_in_full       (x_full[r]),
                .x_in_wdata      (x_wdata[XNB]),
                .x_in_dest_addr  (x_dest[XNB]),

                // Y-in from r's Y neighbour to r
                .y_in_clock      (y_clock[YNB]),
                .y_in_valid      (y_valid[YNB]),
                .y_in_full       (y_full[r]),
                .y_in_wdata      (y_wdata[YNB]),
                .y_in_dest_addr  (y_dest[YNB]),

                // L-out to PE
                .l_out_valid     (pe_out_valid[r]),
                .l_out_full      (pe_out_full[r]),
                .l_out_wdata     (pe_out_wdata[r]),

                // X-out from r to its X neighbour
                .x_out_clock     (x_clock[r]),
                .x_out_valid     (x_valid[r]),
                .x_out_full      (x_full[XNB]),
                .x_out_wdata     (x_wdata[r]),
                .x_out_dest_addr (x_dest[r]),

                // Y-out from r to its Y neighbour
                .y_out_clock     (y_clock[r]),
                .y_out_valid     (y_valid[r]),
                .y_out_full      (y_full[YNB]),
                .y_out_wdata     (y_wdata[r]),
                .y_out_dest_addr (y_dest[r])
            );
        end
    endgenerate

endmodule

`default_nettype wire
