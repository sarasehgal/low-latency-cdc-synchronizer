// =============================================================================
// router_logic.v (formerly noc_router.v)
// single Router for 2x2 NoC
// one per router, and each only differs by MY_ID.
//
//     R0 (00) ---- R1 (01)
//       |            |
//     R2 (10) ---- R3 (11)

//   L - its own PE
//   X - the other router in the same row
//   Y - the other router in the same column
//
// flit width: 5 bit 
//flit strucutre : {dest_addr[1:0], data[2:0]}.
// Routing (XY): if the flit is in the wrong column, send it out to the router of the same Row X
//if the flit is in the column is right but row is wrong, it sends to the router of the same column Y
// if both column and row matches, it sends to its own pe L. 
//   Inputs:  *_in port connects to the read side of an input FIFO.
//            *_in_valid means the FIFO has a flit, *_in_flit is flit data itself
//  and the router sets *_in_ready to take it.
//   Outputs: *_out port connects to the write side of downstream FIFO
//            writes when *_out_full is 0.
//
// Each output port has a round-robin arbiter, a mux, and one pipeline register.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module router_logic #(
    parameter [1:0] MY_ID  = 2'b00,   // id for each router R0=00, R1=01, R2=10, R3=11
    parameter       DATA_W = 3        // 3 payload bits
)(
    input  wire                 clk,
    input  wire                 rst_n,      //active-low reset

    //Input ports to reading FIFO
    input  wire                 l_in_valid,
    input  wire [DATA_W+1:0]    l_in_flit,
    output wire                 l_in_ready,

    input  wire                 x_in_valid,
    input  wire [DATA_W+1:0]    x_in_flit,
    output wire                 x_in_ready,

    input  wire                 y_in_valid,
    input  wire [DATA_W+1:0]    y_in_flit,
    output wire                 y_in_ready,

    //output ports to writing FIFO
    output wire                 l_out_wr_en,
    output wire [DATA_W+1:0]    l_out_wdata,
    input  wire                 l_out_full,

    output wire                 x_out_wr_en,
    output wire [DATA_W+1:0]    x_out_wdata,
    input  wire                 x_out_full,

    output wire                 y_out_wr_en,
    output wire [DATA_W+1:0]    y_out_wdata,
    input  wire                 y_out_full
);

    localparam FLIT_W = DATA_W + 2;
    localparam NP     = 3;            //total no. ports
    localparam P_L    = 0;
    localparam P_X    = 1;
    localparam P_Y    = 2;

    localparam MY_X = MY_ID[0];
    localparam MY_Y = MY_ID[1];

    wire [NP-1:0]     in_valid = {y_in_valid, x_in_valid, l_in_valid};
    wire [NP-1:0]     out_full = {y_out_full, x_out_full, l_out_full};
    wire [FLIT_W-1:0] in_flit [0:NP-1];
    assign in_flit[P_L] = l_in_flit;
    assign in_flit[P_X] = x_in_flit;
    assign in_flit[P_Y] = y_in_flit;
    wire [NP*NP-1:0] req;
    //req[go*3 + gi] 
    genvar gi;
    /*
        Bit	From (input, gi) To (output, go)
        req[0]	L-in	    L-out
        req[1]	X-in	    L-out
        req[2]	Y-in	    L-out
        req[3]	L-in	    X-out
        req[4]	X-in	    X-out
        req[5]	Y-in	    X-out
        req[6]	L-in	    Y-out
        req[7]	X-in	    Y-out
        req[8]	Y-in	    Y-out
    */

    generate
        for (gi = 0; gi < NP; gi = gi + 1) begin : g_rc
            wire dx, dy;
            // Route towards X before Y.
            // MY_X and MY_Y are constants, so these XORs resolve to wires/inverters in synthesis.
            assign dx = in_flit[gi][FLIT_W-2] ^ MY_X;
            assign dy = in_flit[gi][FLIT_W-1] ^ MY_Y;

            assign req[P_X*NP + gi] = in_valid[gi] &  dx;
            assign req[P_Y*NP + gi] = in_valid[gi] & ~dx &  dy;
            assign req[P_L*NP + gi] = in_valid[gi] & ~dx & ~dy;
        end
    endgenerate
    
    // ------------------------------------------------------------------------
    //  arbite (round robin) connnected to mux and 1 pipeline register
    // ------------------------------------------------------------------------
    wire [NP*NP-1:0]  gnt;               // gnt[o*NP + i]
    wire [NP-1:0]     wr_en;
    wire [FLIT_W-1:0] wdata   [0:NP-1];

    genvar o;
    generate
        for (o = 0; o < NP; o = o + 1) begin : g_out
            reg              s1_v;
            reg [FLIT_W-1:0] s1_d;

        
            wire s1_fire = s1_v & ~out_full[o];   // write downstream this cycle
            wire s1_free = ~s1_v | s1_fire;       // empty, or replacing a departing flit

            wire [NP-1:0] arbiter_req = {NP{s1_free}} & req[o*NP +: NP];
            /*
            o	o*3	req[o*3 +: 3]	output_port
            0	0	req[2:0]	    L-out
            1	3	req[5:3]	    X-out
            2	6	req[8:6]	    Y-out
            */
            rr_arbiter #(.N(NP)) u_arb (
                .clk   (clk),
                .rst_n (rst_n),
                .req   (arbiter_req),
                .gnt   (gnt[o*NP +: NP])
            );

            // gnt for this output: 3 bits, at most one set = the input that wins this cycle.
            // gnt = 000 if no input requests this output port, or stage 1 is blocked.
            wire [NP-1:0] my_gnt      = gnt[o*NP +: NP];
            wire          someone_won = |my_gnt;    // 1 if any input won this output

            // Crossbar mux: pass the winning input's flit
            wire [FLIT_W-1:0] xbar;
            assign xbar = in_flit[P_L] & {FLIT_W{my_gnt[0]}} |
                           in_flit[P_X] & {FLIT_W{my_gnt[1]}} |
                           in_flit[P_Y] & {FLIT_W{my_gnt[2]}};

            // Single elastic output stage: hold data while blocked; allow
            // simultaneous downstream write and capture of the next flit.
            // Valid bit (reset)
            always @(posedge clk) begin
                if (!rst_n) begin
                    s1_v <= 1'b0;
                end else begin
                    if (s1_free) s1_v <= someone_won;
                end
            end
            always @(posedge clk) begin
                if (s1_free && someone_won) s1_d <= xbar;
            end

            assign wr_en[o] = s1_fire;
            assign wdata[o] = s1_d;
        end
    endgenerate

    // Each input requests exactly one output, so at most one grant per input.
    wire [NP-1:0] in_ready = gnt[P_L*NP +: NP] | gnt[P_X*NP +: NP] | gnt[P_Y*NP +: NP];

    assign l_in_ready  = in_ready[P_L];
    assign x_in_ready  = in_ready[P_X];
    assign y_in_ready  = in_ready[P_Y];

    assign l_out_wr_en = wr_en[P_L];
    assign x_out_wr_en = wr_en[P_X];
    assign y_out_wr_en = wr_en[P_Y];
    assign l_out_wdata = wdata[P_L];
    assign x_out_wdata = wdata[P_X];
    assign y_out_wdata = wdata[P_Y];

endmodule


// arbiter -- roundrobin

module rr_arbiter #(
    parameter N = 3
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire [N-1:0] req,
    output wire [N-1:0] gnt
);
    reg  [N-1:0]   pri;  

    // Double-width trick
    wire [2*N-1:0] dreq = {req, req};
    wire [2*N-1:0] dgnt = dreq & ~(dreq - {{N{1'b0}}, pri});
    assign gnt          = dgnt[N-1:0] | dgnt[2*N-1:N];


    always @(posedge clk) begin
        if (!rst_n)
            pri <= {{(N-1){1'b0}}, 1'b1};
        else if (|gnt)
            pri <= {gnt[N-2:0], gnt[N-1]};   // The winner's bit moves up one position, with wraparound:
    end
endmodule

`default_nettype wire
