// =============================================================================
// noc_top_tb.sv
// Traffic-generator testbench for the 2x2 NoC top level (noc_top).
//
// Each PE r has, in its own clock domain clk[r]:
//   - a generator that writes flits into router r's L-in FIFO, holding each
//     flit stable until it is accepted (valid && !full at a rising edge);
//   - a sink on router r's L-out write port that drives pe_out_full randomly
//     (backpressure) and checks every delivered flit against a scoreboard.
//
// Flit = {dest_addr[1:0], wdata} with wdata = {src[1:0], seq[DATA_W-3:0]}
// XY routing is deterministic, so flits from one source to one destination must
// arrive in order. The scoreboard keeps one queue per (source, destination).
//
// MULTI_CLK = 0 : all four routers run on the same clock.
// MULTI_CLK = 1 : router frequencies are clk, clk*0.99, clk*0.97, clk*0.7 with
//                 different start phases.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_noc_top #(
    parameter int  DATA_W      = 3,
    parameter int  FIFO_DEPTH  = 16,
    parameter bit  MULTI_CLK   = 0,
    parameter int  NUM_FLITS   = 200,     // flits per source, per traffic phase
    parameter real BASE_PERIOD = 10.0     // ns, period of clk (router 0)
);

    localparam int FLIT_W = DATA_W + 2;
    localparam int SEQ_W  = DATA_W - 2;
    typedef logic [FLIT_W-1:0] flit_t;
    typedef logic [DATA_W-1:0] payload_t;

    // -------------------------------------------------------------------------
    // DUT
    // -------------------------------------------------------------------------
    logic  [3:0]  clk;
    logic         rst_n;
    logic  [3:0]        pe_in_valid;
    flit_t              pe_in_flit       [0:3];   // {dest_addr, wdata}
    wire   [3:0]        pe_in_full;
    wire   [DATA_W-1:0] pe_in_wdata      [0:3];
    wire   [1:0]        pe_in_dest_addr  [0:3];
    wire   [3:0]        pe_out_valid;
    logic  [3:0]        pe_out_full;
    wire   [DATA_W-1:0] pe_out_wdata     [0:3];

    for (genvar gi = 0; gi < 4; gi++) begin : g_split
        assign {pe_in_dest_addr[gi], pe_in_wdata[gi]} = pe_in_flit[gi];
    end

    noc_top #(
        .DATA_W     (DATA_W),
        .FIFO_DEPTH (FIFO_DEPTH)
    ) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .pe_in_valid      (pe_in_valid),
        .pe_in_full       (pe_in_full),
        .pe_in_wdata      (pe_in_wdata),
        .pe_in_dest_addr  (pe_in_dest_addr),
        .pe_out_valid     (pe_out_valid),
        .pe_out_full      (pe_out_full),
        .pe_out_wdata     (pe_out_wdata)
    );

    // -------------------------------------------------------------------------
    // Clocks
    // -------------------------------------------------------------------------
    function automatic real clk_ratio(input int i);
        if (!MULTI_CLK) return 1.0;
        case (i)
            0:       return 1.00;
            1:       return 0.99;
            2:       return 0.97;
            default: return 0.70;
        endcase
    endfunction

    function automatic real clk_phase(input int i);   // ns before first edge
        if (!MULTI_CLK) return 0.0;
        case (i)
            0:       return 0.0;
            1:       return 1.3;
            2:       return 2.7;
            default: return 4.1;
        endcase
    endfunction

    function automatic real clk_period(input int i);
        return BASE_PERIOD / clk_ratio(i);
    endfunction

    genvar gc;
    generate
        for (gc = 0; gc < 4; gc = gc + 1) begin : g_clk
            initial begin
                clk[gc] = 1'b0;
                #(clk_phase(gc));
                forever #(clk_period(gc) / 2.0) clk[gc] = ~clk[gc];
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Traffic configuration (set by the main sequence, read by generators/sinks)
    // -------------------------------------------------------------------------
    int gen_mode;          // 0 = uniform random destination, 1 = hotspot
    int hot_dst;
    int inj_pct;           // chance (%) that an idle source injects a flit
    int sink_pct;          // chance (%) that a sink is not full on a given cycle
    int phase_quota;       // flits each source injects this phase
    int issued [0:3];
    int seq    [0:3];

    // -------------------------------------------------------------------------
    // Scoreboard and statistics
    // -------------------------------------------------------------------------
    payload_t sb_q [0:3][0:3][$];
    realtime sb_t [0:3][0:3][$];
    int      sent_pair [0:3][0:3];
    int      rcvd_pair [0:3][0:3];
    int      sent_total, rcvd_total, errors;
    int      overflow_err, underflow_err, full_cycles;
    real     lat_sum, lat_max;
    int      lat_cnt;

    initial begin
        sent_total = 0; rcvd_total = 0; errors = 0;
        overflow_err = 0; underflow_err = 0; full_cycles = 0;
        lat_sum = 0.0; lat_max = 0.0; lat_cnt = 0;
        gen_mode = 0; hot_dst = 0; inj_pct = 0; sink_pct = 100; phase_quota = 0;
        for (int s = 0; s < 4; s++) begin
            issued[s] = 0;
            seq[s]    = 0;
            for (int d = 0; d < 4; d++) begin
                sent_pair[s][d] = 0;
                rcvd_pair[s][d] = 0;
            end
        end
    end

    function automatic bit sb_empty();
        for (int s = 0; s < 4; s++)
            for (int d = 0; d < 4; d++)
                if (sb_q[s][d].size() != 0) return 1'b0;
        return 1'b1;
    endfunction

    task automatic check_rx(input int dst, input payload_t payload);
        int       src;
        payload_t expected;
        realtime t0;
        real     lat;
        if ($isunknown(payload)) begin
            $display("[FAIL] %0t: PE%0d received payload with unknown bits: %b",
                     $realtime, dst, payload);
            errors++;
            return;
        end
        src = payload[DATA_W-1 -: 2];
        if (sb_q[src][dst].size() == 0) begin
            $display("[FAIL] %0t: PE%0d received unexpected payload %b (src R%0d)",
                     $realtime, dst, payload, src);
            errors++;
            return;
        end
        expected = sb_q[src][dst].pop_front();
        t0    = sb_t[src][dst].pop_front();
        if (payload !== expected) begin
            $display("[FAIL] %0t: R%0d->R%0d out of order or corrupt. Expected %b, got %b",
                     $realtime, src, dst, expected, payload);
            errors++;
        end
        rcvd_total++;
        rcvd_pair[src][dst]++;
        lat     = $realtime - t0;
        lat_sum += lat;
        lat_cnt++;
        if (lat > lat_max) lat_max = lat;
    endtask

    // -------------------------------------------------------------------------
    // Per-PE generator and sink, each in its router's clock domain
    // -------------------------------------------------------------------------
    genvar gs;
    generate
        for (gs = 0; gs < 4; gs = gs + 1) begin : g_pe
            localparam logic [1:0] SRC = gs;

            always @(posedge clk[gs]) begin : pe_proc
                logic [1:0] d;
                int         acc_dst;
                if (!rst_n) begin
                    pe_in_valid[gs]  <= 1'b0;
                    pe_in_flit[gs]   <= '0;
                    pe_out_full[gs]  <= 1'b1;
                end else begin
                    // ---- source: record accepted flit ----
                    if (pe_in_valid[gs] && !pe_in_full[gs]) begin
                        acc_dst = pe_in_flit[gs][FLIT_W-1 -: 2];
                        sb_q[gs][acc_dst].push_back(pe_in_wdata[gs]);
                        sb_t[gs][acc_dst].push_back($realtime);
                        sent_pair[gs][acc_dst]++;
                        sent_total++;
                    end
                    // ---- source: present the next flit (hold while not accepted) ----
                    if (!pe_in_valid[gs] || !pe_in_full[gs]) begin
                        if (issued[gs] < phase_quota && $urandom_range(99) < inj_pct) begin
                            d = (gen_mode == 1) ? hot_dst[1:0] : 2'($urandom_range(3));
                            pe_in_flit[gs]  <= {d, SRC, seq[gs][SEQ_W-1:0]};
                            pe_in_valid[gs] <= 1'b1;
                            issued[gs]++;
                            seq[gs]++;
                        end else begin
                            pe_in_valid[gs] <= 1'b0;
                        end
                    end

                    // ---- sink ----
                    if ($isunknown(pe_out_valid[gs]) || $isunknown(pe_in_full[gs])) begin
                        $display("[FAIL] %0t: X on PE%0d handshake (out_valid=%b in_full=%b)",
                                 $realtime, gs, pe_out_valid[gs], pe_in_full[gs]);
                        errors++;
                    end
                    if (pe_out_valid[gs] && pe_out_full[gs]) begin
                        $display("[FAIL] %0t: R%0d L-out wrote while PE%0d full", $realtime, gs, gs);
                        errors++;
                    end
                    if (pe_out_valid[gs] && !pe_out_full[gs]) begin
                        // Check the route before the wrapper strips the address.
                        if (dut.g_router[gs].u_router.l_out_flit[FLIT_W-1 -: 2] !== SRC) begin
                            $display("[FAIL] %0t: PE%0d received flit addressed to R%b",
                                     $realtime, gs,
                                     dut.g_router[gs].u_router.l_out_flit[FLIT_W-1 -: 2]);
                            errors++;
                        end
                        check_rx(gs, pe_out_wdata[gs]);
                    end
                    pe_out_full[gs] <= !($urandom_range(99) < sink_pct);
                end
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // FIFO protocol monitors (input FIFOs inside each router)
    //   port 0 = L (written by PE r), 1 = X (by R r^1), 2 = Y (by R r^2)
    //   no write while full on router-driven X/Y ports (the PE legitimately
    //   holds valid while full; the FIFO ignores that write), no read while empty
    // -------------------------------------------------------------------------
    genvar gr, gp;
    generate
        for (gr = 0; gr < 4; gr = gr + 1) begin : g_mon_rtr
            for (gp = 0; gp < 3; gp = gp + 1) begin : g_mon_port
                localparam int WSRC = (gp == 0) ? gr : (gp == 1) ? (gr ^ 1) : (gr ^ 2);

                always @(posedge clk[WSRC]) if (rst_n) begin
                    if (dut.g_router[gr].u_router.g_in[gp].u_fifo.full) full_cycles++;
                    if (gp != 0 &&
                        dut.g_router[gr].u_router.g_in[gp].u_fifo.write_en &&
                        dut.g_router[gr].u_router.g_in[gp].u_fifo.full) begin
                        $display("[FAIL] %0t: write to full FIFO R%0d port %0d", $realtime, gr, gp);
                        overflow_err++;
                        errors++;
                    end
                end

                always @(posedge clk[gr]) if (rst_n) begin
                    if (dut.g_router[gr].u_router.g_in[gp].u_fifo.read_en &&
                        dut.g_router[gr].u_router.g_in[gp].u_fifo.empty) begin
                        $display("[FAIL] %0t: read from empty FIFO R%0d port %0d", $realtime, gr, gp);
                        underflow_err++;
                        errors++;
                    end
                end
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Main sequence
    // -------------------------------------------------------------------------
    localparam real SLOW_PERIOD = MULTI_CLK ? (BASE_PERIOD / 0.70) : BASE_PERIOD;
    localparam real PHASE_TIMEOUT = 400000.0;   // ns

    task automatic run_phase(input string name, input int mode, input int hot,
                             input int inj, input int sink, input int quota);
        realtime t_start, t_inj_done;
        int      sent0, rcvd0;
        bit      busy;

        // Configure atomically (no timing control in between).
        gen_mode = mode;
        hot_dst  = hot;
        inj_pct  = inj;
        sink_pct = sink;
        for (int s = 0; s < 4; s++) issued[s] = 0;
        phase_quota = quota;
        lat_sum = 0.0; lat_max = 0.0; lat_cnt = 0;
        sent0 = sent_total; rcvd0 = rcvd_total;
        t_start = $realtime;

        // Wait until every source has injected its quota.
        do begin
            @(posedge clk[0]); #0.001;   // sample after all same-edge updates settle
            busy = 1'b0;
            for (int s = 0; s < 4; s++)
                if (issued[s] < quota || pe_in_valid[s] !== 1'b0) busy = 1'b1;
            if ($realtime - t_start > PHASE_TIMEOUT) begin
                $display("[FAIL] %s: injection timeout (possible deadlock)", name);
                errors++;
                busy = 1'b0;
            end
        end while (busy);
        t_inj_done = $realtime;

        // Drain the network.
        while (!sb_empty()) begin
            @(posedge clk[0]); #0.001;
            if ($realtime - t_start > PHASE_TIMEOUT) begin
                $display("[FAIL] %s: drain timeout, flits stuck in network", name);
                errors++;
                break;
            end
        end

        $display("  %-28s sent %4d  rcvd %4d  inject %8.1f ns  total %8.1f ns  lat avg %6.1f max %6.1f ns",
                 name, sent_total - sent0, rcvd_total - rcvd0,
                 t_inj_done - t_start, $realtime - t_start,
                 (lat_cnt > 0) ? lat_sum / lat_cnt : 0.0, lat_max);
    endtask

    initial begin
        $display("==========================================================================");
        $display(" tb_noc_top: DATA_W=%0d FIFO_DEPTH=%0d NUM_FLITS=%0d MULTI_CLK=%0d",
                 DATA_W, FIFO_DEPTH, NUM_FLITS, MULTI_CLK);
        for (int i = 0; i < 4; i++)
            $display("   R%0d clock: %.4f ns (%.2f x clk), first edge offset %.1f ns",
                     i, clk_period(i), clk_ratio(i), clk_phase(i));
        $display("==========================================================================");

        rst_n = 1'b0;
        for (int s = 0; s < 4; s++) begin
            pe_in_valid[s]  = 1'b0;
            pe_in_flit[s]   = '0;
            pe_out_full[s]  = 1'b1;
        end

        // Hold reset for 20 cycles of the slowest clock, release off any edge.
        #(20.0 * SLOW_PERIOD + 0.123);
        rst_n = 1'b1;
        #(5.0 * SLOW_PERIOD);

        //        name                          mode hot inj sink quota
        run_phase("uniform, full rate",           0,  0, 100, 100, NUM_FLITS);
        run_phase("uniform, sink backpressure",   0,  0,  70,  25, NUM_FLITS);
        run_phase("hotspot -> R3 (slowest)",      1,  3, 100,  60, NUM_FLITS / 2);
        run_phase("hotspot -> R0",                1,  0, 100, 100, NUM_FLITS / 2);
        run_phase("uniform, low rate",            0,  0,  10, 100, NUM_FLITS / 2);

        // Idle: any further arrival would be flagged as unexpected.
        #(50.0 * SLOW_PERIOD);

        $display("--------------------------------------------------------------------------");
        $display(" Flits delivered per source (row) -> destination (column):");
        $display("            R0    R1    R2    R3");
        for (int s = 0; s < 4; s++)
            $display("   R%0d   %5d %5d %5d %5d", s,
                     rcvd_pair[s][0], rcvd_pair[s][1], rcvd_pair[s][2], rcvd_pair[s][3]);
        $display(" FIFO full cycles (summed over 12 FIFOs): %0d", full_cycles);
        $display(" FIFO write-while-full: %0d   read-while-empty: %0d", overflow_err, underflow_err);
        $display(" Summary: Sent = %0d | Received = %0d | Errors = %0d",
                 sent_total, rcvd_total, errors);
        if (errors == 0 && sent_total == rcvd_total && sent_total > 0 && sb_empty())
            $display(" RESULT: PASS");
        else
            $display(" RESULT: FAIL");
        $display("==========================================================================");
        $finish;
    end

endmodule

`default_nettype wire
