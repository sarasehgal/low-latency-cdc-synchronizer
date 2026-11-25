// tb_mpam.sv - tb for mpam_sync (wptr -> rclk)
// compares 1 flop vs normal 2-FF vs mpam, counts X's + latency
// run: xrun -sv +define+META_MODEL mpam_sync.sv tb_mpam.sv +WPER=13.7
// can also set +RPER= +D= +NCYC=
`timescale 1ns/1ps

module tb_mpam;
    localparam W = 5;

    real WPER = 13.7;    // wclk ~73 MHz
    real RPER = 10.0;    // rclk 100 MHz
    real D    = 1.5;     // delay step
    int  NCYC = 200000;

    reg  rst_n = 0;
    reg  wclk = 0, rclk = 0;
    wire clk_int, clk_lag;

    // fake delay line for now (replace w santiago's)
    reg clk_int_r = 0, clk_lag_r = 0;
    always @(rclk) clk_int_r <= #(D)     rclk;
    always @(rclk) clk_lag_r <= #(2.0*D) rclk;
    assign clk_int = clk_int_r;
    assign clk_lag = clk_lag_r;

    // write side: gray ptr, +1 every wclk
    reg [W-1:0] wbin = 0;
    wire [W-1:0] wgray = wbin ^ (wbin >> 1);
    always @(posedge wclk) if (rst_n) wbin <= wbin + 1'b1;

    // dut
    wire [W-1:0] ptr_fast, ptr_shadow;
    wire [1:0]   phase_sel;
    wire         risk, err;
    mpam_sync #(.W(W)) dut (
        .rst_n(rst_n), .clk_lead(rclk), .clk_int(clk_int), .clk_lag(clk_lag),
        .ptr_in(wgray), .ptr_fast(ptr_fast), .ptr_shadow(ptr_shadow),
        .phase_sel(phase_sel), .risk(risk), .err(err));

    // baselines to compare
    wire [W-1:0] b1;                       // 1-FF
    wire [W-1:0] b2a, b2;                  // 2-FF
    mpam_dff #(W) u_b1  (.clk(rclk), .rst_n(rst_n), .d(wgray), .q(b1));
    mpam_dff #(W) u_b2a (.clk(rclk), .rst_n(rst_n), .d(wgray), .q(b2a));
    mpam_dff #(W) u_b2b (.clk(rclk), .rst_n(rst_n), .d(b2a),   .q(b2));

    // check outputs right before next rclk edge
    int fail_1ff = 0, fail_2ff = 0, fail_mpam = 0, err_cnt = 0, risk_cnt = 0;
    int nsel[3] = '{0, 0, 0};
    int cyc = 0;

    // latency tracking
    realtime t_wr [0:31];
    real lat_mpam = 0, lat_2ff = 0;  int n_mpam = 0, n_2ff = 0;
    always @(wgray) t_wr[wgray] = $realtime;

    reg [W-1:0] last_mpam = 0, last_2ff = 0;
    always @(posedge rclk) if (rst_n) begin
        #(RPER - 0.1);
        cyc++;
        if (^b1       === 1'bx) fail_1ff++;
        if (^b2       === 1'bx) fail_2ff++;
        if (^ptr_fast === 1'bx) fail_mpam++;
        if (err)  err_cnt++;
        if (risk) risk_cnt++;
        if (phase_sel < 3) nsel[phase_sel]++;
        // new value seen -> add latency
        if (^ptr_fast !== 1'bx && ptr_fast != last_mpam) begin
            lat_mpam += ($realtime + 0.1) - t_wr[ptr_fast]; n_mpam++;
            last_mpam = ptr_fast;
        end
        if (^b2 !== 1'bx && b2 != last_2ff) begin
            lat_2ff += ($realtime + 0.1) - t_wr[b2]; n_2ff++;
            last_2ff = b2;
        end
    end

    // clocks, reset, run
    initial begin
        void'($value$plusargs("WPER=%f", WPER));
        void'($value$plusargs("RPER=%f", RPER));
        void'($value$plusargs("D=%f",    D));
        void'($value$plusargs("NCYC=%d", NCYC));
        fork
            forever #(WPER/2.0) wclk = ~wclk;
            #(0.37) forever #(RPER/2.0) rclk = ~rclk;   // random offset
        join_none
    end

    initial begin
        $dumpfile("mpam.vcd");
        $dumpvars(0, tb_mpam.wclk, tb_mpam.rclk, tb_mpam.clk_int, tb_mpam.clk_lag,
                     tb_mpam.wgray, tb_mpam.b1, tb_mpam.b2,
                     tb_mpam.ptr_fast, tb_mpam.ptr_shadow,
                     tb_mpam.phase_sel, tb_mpam.risk, tb_mpam.err,
                     tb_mpam.dut.s_lead, tb_mpam.dut.s_int, tb_mpam.dut.s_lag);
        #(5.0*RPER) rst_n = 1;
        #(400.0*RPER) $dumpoff;
        wait (cyc >= NCYC);
        $display("==================================================");
        $display(" MPAM test  rclk %0.2f ns  wclk %0.2f ns  D %0.2f ns", RPER, WPER, D);
        $display(" read cycles checked       : %0d", cyc);
        $display(" consumer saw X (failures) : 1FF %0d | 2FF %0d | MPAM %0d",
                 fail_1ff, fail_2ff, fail_mpam);
        $display(" avg latency (ns)          : 2FF %0.2f | MPAM %0.2f",
                 lat_2ff / n_2ff, lat_mpam / n_mpam);
        $display(" avg latency (rclk cycles) : 2FF %0.2f | MPAM %0.2f",
                 lat_2ff / n_2ff / RPER, lat_mpam / n_mpam / RPER);
        $display(" risky cycles detected     : %0d", risk_cnt);
        $display(" phase used lead/int/lag   : %0d / %0d / %0d", nsel[0], nsel[1], nsel[2]);
        $display(" shadow-sync err flags     : %0d", err_cnt);
        $display("==================================================");
        $finish;
    end
endmodule
