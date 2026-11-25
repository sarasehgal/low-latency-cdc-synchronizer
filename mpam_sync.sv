// mpam - digital part (lin et al jssc 2024)
// 1 per crossing. sample ptr on 3 phases, pick safest one
// TODO match santiago's delay cell
`timescale 1ns/1ps

module mpam_sync #(
    parameter W = 5                   // ptr width
)(
    input  wire         rst_n,
    input  wire         clk_lead,     // local clk
    input  wire         clk_int,      // lead + D
    input  wire         clk_lag,      // lead + 2D
    input  wire [W-1:0] ptr_in,       // gray ptr in
    output reg  [W-1:0] ptr_fast,     // mpam output
    output wire [W-1:0] ptr_shadow,   // 2ff ref
    output reg  [1:0]   phase_sel,    // 0 lead 1 int 2 lag
    output reg          risk,
    output reg          err
);

    // sample x3
    wire [W-1:0] s_lead, s_int, s_lag;
    mpam_dff #(W) u_s_lead (.clk(clk_lead), .rst_n(rst_n), .d(ptr_in), .q(s_lead));
    mpam_dff #(W) u_s_int  (.clk(clk_int),  .rst_n(rst_n), .d(ptr_in), .q(s_int));
    mpam_dff #(W) u_s_lag  (.clk(clk_lag),  .rst_n(rst_n), .d(ptr_in), .q(s_lag));

    // detectors - differ = changed here
`ifdef META_MODEL
    // sim: X = changed
    wire early = ((s_lead ^ s_int) !== {W{1'b0}});
    wire late  = ((s_int  ^ s_lag) !== {W{1'b0}});
`else
    wire early = (s_lead != s_int);
    wire late  = (s_int  != s_lag);
`endif

    // early->lag, late->lead, else int
    reg [1:0] sel;
    always @(*) begin
        if (early)     begin sel = 2'd2; ptr_fast = s_lag;  end
        else if (late) begin sel = 2'd0; ptr_fast = s_lead; end
        else           begin sel = 2'd1; ptr_fast = s_int;  end
    end

    // shadow sync + err
    reg [W-1:0] sh2, fast_q;
    reg         quiet_q;
    always @(posedge clk_lead or negedge rst_n) begin
        if (!rst_n) begin
            sh2       <= {W{1'b0}};
            fast_q    <= {W{1'b0}};
            quiet_q   <= 1'b1;
            phase_sel <= 2'd1;
            risk      <= 1'b0;
            err       <= 1'b0;
        end else begin
            sh2       <= s_lead;
            fast_q    <= ptr_fast;
            quiet_q   <= ~(early | late);
            phase_sel <= sel;
            risk      <= early | late;
            err       <= quiet_q && (fast_q != sh2);
        end
    end
    assign ptr_shadow = sh2;

endmodule

// dff. META_MODEL = fake metastability for sim
module mpam_dff #(parameter W = 1)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire [W-1:0] d,
    output reg  [W-1:0] q
);
`ifdef META_MODEL
    parameter real T_META = 0.4;   // ns
    parameter real TAU    = 2.0;   // ns
    realtime    t_change = -100.0, t_edge = -100.0;
    reg [W-1:0] d_edge;

    function automatic [W-1:0] xmask(input [W-1:0] a);
        for (int i = 0; i < W; i++) xmask[i] = (a[i] === 1'bx);
    endfunction
    function automatic [W-1:0] diff(input [W-1:0] a, input [W-1:0] b);
        for (int i = 0; i < W; i++) diff[i] = (a[i] !== b[i]);
    endfunction

    task automatic go_meta(input [W-1:0] m, input [W-1:0] base);
        realtime tres;
        q <= (base & ~m) | ({W{1'bx}} & m);
        tres = -TAU * $ln(($urandom % 100000 + 1) / 100001.0);
        fork begin
            #(tres);
            for (int i = 0; i < W; i++)
                if (m[i] && q[i] === 1'bx) q[i] <= $urandom % 2;
        end join_none
    endtask

    always @(d) t_change = $realtime;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) q <= {W{1'b0}};
        else begin
            t_edge = $realtime;
            d_edge = d;
            if (^d === 1'bx)                        // X in
                go_meta(xmask(d), d);
            else if (($realtime - t_change) < T_META)
                go_meta(diff(d, q), d);             // too close
            else
                q <= d;
        end
    end
`else
    always @(posedge clk or negedge rst_n)
        if (!rst_n) q <= {W{1'b0}};
        else        q <= d;
`endif
endmodule

// fake delay line for sim (santiago's cell goes here)
module mpam_delay_line #(parameter real D = 1.0)(
    input  wire clk,
    output wire clk_int,
    output wire clk_lag
);
    assign #(D)     clk_int = clk;
    assign #(2.0*D) clk_lag = clk;
endmodule
