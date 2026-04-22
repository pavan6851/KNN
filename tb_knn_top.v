// =============================================================================
//  tb_knn_top.v  -  Self-checking Testbench, 240 samples
//  PMDC Motor Fault Detection
// =============================================================================
`timescale 1ns/1ps
`include "knn_params.vh"

module tb_knn_top;

    parameter CLK_PERIOD = 10;
    // 2 (preprocess) + N_TRAIN + VOTE_WAIT + VOTE + 6 (RESULT hold) + margin
    parameter MAX_WAIT = `N_TRAIN + 20;  // 980

    // ── DUT signals ───────────────────────────────────────────────────────────
    reg  clk, rst_n, in_valid;
    reg  [`N_RAW_IN*32-1:0] feat_in_bus;
    wire result_valid;
    wire [1:0] result_class;

    // ── DUT ───────────────────────────────────────────────────────────────────
    knn_top dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .in_valid    (in_valid),
        .feat_in     (feat_in_bus),
        .result_valid(result_valid),
        .result_class(result_class)
    );

    // ── Clock ─────────────────────────────────────────────────────────────────
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ── Test data ─────────────────────────────────────────────────────────────
    reg [31:0] all_features [0:`N_TEST*`N_RAW_IN-1];
    reg [7:0]  all_labels   [0:`N_TEST-1];

    // ── Counters ──────────────────────────────────────────────────────────────
    integer n_pass, n_fail, n_total, n_timeout;
    integer n_pass_cls  [0:`N_CLASSES-1];
    integer n_total_cls [0:`N_CLASSES-1];
    integer sample, feat, wait_cnt, cls;
    reg [1:0] cap_class;

    // ── Main ──────────────────────────────────────────────────────────────────
    initial begin
        $readmemh(`MEM_TB_VECTORS, all_features);
        $readmemh(`MEM_TB_LABELS,  all_labels);

        rst_n       = 0;
        in_valid    = 0;
        feat_in_bus = 0;
        n_pass = 0; n_fail = 0; n_total = 0; n_timeout = 0;
        for (cls = 0; cls < `N_CLASSES; cls = cls + 1) begin
            n_pass_cls [cls] = 0;
            n_total_cls[cls] = 0;
        end

        repeat(4) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        $display("========================================");
        $display("  KNN FPGA  -  %0d TEST SAMPLES", `N_TEST);
        $display("  K=%0d  N_TRAIN=%0d  N_RAW_IN=%0d  N_LDA=%0d",
                 `K_NEIGHBORS, `N_TRAIN, `N_RAW_IN, `N_LDA);
        $display("========================================");

        for (sample = 0; sample < `N_TEST; sample = sample + 1) begin

            // Pack features into bus
            for (feat = 0; feat < `N_RAW_IN; feat = feat + 1)
                feat_in_bus[feat*32 +: 32] = all_features[sample*`N_RAW_IN + feat];

            // Assert in_valid for 1 clock
            in_valid = 1;
            @(posedge clk); #1;
            in_valid = 0;

            // Wait for result_valid
            wait_cnt = 0;
            while (!result_valid && wait_cnt < MAX_WAIT) begin
                @(posedge clk);
                wait_cnt = wait_cnt + 1;
            end
            #1;

            cap_class = result_class;
            n_total   = n_total + 1;
            n_total_cls[all_labels[sample]] = n_total_cls[all_labels[sample]] + 1;

            if (!result_valid) begin
                $display("[%0d] TIMEOUT  exp=%0d", sample+1, all_labels[sample]);
                n_timeout = n_timeout + 1;
                n_fail    = n_fail + 1;
            end else if (cap_class == all_labels[sample][1:0]) begin
                n_pass = n_pass + 1;
                n_pass_cls[all_labels[sample]] = n_pass_cls[all_labels[sample]] + 1;
            end else begin
                n_fail = n_fail + 1;
                $display("[%0d] FAIL: got=%0d  exp=%0d",
                         sample+1, cap_class, all_labels[sample]);
            end

            // Wait for result_valid to deassert (RESULT state holds 6 cycles)
            // then one extra idle cycle before next sample
            wait (result_valid == 0);
            @(posedge clk);
        end

        $display("========================================");
        $display("  RESULTS");
        $display("========================================");
        $display("  Total   : %0d", n_total);
        $display("  Correct : %0d", n_pass);
        $display("  Wrong   : %0d", n_fail);
        if (n_timeout > 0)
            $display("  Timeouts: %0d  (increase MAX_WAIT)", n_timeout);
        $display("  Accuracy: %0.2f%%", 100.0*n_pass/n_total);
        $display("----------------------------------------");
        $display("  Per-class:");
        $display("    Healthy (0): %0d / %0d", n_pass_cls[0], n_total_cls[0]);
        $display("    Fault 1 (1): %0d / %0d", n_pass_cls[1], n_total_cls[1]);
        $display("    Fault 2 (2): %0d / %0d", n_pass_cls[2], n_total_cls[2]);
        $display("    Fault 3 (3): %0d / %0d", n_pass_cls[3], n_total_cls[3]);
        $display("========================================");
        if (n_pass * 100 / n_total >= 83)
            $display("  Status: PASS");
        else
            $display("  Status: CHECK");
        $display("  Python reference: 85.00%%");
        $display("========================================");
        $finish;
    end

    // ── Watchdog ──────────────────────────────────────────────────────────────
    initial begin
        #(CLK_PERIOD * (`N_TEST * (MAX_WAIT + 30) + 2000));
        $display("WATCHDOG TIMEOUT");
        $finish;
    end

endmodule
