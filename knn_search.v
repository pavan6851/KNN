// =============================================================================
//  knn_search.v  -  KNN Search + Distance-Weighted Vote
//  PMDC Motor Fault Detection
//
//  Fixes applied:
//    1. query registered on in_valid for stability across SEARCH
//    2. train_idx narrowed to 10-bit
//    3. result_valid held HIGH for 6 cycles (RESULT state) so testbench
//       cannot miss the single-cycle pulse
//    4. hold_cnt added for result_valid extension
// =============================================================================
`timescale 1ns/1ps
`include "knn_params.vh"

module knn_search (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    input  wire [`N_LDA*32-1:0] query,
    output reg         result_valid,
    output reg  [1:0]  result_class
);

    // ── Registered query (stable throughout SEARCH) ───────────────────────────
    reg signed [31:0] q0_r, q1_r, q2_r;
    always @(posedge clk) begin
        if (in_valid) begin
            q0_r <= query[31:0];
            q1_r <= query[63:32];
            q2_r <= query[95:64];
        end
    end

    // ── Training data ROMs ────────────────────────────────────────────────────
    reg signed [31:0] train_vec [0:`N_TRAIN*`N_LDA-1];
    reg        [1:0]  train_lbl [0:`N_TRAIN-1];
    initial begin
        $readmemh(`MEM_TRAIN_VEC, train_vec);
        $readmemh(`MEM_TRAIN_LBL, train_lbl);
    end

    // ── FSM states ────────────────────────────────────────────────────────────
    localparam IDLE      = 3'd0,
               SEARCH    = 3'd1,
               VOTE_WAIT = 3'd2,
               VOTE      = 3'd3,
               RESULT    = 3'd4;

    reg [2:0]  state;
    reg [9:0]  train_idx;   // 10 bits covers 0..959
    reg [2:0]  hold_cnt;    // result_valid hold counter

    // ── KNN heap ──────────────────────────────────────────────────────────────
    reg [63:0] knn_dist [0:`K_NEIGHBORS-1];
    reg [1:0]  knn_lbl  [0:`K_NEIGHBORS-1];
    reg [63:0] worst_dist;
    reg [3:0]  worst_idx;

    // ── Current squared Euclidean distance (combinational) ───────────────────
    wire signed [31:0] tv0 = train_vec[train_idx*`N_LDA+0];
    wire signed [31:0] tv1 = train_vec[train_idx*`N_LDA+1];
    wire signed [31:0] tv2 = train_vec[train_idx*`N_LDA+2];

    wire signed [31:0] d0 = q0_r - tv0;
    wire signed [31:0] d1 = q1_r - tv1;
    wire signed [31:0] d2 = q2_r - tv2;

    wire [63:0] sq0      = $signed(d0) * $signed(d0);
    wire [63:0] sq1      = $signed(d1) * $signed(d1);
    wire [63:0] sq2      = $signed(d2) * $signed(d2);
    wire [63:0] cur_dist = sq0 + sq1 + sq2;

    // ── Combinational heap update ─────────────────────────────────────────────
    reg [63:0] nxt_dist [0:`K_NEIGHBORS-1];
    reg [1:0]  nxt_lbl  [0:`K_NEIGHBORS-1];
    reg [63:0] nxt_worst_dist;
    reg [3:0]  nxt_worst_idx;

    always @(*) begin : comb_heap
        reg [63:0] sc_d;
        reg [3:0]  sc_i;
        integer    p;

        for (p = 0; p < `K_NEIGHBORS; p = p + 1) begin
            nxt_dist[p] = knn_dist[p];
            nxt_lbl [p] = knn_lbl [p];
        end
        nxt_worst_dist = worst_dist;
        nxt_worst_idx  = worst_idx;

        if (state == SEARCH && cur_dist < worst_dist) begin
            nxt_dist[worst_idx] = cur_dist;
            nxt_lbl [worst_idx] = train_lbl[train_idx];

            sc_d = nxt_dist[0];
            sc_i = 4'd0;
            for (p = 1; p < `K_NEIGHBORS; p = p + 1) begin
                if (nxt_dist[p] > sc_d) begin
                    sc_d = nxt_dist[p];
                    sc_i = p[3:0];
                end
            end
            nxt_worst_dist = sc_d;
            nxt_worst_idx  = sc_i;
        end
    end

    // ── Distance-weighted vote (combinational) ────────────────────────────────
    reg [63:0] vote_weight [0:`N_CLASSES-1];
    always @(*) begin : comb_vote
        reg [63:0] min_d;
        reg [63:0] w;
        integer    v;

        min_d = knn_dist[0];
        for (v = 1; v < `K_NEIGHBORS; v = v + 1)
            if (knn_dist[v] < min_d) min_d = knn_dist[v];

        for (v = 0; v < `N_CLASSES; v = v + 1)
            vote_weight[v] = 64'd0;

        for (v = 0; v < `K_NEIGHBORS; v = v + 1) begin
            if (knn_dist[v] == 64'd0)
                w = 64'hFFFF_FFFF;
            else if (min_d == 64'd0)
                w = (knn_dist[v] == 64'd0) ? 64'hFFFF_FFFF : 64'd0;
            else
                w = (min_d << 14) / knn_dist[v];
            vote_weight[knn_lbl[v]] = vote_weight[knn_lbl[v]] + w;
        end
    end

    // ── Argmax (combinational) ────────────────────────────────────────────────
    reg [63:0] best_wt;
    reg [1:0]  best_cls;
    integer    ar;
    always @(*) begin : comb_argmax
        best_wt  = 64'd0;
        best_cls = 2'd0;
        for (ar = 0; ar < `N_CLASSES; ar = ar + 1) begin
            if (vote_weight[ar] > best_wt) begin
                best_wt  = vote_weight[ar];
                best_cls = ar[1:0];
            end
        end
    end

    // ── Clocked FSM ───────────────────────────────────────────────────────────
    integer j;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            result_valid <= 1'b0;
            result_class <= 2'd0;
            train_idx    <= 10'd0;
            hold_cnt     <= 3'd0;
            worst_dist   <= 64'hFFFF_FFFF_FFFF_FFFF;
            worst_idx    <= 4'd0;
            for (j = 0; j < `K_NEIGHBORS; j = j + 1) begin
                knn_dist[j] <= 64'hFFFF_FFFF_FFFF_FFFF;
                knn_lbl [j] <= 2'd0;
            end
        end else begin

            case (state)

                IDLE: begin
                    result_valid <= 1'b0;
                    if (in_valid) begin
                        train_idx  <= 10'd0;
                        worst_dist <= 64'hFFFF_FFFF_FFFF_FFFF;
                        worst_idx  <= 4'd0;
                        for (j = 0; j < `K_NEIGHBORS; j = j + 1) begin
                            knn_dist[j] <= 64'hFFFF_FFFF_FFFF_FFFF;
                            knn_lbl [j] <= 2'd0;
                        end
                        state <= SEARCH;
                    end
                end

                SEARCH: begin
                    for (j = 0; j < `K_NEIGHBORS; j = j + 1) begin
                        knn_dist[j] <= nxt_dist[j];
                        knn_lbl [j] <= nxt_lbl [j];
                    end
                    worst_dist <= nxt_worst_dist;
                    worst_idx  <= nxt_worst_idx;

                    if (train_idx == `N_TRAIN - 1)
                        state <= VOTE_WAIT;
                    else
                        train_idx <= train_idx + 1;
                end

                VOTE_WAIT: begin
                    // Pipeline bubble: let last knn_dist[] writes settle
                    state <= VOTE;
                end

                VOTE: begin
                    result_class <= best_cls;
                    result_valid <= 1'b1;
                    hold_cnt     <= 3'd0;
                    state        <= RESULT;
                end

                // Hold result_valid HIGH for 6 cycles so testbench cannot
                // miss the pulse due to posedge sampling race
                RESULT: begin
                    result_valid <= 1'b1;
                    if (hold_cnt == 3'd5) begin
                        result_valid <= 1'b0;
                        hold_cnt     <= 3'd0;
                        state        <= IDLE;
                    end else begin
                        hold_cnt <= hold_cnt + 1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
