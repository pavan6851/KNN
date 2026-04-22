// =============================================================================
//  knn_preprocess.v  -  LDA Projection Only
//  PMDC Motor Fault Detection
//
//  Pipeline: 2 clock cycles latency
//    Cycle 1: subtract LDA mean
//    Cycle 2: LDA matrix-vector multiply
// =============================================================================
`timescale 1ns/1ps
`include "knn_params.vh"

module knn_preprocess (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    input  wire [`N_RAW_IN*32-1:0] feat_in,
    output reg  [`N_LDA*32-1:0]    lda_out,
    output reg                     out_valid
);

    // ── Unpack input bus ──────────────────────────────────────────────────────
    wire signed [31:0] feat_arr [0:`N_RAW_IN-1];
    genvar gi;
    generate
        for (gi = 0; gi < `N_RAW_IN; gi = gi + 1) begin : unpack
            assign feat_arr[gi] = feat_in[(gi+1)*32-1 : gi*32];
        end
    endgenerate

    // ── Parameter ROMs ────────────────────────────────────────────────────────
    reg signed [31:0] lda_mean     [0:`N_RAW_IN-1];
    reg signed [31:0] lda_scalings [0:`N_RAW_IN*`N_LDA-1];
    initial begin
        $readmemh(`MEM_LDA_MEAN,  lda_mean);
        $readmemh(`MEM_LDA_SCALE, lda_scalings);
    end

    // ── Stage 1: subtract LDA mean ────────────────────────────────────────────
    // lda_mean is Q16.16, feat_arr is Q8.24
    // shift lda_mean left 8 to align to Q8.24
    reg signed [31:0] x_c [0:`N_RAW_IN-1];
    reg               valid_s1;
    integer           i1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_s1 <= 1'b0;
        end else begin
            valid_s1 <= in_valid;
            if (in_valid) begin
                for (i1 = 0; i1 < `N_RAW_IN; i1 = i1 + 1)
                    x_c[i1] <= feat_arr[i1] - ($signed(lda_mean[i1]) <<< 8);
            end
        end
    end

    // ── Stage 2: LDA projection ───────────────────────────────────────────────
    // x_c: Q8.24, scalings: Q16.16
    // product: Q24.40, shift right 16 -> Q8.24, take lower 32 bits
    integer           i2, k2;
    reg signed [63:0] acc;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            lda_out   <= {(`N_LDA*32){1'b0}};
        end else begin
            out_valid <= valid_s1;
            if (valid_s1) begin
                for (k2 = 0; k2 < `N_LDA; k2 = k2 + 1) begin
                    acc = 64'sd0;
                    for (i2 = 0; i2 < `N_RAW_IN; i2 = i2 + 1)
                        acc = acc + (($signed(x_c[i2]) *
                                      $signed(lda_scalings[i2*`N_LDA + k2]))
                                     >>> 16);
                    lda_out[(k2+1)*32-1 -: 32] <= acc[31:0];
                end
            end
        end
    end

endmodule
