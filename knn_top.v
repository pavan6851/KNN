// =============================================================================
//  knn_top.v  -  KNN Inference Top Level
//  PMDC Motor Fault Detection
// =============================================================================
`timescale 1ns/1ps
`include "knn_params.vh"

module knn_top (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    input  wire [`N_RAW_IN*32-1:0] feat_in,
    output wire        result_valid,
    output wire [1:0]  result_class
);

    wire [`N_LDA*32-1:0] lda_bus;
    wire                 prep_valid;

    knn_preprocess u_prep (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (in_valid),
        .feat_in  (feat_in),
        .lda_out  (lda_bus),
        .out_valid(prep_valid)
    );

    knn_search u_search (
        .clk         (clk),
        .rst_n       (rst_n),
        .in_valid    (prep_valid),
        .query       (lda_bus),
        .result_valid(result_valid),
        .result_class(result_class)
    );

endmodule
