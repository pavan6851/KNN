`ifndef KNN_PARAMS_VH
`define KNN_PARAMS_VH

`define N_RAW_IN        30
`define N_LDA           3
`define N_TRAIN         960
`define N_TEST          240
`define K_NEIGHBORS     15
`define N_CLASSES       4
`define Q_PARAM         24

`define MEM_LDA_MEAN   "F:/FPGA Based Fault Detection/AI Analysis/KNN NEW/PARAMS NEW/fp_lda_mean.mem"
`define MEM_LDA_SCALE  "F:/FPGA Based Fault Detection/AI Analysis/KNN NEW/PARAMS NEW/fp_lda_scalings.mem"
`define MEM_TRAIN_VEC  "F:/FPGA Based Fault Detection/AI Analysis/KNN NEW/PARAMS NEW/fp_train_vectors.mem"
`define MEM_TRAIN_LBL  "F:/FPGA Based Fault Detection/AI Analysis/KNN NEW/PARAMS NEW/fp_train_labels.mem"
`define MEM_TB_VECTORS "F:/FPGA Based Fault Detection/AI Analysis/KNN NEW/PARAMS NEW/tb_vectors_full.mem"
`define MEM_TB_LABELS  "F:/FPGA Based Fault Detection/AI Analysis/KNN NEW/PARAMS NEW/tb_labels_full.mem"

`endif
