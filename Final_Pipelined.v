`timescale 1ns/1ps

//=====================================================================
// 1) 2:1 multiplexer
//=====================================================================
module mux2x1 #(
  parameter w = 1
)(
  input  wire [w-1:0] a,
  input  wire [w-1:0] b,
  input  wire         sel,
  output reg  [w-1:0] o
);
  always @(*) o = sel ? b : a;
endmodule

//=====================================================================
// 2) 4:1 multiplexer with enable
//=====================================================================
module mux4x1 #(
  parameter w = 1
)(
  input  wire [w-1:0] a,
  input  wire [w-1:0] b,
  input  wire [w-1:0] c,
  input  wire [w-1:0] d,
  input  wire         en,
  input  wire [1:0]   sel,
  output reg  [w-1:0] o
);
  always @(*) begin
    if (en) begin
      case (sel)
        2'b00: o = a;
        2'b01: o = b;
        2'b10: o = c;
        2'b11: o = d;
        default: o = {w{1'b0}};
      endcase
    end else
      o = {w{1'b0}};
  end
endmodule

//=====================================================================
// 3) Instruction Fetch Unit
//=====================================================================
module instructionfetch (
  input  wire        clk,
  input  wire        rst,
  input  wire        isbranchtaken,
  input  wire [31:0] branchpc,
  output wire [31:0] instruct,
  output reg  [31:0] lo
);
  reg  [7:0] instrumem [0:255];
  wire [31:0] next_pc;

  mux2x1 #(.w(32)) pc_mux (
    .a(lo + 32'd4),
    .b(branchpc),
    .sel(isbranchtaken),
    .o(next_pc)
  );

  always @(posedge clk or posedge rst) begin
    if (rst)   lo <= 32'd0;
    else       lo <= next_pc;
  end

  assign instruct = {
    instrumem[lo],
    instrumem[lo+1],
    instrumem[lo+2],
    instrumem[lo+3]
  };
endmodule

//=====================================================================
// 4) Control Unit
//=====================================================================
module control_unit (
  input  wire [31:0] inst,
  output reg         isSt,
  output reg         isLd,
  output reg         isBeq,
  output reg         isBgt,
  output reg         isRet,
  output reg         isImmediate,
  output reg         isWb,
  output reg         isUbranch,
  output reg         isCall,
  output reg         isAdd,
  output reg         isSub,
  output reg         isCmp,
  output reg         isMul,
  output reg         isDiv,
  output reg         isMod,
  output reg         isLsl,
  output reg         isLsr,
  output reg         isAsr,
  output reg         isOr,
  output reg         isAnd,
  output reg         isNot,
  output reg         isMov
);
  wire op1 = inst[27];
  wire op2 = inst[28];
  wire op3 = inst[29];
  wire op4 = inst[30];
  wire op5 = inst[31];

  always @(*) begin
    isSt        = ~op5 &  op4 &  op3 &  op2 &  op1;
    isLd        = ~op5 &  op4 &  op3 &  op2 & ~op1;
    isBeq       =  op5 & ~op4 & ~op3 & ~op2 & ~op1;
    isBgt       =  op5 & ~op4 & ~op3 & ~op2 &  op1;
    isRet       =  op5 & ~op4 &  op3 & ~op2 & ~op1;
    isImmediate =  inst[26];
     isWb=(~(op5|((~op5)&op3&op1&(op4|(~op2))))|(op5&(~op4)&(~op3)&op2&op1));
    isUbranch   =  op5 & ~op4 & ((~op3 & op2) | (op3 & ~op2 & ~op1));
    isCall      =  op5 & ~op4 & ~op3 &  op2 &  op1;
      isAdd =(((~op5) & (~op4) & (~op3) & (~op2) & (~op1)) | ((~op5) & op4 & op3 & op2));
    isSub       = ~op5 & ~op4 & ~op3 & ~op2 &  op1;
    isCmp       = ~op5 & ~op4 &  op3 & ~op2 &  op1;
    isMul       = ~op5 & ~op4 & ~op3 &  op2 & ~op1;
    isDiv       = ~op5 & ~op4 & ~op3 &  op2 &  op1;
    isMod       = ~op5 & ~op4 &  op3 & ~op2 & ~op1;
    isLsl       = ~op5 &  op4 & ~op3 &  op2 & ~op1;
    isLsr       = ~op5 &  op4 & ~op3 &  op2 &  op1;
    isAsr       = ~op5 &  op4 &  op3 & ~op2 & ~op1;
    isOr        = ~op5 & ~op4 &  op3 &  op2 &  op1;
    isAnd       = ~op5 & ~op4 &  op3 &  op2 & ~op1;
    isNot       = ~op5 &  op4 & ~op3 & ~op2 & ~op1;
    isMov       = ~op5 &  op4 & ~op3 & ~op2 &  op1;
  end
endmodule

//=====================================================================
// 5) Operand Fetch Unit
//=====================================================================
module OperandFetchUnit (
  input  wire        isRet,
  input  wire        isSt,
  input  wire [3:0]  rs1,
  input  wire [3:0]  rs2,
  input  wire [3:0]  rd,
  input  wire [3:0]  ra,
  output wire [3:0]  inpregfile1,
  output wire [3:0]  inpregfile2
);
  mux2x1 #(.w(4)) m1 (.a(rs1), .b(ra), .sel(isRet), .o(inpregfile1));
  mux2x1 #(.w(4)) m2 (.a(rs2), .b(rd), .sel(isSt),  .o(inpregfile2));
endmodule

//=====================================================================
// 6) Register File
//=====================================================================
module Register_file (
  input  wire        clock,
  input  wire        reset,
  input  wire [3:0]  reg_rd1,
  input  wire [3:0]  reg_rd2,
  output wire [31:0] reg_rd1_out,
  output wire [31:0] reg_rd2_out,
  input  wire [3:0]  reg_wr1,
  input  wire [31:0] reg_wr1_data,
  input  wire        wr1_enable
);
  reg [31:0] register [0:15];
  integer i;
  assign reg_rd1_out = register[reg_rd1];
  assign reg_rd2_out = register[reg_rd2];

  always @(posedge clock or posedge reset) begin
    if (reset) begin
      for (i=0; i<16; i=i+1) register[i] <= 32'd0;
    end else if (wr1_enable) begin
      register[reg_wr1] <= reg_wr1_data;
    end
  end
endmodule

//=====================================================================
// 7) Branch Unit
//=====================================================================
module branchunit(
    input [31:0] branchTarget,
    input [31:0] op1,
    input isBeq,isBgt,isUbranch,
    input [1:0] flag,
    input isRet,
    input isCall,
    output [31:0] branchPC,
    output isBranchTaken
);
mux2x1 #(.w(32)) m1 (
    .a(branchTarget),
    .b(op1),
    .sel(isRet),
    .o(branchPC)
);
wire fa,sa;
assign fa=isBeq&flag[0];
assign sa=isBgt&flag[1];
assign isBranchTaken=isUbranch|fa|sa|isCall;
endmodule
//=====================================================================
// 8) Immediate Generator (stub)
//=====================================================================
module imm_gen(
    input  [31:0] inst,
    output [31:0] immx,
    input  [31:0] pc,
    output [31:0] branchTarget
);
wire [17:0] imm_field = inst[17:0];
wire [26:0] off = inst[26:0];
wire [31:0] exof = { {3{off[26]}},off, 2'b00 };
wire [15:0] imm16 = imm_field[15:0];
wire mod_u = imm_field[16];
wire mod_h = imm_field[17];
reg [31:0] immx_reg;
always @* begin
    case({mod_h, mod_u})
        2'b00: begin
        immx_reg = {{16{imm16[15]}}, imm16};
        end
        2'b01: begin
            immx_reg = {16'b0, imm16};
        end
        2'b10: begin
            immx_reg = {imm16, 16'b0};
        end
        2'b11: begin
            immx_reg = {{16{imm16[15]}}, imm16};
        end
        default: immx_reg = 32'b0;
    endcase
end

assign immx = immx_reg;
assign branchTarget = pc + exof;
endmodule


//=====================================================================
// 9) ALU Submodules
//=====================================================================
module fulladder (
  input  wire a, b, cin,
  output wire s, cout
);
  assign s    = a ^ b ^ cin;
  assign cout = (a & b) | (b & cin) | (cin & a);
endmodule

module adder_subtractor (
  input  wire [31:0] a, b,
  input  wire        s,
  output wire [31:0] sum,
  output wire        cout
);
  wire [31:0] bm = b ^ {32{s}};
  wire [32:0] c;
  assign c[0] = s;
  genvar i;
  generate
    for (i=0; i<32; i=i+1) begin
      fulladder fa (
        .a(a[i]), .b(bm[i]), .cin(c[i]),
        .s(sum[i]), .cout(c[i+1])
      );
    end
  endgenerate
  assign cout = c[32];
endmodule

module muxx4x1 #(
  parameter W=1
)(
     input  wire [W-1:0] a,        
    input  wire [W-1:0] b,
    input  wire [W-1:0] c,
    input  wire [W-1:0] d,
    input  wire        e,       
    input  wire [1:0]  sel,      
    output reg  [W-1:0] o   
);
    always @(*) begin
    if(e) begin
      case (sel)
        2'b00: o = a;
        2'b01: o = b;
        2'b10: o = c;
        2'b11: o = d;
        default: o = {W{1'b0}};
      endcase
    end else begin
      o = {W{1'b0}}; 
    end
end

endmodule

module Adder (
  input  wire [31:0] a, b,
  input  wire        isAdd, isSub, isCmp, isLd, isSt,
  output wire [31:0] sum,
  output wire        cout, cmp_g, cmp_e
);
  // Determine operation type
  wire [1:0] op = isAdd ? 2'b01
                : isSub ? 2'b10
                : isCmp ? 2'b11
                : 2'b00;
  wire sub_ctrl = op[1];
  wire [31:0] mid;
  wire        mid_c;
  
  // Perform addition/subtraction
  adder_subtractor u0(.a(a), .b(b), .s(sub_ctrl), .sum(mid), .cout(mid_c));

  // Critical fix: For load/store, pass through the address value 
  // even when op is 2'b00
  assign sum   = (isLd || isSt) ? a : (op != 2'b00) ? mid : 32'd0;
  assign cout  = (op != 2'b00) ? mid_c : 1'b0;

  // Comparison outputs are enabled only for the compare operation
  assign cmp_g = (op == 2'b11) ? ~mid[31]      : 1'b0;
  assign cmp_e = (op == 2'b11) ? ~(|(a ^ b))   : 1'b0;
endmodule

module Mul (
  input  wire [31:0] a, b,
  input  wire        isMul,
  output wire [31:0] mo
);
  assign mo = isMul ? a * b : 32'd0;
endmodule

module Divider (
  input  wire [31:0] a, b,
  input  wire        isDiv, isMod,
  output wire [31:0] o
);
  assign o = isDiv ? (a / b) : isMod ? (a % b) : 32'd0;
endmodule

module Logical_unit (
  input  wire [31:0] a, b,
  input  wire        isOr, isNot, isAnd,
  output wire [31:0] o
);
  assign o = isOr  ? (a | b)
           : isNot ? (~a)
           : isAnd ? (a & b)
           : 32'd0;
endmodule

module Mov (
  input  wire [31:0] b,
  input  wire        isMov,
  output wire [31:0] o
);
  assign o = isMov ? b : 32'd0;
endmodule

module unified_shift_register (
    input  wire [31:0] a,
    input  wire [31:0] b,
    input  wire        isAsl,
    input  wire        isAsr,
    input  wire        isLsl,
    input  wire        isLsr,
    output wire [31:0] o
);

  // determine shift type: 00=asl, 01=asr, 10=lsr, 11=lsl
  wire [1:0] s = isAsl ? 2'b00 :
                 isAsr ? 2'b01 :
                 isLsr ? 2'b10 :
                 isLsl ? 2'b11 :
                         2'b00;
                       
  // Original sign bit for ASR operations - this remains constant throughout all stages
  wire sign_bit = a[31];

  genvar i;

  // 16-bit stage
  wire [31:0] shifted16;
  generate
    for (i = 0; i < 32; i = i + 1) begin : stage16_compute
      if (i < 16) begin
        // Lower bits - shifted data comes from higher bits
        muxx4x1 #(.W(1)) u_mux16 (
          .a(1'b0),                 // ASL - fill with 0
          .b(i+16 < 32 ? a[i+16] : sign_bit), // ASR - data from upper half with sign ext
          .c(i+16 < 32 ? a[i+16] : 1'b0),     // LSR - data from upper half with 0 ext
          .d(1'b0),                 // LSL - fill with 0
          .e(1'b1),
          .sel(s),
          .o(shifted16[i])
        );
      end else begin
        // Upper bits - may need sign extension for ASR
        muxx4x1 #(.W(1)) u_mux16 (
          .a(1'b0),                 // ASL - fill with 0
          .b(s == 2'b01 ? sign_bit : 1'b0), // ASR - sign extend all upper bits
          .c(1'b0),                 // LSR - fill with 0
          .d(i-16 < 16 ? a[i-16] : 1'b0), // LSL - data from lower half
          .e(1'b1),
          .sel(s),
          .o(shifted16[i])
        );
      end
    end
  endgenerate

  // select between original or 16-bit shifted
  wire [31:0] shift_16_f;
  mux2x1 #(.w(32)) mux16_sel (
    .a(a), .b(shifted16), .sel(b[4]), .o(shift_16_f)
  );

  // 8-bit stage
  wire [31:0] shifted8;
  generate
    for (i = 0; i < 32; i = i + 1) begin : stage8_compute
      if (i < 8) begin
        // Lower bits - shifted data comes from higher bits
        muxx4x1 #(.W(1)) u_mux8 (
          .a(1'b0),                  // ASL - fill with 0
          .b(i+8 < 32 ? shift_16_f[i+8] : sign_bit), // ASR with sign ext
          .c(i+8 < 32 ? shift_16_f[i+8] : 1'b0),     // LSR with 0 ext
          .d(1'b0),                  // LSL - fill with 0
          .e(1'b1),
          .sel(s),
          .o(shifted8[i])
        );
      end else if (i < 24) begin
        // Middle bits - normal shifting
        muxx4x1 #(.W(1)) u_mux8 (
          .a(shift_16_f[i-8]),       // ASL - data from lower section
          .b(i+8 < 32 ? shift_16_f[i+8] : sign_bit), // ASR with sign ext
          .c(i+8 < 32 ? shift_16_f[i+8] : 1'b0),     // LSR with 0 ext
          .d(shift_16_f[i-8]),       // LSL - data from lower section
          .e(1'b1),
          .sel(s),
          .o(shifted8[i])
        );
      end else begin
        // Upper bits - may need sign extension for ASR
        muxx4x1 #(.W(1)) u_mux8 (
          .a(shift_16_f[i-8]),       // ASL - data from lower section
          .b(s == 2'b01 ? sign_bit : 1'b0), // ASR - sign extend
          .c(1'b0),                  // LSR - fill with 0
          .d(shift_16_f[i-8]),       // LSL - data from lower section
          .e(1'b1),
          .sel(s),
          .o(shifted8[i])
        );
      end
    end
  endgenerate

  // select between 16-stage or 8-stage
  wire [31:0] shift_8_f;
  mux2x1 #(.w(32)) mux8_sel (
    .a(shift_16_f), .b(shifted8), .sel(b[3]), .o(shift_8_f)
  );

  // 4-bit stage
  wire [31:0] shifted4;
  generate
    for (i = 0; i < 32; i = i + 1) begin : stage4_compute
      if (i < 4) begin
        // Lower bits - shifted data comes from higher bits
        muxx4x1 #(.W(1)) u_mux4 (
          .a(1'b0),                 // ASL - fill with 0
          .b(i+4 < 32 ? shift_8_f[i+4] : sign_bit), // ASR with sign ext
          .c(i+4 < 32 ? shift_8_f[i+4] : 1'b0),     // LSR with 0 ext
          .d(1'b0),                 // LSL - fill with 0
          .e(1'b1),
          .sel(s),
          .o(shifted4[i])
        );
      end else if (i < 28) begin
        // Middle bits - normal shifting
        muxx4x1 #(.W(1)) u_mux4 (
          .a(shift_8_f[i-4]),       // ASL - data from lower section
          .b(i+4 < 32 ? shift_8_f[i+4] : sign_bit), // ASR with sign ext
          .c(i+4 < 32 ? shift_8_f[i+4] : 1'b0),     // LSR with 0 ext
          .d(shift_8_f[i-4]),       // LSL - data from lower section
          .e(1'b1),
          .sel(s),
          .o(shifted4[i])
        );
      end else begin
        // Upper bits - may need sign extension for ASR
        muxx4x1 #(.W(1)) u_mux4 (
          .a(shift_8_f[i-4]),       // ASL - data from lower section
          .b(s == 2'b01 ? sign_bit : 1'b0), // ASR - sign extend
          .c(1'b0),                 // LSR - fill with 0
          .d(shift_8_f[i-4]),       // LSL - data from lower section
          .e(1'b1),
          .sel(s),
          .o(shifted4[i])
        );
      end
    end
  endgenerate

  // select between 8-stage or 4-stage
  wire [31:0] shift_4_f;
  mux2x1 #(.w(32)) mux4_sel (
    .a(shift_8_f), .b(shifted4), .sel(b[2]), .o(shift_4_f)
  );

  // 2-bit stage
  wire [31:0] shifted2;
  generate
    for (i = 0; i < 32; i = i + 1) begin : stage2_compute
      if (i < 2) begin
        // Lower bits - shifted data comes from higher bits
        muxx4x1 #(.W(1)) u_mux2 (
          .a(1'b0),                 // ASL - fill with 0
          .b(i+2 < 32 ? shift_4_f[i+2] : sign_bit), // ASR with sign ext
          .c(i+2 < 32 ? shift_4_f[i+2] : 1'b0),     // LSR with 0 ext
          .d(1'b0),                 // LSL - fill with 0
          .e(1'b1),
          .sel(s),
          .o(shifted2[i])
        );
      end else if (i < 30) begin
        // Middle bits - normal shifting
        muxx4x1 #(.W(1)) u_mux2 (
          .a(shift_4_f[i-2]),       // ASL - data from lower section
          .b(i+2 < 32 ? shift_4_f[i+2] : sign_bit), // ASR with sign ext
          .c(i+2 < 32 ? shift_4_f[i+2] : 1'b0),     // LSR with 0 ext
          .d(shift_4_f[i-2]),       // LSL - data from lower section
          .e(1'b1),
          .sel(s),
          .o(shifted2[i])
        );
      end else begin
        // Upper bits - may need sign extension for ASR
        muxx4x1 #(.W(1)) u_mux2 (
          .a(shift_4_f[i-2]),       // ASL - data from lower section
          .b(s == 2'b01 ? sign_bit : 1'b0), // ASR - sign extend
          .c(1'b0),                 // LSR - fill with 0
          .d(shift_4_f[i-2]),       // LSL - data from lower section
          .e(1'b1),
          .sel(s),
          .o(shifted2[i])
        );
      end
    end
  endgenerate

  // select between 4-stage or 2-stage
  wire [31:0] shift_2_f;
  mux2x1 #(.w(32)) mux2_sel (
    .a(shift_4_f), .b(shifted2), .sel(b[1]), .o(shift_2_f)
  );

  // 1-bit stage
  wire [31:0] shifted1;
  generate
    for (i = 0; i < 32; i = i + 1) begin : stage1_compute
      if (i < 1) begin
        // Lowest bit - shifted data comes from higher bits
        muxx4x1 #(.W(1)) u_mux1 (
          .a(1'b0),                 // ASL - fill with 0
          .b(i+1 < 32 ? shift_2_f[i+1] : sign_bit), // ASR with sign ext
          .c(i+1 < 32 ? shift_2_f[i+1] : 1'b0),     // LSR with 0 ext
          .d(1'b0),                 // LSL - fill with 0
          .e(1'b1),
          .sel(s),
          .o(shifted1[i])
        );
      end else if (i < 31) begin
        // Middle bits - normal shifting
        muxx4x1 #(.W(1)) u_mux1 (
          .a(shift_2_f[i-1]),       // ASL - data from lower section
          .b(i+1 < 32 ? shift_2_f[i+1] : sign_bit), // ASR with sign ext
          .c(i+1 < 32 ? shift_2_f[i+1] : 1'b0),     // LSR with 0 ext
          .d(shift_2_f[i-1]),       // LSL - data from lower section
          .e(1'b1),
          .sel(s),
          .o(shifted1[i])
        );
      end else begin
        // MSB - may need sign extension for ASR
        muxx4x1 #(.W(1)) u_mux1 (
          .a(shift_2_f[i-1]),       // ASL - data from lower section
          .b(s == 2'b01 ? sign_bit : 1'b0), // ASR - sign extend with original sign bit
          .c(1'b0),                 // LSR - fill with 0
          .d(shift_2_f[i-1]),       // LSL - data from lower section
          .e(1'b1),
          .sel(s),
          .o(shifted1[i])
        );
      end
    end
  endgenerate

  // select between 2-stage or 1-stage
  wire [31:0] shift_1_f;
  mux2x1 #(.w(32)) mux1_sel (
    .a(shift_2_f), .b(shifted1), .sel(b[0]), .o(shift_1_f)
  );

  // if shift amount >= 32, result depends on shift type
  wire moreshift = |b[31:5];
  reg [31:0] full_shift;
  
  // Handle shifts >= 32 bits
  integer j;
  always @(*) begin
    for (j = 0; j < 32; j = j + 1) begin
      full_shift[j] = (s == 2'b01) ? sign_bit : 1'b0;
    end
  end
  
  assign o = moreshift ? full_shift : shift_1_f;
endmodule
//=====================================================================
// 10) Top-level ALU
//=====================================================================
module ALU (
  input  wire [31:0] a, b,
  input  wire        isAdd, isSub, isCmp,
  input  wire        isMul, isDiv, isMod,
  input  wire        isOr,  isNot, isAnd, isMov,
  input  wire        isAsl, isAsr, isLsr, isLsl,
  input wire         isSt,isLd,
  input [3:0] rs11,rs22,
  output reg  [31:0] aluResult,
  output wire        cout, cmp_g, cmp_e
);
  wire [31:0] add_out, mul_out, div_out, log_out, mov_out, shift_out;
   Adder  u_add(.a(a), .b(b),.isSt(isSt),.isLd(isLd), .isAdd(isAdd), .isSub(isSub), .isCmp(isCmp),
               .sum(add_out), .cout(cout), .cmp_g(cmp_g), .cmp_e(cmp_e));
  Mul    u_mul(.a(a), .b(b), .isMul(isMul), .mo(mul_out));
  Divider u_div(.a(a), .b(b), .isDiv(isDiv), .isMod(isMod), .o(div_out));
  Logical_unit u_log(.a(a), .b(b), .isOr(isOr), .isNot(isNot), .isAnd(isAnd), .o(log_out));
  Mov    u_mov(.b(b), .isMov(isMov), .o(mov_out));
  unified_shift_register u_sh(.a(a), .b(b),
                              .isAsl(isAsl), .isAsr(isAsr),
                              .isLsr(isLsr), .isLsl(isLsl),
                              .o(shift_out));

  always @(*) begin
    case (1'b1)
      isAdd, isSub, isCmp: aluResult = add_out;
      isMul:               aluResult = mul_out;
      isDiv, isMod:        aluResult = div_out;
      isOr, isNot, isAnd:  aluResult = log_out;
      isMov:               aluResult = mov_out;
      isAsl, isAsr, isLsr, isLsl: aluResult = shift_out;
      default:             aluResult = 32'd0;
    endcase
  end
endmodule

//=====================================================================
// 11) Data Memory Access Unit
//=====================================================================
module memoryaccessunit (
  input  wire [31:0] op2,
  input  wire [31:0] aluResult,
  input  wire        isLd, isSt,
  input  wire        clk,
  output wire [31:0] ldresult
);
  reg [31:0] datamemory [0:1023];
  reg [31:0] mdr;

  // Make load operations combinational instead of sequential
  assign ldresult = isLd ? datamemory[aluResult] : 32'd0;

  // Store operations remain sequential (on clock edge)
  always @(posedge clk) begin
    if (isSt) datamemory[aluResult] <= op2;
  end 
endmodule

//=====================================================================
// 12) Writeback Muxes
//=====================================================================
module regwriteback (
  input  wire [31:0] aluResult,
  input  wire [31:0] ldResult,
  input  wire [31:0] prpc,
  input  wire        isLd, isCall, isWb,
  input  wire [3:0]  ra, rd,
  output wire [3:0]  writeadd,
  output wire [31:0] writedata
);
  wire [1:0] sel = {isCall, isLd};

  mux4x1 #(.w(32)) wb_mux (
    .a(aluResult),
    .b(ldResult),
    .c(prpc),
    .d(32'd0),
    .en(isWb),
    .sel(sel),
    .o(writedata)
  );
  mux2x1 #(.w(4)) addr_mux (
    .a(rd),
    .b(ra),
    .sel(isCall),
    .o(writeadd)
  );
endmodule



module flag_reg (
input clk, 
  input isCmp,
  input cmp_g,
  input cmp_e,
  output reg ocmp_g,
  output reg ocmp_e 

);
always @(posedge clk)
begin
if(isCmp)
begin
  ocmp_g<=cmp_g;
  ocmp_e<=cmp_e;
end
end

endmodule








// All Registers 
module pipo_IF_OF (
  input [31:0] pc,
  input [31:0] instruction,
  input clk,
  input rst,
  output reg [31:0] pcc,
  output reg [31:0] instructiono
);
  always @(posedge clk or posedge rst) begin
      if (rst) begin
          pcc <= 32'b0;
          instructiono <= 32'b0;
      end else begin
          pcc <= pc;
          instructiono <= instruction;
      end
  end 
endmodule

module pipo_OF_EX(
  input clk,
  input rst,
  input [31:0] pc,
  input [31:0] instruction,
  input [31:0] a, b, op2,
  input [21:0] control,
  input [31:0] branchTarget,
  output reg [31:0] pcc, instr, ao, bo, oppo, co, bto
);
  always @(posedge clk or posedge rst) begin
      if (rst) begin
          pcc  <= 32'b0;
          instr<= 32'b0;
          ao   <= 32'b0;
          bo   <= 32'b0;
          oppo <= 32'b0;
          co   <= 22'b0;
          bto  <= 32'b0;
      end else begin
          pcc  <= pc;
          instr<= instruction;
          ao   <= a;
          bo   <= b;
          oppo <= op2;
          co   <= control;
          bto  <= branchTarget;
      end
  end   
endmodule

module EX_MA (
  input clk,
  input rst,
  input [31:0] pc, aluresult, op2, instruction,
  input [21:0] control,
  output reg [31:0] pcc, alo, opo, io,
  output reg [21:0] co
);
  always @(posedge clk or posedge rst) begin
      if (rst) begin
          pcc <= 32'b0;
          alo <= 32'b0;
          opo <= 32'b0;
          io  <= 32'b0;
          co  <= 22'b0;
      end else begin
          pcc <= pc;
          alo <= aluresult;
          opo <= op2;
          io  <= instruction;
          co  <= control;
      end
  end
endmodule

module MA_RW (
  input clk, rst,
  input [31:0] pc, ldresult, aluresult, instruction,
  input  wire [21:0] control,
  output reg [31:0] pcc, ldo, alo, io,
  output reg [21:0] co
);
  always @(posedge clk or posedge rst) begin
      if (rst) begin
          pcc <= 32'b0;
          ldo <= 32'b0;
          alo <= 32'b0;
          io  <= 32'b0;
          co  <= 22'd0;
      end else begin
          pcc <= pc;
          ldo <= ldresult;
          alo <= aluresult;
          io  <= instruction;
          co  <= control;
      end
  end
endmodule


  module Final_Pipelined (
  input  wire        clk,
  input  wire        rst
);

  // === IF Stage ===
  wire        isBranchTaken_IF;
  wire [31:0] branchPC_IF, instruction_IF, pc_IF;
 wire [31:0] branchPC_EX;
  wire        isBranchTaken_EX;
  instructionfetch IF (
    .clk(clk),
    .rst(rst),
    .isbranchtaken(isBranchTaken_EX),
    .branchpc(branchPC_EX),
    .instruct(instruction_IF),
    .lo(pc_IF)
  );

  // IF/ID Pipeline Register
  wire [31:0] pc_ID, instr_ID;
  pipo_IF_OF IF_ID (
    .pc(pc_IF),
    .instruction(instruction_IF),
    .clk(clk),
    .rst(rst),
    .pcc(pc_ID),
    .instructiono(instr_ID)
  );

  // === ID Stage ===
  // Control Unit
  wire isSt_ID, isLd_ID, isBeq_ID, isBgt_ID, isRet_ID;
  wire isImmediate_ID, isWb_ID, isUbranch_ID, isCall_ID;
  wire isAdd_ID, isSub_ID, isCmp_ID;
  wire isMul_ID, isDiv_ID, isMod_ID;
  wire isLsl_ID, isLsr_ID, isAsr_ID;
  wire isOr_ID, isAnd_ID, isNot_ID, isMov_ID;

  control_unit CU (
    .inst(instr_ID),
    .isSt(isSt_ID), .isLd(isLd_ID), .isBeq(isBeq_ID), .isBgt(isBgt_ID), .isRet(isRet_ID),
    .isImmediate(isImmediate_ID), .isWb(isWb_ID), .isUbranch(isUbranch_ID), .isCall(isCall_ID),
    .isAdd(isAdd_ID), .isSub(isSub_ID), .isCmp(isCmp_ID),
    .isMul(isMul_ID), .isDiv(isDiv_ID), .isMod(isMod_ID),
    .isLsl(isLsl_ID), .isLsr(isLsr_ID), .isAsr(isAsr_ID),
    .isOr(isOr_ID),   .isAnd(isAnd_ID), .isNot(isNot_ID), .isMov(isMov_ID)
  );

  // Pack control signals into vector
  wire [21:0] ctrl_ID = { isSt_ID, isLd_ID, isBeq_ID, isBgt_ID, isRet_ID,
                          isImmediate_ID, isWb_ID, isUbranch_ID, isCall_ID,
                          isAdd_ID, isSub_ID, isCmp_ID, isMul_ID, isDiv_ID,
                          isMod_ID, isLsl_ID, isLsr_ID, isAsr_ID,
                          isOr_ID, isAnd_ID, isNot_ID, isMov_ID };

  // Operand Fetch and Register File
  wire [3:0] rd_ID   = instr_ID[25:22];
  wire [3:0] rs1_ID  = instr_ID[21:18];
  wire [3:0] rs2_ID  = instr_ID[17:14];
  wire [3:0] ra      = 4'd15;
  wire [3:0] inp1_ID, inp2_ID;
  wire [31:0] reg1_out_ID, reg2_out_ID;

  OperandFetchUnit OFU (
    .isRet(isRet_ID), .isSt(isSt_ID),
    .rs1(rs1_ID), .rs2(rs2_ID), .rd(rd_ID), .ra(ra),
    .inpregfile1(inp1_ID), .inpregfile2(inp2_ID)
  );
  wire [21:0] ctrl_WB;
  wire [31:0] wb_data;
  wire [3:0]  wb_addr;
  Register_file RF (
    .clock(clk), .reset(rst),
    .reg_rd1(inp1_ID), .reg_rd2(inp2_ID),
    .reg_rd1_out(reg1_out_ID), .reg_rd2_out(reg2_out_ID),
    .reg_wr1(wb_addr), .reg_wr1_data(wb_data), .wr1_enable(ctrl_WB[15])
  );

  // Immediate Generation
  wire [31:0] imm_ID, branchTarget_ID;
  imm_gen IMG (
    .inst(instr_ID),
    .immx(imm_ID),
    .pc(pc_ID),
    .branchTarget(branchTarget_ID)
  );

  // === ID/EX Pipeline Register ===
  wire [31:0] pc_EX, instr_EX;
  wire [31:0] reg1_EX, reg2_EX, imm_EX, branchTarget_EX;
  wire [21:0] ctrl_EX;
  pipo_OF_EX ID_EX (
    .clk(clk), .rst(rst),
    .pc(pc_ID), .instruction(instr_ID),
    .a(reg1_out_ID), .b(reg2_out_ID), .op2(imm_ID),
    .control(ctrl_ID), .branchTarget(branchTarget_ID),
    .pcc(pc_EX), .instr(instr_EX), .ao(reg1_EX), .bo(reg2_EX),
    .oppo(imm_EX), .co(ctrl_EX), .bto(branchTarget_EX)
  );

  // === EX Stage ===
  // Unpack control signals
  wire isSt_EX        = ctrl_EX[21];
  wire isLd_EX        = ctrl_EX[20];
  wire isBeq_EX       = ctrl_EX[19];
  wire isBgt_EX       = ctrl_EX[18];
  wire isRet_EX       = ctrl_EX[17];
  wire isImmediate_EX = ctrl_EX[16];
  wire isWb_EX        = ctrl_EX[15];
  wire isUbranch_EX   = ctrl_EX[14];
  wire isCall_EX      = ctrl_EX[13];
  wire isAdd_EX       = ctrl_EX[12];
  wire isSub_EX       = ctrl_EX[11];
  wire isCmp_EX       = ctrl_EX[10];
  wire isMul_EX       = ctrl_EX[9];
  wire isDiv_EX       = ctrl_EX[8];
  wire isMod_EX       = ctrl_EX[7];
  wire isLsl_EX       = ctrl_EX[6];
  wire isLsr_EX       = ctrl_EX[5];
  wire isAsr_EX       = ctrl_EX[4];
  wire isOr_EX        = ctrl_EX[3];
  wire isAnd_EX       = ctrl_EX[2];
  wire isNot_EX       = ctrl_EX[1];
  wire isMov_EX       = ctrl_EX[0];

  // ALU operand B MUX
  wire [31:0] aluB_EX;
  mux2x1 #(.w(32)) imm_mux_EX (
    .a(reg2_EX),
    .b(imm_EX),
    .sel(isImmediate_EX),
    .o(aluB_EX)
  );

  // ALU
  wire [31:0] aluResult_EX;
  wire        cout_EX, cmp_g_EX, cmp_e_EX;

  ALU ALU_E (
    .a(reg1_EX), .b(aluB_EX),
    .isAdd(isAdd_EX), .isSub(isSub_EX), .isCmp(isCmp_EX),
    .isMul(isMul_EX), .isDiv(isDiv_EX), .isMod(isMod_EX),
    .isOr(isOr_EX), .isNot(isNot_EX), .isAnd(isAnd_EX), .isMov(isMov_EX),
    .isAsl(isLsl_EX), .isAsr(isAsr_EX), .isLsr(isLsr_EX), .isLsl(isLsl_EX),
    .isSt(isSt_EX), .isLd(isLd_EX),
    .rs11(), .rs22(),
    .aluResult(aluResult_EX),
    .cout(cout_EX), .cmp_g(cmp_g_EX), .cmp_e(cmp_e_EX)
  );

  // Branch Unit
  wire ocmp_g_EX, ocmp_e_EX;
  wire [1:0] flag_EX;
  flag_reg FLG (
    .clk(clk), .isCmp(isCmp_EX),
    .cmp_g(cmp_g_EX), .cmp_e(cmp_e_EX),
    .ocmp_g(ocmp_g_EX), .ocmp_e(ocmp_e_EX)
  );
  assign flag_EX = {ocmp_e_EX, ocmp_g_EX};

 
 
  branchunit BU (
    .branchTarget(branchTarget_EX),
    .op1(reg1_EX),
    .isBeq(isBeq_EX), .isBgt(isBgt_EX), .isUbranch(isUbranch_EX),
    .flag(flag_EX), .isRet(isRet_EX), .isCall(isCall_EX),
    .branchPC(branchPC_EX),
    .isBranchTaken(isBranchTaken_EX)
  );

  // === EX/MEM Pipeline Register ===
  wire [31:0] pc_MEM, aluOut_MEM, op2_MEM, instr_MEM, branchPC_MEM;
  wire [21:0] ctrl_MEM;
  EX_MA EX_MEM (
    .clk(clk), .rst(rst),
    .pc(pc_EX), .aluresult(aluResult_EX), .op2(reg2_EX), .instruction(instr_EX),
    .control(ctrl_EX),
    .pcc(pc_MEM), .alo(aluOut_MEM), .opo(op2_MEM), .io(instr_MEM), .co(ctrl_MEM)
  );

  // === MEM Stage ===
  wire [31:0] ldResult_MEM;
  memoryaccessunit MA (
    .op2(op2_MEM),
    .aluResult(aluOut_MEM),
    .isLd(ctrl_MEM[20]), .isSt(ctrl_MEM[21]),
    .clk(clk),
    .ldresult(ldResult_MEM)
  );

  // Pass branch information to IF stage
  assign branchPC_IF      = branchPC_EX;
  assign isBranchTaken_IF = isBranchTaken_EX;

  // === MEM/WB Pipeline Register ===
  wire [31:0] pc_WB, ldOut_WB, aluOut_WB,instr_WB;
  MA_RW MEM_WB (
    .clk(clk), .rst(rst),
    .pc(pc_MEM), .ldresult(ldResult_MEM), .aluresult(aluOut_MEM), .instruction(instr_MEM),
    .control(ctrl_MEM),
    .pcc(pc_WB), .ldo(ldOut_WB), .alo(aluOut_WB),.io(instr_WB), .co(ctrl_WB)
  );
  regwriteback WB (
    .aluResult(aluOut_WB),
    .ldResult(ldOut_WB),
    .prpc(pc_WB + 32'd4),
    .isLd(ctrl_WB[20]), .isCall(ctrl_WB[13]), .isWb(ctrl_WB[15]),
    .ra(ra), .rd(instr_WB[25:22]),
    .writedata(wb_data),
    .writeadd(wb_addr)
  );
endmodule