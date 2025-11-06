//`timescale 1ns/1ps
//module Processor_tb;
//  // Testbench signals
//  reg clk;
//  reg rst;
  
//  // Instantiate the processor
//  Processor dut (
//    .clk(clk),
//    .rst(rst)
//  );
//  // Clock generator
//  initial begin
//    clk = 0;
//    forever #5 clk = ~clk; // 100MHz clock
//  end
//  // Instruction memory initialization
//  // Loading add test instructions
//  initial begin
//    // Initialize instruction memory with ADD instructions
//    dut.IF.instrumem[0] = 8'h4C;
//    dut.IF.instrumem[1] = 8'h40;
//    dut.IF.instrumem[2] = 8'h00;
//    dut.IF.instrumem[3] = 8'h00;
    
//    // ADD R2, R0, #5 (R2 = 0 + 5 = 5) - Immediate
//    dut.IF.instrumem[4] = 8'h00;
//    dut.IF.instrumem[5] = 8'h00;
//    dut.IF.instrumem[6] = 8'h00;
//    dut.IF.instrumem[7] = 8'h00;
    
//    // ADD R3, R2, R2 (R3 = 5 + 5 = 10)
//    dut.IF.instrumem[8] = 8'h00;
//    dut.IF.instrumem[9] = 8'h00;
//    dut.IF.instrumem[10] = 8'h00;
//    dut.IF.instrumem[11] = 8'h00;
    
//    // ADD R4, R3, #15 (R4 = 10 + 15 = 25) - Immediate
//    dut.IF.instrumem[12] = 8'h00;
//    dut.IF.instrumem[13] = 8'h00;
//    dut.IF.instrumem[14] = 8'h00;
//    dut.IF.instrumem[15] = 8'h00;
    
//    dut.IF.instrumem[16] = 8'h04;
//    dut.IF.instrumem[17] = 8'h84;
//    dut.IF.instrumem[18] = 8'h00;
//    dut.IF.instrumem[19] = 8'h05;
    
//    // ADD R3, R2, R2 (R3 = 5 + 5 = 10)
//    dut.IF.instrumem[20] = 8'h00;
//    dut.IF.instrumem[21] = 8'h00;
//    dut.IF.instrumem[22] = 8'h00;
//    dut.IF.instrumem[23] = 8'h00;
    
//    // ADD R4, R3, #15 (R4 = 10 + 15 = 25) - Immediate
//    dut.IF.instrumem[24] = 8'h00;
//    dut.IF.instrumem[25] = 8'h00;
//    dut.IF.instrumem[26] = 8'h00;
//    dut.IF.instrumem[27] = 8'h00;
    
//    // ADD R5, R4, R3 (R5 = 25 + 10 = 35)
//    dut.IF.instrumem[28] = 8'h00;
//    dut.IF.instrumem[29] = 8'h00;
//    dut.IF.instrumem[30] = 8'h00;
//    dut.IF.instrumem[31] = 8'h00;
    
//    dut.IF.instrumem[32] = 8'h00;
//    dut.IF.instrumem[33] = 8'hC8;
//    dut.IF.instrumem[34] = 8'h80;
//    dut.IF.instrumem[35] = 8'h00;
//  end
  
//  // Data memory initialization
//  initial begin
    
//    // Monitor basic register results
//    $monitor("Time=%0t | PC=%0d | R1=%0d | R2=%0d | R3=%0d | R4=%0d | R5=%0d", 
//             $time,
//             dut.IF.PC,
//             dut.RF.register[1], 
//             dut.RF.register[2], 
//             dut.RF.register[3], 
//             dut.RF.register[4], 
//             dut.RF.register[5]);
//  end
  
//  // Continuous monitoring of PC and ADD operations
//  always @(posedge clk) begin
//    // Display PC value at every clock cycle
//    $display("Time=%0t | PC=%0d", $time, dut.IF.PC);
    
//    // Check if ADD operation is in Execute stage
//    // Assuming you have a signal in your Control unit that indicates an ADD operation
//    // Replace dut.EX.Control.ADD_signal with the actual signal name in your design
//    if (dut.EX.Control.ADD_signal) begin
//      $display("Time=%0t | PC=%0d | ADD operation detected in Execute stage", $time, dut.IF.PC);
//    end
    
//    // Alternative approach if you don't have a specific ADD signal
//    // Check the ALU operation code for ADD (typically 0x20 for ADD)
//    if (dut.EX.ALUControl == 4'b0010 || dut.EX.ALUOp == 2'b10) begin // Adjust based on your actual ALU control encoding
//      $display("Time=%0t | PC=%0d | ADD operation detected (via ALU control)", $time, dut.IF.PC);
//    end
//  end
  
//  // Test sequence
//  initial begin
//    // Apply reset
//    rst = 1;
//    #15 rst = 0;
    
//    // Wait for pipeline to complete all instructions
//    // 5 instructions + 5 pipeline stages = 10 cycles
//    #100;
    
//    // Check results
//    if (dut.RF.register[1] !== 32'd0) $display("ERROR: R1 should be 0, got %0d", dut.RF.register[1]);
//    else $display("✓ R1 = 0 (PASSED)");
    
//    if (dut.RF.register[2] !== 32'd5) $display("ERROR: R2 should be 5, got %0d", dut.RF.register[2]);
//    else $display("✓ R2 = 5 (PASSED)");
    
//    if (dut.RF.register[3] !== 32'd10) $display("ERROR: R3 should be 10, got %0d", dut.RF.register[3]);
//    else $display("✓ R3 = 10 (PASSED)");
    
//    if (dut.RF.register[4] !== 32'd25) $display("ERROR: R4 should be 25, got %0d", dut.RF.register[4]);
//    else $display("✓ R4 = 25 (PASSED)");
    
//    if (dut.RF.register[5] !== 32'd35) $display("ERROR: R5 should be 35, got %0d", dut.RF.register[5]);
//    else $display("✓ R5 = 35 (PASSED)");
    
//    $display("ADD operation test completed");
//    #20 $finish;
//  end
//endmodule




`timescale 1ns/1ps
module tb_processor;
  // Testbench signals
  reg clk;
  reg rst;
  
  // Instantiate the processor
  Final_Pipelined dut (
    .clk(clk),
    .rst(rst)
  );
  
  // Clock generator
  initial begin
    clk = 0;
    forever #5 clk = ~clk; // 100MHz clock
  end
  
  // Instruction memory initialization
  initial begin
     dut.IF.instrumem[0] = 8'h4C;
    dut.IF.instrumem[1] = 8'h80;
    dut.IF.instrumem[2] = 8'h00;
    dut.IF.instrumem[3] = 8'h05;
    
    // ADD R2, R0, #5 (R2 = 0 + 5 = 5) - Immediate
    dut.IF.instrumem[4] = 8'h00;
    dut.IF.instrumem[5] = 8'h00;
    dut.IF.instrumem[6] = 8'h00;
    dut.IF.instrumem[7] = 8'h00;
    
    // ADD R3, R2, R2 (R3 = 5 + 5 = 10)
    dut.IF.instrumem[8] = 8'h00;
    dut.IF.instrumem[9] = 8'h00;
    dut.IF.instrumem[10] = 8'h00;
    dut.IF.instrumem[11] = 8'h00;
    
    // ADD R4, R3, #15 (R4 = 10 + 15 = 25) - Immediate
    dut.IF.instrumem[12] = 8'h00;
    dut.IF.instrumem[13] = 8'h00;
    dut.IF.instrumem[14] = 8'h00;
    dut.IF.instrumem[15] = 8'h00;
    
    dut.IF.instrumem[16] = 8'h4C;
    dut.IF.instrumem[17] = 8'h40;
    dut.IF.instrumem[18] = 8'h00;
    dut.IF.instrumem[19] = 8'h05;
    
       // ADD R2, R0, #5 (R2 = 0 + 5 = 5) - Immediate
    dut.IF.instrumem[20] = 8'h00;
    dut.IF.instrumem[21] = 8'h00;
    dut.IF.instrumem[22] = 8'h00;
    dut.IF.instrumem[23] = 8'h00;
    
    // ADD R3, R2, R2 (R3 = 5 + 5 = 10)
    dut.IF.instrumem[24] = 8'h00;
    dut.IF.instrumem[25] = 8'h00;
    dut.IF.instrumem[26] = 8'h00;
    dut.IF.instrumem[27] = 8'h00;
    
    // ADD R4, R3, #15 (R4 = 10 + 15 = 25) - Immediate
    dut.IF.instrumem[28] = 8'h00;
    dut.IF.instrumem[29] = 8'h00;
    dut.IF.instrumem[30] = 8'h00;
    dut.IF.instrumem[31] = 8'h00;
    
     dut.IF.instrumem[32] = 8'h28;
    dut.IF.instrumem[33] = 8'h40;
    dut.IF.instrumem[34] = 8'h80;
    dut.IF.instrumem[35] = 8'h00;
    
      dut.IF.instrumem[16] = 8'h80;
    dut.IF.instrumem[17] = 8'h00;
    dut.IF.instrumem[18] = 8'h00;
    dut.IF.instrumem[19] = 8'h03;
    
       // ADD R2, R0, #5 (R2 = 0 + 5 = 5) - Immediate
    dut.IF.instrumem[20] = 8'h00;
    dut.IF.instrumem[21] = 8'h00;
    dut.IF.instrumem[22] = 8'h00;
    dut.IF.instrumem[23] = 8'h00;
    
    
    // ADD R4, R3, #15 (R4 = 10 + 15 = 25) - Immediate
    dut.IF.instrumem[24] = 8'h00;
    dut.IF.instrumem[25] = 8'hc8;
    dut.IF.instrumem[26] = 8'h80;
    dut.IF.instrumem[27] = 8'h00;
    
 
  end
  
  // Track PC and control signals
  always @(posedge clk) begin
    // Print PC (Program Counter) at every clock cycle
    $display("Time=%0t | PC=%0d", $time, dut.IF.lo);
    
    // Check for ADD operation in Control unit signals
    // We look at the Execute stage signals that show control info
    if (dut.ID_EX.co[12]) begin  // isAdd_EX signal is bit 12 of control vector
      $display("Time=%0t | PC=%0d | ADD operation detected in Execute stage", 
               $time, dut.ID_EX.pcc);
    end
  end
  
  // Final register value check
  initial begin
    // Apply reset
    rst = 1;
    #15 rst = 0;
    
    // Wait for pipeline to complete all instructions
    // 5 instructions + 5 pipeline stages = 10 cycles at minimum
    #200;
    
    // Display final register values
    $display("\n=== Final Register Values ===");
    $display("R1 = %0d", dut.RF.register[1]);
    $display("R2 = %0d", dut.RF.register[2]);
    $display("R3 = %0d", dut.RF.register[3]);
    $display("R4 = %0d", dut.RF.register[4]);
    $display("R5 = %0d", dut.RF.register[5]);
    
    // Check expected results
    if (dut.RF.register[2] !== 32'd5) $display("ERROR: R2 should be 5, got %0d", dut.RF.register[2]);
    else $display("✓ R2 = 5 (PASSED)");
    
    if (dut.RF.register[3] !== 32'd10) $display("ERROR: R3 should be 10, got %0d", dut.RF.register[3]);
    else $display("✓ R3 = 10 (PASSED)");
    
    if (dut.RF.register[4] !== 32'd25) $display("ERROR: R4 should be 25, got %0d", dut.RF.register[4]);
    else $display("✓ R4 = 25 (PASSED)");
    
    if (dut.RF.register[5] !== 32'd35) $display("ERROR: R5 should be 35, got %0d", dut.RF.register[5]);
    else $display("✓ R5 = 35 (PASSED)");
    
    $display("ADD operation test completed");
    $finish;
  end
endmodule