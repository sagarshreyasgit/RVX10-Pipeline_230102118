`timescale 1ns / 1ps
//======================================================================
// Control Pipeline Register: ID/EX
// Purpose : Holds decoded control signals for the Execute (EX) stage.
// Features:
//   - Asynchronous reset
//   - Synchronous flush (clear)
//   - Valid bit propagation
//======================================================================
module ctrl_ID_EX (
  input  logic        clk, reset, flush,
  // --- Incoming control signals from ID ---
  input  logic        RegWriteD, MemWriteD, JumpD, BranchD, ALUSrcD,
  input  logic [1:0]  ResultSrcD,
  input  logic [4:0]  ALUControlD,     // 5 bits for RVX10 extended ALU ops
  // --- Outgoing latched signals to EX ---
  output logic        RegWriteE, MemWriteE, JumpE, BranchE, ALUSrcE,
  output logic [1:0]  ResultSrcE,
  output logic [4:0]  ALUControlE
);

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      RegWriteE   <= 1'b0;
      MemWriteE   <= 1'b0;
      JumpE       <= 1'b0;
      BranchE     <= 1'b0;
      ALUSrcE     <= 1'b0;
      ResultSrcE  <= 2'b00;
      ALUControlE <= 5'b00000;
    end
    else if (flush) begin
      // Flush on branch/jump or hazard: insert NOP control
      RegWriteE   <= 1'b0;
      MemWriteE   <= 1'b0;
      JumpE       <= 1'b0;
      BranchE     <= 1'b0;
      ALUSrcE     <= 1'b0;
      ResultSrcE  <= 2'b00;
      ALUControlE <= 5'b00000;
    end
    else begin
      RegWriteE   <= RegWriteD;
      MemWriteE   <= MemWriteD;
      JumpE       <= JumpD;
      BranchE     <= BranchD;
      ALUSrcE     <= ALUSrcD;
      ResultSrcE  <= ResultSrcD;
      ALUControlE <= ALUControlD;
    end
  end

endmodule
