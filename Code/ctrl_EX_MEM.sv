`timescale 1ns / 1ps
//======================================================================
// Control Pipeline Register: EX/MEM
// Purpose : Carries control signals to Memory (MEM) stage.
// Features:
//   - Asynchronous reset
//   - Simple latching (no flush needed here, already handled earlier)
//======================================================================
module ctrl_EX_MEM (
  input  logic        clk, reset,
  // --- Incoming control signals from EX ---
  input  logic        RegWriteE, MemWriteE,
  input  logic [1:0]  ResultSrcE,
  // --- Outgoing latched signals to MEM ---
  output logic        RegWriteM, MemWriteM,
  output logic [1:0]  ResultSrcM
);

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      RegWriteM  <= 1'b0;
      MemWriteM  <= 1'b0;
      ResultSrcM <= 2'b00;
    end
    else begin
      RegWriteM  <= RegWriteE;
      MemWriteM  <= MemWriteE;
      ResultSrcM <= ResultSrcE;
    end
  end

endmodule
