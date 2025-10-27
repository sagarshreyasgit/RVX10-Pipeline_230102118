`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.10.2025 18:58:30
// Design Name: 
// Module Name: ctrl_MEM_WB
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

//======================================================================
// Control Pipeline Register: MEM/WB
// Purpose : Holds final write-back control signals for WB stage.
//======================================================================
module ctrl_MEM_WB (
  input  logic        clk, reset,
  // --- Incoming control signals from MEM ---
  input  logic        RegWriteM,
  input  logic [1:0]  ResultSrcM,
  // --- Outgoing latched signals to WB ---
  output logic        RegWriteW,
  output logic [1:0]  ResultSrcW
);

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      RegWriteW  <= 1'b0;
      ResultSrcW <= 2'b00;
    end
    else begin
      RegWriteW  <= RegWriteM;
      ResultSrcW <= ResultSrcM;
    end
  end

endmodule
