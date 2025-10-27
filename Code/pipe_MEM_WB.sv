`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.10.2025 18:08:26
// Design Name: 
// Module Name: pipe_MEM_WB
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
// Pipeline Register: MEM/WB
// Purpose : Passes memory read data, ALU result, destination register,
//           and PC+4 to Writeback stage.
//======================================================================
module pipe_MEM_WB (
  input  logic        clk, reset,
  input  logic [31:0] ReadDataM, ALUResultM, PCPlus4M,
  input  logic [4:0]  RdM,
  output logic [31:0] ReadDataW, ALUResultW, PCPlus4W,
  output logic [4:0]  RdW
);

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      ReadDataW  <= 32'b0;
      ALUResultW <= 32'b0;
      PCPlus4W   <= 32'b0;
      RdW        <= 5'b0;
    end 
    else begin
      ReadDataW  <= ReadDataM;
      ALUResultW <= ALUResultM;
      PCPlus4W   <= PCPlus4M;
      RdW        <= RdM;
    end
  end

endmodule

