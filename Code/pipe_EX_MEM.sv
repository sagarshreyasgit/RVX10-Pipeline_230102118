`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.10.2025 18:08:26
// Design Name: 
// Module Name: pipe_EX_MEM
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
// Pipeline Register: EX/MEM
// Purpose : Transfers ALU result, data to write, destination register,
//           and PC+4 to Memory stage.
//======================================================================
module pipe_EX_MEM (
  input  logic        clk, reset,
  input  logic [31:0] ALUResultE, WriteDataE, PCPlus4E,
  input  logic [4:0]  RdE,
  output logic [31:0] ALUResultM, WriteDataM, PCPlus4M,
  output logic [4:0]  RdM
);

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      ALUResultM <= 32'b0;
      WriteDataM <= 32'b0;
      PCPlus4M   <= 32'b0;
      RdM        <= 5'b0;
    end 
    else begin
      ALUResultM <= ALUResultE;
      WriteDataM <= WriteDataE;
      PCPlus4M   <= PCPlus4E;
      RdM        <= RdE;
    end
  end

endmodule

