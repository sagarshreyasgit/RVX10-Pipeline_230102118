`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.10.2025 17:50:41
// Design Name: 
// Module Name: alu
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


module alu(input  logic [31:0] a, b,
           input  logic [4:0]  alucontrol, // MODIFIED: Widened to 5 bits
           output logic [31:0] result,
           output logic        zero);

  logic [31:0] condinvb, sum;
  logic        v;              // overflow
  logic        isAddSub;       // true when is add or subtract operation
  logic signed [31:0] signed_a, signed_b;
  logic [4:0] shamt;

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum = a + condinvb + alucontrol[0];
  assign isAddSub = (alucontrol == 5'b00000) || (alucontrol == 5'b00001);

  assign signed_a = a;
  assign signed_b = b;
  assign shamt = b[4:0];
  
  always_comb
    case (alucontrol)
      // Base ISA operations
      5'b00000: result = sum;         // add
      5'b00001: result = sum;         // subtract
      5'b00010: result = a & b;       // and
      5'b00011: result = a | b;       // or
      5'b00101: result = sum[31] ^ v; // slt
      5'b00100: result = a ^ b;       // xor
      5'b00110: result = a << b[4:0]; // sll
      5'b00111: result = a >> b[4:0]; // srl
      // RVX10 custom operations
      5'b01000: result = a & ~b;                                                     // ANDN
      5'b01001: result = a | ~b;                                                     // ORN
      5'b01010: result = ~(a ^ b);                                                   // XNOR
      5'b01011: result = (signed_a < signed_b) ? a : b;                              // MIN
      5'b01100: result = (signed_a > signed_b) ? a : b;                              // MAX
      5'b01101: result = (a < b) ? a : b;                                            // MINU
      5'b01110: result = (a > b) ? a : b;                                            // MAXU
      5'b01111: result = (shamt == 5'b0) ? a : (a << shamt) | (a >> (32 - shamt));    // ROL
      5'b10000: result = (shamt == 5'b0) ? a : (a >> shamt) | (a << (32 - shamt));    // ROR
      5'b10001: result = (signed_a >= 32'sd0) ? a : -signed_a;                       // ABS
      default: result = 32'bx;
    endcase

  assign zero = (result == 32'b0);
  assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
  
endmodule
