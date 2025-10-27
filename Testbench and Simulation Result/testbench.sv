`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.10.2025 21:54:12
// Design Name: 
// Module Name: testbench
// Project Name: RVX10-P
// Target Devices: 
// Tool Versions: 
// Description: Testbench for the 'top' module of the RISC-V pipelined processor.
// 
// Dependencies: top.sv
// 
//////////////////////////////////////////////////////////////////////////////////

module testbench();

  // --- Testbench-side signals ---
  logic        clk;
  logic        reset;
  logic [31:0] WriteData;
  logic [31:0] DataAdr;
  logic        MemWrite;
  
  // --- FIX: Variables moved from 'always' block to module level ---
  logic [31:0] total_cycles_at_stop;
  logic [31:0] total_instrs_at_stop;
  real         final_cpi;
  // -------------------------------------------------------------

  // --- Instantiate the Device Under Test (DUT) ---
  // This is your 'top.sv' module
  top dut (
    .clk(clk),
    .reset(reset),
    .WriteData(WriteData),
    .DataAdr(DataAdr),
    .MemWrite(MemWrite)
  );

  // --- Clock Generation ---
  // 10ns clock period (100MHz)
  initial begin
    clk <= 0;
  end
  always #5 clk <= ~clk;

  // --- Reset Generation ---
  initial begin
    reset <= 1;
    #20; // Hold reset for 22ns (covers a few clock edges)
    reset <= 0;
  end

  // --- Result Monitoring and Checking ---
  always @(negedge clk) begin
    
    // Check only if a memory write is happening
    if (MemWrite) begin

      // --- Success Condition ---
      if (DataAdr == 100 && WriteData == 25) begin
        $display("-------------------------------------------");
        $display(">>> SIMULATION SUCCEEDED <<<");
        $display("Test Passed: Correctly wrote 25 to address 100.");
        $display("Simulation stopped at time %t.", $time);
        
        // --- (Optional) Performance Counter Report ---
        // Access counters via hierarchy: testbench -> top (dut) -> riscvpipeline (core_inst)
        
        // Note: Declarations were moved to the top of the module
        
        total_cycles_at_stop = dut.core_inst.total_cycles;
        total_instrs_at_stop = dut.core_inst.retired_instructions;

        // Calculate CPI (Cycles Per Instruction)
        if (total_instrs_at_stop > 0) begin
          final_cpi = $itor(total_cycles_at_stop) / $itor(total_instrs_at_stop);
        end else begin
          final_cpi = 0.0;
        end

        $display("\n--- PERFORMANCE (BONUS) ---");
        $display("  Total Execution Cycles: %0d", total_cycles_at_stop);
        $display("  Total Retired Instructions: %0d", total_instrs_at_stop);
        $display("  Average CPI (Cycles/Instr): %f", final_cpi);
        $display("-------------------------------------------");
      end 
      
      // --- Failure Condition ---
      // This ignores the intermediate write to address 96 from the test program
      else if (DataAdr != 96) begin
        $display("-------------------------------------------");
        $display(">>> SIMULATION FAILED <<<");
        $display("Test Failed: Wrote %0d to unexpected address %0d.", WriteData, DataAdr);
        $display("Simulation stopped at time %t.", $time);
        $display("-------------------------------------------");
        $stop; // End simulation
      end
    end
  end

endmodule




