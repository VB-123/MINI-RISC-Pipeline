`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 13.02.2025 09:49:54
// Design Name: 
// Module Name: memory_tb
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


module memory_tb
  # (parameter ADDR_WIDTH = 11,  // adress is 11 bits
     parameter DATA_WIDTH = 16,  // data is 16 bits
     // 4Kb memory with 16 bit word size implies 2048 words (2^11)
     parameter DEPTH = 2**ADDR_WIDTH     
    );
  reg clk = 0;
  reg write_en;
  reg [ADDR_WIDTH-1:0] address;
  reg [DATA_WIDTH-1:0] data_in;
  wire [DATA_WIDTH-1:0] read_data;

  memory u0 (
    .clk(clk),
    .write_en(write_en),
    .address(address),
    .data_in(data_in),
    .read_data(read_data)
  );

  always #10 clk = ~clk;
  reg [DATA_WIDTH-1:0] expected_mem [0:DEPTH-1];
  reg error_found = 0;
  integer i = 0;

  initial begin
    // ----------------------------
    // WRITE PHASE
    // ----------------------------
    for (i = 0; i < DEPTH; i= i+1) begin
      expected_mem[i] <= i;
    end

    
    #10 write_en <= 1;
    for (i = 0; i < DEPTH; i= i+1) begin
      address <= i;
      data_in <= expected_mem[i];
      @(posedge clk);
    end

      // ----------------------------
      // READ & VERIFY PHASE
      // ----------------------------

      
    #10 write_en <= 0;
    for (i = 0; i < DEPTH; i= i+1) begin
      address <= i;
      @(posedge clk);
      if (read_data !== expected_mem[i]) begin
        $display("ERROR at address %0d: expected 0x%0h, got 0x%0h", i, expected_mem[i], read_data);
        error_found <= 1;
      end
    end

    if (!error_found) $display("No Errors Found!");
    #20 $finish;
  end
endmodule
