`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 15.02.2025 17:16:44
// Design Name: 
// Module Name: program_counter
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

module program_counter (
  input wire clk,
  input wire rst,
  input wire inc,
  input wire branch_en,
  input wire [10:0] branch_addr,
  output reg [10:0] current_addr
);

  always @(posedge clk or posedge rst) begin
    if (rst) begin
      current_addr <= 11'b0; // Explicitly initialize to 0 on reset
      $display("PC RESET to 0");
    end else if (branch_en) begin
      current_addr <= branch_addr;
      $display("PC BRANCH to %d", branch_addr);
    end else if (inc) begin
      current_addr <= current_addr + 11'b1;
      $display("PC INCREMENT to %d", current_addr + 11'b1);
    end
  end
endmodule
