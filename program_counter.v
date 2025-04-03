`timescale 1ps / 1ps
module program_counter (
  input wire clk,
  input wire rst,
  input wire inc,
  input wire branch_en,
  input wire halt, 
  input wire [10:0] branch_addr,
  output reg [10:0] current_addr
);
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      current_addr <= 11'b0;
    end else begin
      if (halt) begin
        current_addr <= current_addr; 
      end else if (branch_en) begin
        current_addr <= branch_addr;
      end else if (inc) begin
        current_addr <= current_addr + 11'b1;
      end else begin
      end
    end
  end
endmodule