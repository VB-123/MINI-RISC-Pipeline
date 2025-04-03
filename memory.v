`timescale 1ps/1ps
`include "parameters.v"
module memory (
    input wire clk,
    input wire write_en,
    input wire [`ADDR_WIDTH-1:0] read_address,
    input wire [`ADDR_WIDTH-1:0] write_address,
    input wire [`DATA_WIDTH-1:0] data_in,
    output wire [`DATA_WIDTH-1:0] data_out
);
  reg [`DATA_WIDTH-1:0] mem[0:`MEMORY_DEPTH-1];

  assign data_out = mem[read_address];
  always @(posedge clk) begin
    if (write_en) mem[write_address] <= data_in;
  end


endmodule