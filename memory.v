`include "parameters.v"
//////////////////////////////////////////////////////////////////////////////////
//Company: 
// Engineer: 
// 
// Create Date: 13.02.2025 09:26:39
// Design Name: 
// Module Name: memory
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

module memory (
    input wire clk,
    input wire write_en,
    input wire [`ADDR_WIDTH-1:0] read_address,
    input wire [`ADDR_WIDTH-1:0] write_address,
    input wire [`DATA_WIDTH-1:0] data_in,
    output wire [`DATA_WIDTH-1:0] data_out
);

  reg [`DATA_WIDTH-1:0] mem[0:`MEMORY_DEPTH-1];  // 11 bits to index

  assign data_out = mem[read_address];
  always @(posedge clk) begin
    if (write_en) mem[write_address] <= data_in;
  end


endmodule