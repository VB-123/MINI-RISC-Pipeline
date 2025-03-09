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


module program_counter(
    input wire clk,
    input wire rst,
    input wire inc,
    input wire branch_en,
    input wire [10:0] branch_addr,
    output reg [10:0] current_addr
);
    always @(posedge clk) begin
        if (rst) begin
            current_addr <= 11'b0; 
        end
        else begin
            if (inc) begin
                if (branch_en) 
                    current_addr <= branch_addr;
                else 
                    current_addr <= current_addr + 1;
            end
        end
    end
endmodule
