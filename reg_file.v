module reg_file (
    input wire clk,  // clock
    input wire rst,  // RESET
    input wire [1:0] write_en_0,  // Read/ Write signal, conected to control unit
    input wire [1:0] write_en_1,  // Read/ Write signal, conected to control unit
    input wire [2:0] read_addr_0,  // Register Select for reading data, read port 0
    input wire [2:0] read_addr_1,  // Register Select for reading data, read port 1
    input wire [2:0] reg_write_addr_0,  // Register Select for writing data 
    input wire [2:0] reg_write_addr_1,  // Register Select for writing data 
    input wire [15:0] data_in_0,// Data to be written to the register selected, write port 0,(active when write_en = 01)
    input wire [15:0] data_in_1,// Data to be written to the register selected, write port 1,(active when write_en = 11)
    output [15:0] read_data_0,  // Data read from the selected register, read port 0
    output [15:0] read_data_1  // Data read from the selected register, read port 1
);
  // Register file array
  reg [15:0] registers[0:7];

  // Reset and write logic

  always @(posedge clk or posedge rst) begin
    if (rst) begin
      // Reset all registers to 0
      registers[0] <= 16'b0;
      registers[1] <= 16'b0;
      registers[2] <= 16'b0;
      registers[3] <= 16'b0;
      registers[4] <= 16'b0;
      registers[5] <= 16'b0;
      registers[6] <= 16'b0;
      registers[7] <= 16'b0;
    end
    case (write_en_0)
      2'b00: begin
      end
      2'b01: registers[reg_write_addr_0][7:0] <= data_in_0[7:0];
      2'b10: registers[reg_write_addr_0][15:8] <= data_in_0[15:8];
      2'b11: registers[reg_write_addr_0] <= data_in_0;
    endcase

    case (write_en_1)
      2'b00: begin
      end
      2'b01: registers[reg_write_addr_1][7:0] <= data_in_1[7:0];
      2'b10: registers[reg_write_addr_1][15:8] <= data_in_1[15:8];
      2'b11: registers[reg_write_addr_1] <= data_in_1;
    endcase

  end
  // Read logic
  assign read_data_0 = (rst) ? 16'b0 : registers[read_addr_0];
  assign read_data_1 = (rst) ? 16'b0 : registers[read_addr_1];
endmodule
