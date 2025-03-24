module reg_file (
    input wire clk,
    input wire rst,
    // Read ports
    input wire [2:0] read_addr_0,
    input wire [2:0] read_addr_1,
    output wire [15:0] read_data_0,
    output wire [15:0] read_data_1,
    
    // Write ports
    input wire reg_write_en,           // Enable writing to registers
    input wire [1:0] write_mode,       // 00: no write, 01: write through port 1, 11: write through port 1 and port 2
    input wire [2:0] reg_write_addr_0, // Address for first register to write
    input wire [2:0] reg_write_addr_1, // Address for second register to write (for high byte in multi-register ops)
    input wire [15:0] data_in_0,       // Data to write to first register
    input wire [15:0] data_in_1        // Data to write to second register
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
   else if (reg_write_en) begin
        case (write_mode)
            2'b01: registers[reg_write_addr_0] <= data_in_0;   // Write low byte
            2'b11: begin
                registers[reg_write_addr_0] <= data_in_0; 
                registers[reg_write_addr_1] <= data_in_1; 
            end
            default: ; // No write
        endcase
    end
  end
  assign read_data_0 = registers[read_addr_0];
  assign read_data_1 = registers[read_addr_1];
endmodule
