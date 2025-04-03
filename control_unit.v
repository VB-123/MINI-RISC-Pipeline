`include "parameters.v"
module control_unit (
    input wire clk,
    input wire [4:0] opcode,      // from instruction register
    input wire [15:0] flag_reg_values,
    input wire [3:0] flag_index,
    // Control signals
    output reg alu_src,           // Select between register or immediate for ALU
    output reg read_write,        // Register file read/write select (00: read, 01: write low byte, 11: write high byte, 10: unused)
    output reg mem_write,         // Memory write enable
    output reg mem_to_reg,        // Select between ALU result or memory data for register write
    output reg branch_en,            // Branch instruction
    output reg inc_pc,            // Increment PC
    output reg jump,              // Jump instruction (added for hazard unit)
    output reg mem_read,          // Memory read (added for hazard unit)
    output reg alu_op,            // ALU operation control
    output reg io_op,             // IO operation
    output reg [1:0] write_mode   // Write mode for register
);

  reg flag_value;
  reg [1:0] branch_cooldown;
  always @(*) begin
    // Default control signals
    alu_src = 1'b0;      // Use register value by default
    read_write = 1'b0;  // Register read by default
    mem_write = 1'b0;    // No memory write by default
    mem_to_reg = 1'b0;   // Use ALU result by default
    branch_en = 1'b0;       // Not a branch instruction by default
    alu_op = 1'b0; 
    inc_pc = 1'b1;       // Default to incrementing PC unless explicitly disabled
    jump = 1'b0;         // Not a jump instruction by default
    mem_read = 1'b0;     // No memory read by default
    write_mode = 2'b00;  // Default write mode
    io_op = 1'b0;
    flag_value = 1'b0;
    case (opcode)
      // ALU operations
      `ADD,`SUB, `DIV, `NOT, `AND, `OR, `XOR, `INC, `RR, `RL, `SETB, `CLRB, `CPLB: begin
        read_write = 1'b1;
        alu_src = 1'b0;
        alu_op = 1'b1;
        write_mode = 2'b01;
      end
      `CMP: begin
        read_write = 1'b0;
        alu_src = 1'b0;
        alu_op = 1'b1;
        write_mode = 2'b00;
      end

      `MUL: begin
        read_write = 1'b1;
        alu_src = 1'b0;
        alu_op = 1'b1;
        write_mode = 2'b11;
      end
      
      // Flag operations
      `SETF, `CLRF, `CPLF: begin
        flag_value = flag_reg_values [flag_index];
        alu_op = 1'b1;
      end
      
      // Memory operations
      `LOAD: begin
        read_write = 1'b1;
        mem_to_reg = 1'b1;
        mem_read = 1'b1;
        write_mode = 2'b11; 
        alu_op = 1'b0;
      end
      
      `STORE: begin
        read_write = 1'b0;
        mem_write = 1'b1;
        alu_op = 1'b0;
      end
      
      // Immediate load operations
      `LBL: begin
        read_write = 1'b1;
        alu_src = 1'b1;
        write_mode = 2'b01;
        alu_op = 1'b0;
      end
      
      `LBH: begin
        read_write = 1'b1;
        alu_src = 1'b1;
        write_mode = 2'b01;
        alu_op = 1'b0;
      end
      
      // Register transfer operations
      `MOV: begin
        read_write = 1'b1;
        alu_src = 1'b0;
        write_mode = 2'b01;
        alu_op = 1'b0;
      end
      
      // Branch/Jump operations
      `JF: begin
        alu_src = 1'b0;
        read_write = 1'b0;
        mem_write = 1'b0;
        mem_to_reg = 1'b0;
        branch_en = 1'b0;
        inc_pc = 1'b1;
        jump = 1'b0;
        mem_read = 1'b0;
        alu_op = 1'b0;
        write_mode = 2'b00;
        flag_value = flag_reg_values[flag_index];

        if (flag_value) begin
            branch_en = 1'b1;
            jump = 1'b1;
            inc_pc = 1'b0;
            alu_op = 1'b0; 
        end
      end
      
      `LOADBR: begin
        read_write = 1'b0;
        write_mode = 2'b00;
        alu_op = 1'b0;
      end

      `NOP: begin
      end
      
      // I/O operations
      `MOVIN: begin
        read_write = 1'b1;
        write_mode = 2'b01;
        io_op = 1'b1;
      end

      `MOVOUT: begin
        read_write = 1'b0;
        write_mode = 2'b00;
        io_op = 1'b0;
      end

      `MOVB: begin
        alu_op = 1'b1;
        alu_src = 1'b1;
        io_op = 1'b1;
      end

      `HALT: begin
        inc_pc = 1'b0;  // Do not increment PC
      end
      
      default: begin
        alu_op = 1'b0;
      end
    endcase
  end
endmodule