I'll help you understand how to implement proper forwarding and hazard detection units for your processor. These are critical components for handling data hazards in a pipelined design.

## Understanding the Issues

From examining your processor.v file, I can see you've already defined a `HAZARD_Unit` and have connections for forwarding (forward_A, forward_B), but the implementation is missing or not working correctly. Let's focus on how to properly implement these components.

## Forwarding Unit Implementation

The forwarding unit detects and resolves data hazards by bypassing results from later pipeline stages back to the execute stage.

### Key Signals to Monitor:
1. **Register addresses being read** in the execute stage (rs1_E, rs2_E)
2. **Register being written to** in the memory/writeback stage (reg_write_addr_W)
3. **Register being written to** in the execute/memory stage (reg_write_addr_E)
4. **Register write enable signals** for both stages to ensure the write is happening

### Implementation Approach:

1. **Forward from Writeback to Execute (forward_A/B = 01)**:
   - If rs1_E == reg_write_addr_W && write_mode_W[0] == 1
   - If rs2_E == reg_write_addr_W && write_mode_W[0] == 1

2. **Forward from Memory to Execute (forward_A/B = 10)**:
   - If rs1_E == reg_write_addr_E && write_mode_E[0] == 1
   - If rs2_E == reg_write_addr_E && write_mode_E[0] == 1

3. **No Forwarding (forward_A/B = 00)**:
   - Default case when no hazard is detected

You'll need to extract rs1_E and rs2_E from your opcode_E, since they're currently not explicitly defined in your pipeline registers.

## Hazard Detection Unit Implementation

The hazard detection unit identifies situations where forwarding cannot resolve a hazard, requiring pipeline stalls.

### Key Hazards to Detect:

1. **Load-Use Hazard**: When an instruction depends on the result of a load instruction immediately before it.
   - If (mem_read_E == 1) && ((rs1_D == reg_write_addr_E) || (rs2_D == reg_write_addr_E))
   - Action: Stall the pipeline (stall_F = 1, stall_D = 1)

2. **Control Hazards**: For branch/jump instructions.
   - If (branch_D == 1 || jump_D == 1)
   - Action: Flush the decode stage (flush_D = 1)

### Implementation Approach:

```
// Simplified pseudo-code for hazard detection
always @(*) begin
    // Default: no stalls or flushes
    stall_F = 0;
    stall_D = 0;
    flush_F = 0;
    flush_D = 0;
    
    // Load-use hazard
    if (mem_read_E && ((rs1_D == reg_write_addr_E) || (rs2_D == reg_write_addr_E))) begin
        stall_F = 1;
        stall_D = 1;
    end
    
    // Control hazard
    if (branch_D || jump_D) begin
        flush_D = 1;
        // Only flush fetch if we're actually taking the branch
        if (branch_taken) flush_F = 1;
    end
end
```

For branch_taken logic, you'll need to evaluate this based on your condition codes and branch type.

## Key Connections to Check

In your processor.v file, ensure these connections are correct:

1. **Forward_A and Forward_B connections**:
   ```verilog
   assign fwd_A = (forward_A == 2'b10) ? alu_result_0_E :
                 (forward_A == 2'b01) ? reg_write_data_0_W :
                 reg_data_1_E;
   ```

2. **Hazard Unit inputs**:
   - Make sure rs1_D, rs2_D, reg_write_addr_E, and reg_write_addr_W are properly extracted and connected
   - Ensure write_enable signals (read_write_E, read_write_W) are connected properly

3. **Pipeline Stalling Mechanism**:
   - Check that stall signals properly prevent pipeline registers from updating
   - Ensure PC is not incremented when stalling

## Implementation Guidelines

1. **Extract source registers in execute stage**:
   You need to add source register fields to the DE_Register to properly track dependencies.

2. **Fix HAZARD_Unit interface**:
   ```verilog
   module HAZARD_Unit(
       input wire [4:0] opcode_D, opcode_E,
       input wire [2:0] source_reg1_D, source_reg2_D, 
       input wire [2:0] rd_E, rd_W,
       input wire mem_read_E,
       input wire branch_D, jump_hzd,
       input wire reg_write_E, reg_write_W,
       
       output reg stall_F, stall_D,
       output reg flush_F, flush_D,
       output reg [1:0] forward_A, forward_B
   );
   ```

3. **Implement the hazard detection logic**:
   ```verilog
   // Load-use hazard detection
   always @(*) begin
       stall_F = 1'b0;
       stall_D = 1'b0;
       
       // Load followed by use
       if (mem_read_E && 
          ((source_reg1_D == rd_E) || (source_reg2_D == rd_E))) begin
           stall_F = 1'b1;
           stall_D = 1'b1;
       end
   end
   ```

4. **Implement the forwarding logic**:
   ```verilog
   // Forwarding logic
   always @(*) begin
       forward_A = 2'b00;  // Default: no forwarding
       forward_B = 2'b00;
       
       // Forward from EX/MEM stage
       if (reg_write_E && (rd_E != 0) && 
          (rd_E == source_reg1_E)) begin
           forward_A = 2'b10;
       end
       
       // Forward from MEM/WB stage
       else if (reg_write_W && (rd_W != 0) && 
               (rd_W == source_reg1_E)) begin
           forward_A = 2'b01;
       end
       
       // Similar logic for forward_B
       // ...
   end
   ```

By carefully implementing these components and ensuring the proper connections, you should be able to resolve the data hazards in your processor design. Make sure to carefully trace through each signal to ensure they're properly connected between pipeline stages.