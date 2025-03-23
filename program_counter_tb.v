// Standalone testbench to verify key components
module pc_verify_tb;
  reg clk, rst, inc, branch_en, halt;
  reg [10:0] branch_addr;
  wire [10:0] current_addr;
  
  // Instantiate just the program counter
  program_counter pc_test(
    .clk(clk),
    .rst(rst),
    .inc(inc),
    .branch_en(branch_en),
    .halt(halt),
    .branch_addr(branch_addr),
    .current_addr(current_addr)
  );
  
  // Clock generation
  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end
  
  // Test sequence
  initial begin
    // Initialize all inputs
    rst = 1;
    inc = 0;
    branch_en = 0;
    halt = 0;
    branch_addr = 11'h100;
    
    // Release reset after 2 cycles
    repeat(2) @(posedge clk);
    rst = 0;
    
    // Test increment
    repeat(2) @(posedge clk);
    inc = 1;
    $display("Testing increment");
    
    repeat(3) @(posedge clk);
    
    // Test branch
    inc = 0;
    branch_en = 1;
    $display("Testing branch");
    
    repeat(2) @(posedge clk);
    
    // Test halt
    branch_en = 0;
    halt = 1;
    $display("Testing halt");
    
    repeat(2) @(posedge clk);
    
    $finish;
  end
  
  // Monitor PC value
  always @(posedge clk) begin
    $display("PC Value: %d", current_addr);
  end
endmodule