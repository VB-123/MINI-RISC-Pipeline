
module program_counter_tb();
    reg clk;
    reg rst;
    reg inc;
    reg branch_en;
    reg [10:0] branch_addr;
    wire [10:0] current_addr;

    program_counter pc(
        .clk(clk),
        .rst(rst),
        .inc(inc),
        .branch_en(branch_en),
        .branch_addr(branch_addr),
        .current_addr(current_addr)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 100MHz clock (period = 10ns)
    end

    // Tests
    initial begin
        $dumpfile("program_counter_tb.vcd");
        $dumpvars(0, program_counter_tb);
        
        // Initialize signals
        rst = 0;
        inc = 0;
        branch_en = 0;
        branch_addr = 11'b0;
        
        // Test 1: Reset behavior
        #20 rst = 1;
        #20 rst = 0;
        
        // Test 2: Normal increment
        inc = 1;
        #10;
        if (current_addr !== 11'd1) 
            $display("Test 2 Failed: Expected 1, Got %d", current_addr);
        
        // Test 3: Branch operation
        branch_en = 1;
        branch_addr = 11'd20;
        #10;
        if (current_addr !== 11'd20) 
            $display("Test 3 Failed: Expected 20, Got %d", current_addr);
        
        // Test 4: Continue normal operation after branch
        branch_en = 0;
        #10;
        if (current_addr !== 11'd21) 
            $display("Test 4 Failed: Expected 21, Got %d", current_addr);
        
        // Test 5: No increment when inc is 0
        inc = 0;
        #10;
        if (current_addr !== 11'd21) 
            $display("Test 5 Failed: Expected 21, Got %d", current_addr);
        
        // End simulation
        #20 $finish;
    end

    // Monitor changes
    initial begin
        $monitor("Time=%0t rst=%b inc=%b branch_en=%b current_addr=%d branch_addr=%d",
                 $time, rst, inc, branch_en, current_addr, branch_addr);
    end
endmodule
