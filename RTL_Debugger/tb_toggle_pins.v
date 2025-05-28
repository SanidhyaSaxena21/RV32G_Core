`timescale 1ns / 1ps

module tb_toggle_pins;

    // Differential clock signals
    reg SYSCLK_P;
    reg SYSCLK_N;

    // Output wires
    wire TDI;
    wire TD0;
    wire TMS;
    wire TCK;

    // Instantiate the Device Under Test (DUT)
    toggle_pins dut (
        .SYSCLK_P(SYSCLK_P),
        .SYSCLK_N(SYSCLK_N),
        .TDI(TDI),
        .TD0(TD0),
        .TMS(TMS),
        .TCK(TCK)
    );

    // Generate a 200 MHz differential clock: 5 ns period
    initial begin
        SYSCLK_P = 0;
        SYSCLK_N = 1;
        forever begin
            #2.5 SYSCLK_P = ~SYSCLK_P;
            SYSCLK_N = ~SYSCLK_P;
        end
    end

    // Simulation control
    initial begin
        $display("Starting simulation...");
        #1000; // Run for 1000 ns
        $display("Simulation complete.");
        $finish;
    end

    // Monitor output pins
    initial begin
        $monitor("Time = %0t | TDI = %b, TD0 = %b, TMS = %b, TCK = %b", $time, TDI, TD0, TMS, TCK);
    end

endmodule
