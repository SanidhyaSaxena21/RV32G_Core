# Vivado TCL Script to automate RISC-V test cases

# Open the project
#open_project processor.xpr


# Default: Run all test cases
set test_mode "all"

# Parse command-line arguments
if { $argc >= 1 } {
    set test_mode [lindex $argv 0]
}


# Set the test cases directory
set test_cases_dir "../TestCases/RISC-V/test/mem_files"
set testbench "tb_platform"
set instr_mem_file "../TestCases/instruction_csr_test.mem"
set data_mem_file "../TestCases/data.mem"

# Clear the log file at the beginning
set results_file [open "test_results.log" w]
puts $results_file "Test Case Results:\n==================="
close $results_file  ;# Close immediately after clearing

# Create a results file
#set results_file [open "test_results.log" w]
#puts $results_file "Test Case Results:\n==================="

# Get all .mem files or a single specified test case
if { $test_mode eq "all" } {
    set mem_files [glob -nocomplain -directory $test_cases_dir *.mem]
    # Ensure we only get test cases, not _data.mem files
    set filtered_mem_files {}

    foreach file $mem_files {
        if { ![regexp {_data\.mem$} $file] } {
            lappend filtered_mem_files $file
        }
    }

    set mem_files $filtered_mem_files

    # Check if we have at least one valid test case
    if { [llength $mem_files] == 0 } {
        puts "ERROR: No valid test case found in $test_cases_dir!"
        exit
    } 
} else {
    set mem_files "$test_cases_dir/$test_mode"
    if { ![file exists $mem_files] } {
        puts "ERROR: Test case $test_mode not found in $test_cases_dir!"
        exit
    }
    set mem_files $test_case_file
}


# Flag to control simulation closing behavior
set debug_mode 0  ;# Default: close_sim enabled
if { $test_mode ne "all" } {
    set debug_mode 1  ;# If running a specific test case, disable close_sim
}

# Get all .mem files
#set mem_files [glob -nocomplain -directory $test_cases_dir *.mem]

# Iterate over each test case
foreach mem_file $mem_files {
    # Extract the test name
    set test_name [file rootname [file tail $mem_file]]

    # Overwrite instr_mem.mem with current test file
    file copy -force $mem_file $instr_mem_file
    puts "Loaded test case: $test_name"

  # Check if a corresponding data memory file exists
    set data_mem_testcase "${test_cases_dir}/${test_name}_data.mem"
    if { [file exists $data_mem_testcase] } {
        file copy -force $data_mem_testcase $data_mem_file
        puts "Loaded data memory file: $data_mem_testcase"
    } else {
        puts "No data memory file found for $test_name, keeping default data.mem"
    }
    
   
   # Open log file in append mode for each test case
    set results_file [open "test_results.log" a]
    puts $results_file "\nRunning Test: $test_name"
    puts $results_file "========================="
    flush $results_file 
    # Load .mem file into the instruction memory
    #set_property generic "INSTR_INPUT=./mem_files/add.mem" [get_filesets sim_1]

    # Launch Simulation
    #restart
    launch_simulation

    puts "Checking PC_HALT_BREAK values..."

    # Monitor PC_HALT_BREAK with a timeout of 200us
    set sim_time 0          ;# Simulation time counter in ns
    set max_time 200000     ;# 200us = 200000ns

    # Monitor PC_HALT_BREAK and stop when it goes high
    while { [get_value {/tb_platform/riscv_platform/cpu1/fdem/Pipeline/PC_HALT_BREAK}] == 0 } {
        run 100ns  ;# Run in small time steps to check halt condition
        #puts "INSIDE LOOOPPPPPPP..."
        set sim_time [expr $sim_time + 100]
    }

    # If timeout is reached and PC_HALT_BREAK is still 0, log the failure and continue
    if { $sim_time >= $max_time } {
        puts "ERROR: Max simulation time exceeded for $test_name!"
        puts $results_file "$test_name: FAIL (PC_HALT_BREAK did not rise within 200us)"
        flush $results_file
        close $results_file
        stop
        close_sim
        continue  ;# Skip the rest of the test case and move to the next
    }


    run 50ns

    puts "Checking register values..."

    # Run for 100,000 cycles or until stop condition
    #run 100us

    # Read intermediate signals
    # Once PC_HALT_BREAK is high, check register values
    set expected_value_x10 00000000    ;# Expected value for register x10
    set expected_value_x17 0000005d   ;# Expected value for register x17 (93 in hex)

    set actual_value_x10 [get_value {/tb_platform/riscv_platform/cpu1/fdem/Pipeline/RF/MEM[10]}]
    set actual_value_x17 [get_value {/tb_platform/riscv_platform/cpu1/fdem/Pipeline/RF/MEM[17]}]
    set PC_BREAK         [get_value {/tb_platform/riscv_platform/cpu1/fdem/Pipeline/ID/PC__IF_ID}]

    puts $results_file "Value of PC_BREAK for $test_name is $PC_BREAK"

    # Compare results
    if { ($actual_value_x10 == $expected_value_x10) && ($actual_value_x17 == $expected_value_x17) } {
        puts "TEST CASE PASSED"
        puts $results_file "$test_name: PASS"
    } else {
        puts "TEST CASE FAILED"
        puts $results_file "$test_name: FAIL (Expected x10=$expected_value_x10, x17=$expected_value_x17, Got x10=$actual_value_x10, x17=$actual_value_x17)"
    }

    flush $results_file  ;# Ensure immediate writing
    close $results_file  ;# Close the log file after writing
    # Stop Simulation
    stop
    puts "Stopping the simulation"

    # Close simulation only if debug mode is OFF
    if { $debug_mode == 0 } {
        close_sim
    }
    #close_sim

}

# Close the results file
close $results_file

# Print final results
puts "Test Execution Completed. Check test_results.log for details."

exit

