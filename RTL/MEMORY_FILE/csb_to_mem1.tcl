#!/usr/bin/env tclsh
# Function to convert a decimal number to a 4-digit hexadecimal string
proc generate_mem_file {csv_file mem_file} {
    # Open the CSV file for reading
    set csv [open $csv_file r]
    set lines [split [read $csv] "\n"]
    close $csv

    # Open the .mem file for writing
    set mem [open $mem_file w]

    # Initialize variables
    set current_address 0
    set max_address 0

    # Parse the CSV file and store the data in a dictionary
    array set data_dict {}
    foreach line $lines {
        if {[string trim $line] eq ""} {continue}
        lassign [split $line ","] address data
        set address [string trim $address]
        set data [string trim $data]
        # Convert address to decimal if it's in hexadecimal
        if {[string match "0x*" $address]} {
            set address [expr {0x$address}]
        }
        # Remove '0x' prefix from data if present
        if {[string match "0x*" $data]} {
            set data [string range $data 2 end]
        }
        set data_dict($address) $data
        if {$address > $max_address} {
            set max_address $address
        }
    }

    # Write the .mem file
    for {set i 0} {$i <= $max_address} {incr i} {
        if {[info exists data_dict($i)]} {
            puts $mem $data_dict($i)
        } else {
            puts $mem "00000000"
        }
    }

    # Close the .mem file
    close $mem
}
# Example usage
set csv_file "Instruction_memory.csv"  
set mem_file "pte.mem" 

generate_mem_file $csv_file $mem_file
puts "Memory file $mem_file generated successfully."
