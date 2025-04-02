#!/usr/bin/env tclsh
# File: csv_to_mem.tcl
# Description: Converts a CSV file with memory addresses and values into a .mem file for Vivado.

# Function to read the CSV file and generate the .mem file
proc generate_mem_file {csv_file mem_file} {
    # Declare memory_data as an array
    array set memory_data {}

    # Open the CSV file for reading
    set fp [open $csv_file r]

    # Read the CSV file line by line
    while {[gets $fp line] >= 0} {
        # Skip empty lines and comments
        if {[string trim $line] eq "" || [string index $line 0] eq "#"} {
            continue
        }

        # Parse the address and value (assuming CSV format: address,value)
        set fields [split $line ","]
        if {[llength $fields] != 2} {
            puts "Error: Invalid line format: $line"
            continue
        }
        set address [string trim [lindex $fields 0]]
        set value [string trim [lindex $fields 1]]

        # Store the value in the array with the address as the key
        set memory_data($address) $value
    }

    # Close the CSV file
    close $fp

    # Open the .mem file for writing
    set mem_fp [open $mem_file w]

    # Write a header for the .mem file
    puts $mem_fp "// Memory initialization file for Vivado"
    puts $mem_fp "// Format: one word per line in hexadecimal"

    # Get the sorted list of addresses
    set sorted_addresses [lsort -integer [array names memory_data]]

    # Write the memory data to the .mem file
    foreach address $sorted_addresses {
        # Get the value and format it as hexadecimal
        set value [format "%X" [expr {int($memory_data($address))}]]
        puts $mem_fp $value
    }

    # Close the .mem file
    close $mem_fp

    puts "Successfully generated $mem_file from $csv_file"
}

# Main script
if {[llength $argv] != 2} {
    puts "Usage: ./csv_to_mem.tcl <input.csv> <output.mem>"
    exit 1
}

set csv_file [lindex $argv 0]
set mem_file [lindex $argv 1]

generate_mem_file $csv_file $mem_file

