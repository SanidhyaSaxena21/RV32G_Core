# Define the input CSV file
set csv_file "Instruction_memory.csv"

# Open the CSV file for reading
set file_id [open $csv_file r]

# Create or open the Instruction_Memory.mif file
set mif_file [open "Instruction_Memory.mif" w]

# Write the header for the MIF file
# 
puts $mif_file "WIDTH=16;"          
puts $mif_file "DEPTH=256;"       
puts $mif_file ""
puts $mif_file "ADDRESS_RADIX=HEX;"
puts $mif_file "DATA_RADIX=HEX;"
puts $mif_file ""
puts $mif_file "CONTENT"
puts $mif_file "BEGIN"

# Read the CSV file line by line
while {[gets $file_id line] != -1} {
    # Split the line into memory location and value
    set values [split $line ","] 
    set memory_location [lindex $values 0]
    set memory_value [lindex $values 1]

    # Write the memory location and value into the MIF file
    puts $mif_file "$memory_location : $memory_value;"
}

# Close the files
close $file_id
puts $mif_file "END;"
close $mif_file
