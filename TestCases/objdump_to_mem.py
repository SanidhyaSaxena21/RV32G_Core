import re

# Input and output files
objdump_file = "entry_asm.objdump"
mem_file = "entry_asm.mem"

# Open objdump file and extract instructions
with open(objdump_file, "r") as infile, open(mem_file, "w") as outfile:
    for line in infile:
        match = re.search(r'\s([0-9a-f]{8})\s', line)  # Match 8-digit hex instruction
        if match:
            outfile.write(match.group(1) + "\n")  # Write instruction to .mem file

print(f"Memory file '{mem_file}' has been generated successfully!")

