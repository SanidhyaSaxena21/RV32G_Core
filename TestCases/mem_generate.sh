#!/bin/bash

# Input file containing objdump output
DUMP_FILE="handler.dump"
MEM_FILE="handler.mem"

# Extract hexadecimal values and save to .mem file
grep '^[[:space:]]*c0' "$DUMP_FILE" | awk '{print $2}' > "$MEM_FILE"

echo "Extraction complete. Output saved to: $MEM_FILE"

