#!/bin/bash

# Check if at least one linker script and one assembly file are provided
if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <linker_file.ld> <assembly_file1.s> [assembly_file2.s ...]"
    exit 1
fi

# Extract linker script (first argument)
LINKER_SCRIPT=$1
shift # Remove first argument from list

# Remaining arguments are assembly files
ASM_FILES="$@"

# Define output filenames
ELF_FILE="program.elf"
BIN_FILE="program.bin"
MEM_FILE="program.mem"
OBJDUMP_FILE="program.objdump"
TEXT_BIN="text_section.bin"
HANDLER_BIN="handler_section.bin"
HANDLER_ELF="handler_section.elf"
HANDLER_DUMP="handler_section.dump"
TEXT_MEM="instruction.mem"
HANDLER_MEM="handler.mem"

echo "Compiling assembly files into ELF..."
riscv64-unknown-elf-gcc -march=rv32im -mabi=ilp32 -nostartfiles -nodefaultlibs -nostdlib -T "$LINKER_SCRIPT" -o "$ELF_FILE" $ASM_FILES

# Check if ELF generation was successful
if [ $? -ne 0 ]; then
    echo "Error: ELF generation failed."
    exit 1
fi

echo "Generating objdump..."
riscv64-unknown-elf-objdump -D "$ELF_FILE" > "$OBJDUMP_FILE"
cat $OBJDUMP_FILE

echo "Extracting .text and .handler sections using objcopy..."
riscv64-unknown-elf-objcopy -j .text -O binary "$ELF_FILE" "$TEXT_BIN"
riscv64-unknown-elf-objcopy -j .handler -O elf32-littleriscv "$ELF_FILE" "$HANDLER_ELF"
riscv64-unknown-elf-objdump -D "$HANDLER_ELF" > "$HANDLER_DUMP"
cat "$HANDLER_DUMP"

#riscv64-unknown-elf-objcopy -O binary --only-section=.text "$ELF_FILE" "$TEXT_BIN"
#riscv64-unknown-elf-objcopy -O binary --only-section=.handler "$ELF_FILE" "$HANDLER_BIN"

echo "Converting sections to .mem format using hexdump..."
hexdump -v -e '/4 "%08x\n"' "$TEXT_BIN" > "$TEXT_MEM"
# Extract hexadecimal values and save to .mem file
grep '^[[:space:]]*c0' "$HANDLER_DUMP" | awk '{print $2}' > "$HANDLER_MEM"

# Check if files are created successfully
if [ -s "$TEXT_MEM" ]; then
    echo "Successfully extracted .text section to '$TEXT_MEM'"
else
    echo "Warning: '$TEXT_MEM' is empty! Check section header format."
fi

if [ -s "$HANDLER_MEM" ]; then
    echo "Successfully extracted .handler section to '$HANDLER_MEM'"
else
    echo "Warning: '$HANDLER_MEM' is empty! Check section header format."
fi

echo "Converting ELF to binary..."
riscv64-unknown-elf-objcopy -O binary "$ELF_FILE" "$BIN_FILE"

echo "Generating .mem file..."
hexdump -v -e '/4 "%08x\n"' "$BIN_FILE" > "$MEM_FILE"

echo "Generated .mem file:"
#cat "$MEM_FILE"

echo "Process completed successfully!"

