#!/bin/bash
# Compile all RISC-V test cases and generate .mem files

RISCV_GCC=riscv64-unknown-elf-gcc
OBJCOPY=riscv64-unknown-elf-objcopy
OBJDUMP=riscv64-unknown-elf-objdump

# Check if at least one linker script and one assembly file are provided
if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <linker_file.ld> <assembly_file1.s> [assembly_file2.s ...]"
    exit 1
fi

# Extract linker script (first argument)
LINKER_SCRIPT=$1
shift # Remove first argument from list

# Remaining arguments are assembly files
START_ASM="$@"


TEST_DIR=./extra
MEM_DIR=./mem_files
mkdir -p $MEM_DIR

for file in $TEST_DIR/*.s; do
    test_name=$(basename $file .s)
    
    # Assemble and link
    $RISCV_GCC -march=rv32i -mabi=ilp32 -nostdlib -nodefaultlibs -nostartfiles -T "$LINKER_SCRIPT" -o $MEM_DIR/$test_name.elf "$START_ASM" $file
    
    # Convert to binary
    $OBJCOPY -O binary $MEM_DIR/$test_name.elf $MEM_DIR/$test_name.bin
   
    #OBJDUMP for Visulisation
    $OBJDUMP -D $MEM_DIR/$test_name.elf > $MEM_DIR/$test_name.objdump
    # Convert to memory format
    od -An -tx4 -w4 -v $MEM_DIR/$test_name.bin | awk '{print $1}' > $MEM_DIR/$test_name.mem

    # Extract the .data section separately (if it exists)
    DATA_BIN="$MEM_DIR/${test_name}_data.bin"
    DATA_MEM="$MEM_DIR/${test_name}_data.mem"

    $OBJCOPY --dump-section .data=$DATA_BIN $MEM_DIR/$test_name.elf 2>/dev/null

    if [ -s "$DATA_BIN" ]; then
        # Convert .data binary to memory format
        od -An -tx4 -w4 -v $DATA_BIN | awk '{print $1}' > $DATA_MEM
        echo "Extracted .data section to $DATA_MEM"
    else
        echo "No .data section found in $test_name"
        rm -f "$DATA_BIN"
    fi

    echo "Generated $test_name.mem"
done

