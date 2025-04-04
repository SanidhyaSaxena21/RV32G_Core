with open("entry_asm.bin", "rb") as bin_file, open("entry_asm.mem", "w") as mem_file:
    while (word := bin_file.read(4)):  # Read 4 bytes (32 bits) at a time
        mem_file.write(word.hex() + "\n")  # Write as hex string

