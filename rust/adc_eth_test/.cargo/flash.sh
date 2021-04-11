#!/usr/bin/env bash

set -e

# Create bin file
riscv64-unknown-elf-objcopy $1 -O binary $1.bin

# Program Colorlight
ecpprog -o 0x00100000 $1.bin
