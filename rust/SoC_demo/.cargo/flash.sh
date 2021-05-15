#!/usr/bin/env bash

set -e

# Create bin file
riscv64-unknown-elf-objcopy $1 -O binary $1.bin

# Program Colorlight
ecpprog -o 0x00100000 -d i:0x0403:0x6014:1 $1.bin
