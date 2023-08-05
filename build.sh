make clean && make all && arm-none-eabi-size stm32-can-dpf.elf && [[ $1 == "f" ]] && st-flash write stm32-can-dpf.bin 0x08000000
