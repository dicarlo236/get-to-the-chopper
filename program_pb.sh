#!/bin/bash

avr-gcc test.c -DF_CPU=16000000 -mmcu=atmega328pb -c -B /home/jared/328_pb/packs/Atmel.ATmega_DFP.1.2.203.atpack_FILES/gcc/dev/atmega328pb/
avr-gcc -o test.elf test.o -DF_CPU=16000000 -mmcu=atmega328pb -B /home/jared/328_pb/packs/Atmel.ATmega_DFP.1.2.203.atpack_FILES/gcc/dev/atmega328pb/
avr-objcopy -O ihex test.elf test.hex
avrdude -c usbasp -p atmega328pb -B 60 -U flash:w:test.hex 

