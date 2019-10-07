#adapted from https://github.com/catch22eu/pru_shm_c/blob/master/Makefile

pru_options 		= --silicon_version=2 --hardware_mac=on -i/usr/include/arm-linux-gnueabihf/include -i/usr/include/arm-linux-gnueabihf/lib
pru_compiler 		= /usr/bin/clpru
pru_hex_converter 	= /usr/bin/hexpru

all: exports host

host: Encoder_data.bin Encoder_code.bin IRIG_data.bin IRIG_code.bin Beaglebone_Encoder_DAQ.c 
	gcc -std=gnu11 Beaglebone_Encoder_DAQ.c -o Beaglebone_Encoder_DAQ -lprussdrv -lrt

#
# Here's the PRU code generation part
#

exports:
	@export PRU_SDK_DIR=/usr
	@export PRU_CGT_DIR=/usr/include/arm-linux-gnueabihf

IRIG.obj: IRIG_Detection.c
	$(pru_compiler) $(pru_options) --opt_level=off --output_file=IRIG.obj -c IRIG_Detection.c
	
IRIG.elf: IRIG.obj 
	$(pru_compiler) $(pru_options) -z IRIG.obj -llibc.a -m IRIG.map -o IRIG.elf AM335x_PRU.cmd --quiet 

IRIG_code.bin IRIG_data.bin: IRIG.cmd IRIG.elf
	$(pru_hex_converter) IRIG.cmd ./IRIG.elf --quiet

Encoder.obj: Encoder_Detection.c
	$(pru_compiler) $(pru_options) --opt_level=off --output_file=Encoder.obj -c Encoder_Detection.c
	
Encoder.elf: Encoder.obj 
	$(pru_compiler) $(pru_options) -z Encoder.obj -llibc.a -m Encoder.map -o Encoder.elf AM335x_PRU.cmd --quiet 

Encoder_code.bin Encoder_data.bin: Encoder.cmd Encoder.elf
	$(pru_hex_converter) Encoder.cmd ./Encoder.elf --quiet

clean:
	rm IRIG.obj
	rm IRIG.elf
	rm IRIG.map
	rm Encoder.obj
	rm Encoder.elf
	rm Encoder.map
