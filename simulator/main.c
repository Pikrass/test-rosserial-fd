#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "simavr/sim_avr.h"
#include "simavr/sim_elf.h"
#include "simavr/sim_irq.h"
#include "simavr/sim_io.h"
#include "simavr/sim_gdb.h"
#include "simavr/avr_ioport.h"
#include "simavr/parts/uart_pty.h"

void print_usage(char *argv0)
{
	fprintf(stderr, "Usage: %s [-g] path/to/program.elf\n", argv0);
}

int main(int argc, char **argv)
{
	int debug = 0;
	char *elf_path;

	if (argc < 2) {
		print_usage(argv[0]);
		return 1;
	}

	if (strcmp(argv[1], "-g") == 0)
		debug = 1;

	if (debug && argc < 3) {
		print_usage(argv[0]);
		return 1;
	}

	if (debug)
		elf_path = argv[2];
	else
		elf_path = argv[1];

	elf_firmware_t elf;
	elf_read_firmware(elf_path, &elf);
	elf.frequency = 16000000;

	avr_t *avr;
	avr = avr_make_mcu_by_name("atmega2560");
	avr_init(avr);
	avr_load_firmware(avr, &elf);

	if (debug) {
		avr->gdb_port = 1234;
		avr->state = cpu_Stopped;
		avr_gdb_init(avr);
	}

	uart_pty_t uart_pty;
	uart_pty_init(avr, &uart_pty);
	uart_pty_connect(&uart_pty, '0');

	int state = cpu_Running;
	while (state != cpu_Done && state != cpu_Crashed)
		state = avr_run(avr);
}
