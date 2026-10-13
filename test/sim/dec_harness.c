/* Runs dec_test_fw.elf in simavr and reports its verdict. */
#include <stdio.h>
#include <string.h>
#include "sim_avr.h"
#include "sim_elf.h"
#include "avr_ioport.h"

#define _BV(b) (1 << (b))

int main(int argc, char** argv)
{
	elf_firmware_t fw;
	memset(&fw, 0, sizeof fw);
	if (argc < 2 || elf_read_firmware(argv[1], &fw)) { fprintf(stderr, "cannot read firmware\n"); return 2; }
	avr_t* avr = avr_make_mcu_by_name("atmega32u4");
	avr_init(avr);
	avr_load_firmware(avr, &fw);
	avr->frequency = 16000000;
	avr->log = 0;

	while (!(avr->data[0x2B] & _BV(1)))          /* PORTD.1: done */
	{
		int st = avr_run(avr);
		if (avr->data[0x2B] & _BV(1))
		  break;
		/* (simavr reports the final "for (;;)" with interrupts off as Done) */
		if (st == cpu_Done || st == cpu_Crashed || avr->cycle > 4000000000ULL)
		{ printf("decoder test: did not finish (state %d, cycle %llu, pc %04x, GPIOR2 %u)\nFAIL\n", st, (unsigned long long)avr->cycle, avr->pc, avr->data[0x4B]); return 1; }
	}
	if (avr->data[0x2B] & _BV(0))
	{
		printf("decoder test: asm and C decoders differ (config %u, round %u)\nFAIL\n",
		       avr->data[0x3E + 0x0C], avr->data[0x3E]);
		return 1;
	}
	printf("decoder test: asm == C over %u batches\nPASS\n", avr->data[0x4B] << 6);
	return 0;
}
