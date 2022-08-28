#define _POSIX_C_SOURCE 200809L
#define _GNU_SOURCE

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <getopt.h>
#include <poll.h>
#include <signal.h>
#include <fcntl.h>
#include <netdb.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <urjtag/urjtag.h>
#include <inttypes.h>

#define DBG_WB_ADDR		0x00
#define DBG_WB_DATA		0x01
#define DBG_WB_CTRL		0x02

#define DBG_CORE_CTRL		0x10
#define  DBG_CORE_CTRL_STOP		(1 << 0)
#define  DBG_CORE_CTRL_RESET		(1 << 1)
#define  DBG_CORE_CTRL_ICRESET		(1 << 2)
#define  DBG_CORE_CTRL_STEP		(1 << 3)
#define  DBG_CORE_CTRL_START		(1 << 4)

#define DBG_CORE_STAT		0x11
#define  DBG_CORE_STAT_STOPPING		(1 << 0)
#define  DBG_CORE_STAT_STOPPED		(1 << 1)
#define  DBG_CORE_STAT_TERM		(1 << 2)

#define DBG_CORE_NIA		0x12
#define DBG_CORE_MSR		0x13

#define DBG_CORE_GSPR_INDEX	0x14
#define DBG_CORE_GSPR_DATA	0x15

#define DBG_LOG_ADDR		0x16
#define DBG_LOG_DATA		0x17
#define DBG_LOG_TRIGGER		0x18

static bool debug = true;

static void check(int r, const char *failstr)
{
	if (r >= 0)
		return;
	fprintf(stderr, "Error %s\n", failstr);
	exit(1);
}

static urj_chain_t *jc;

static int common_jtag_init(const char *target, int freq)
{
	const char *sep;
	const char *cable;
	const int max_params = 20;
	char *params[max_params+1];
	int rc;

	if (!target)
		target = "probe";
	memset(params, 0x0, sizeof(params));
	sep = strchr(target, ' ');
	cable = strndup(target, sep - target);
	if (sep && *sep) {
		char *param_str = strdup(sep);
		char *s = param_str;
		for (int i = 0; *s; s++) {
			if (*s == ' ') {
				if (i >= max_params) {
					fprintf(stderr, "Too many jtag cable params\n");
					return -1;
				}
				*s = '\0';
				params[i] = s+1;
				i++;
			}
		}
	}
	if (debug)
		printf("Opening jtag backend cable '%s'\n", cable);

	jc = urj_tap_chain_alloc();
	if (!jc) {
		fprintf(stderr, "Failed to alloc JTAG\n");
		return -1;
	}
	jc->main_part = 0;

	rc = urj_tap_chain_connect(jc, cable, params);
	if (rc != URJ_STATUS_OK) {
		fprintf(stderr, "JTAG cable detect failed: %s\n", urj_error_describe());
		return -1;
	}

	if (freq) {
		urj_tap_cable_set_frequency(jc->cable, freq);
	}

	return 0;
}

int bscane2_init(const char *target, int freq)
{
	urj_part_t *p;
	uint32_t id;
	int rc;

	rc = common_jtag_init(target, freq);
	if (rc < 0) {
	    return rc;
	}

	/* XXX Hard wire part 0, that might need to change (use params and detect !) */
	rc = urj_tap_manual_add(jc, 6);
	if (rc < 0) {
		fprintf(stderr, "JTAG failed to add part !\n");
		return -1;
	}
	if (jc->parts == NULL || jc->parts->len == 0) {
		fprintf(stderr, "JTAG Something's wrong after adding part !\n");
		return -1;
	}
	urj_part_parts_set_instruction(jc->parts, "BYPASS");

	jc->active_part = 0;

	p = urj_tap_chain_active_part(jc);
	if (!p) {
		fprintf(stderr, "Failed to get active JTAG part\n");
		return -1;
	}
	rc = urj_part_data_register_define(p, "IDCODE_REG", 32);
	if (rc != URJ_STATUS_OK) {
		fprintf(stderr, "JTAG failed to add IDCODE_REG register !\n");
		return -1;
	}
	if (urj_part_instruction_define(p, "IDCODE", "001001", "IDCODE_REG") == NULL) {
		fprintf(stderr, "JTAG failed to add IDCODE instruction !\n");
		return -1;
	}
	rc = urj_part_data_register_define(p, "USER2_REG", 74);
	if (rc != URJ_STATUS_OK) {
		fprintf(stderr, "JTAG failed to add USER2_REG register !\n");
		return -1;
	}
	if (urj_part_instruction_define(p, "USER2", "000011", "USER2_REG") == NULL) {
		fprintf(stderr, "JTAG failed to add USER2 instruction !\n");
		return -1;
	}
	urj_part_set_instruction(p, "IDCODE");
	urj_tap_chain_shift_instructions(jc);
	urj_tap_chain_shift_data_registers(jc, 1);
        id = urj_tap_register_get_value(p->active_instruction->data_register->out);
	printf("Found device ID: 0x%08x\n", id);
	urj_part_set_instruction(p, "USER2");
	urj_tap_chain_shift_instructions(jc);

	return 0;
}

int jtag_command(uint8_t op, uint8_t addr, uint64_t *data)
{
	urj_part_t *p = urj_tap_chain_active_part(jc);
	urj_part_instruction_t *insn;
	urj_data_register_t *dr;
	uint64_t d = data ? *data : 0;
	int rc;

	if (!p)
		return -1;
	insn = p->active_instruction;
	if (!insn)
		return -1;
	dr = insn->data_register;
	if (!dr)
		return -1;
	rc = urj_tap_register_set_value_bit_range(dr->in, op, 1, 0);
	if (rc != URJ_STATUS_OK)
		return -1;
	rc = urj_tap_register_set_value_bit_range(dr->in, d, 65, 2);
	if (rc != URJ_STATUS_OK)
		return -1;
	rc = urj_tap_register_set_value_bit_range(dr->in, addr, 73, 66);
	if (rc != URJ_STATUS_OK)
		return -1;
	rc = urj_tap_chain_shift_data_registers(jc, 1);
	if (rc != URJ_STATUS_OK)
		return -1;
	rc = urj_tap_register_get_value_bit_range(dr->out, 1, 0);
	if (data)
		*data = urj_tap_register_get_value_bit_range(dr->out, 65, 2);
	return rc;
}

int dmi_read(uint8_t addr, uint64_t *data)
{
	int rc;

	rc = jtag_command(1, addr, data);
	if (rc < 0)
		return rc;
	for (;;) {
		rc = jtag_command(0, 0, data);
		if (rc < 0)
			return rc;
		if (rc == 0)
			return 0;
		if (rc != 3)
			fprintf(stderr, "Unknown status code %d !\n", rc);
	}
}

int dmi_write(uint8_t addr, uint64_t data)
{
	int rc;

	rc = jtag_command(2, addr, &data);
	if (rc < 0)
		return rc;
	for (;;) {
		rc = jtag_command(0, 0, NULL);
		if (rc < 0)
			return rc;
		if (rc == 0)
			return 0;
		if (rc != 3)
			fprintf(stderr, "Unknown status code %d !\n", rc);
	}
}

#define LOG_STOP	0x80000000ull

void log_start(void)
{
	check(dmi_write(DBG_LOG_ADDR, 0), "writing LOG_ADDR");
}

void log_stop(void)
{
	uint64_t lsize, laddr, waddr;

	check(dmi_write(DBG_LOG_ADDR, LOG_STOP), "writing LOG_ADDR");
	check(dmi_read(DBG_LOG_ADDR, &laddr), "reading LOG_ADDR");
	waddr = laddr >> 32;
	for (lsize = 1; lsize; lsize <<= 1)
		if ((waddr >> 1) < lsize)
			break;
	waddr &= ~lsize;
	printf("Log size = %" PRIu64 " entries, ", lsize);
	printf("write ptr = %" PRIx64 "\n", waddr);
}

void log_dump(const char *filename)
{
	FILE *f;
	uint64_t lsize, laddr, waddr;
	uint64_t orig_laddr;
	uint64_t i, ldata;

	f = fopen(filename, "w");
	if (f == NULL) {
		fprintf(stderr, "Failed to create '%s': %s\n", filename,
			strerror(errno));
		exit(1);
	}

	check(dmi_read(DBG_LOG_ADDR, &orig_laddr), "reading LOG_ADDR");
	if (!(orig_laddr & LOG_STOP))
		check(dmi_write(DBG_LOG_ADDR, LOG_STOP), "writing LOG_ADDR");

	waddr = orig_laddr >> 32;
	for (lsize = 1; lsize; lsize <<= 1)
		if ((waddr >> 1) < lsize)
			break;
	waddr &= ~lsize;
	printf("Log size = %" PRIu64 " entries\n", lsize);

	laddr = LOG_STOP | (waddr << 2);
	check(dmi_write(DBG_LOG_ADDR, laddr), "writing LOG_ADDR");

	for (i = 0; i < lsize * 4; ++i) {
		check(dmi_read(DBG_LOG_DATA, &ldata), "reading LOG_DATA");
		if (fwrite(&ldata, sizeof(ldata), 1, f) != 1) {
			fprintf(stderr, "Write error on %s\n", filename);
			exit(1);
		}
		if (!(i % 128)) {
			printf("%" PRIu64 "...\r", i * 8);
			fflush(stdout);
		}
	}
	fclose(f);
	printf("%" PRIu64 " done\n", lsize * 32);

	check(dmi_write(DBG_LOG_ADDR, orig_laddr), "writing LOG_ADDR");
}

void ltrig_show(void)
{
	uint64_t trig;

	check(dmi_read(DBG_LOG_TRIGGER, &trig), "reading LOG_TRIGGER");
	if (trig & 1)
		printf("log stop trigger at %" PRIx64, trig & ~3);
	else
		printf("log stop trigger disabled");
	printf(", %striggered\n", (trig & 2? "": "not "));
}

void ltrig_off(void)
{
	check(dmi_write(DBG_LOG_TRIGGER, 0), "writing LOG_TRIGGER");
}

void ltrig_set(uint64_t addr)
{
	check(dmi_write(DBG_LOG_TRIGGER, (addr & ~(uint64_t)2) | 1), "writing LOG_TRIGGER");
}

