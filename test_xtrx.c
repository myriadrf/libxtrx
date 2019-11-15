/*
 * general xtrx test source file
 * Copyright (c) 2017 Sergey Kostanbaev <sergey.kostanbaev@fairwaves.co>
 * For more information, please visit: http://xtrx.io
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */
#include <xtrxll_port.h>

#define _GNU_SOURCE
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "xtrx_api.h"

#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>

#include <string.h>
#include <inttypes.h>

#include <assert.h>
#include <xtrxll_log.h>
#include <stdbool.h>

static uint64_t grtime(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC_RAW, &ts);

	return (uint64_t)ts.tv_sec * (uint64_t)1000000000 + (uint64_t)ts.tv_nsec;
}

double parse_val(const char* a)
{
	double z = atof(a);
	return z;
}

static int rx_rep_gtime = 0;
static uint32_t s_tx_skip = 8192;
static unsigned s_logp = 0;

enum {
	BUF_TO_FLUSH = 32,
	MAX_DEVS     = 8,
};

struct rx_flush_data {
	void *buffer_ptr[MAX_DEVS*2][BUF_TO_FLUSH];
	FILE *out_files[MAX_DEVS*2];

	unsigned num_chans;
	unsigned buf_ptr;
	unsigned buf_max;

	unsigned wr_sz;

	unsigned put_cnt;
	unsigned get_cnt;

	bool stop;
	bool fiop;

	sem_t sem_read_rx;
	sem_t sem_read_wr;
};

char* alloc_flush_data_bufs(struct rx_flush_data *pd,
							unsigned buffer_count,
							unsigned num_chans,
							unsigned slice_sz,
							const char* tag,
							const char* basename,
							const char* fattr,
							bool rd)
{
	pd->buf_max = buffer_count;
	pd->buf_ptr = 0;
	pd->num_chans = num_chans;
	pd->stop = false;
	pd->fiop = basename != NULL;
	pd->put_cnt = 0;
	pd->get_cnt = 0;
	pd->wr_sz = slice_sz;
	sem_init(&pd->sem_read_rx, 0, (rd) ? (buffer_count) : 0);
	sem_init(&pd->sem_read_wr, 0, (!rd) ? (buffer_count) : 0);

	size_t rx_bufsize  = num_chans * pd->buf_max * pd->wr_sz;
	size_t rx_bufslice = pd->wr_sz;

	char* out_buffs = (char*)malloc(rx_bufsize);
	if (out_buffs == NULL) {
		fprintf(stderr, "Unable to create %s buffers (size=%.3fMB)\n", tag,
				(float)rx_bufsize / 1024 / 1024);
		return NULL;
	}

	char* bptr = out_buffs;
	for (unsigned p = 0; p < num_chans; p++) {
		char filename_buf[256];
		const char* filename = (num_chans == 1) ? basename : filename_buf;
		if (num_chans > 1) {
			snprintf(filename_buf, sizeof(filename_buf), "%s_%d",
					 basename, p);
		}
		if (basename) {
			pd->out_files[p] = fopen(filename, fattr);
			if (pd->out_files[p] == NULL) {
				fprintf(stderr, "Unable to open file: %s! error: %d\n",
						filename, errno);
				exit(EXIT_FAILURE);
			}
		}

		for (unsigned b = 0; b < pd->buf_max; b++) {
			pd->buffer_ptr[p][b] = bptr;
			bptr += rx_bufslice;
		}
	}

	return out_buffs;
}

void* thread_rx_to_file(void* obj)
{
	struct rx_flush_data *rxd = (struct rx_flush_data *)obj;
	int res;

	fprintf(stderr, "RX_TO_FILE: thread start\n");

	for (;;) {
		res = sem_wait(&rxd->sem_read_wr);
		if (res) {
			fprintf(stderr, "RX_TO_FILE: sem wait error!\n");
			return 0;
		}
		if ((rxd->put_cnt == rxd->get_cnt) && rxd->stop) {
			fprintf(stderr, "RX_TO_FILE: thread exit: %d:%d\n", rxd->put_cnt, rxd->get_cnt);
			return 0;
		}

		for (unsigned i = 0; i <rxd->num_chans; i++) {
			size_t ret = fwrite(rxd->buffer_ptr[i][rxd->buf_ptr], rxd->wr_sz, 1, rxd->out_files[i]);
			if (ret != 1) {
				fprintf(stderr, "RX_TO_FILE: write error %u != expected %u\n", (unsigned)ret, rxd->wr_sz);
			}
		}
		sem_post(&rxd->sem_read_rx);
		rxd->buf_ptr = (rxd->buf_ptr + 1) % rxd->buf_max;

		//fprintf(stderr, "RX_TO_FILE: buffer %u written\n", rxd->get_cnt);
		rxd->get_cnt++;
	}
}

// Read file and flush into buffers
void* thread_file_to_tx(void* obj)
{
	struct rx_flush_data *txd = (struct rx_flush_data *)obj;
	int res;

	fprintf(stderr, "FILE_TO_TX: thread start\n");

	for (;;) {
		res = sem_wait(&txd->sem_read_rx);
		if (res) {
			fprintf(stderr, "FILE_TO_TX: sem wait error!\n");
			return 0;
		}
		if ((txd->put_cnt == txd->get_cnt) && txd->stop) {
			fprintf(stderr, "FILE_TO_TX: thread exit: %d:%d\n", txd->put_cnt, txd->get_cnt);
			return 0;
		}

		for (unsigned i = 0; i <txd->num_chans; i++) {
			size_t ret = fread(txd->buffer_ptr[i][txd->buf_ptr], txd->wr_sz, 1, txd->out_files[i]);
			if (ret != 1) {
				fprintf(stderr, "FILE_TO_TX: read error %u != expected %u\n", (unsigned)ret, txd->wr_sz);

				// Restart
				fseek(txd->out_files[i], 0, SEEK_SET);
				ret = fread(txd->buffer_ptr[i][txd->buf_ptr], txd->wr_sz, 1, txd->out_files[i]);
				if (ret != 1) {
					txd->stop = true;
					return 0;
				}
			}
		}
		sem_post(&txd->sem_read_wr);
		txd->buf_ptr = (txd->buf_ptr + 1) % txd->buf_max;

		fprintf(stderr, "FILE_TO_TX: buffer %u written\n", txd->get_cnt);
		txd->get_cnt++;
	}
}

static const char* generate_getopt_string(const struct option *lo)
{
	static char mstr[500];
	char options[256] = {0};
	char* str = mstr;

	for (;lo->name; ++lo) {
		if (options[lo->val]++) {
			fprintf(stderr, "ambigous option '%c', second usage '%s'\n",
					lo->val, lo->name);
			exit(EXIT_FAILURE);
		}
		*str++ = lo->val;
		if (lo->has_arg)
			*str++ = ':';
	}
	*str = 0;

	assert(str - mstr < sizeof(mstr));
	return mstr;
}

static void generate_help(const struct option *lo)
{
	for (;lo->name; ++lo) {
		fprintf(stderr, "  --%s|-%c\t%s\n",
				lo->name,
				lo->val,
				(lo->has_arg == 2) ? "[value]" :
									 (lo->has_arg == 1) ? "value" : "");
	}
}

static void fill_hmft(int arg, unsigned *shs, xtrx_host_format_t *hf)
{
	switch (arg) {
	case 8: *shs = sizeof(float) * 2;   *hf = XTRX_IQ_FLOAT32; break;
	case 4: *shs = sizeof(int16_t) * 2; *hf = XTRX_IQ_INT16; break;
	case 2: *shs = sizeof(int8_t) * 2;  *hf = XTRX_IQ_INT8; break;
	default: fprintf(stderr, "Unsupported host type, should be 8, 4 or 2\n"); exit(EXIT_FAILURE);
	}
}

struct stream_data {
	struct xtrx_dev *dev;

	uint64_t cycles;
	uint64_t samples_per_cyc;
	double ramplerate;
	unsigned slice_sz;

	bool mux_demux; // Data multiplexing/demultiplexing
	bool siso;      // Device operatig mode
	bool flush_data;
	unsigned sample_host_size;

	unsigned out_overruns;

	size_t out_tm_diff;

	uint64_t out_samples_per_dev;
	uint64_t out_time;
};

int stream_rx(struct stream_data* sdata,
			  struct rx_flush_data* rxd)
{
	int res = 0;
	void* stream_buffers[2 * MAX_DEVS];
	unsigned buf_cnt = rxd->num_chans;
	uint64_t zero_inserted = 0;
	int overruns = 0;
	uint64_t rx_processed = 0;
	uint64_t sp = grtime();

	uint64_t st = sp;
	uint64_t abpkt = sp;
	unsigned binx = 0;
	unsigned dev_count = rxd->num_chans / (sdata->mux_demux ? 2 : 1);

	fprintf(stderr, "RX CYCLES=%" PRIu64 " SAMPLES=%" PRIu64 " SLICE=%u (PARTS=%" PRIu64 ")\n",
			sdata->cycles,
			sdata->samples_per_cyc, sdata->slice_sz,
			sdata->samples_per_cyc / sdata->slice_sz);


	for (uint64_t p = 0; p < sdata->cycles; p++) {
		// Get new buffer to writing to
		if (sdata->flush_data) {
			res = sem_trywait(&rxd->sem_read_rx);
			if (res) {
				fprintf(stderr, "RX rate is too much; RX_TO_FILE thread is lagging behind\n");
				goto falied_stop_rx;
			}
		}

		for (unsigned bc = 0; bc < buf_cnt; bc++) {
			stream_buffers[bc] = rxd->buffer_ptr[bc][binx];
		}

		for (uint64_t h = 0; h < sdata->samples_per_cyc / sdata->slice_sz; h++) {
			xtrx_recv_ex_info_t ri;
			size_t rem = sdata->samples_per_cyc - h * sdata->slice_sz;
			if (rem > sdata->slice_sz)
				rem = sdata->slice_sz;

			ri.samples = rem / ((sdata->mux_demux) ? 2 : 1);
			ri.buffer_count = buf_cnt;
			ri.buffers = stream_buffers;
			ri.flags = 0;

			if (rx_rep_gtime) {
				ri.flags |= RCVEX_REPORT_GTIME;
			}

			uint64_t sa = grtime();
			uint64_t da = sa - sp;
			res = xtrx_recv_sync_ex(sdata->dev, &ri);
			sp = grtime();
			uint64_t sb = sp - sa;

			rx_processed += ri.out_samples * ((sdata->mux_demux) ? 2 : 1);

			abpkt += 1e9 * ri.samples * ri.buffer_count / dev_count / sdata->ramplerate / (sdata->siso ? 1 : 2);
			if (s_logp == 0 || p % s_logp == 0)
				fprintf(stderr, "PROCESSED RX SLICE %" PRIu64 " /%" PRIu64 ":"
								" res %d TS:%8" PRIu64 " %c%c  %6" PRId64 " us"
								" DELTA %6" PRId64 " us LATE %6" PRId64 " us"
								" %d samples\n",
						p, h, res, ri.out_first_sample,
						(ri.out_events & RCVEX_EVENT_OVERFLOW)    ? 'O' : ' ',
						(ri.out_events & RCVEX_EVENT_FILLED_ZERO) ? 'Z' : ' ',
						sb / 1000, da / 1000, (int64_t)(sp - abpkt) / 1000, ri.out_samples);
			if (res) {
				fprintf(stderr, "Failed xtrx_recv_sync: %d\n", res);
				goto falied_stop_rx;
			}

			if (ri.out_events & RCVEX_EVENT_OVERFLOW) {
				overruns++;
				zero_inserted += (ri.out_resumed_at - ri.out_overrun_at);
			}
			for (unsigned bc = 0; bc < buf_cnt; bc++) {
				stream_buffers[bc] += ri.samples * sdata->sample_host_size;
			}
		}

		if (sdata->flush_data) {
			binx = (binx + 1) % rxd->buf_max;
			rxd->put_cnt++;

			res = sem_post(&rxd->sem_read_wr);
			if (res) {
				fprintf(stderr, "RX unable to post buffers!\n");
				goto falied_stop_rx;
			}
		}
	}
falied_stop_rx:
	sdata->out_overruns = overruns;
	sdata->out_samples_per_dev = rx_processed;
	sdata->out_tm_diff = grtime() - st;
	return res;
}

uint64_t s_tx_start_ts;
bool s_tx_nodiscard;

int stream_tx(struct stream_data* sdata,
			  struct rx_flush_data* rxd)
{
	int res = 0;
	const void* stream_buffers[2 * MAX_DEVS];
	unsigned buf_cnt = rxd->num_chans;
	int underruns = 0;
	uint64_t sp = grtime();
	uint64_t tx_processed = 0;
	uint64_t st = sp;
	uint64_t abpkt = sp;
	unsigned binx = 0;
	unsigned dev_count = rxd->num_chans / (sdata->mux_demux ? 2 : 1);
	uint64_t tx_sent_samples = s_tx_start_ts;

	fprintf(stderr, "TX CYCLES=%" PRIu64 " SAMPLES=%" PRIu64 " SLICE=%u (PARTS=%" PRIu64 ")\n",
			sdata->cycles,
			sdata->samples_per_cyc, sdata->slice_sz,
			sdata->samples_per_cyc / sdata->slice_sz);

	for (uint64_t p = 0; p < sdata->cycles; p++) {
		// Get new buffer to writing to
		if (sdata->flush_data) {
			res = sem_trywait(&rxd->sem_read_wr);
			if (res) {
				fprintf(stderr, "TX rate is too much; FILE_TO_TX thread is lagging behind\n");
				goto falied_stop_tx;
			}
		}

		for (unsigned bc = 0; bc < buf_cnt; bc++) {
			stream_buffers[bc] = rxd->buffer_ptr[bc][binx];
		}

		for (uint64_t h = 0; h < sdata->samples_per_cyc / sdata->slice_sz; h++) {
			size_t rem = sdata->samples_per_cyc - h * sdata->slice_sz;
			if (rem > sdata->slice_sz)
				rem = sdata->slice_sz;

			xtrx_send_ex_info_t nfo;
			nfo.samples = rem / ((sdata->mux_demux) ? 2 : 1);
			nfo.flags = XTRX_TX_DONT_BUFFER;
			if (s_tx_nodiscard)
				nfo.flags |= XTRX_TX_NO_DISCARD;
			nfo.ts = tx_sent_samples;
			nfo.buffers = (const void* const*)stream_buffers;
			nfo.buffer_count = buf_cnt;
			nfo.timeout = 0;
			nfo.out_txlatets = 0;

			uint64_t sa = grtime();
			uint64_t da = sa - sp;
			res = xtrx_send_sync_ex(sdata->dev, &nfo);
			sp = grtime();
			uint64_t sb = sp - sa;

			abpkt += 1e9 * nfo.samples * nfo.buffer_count / dev_count / sdata->ramplerate / (sdata->siso ? 1 : 2);
			if (s_logp == 0 || p % s_logp == 0) {
				fprintf(stderr, "PROCESSED TX SLICE %" PRIu64 "/%" PRIu64 ":"
								" res %d TS:%8" PRIu64 " %c%c  %6" PRId64 " us"
								" DELTA %6" PRId64 " us LATE %6" PRId64 " us  %d x %d samples (%d)\n",
								p, h, res, nfo.out_txlatets,
								(nfo.out_flags & XTRX_TX_DISCARDED_TO)    ? 'D' : ' ',
								' ',
								sb / 1000, da / 1000, (int64_t)(sp - abpkt) / 1000, nfo.out_samples, nfo.buffer_count, nfo.samples);
			}
			if (res) {
				fprintf(stderr, "Failed xtrx_recv_sync: %d\n", res);
				goto falied_stop_tx;
			}

			for (unsigned bc = 0; bc < buf_cnt; bc++) {
				stream_buffers[bc] += nfo.samples * sdata->sample_host_size;
			}
			if (nfo.out_flags) {
				underruns++;
			}
			tx_processed += nfo.out_samples * ((sdata->mux_demux) ? 2 : 1);
			tx_sent_samples += nfo.samples * (nfo.buffer_count / dev_count) / (sdata->siso ? 1 : 2);
		}

		if (sdata->flush_data) {
			binx = (binx + 1) % rxd->buf_max;
			rxd->put_cnt++;

			res = sem_post(&rxd->sem_read_rx);
			if (res) {
				fprintf(stderr, "TX unable to post buffers!\n");
				goto falied_stop_tx;
			}
		}
	}

falied_stop_tx:
	sdata->out_overruns = underruns;
	sdata->out_samples_per_dev = tx_processed;
	sdata->out_tm_diff = grtime() - st;
	return res;
}

void parse_rxgain(const char* fmt, int *lna, int *pga, int *tia)
{
	int res;
	res = sscanf(fmt, "%d:%d:%d", lna, pga, tia);
	if (res == 3)
		return;

	*tia = 9;
	res = sscanf(fmt, "%d:%d", lna, pga);
	if (res == 2)
		return;

	*pga = 0;
	*lna = atoi(fmt);
}

int main(int argc, char** argv)
{
	struct xtrx_dev *dev;
	int opt;
	uint64_t rx_tm = 0;
	uint64_t tx_tm = 0;
	const char* device = NULL;

	int multidev = 0;
	double rxsamplerate = 4.0e6, actual_rxsample_rate = 0;
	double txsamplerate = 4.0e6, actual_txsample_rate = 0;
	double rxfreq = 900e6, rxactualfreq = 0;
	double txfreq = 450e6, txactualfreq = 0;
	double rxbandwidth = 2e6, actual_rxbandwidth = 0;
	double txbandwidth = 2e6, actual_txbandwidth = 0;
	double actuallnagain = 0;
	xtrx_channel_t ch = XTRX_CH_ALL;
	xtrx_wire_format_t rx_wire_fmt = XTRX_WF_16;
	xtrx_host_format_t rx_host_fmt = XTRX_IQ_INT16;
	xtrx_wire_format_t tx_wire_fmt = XTRX_WF_16;
	xtrx_host_format_t tx_host_fmt = XTRX_IQ_INT16;
	uint64_t samples = 16384;
	unsigned rx_slice = samples;
	unsigned tx_slice = samples;
	unsigned rx_sample_host_size = sizeof(float) * 2;
	unsigned tx_sample_host_size = sizeof(float) * 2;

	int loglevel = 2;
	int mimomode = 0;
	int tx_repeat_mode = 0;
	int rxlfsr = 0;
	int loopback = 0;
	int dmatx = 0;
	int dmarx = 0;
	int tx_siso = 0;
	int rx_siso = 0;
	int tx_swap_ab = 0;
	int rx_swap_ab = 0;
	int tx_swap_iq = 0;
	int rx_swap_iq = 0;
	int rx_tst_a = 0;
	int rx_tst_b = 0;
	int tx_tst_a = 0;
	int tx_tst_b = 0;
	int tx_packet_size = 0;
	int rx_packet_size = 0;
	int vio = 0;
	int tx_nodiscard = 0;

	unsigned samples_flag = 0;
	unsigned refclk = 0;
	unsigned cycles = 1;
	unsigned rx_skip = 8192;
	unsigned tx_skip = 8192;
	unsigned logp = 0;
	double master_in = 0;
	int extclk = 0;
	const char* out_name = NULL;
	const char* in_name = NULL;
	unsigned bufs_cnt = 16;
	int txgain = 0;
	int rxgain_lna = 15;
	int rxgain_pga = 0;
	int rxgain_tia = 9;
	int gtime = 0;
	int gmode = 0;

	struct option long_options[] = {
	{"cycles",  required_argument, 0,   'C' },
	{"refclk",  required_argument, 0,   'c' },
	{"srflags", required_argument, 0,   'p' },
	{"loopback",no_argument,       0,   'P' },
	{"txrepeat",no_argument,       0,   'R' },
	{"rxlfsr",  no_argument,       0,   'r' },
	{"mimomode",no_argument,       0,   'M' },
	{"extclk",  no_argument,       0,   'm' },
	{"loglevel",required_argument, 0,   'l' },
	{"logp",    required_argument, 0,   'L' },
	{"multdevs",required_argument, 0,   'd' },
	{"device",  required_argument, 0,   'D' },
	{"samples", required_argument,  0,   'N' },
	{"rx_bufs", required_argument,  0,   'n' },
	{"out",     required_argument,  0,   'o' },
	{"in",     required_argument,  0,   'O' },
	{"master",  required_argument,  0,   'y' },
	{"vio",     required_argument,  0,   'Y' },
	// symmetric for RX & TX
	{"txpkt",   required_argument, 0,   'Z' },
	{"rxpkt",   required_argument, 0,   'z' },
	{"txslice", required_argument, 0,   'E' },
	{"rxslice", required_argument, 0,   'e' },
	{"skiptx",  required_argument, 0,   'K' },
	{"skiprx",  required_argument, 0,   'k' },
	{"dmatx",   no_argument,       0,   'T' },
	{"dmarx",   no_argument,       0,   't' },
	{"rxrate",  required_argument, 0, 's' },
	{"txrate",  required_argument, 0, 'S' },
	{"rxfreq",  required_argument,  0,   'f' },
	{"txfreq",  required_argument,  0,   'F' },
	{"rxbandwidth",required_argument, 0,   'b' },
	{"txbandwidth",required_argument, 0,   'B' },
	{"rxwfmt",  required_argument,  0,   'x' },
	{"txwfmt",  required_argument,  0,   'X' },
	{"rxhfmt",  required_argument,  0,   'h' },
	{"txhfmt",  required_argument,  0,   'H' },
	{"rx_siso", no_argument,        0,   'i' },
	{"tx_siso", no_argument,        0,   'I' },
	{"rxswapab",no_argument,        0,   'w' },
	{"txswapab",no_argument,        0,   'W' },
	{"rxswapiq",no_argument,        0,   'q' },
	{"txswapiq",no_argument,        0,   'Q' },
	{"rxtsta",  no_argument,        0,   'a' },
	{"txtsta",  no_argument,        0,   'A' },
	{"txnodis", no_argument,        0,   'U' },
	{"samples", required_argument,  0,   'u' },
	{"rxgain",  required_argument,  0,   'g' },
	{"txgain",  required_argument,  0,   'G' },
	{"gtime",   required_argument,  0,   'j' },
	{"gmode",   required_argument,  0,   'J' },
	{0,         0,                  0,    0  }
};

	fill_hmft(4, &rx_sample_host_size, &rx_host_fmt);
	fill_hmft(4, &tx_sample_host_size, &tx_host_fmt);

	int option_index = 0;
	while ((opt = getopt_long(argc, argv,
							  generate_getopt_string(long_options),
							  long_options,
							  &option_index)) != -1) {
		switch (opt) {
		case 0:
			printf("option %s", long_options[option_index].name);
			if (optarg) {
				*long_options[option_index].flag = atoi(optarg);
			}
			if (optarg) {
				printf(" with arg %s", optarg);
			}
			printf("\n");
			break;
		case 'C':	cycles = atoi(optarg);		break;
		case 'c':	refclk = atoi(optarg);		break;

		case 'p': samples_flag = atoi(optarg);	break;
		case 'P': loopback = 1;					break;
		case 'R': tx_repeat_mode = 1;			break;
		case 'r': rxlfsr = 1;					break;
		case 'M': mimomode = 1;					break;
		case 'm': extclk = 1;					break;
		case 'l': loglevel = (atoi(optarg));	break;
		case 'L': logp = (atoi(optarg));		break;
		case 'd': multidev = 1; device = optarg;break;
		case 'D': device = optarg;				break;
		case 'x':
			switch (atoi(optarg)) {
			case 8: rx_wire_fmt = XTRX_WF_8; break;
			case 12: rx_wire_fmt = XTRX_WF_12; break;
			default: rx_wire_fmt = XTRX_WF_16; break;
			}
			break;
		case 'N': samples = atoll(optarg);	break;
		case 'n': bufs_cnt = atoi(optarg); break;

		case 'o': out_name = optarg;			break;
		case 'O': in_name = optarg;				break;

		case 'Z': tx_packet_size = atoi(optarg); break;
		case 'z': rx_packet_size = atoi(optarg); break;

		case 'K': tx_skip = atoi(optarg);		break;
		case 'k': rx_skip = atoi(optarg);		break;

		case 'E': tx_slice = atoi(optarg);	break;
		case 'e': rx_slice = atoi(optarg);		break;

			// symmetric flags for TX & RX
		case 'h':
			fill_hmft(atoi(optarg), &rx_sample_host_size, &rx_host_fmt);
			break;
		case 'H':
			fill_hmft(atoi(optarg), &tx_sample_host_size, &tx_host_fmt);
			break;

		case 'i':	rx_siso = 1;			break;
		case 'I':	tx_siso = 1;			break;

		case 'w':	rx_swap_ab = 1;			break;
		case 'W':	tx_swap_ab = 1;			break;

		case 'q':	rx_swap_iq = 1;			break;
		case 'Q':	tx_swap_iq = 1;			break;

		case 'a':	rx_tst_a = 1;			break;
		case 'A':	tx_tst_a = 1;			break;

		case 's':	rxsamplerate = parse_val(optarg);	break;
		case 'S':	txsamplerate = parse_val(optarg);	break;

		case 'f':	rxfreq = parse_val(optarg);	break;
		case 'F':	txfreq = parse_val(optarg);	break;

		case 'b':	rxbandwidth = parse_val(optarg);	break;
		case 'B':	txbandwidth = parse_val(optarg);	break;

		case 't':	dmarx = 1;					break;
		case 'T':	dmatx = 1;					break;

		case 'g': parse_rxgain(optarg, &rxgain_lna, &rxgain_pga, &rxgain_tia); break;
		case 'G': txgain = atoi(optarg);	break;

		case 'y': master_in = atof(optarg); break;
		case 'Y': vio = atoi(optarg); break;

		case 'u': samples = atoll(optarg); break;
		case 'U': tx_nodiscard = 1; break;

		case 'j':	gtime = parse_val(optarg);	break;
		case 'J':	gmode = parse_val(optarg);	break;
		default: /* '?' */
			fprintf(stderr, "Usage: %s <options>\n", argv[0]);
			generate_help(long_options);
			exit(EXIT_FAILURE);
		}
	}

	if (samples < rx_slice) {
		rx_slice = samples;
	}
	if (samples < tx_slice) {
		tx_slice = samples;
	}

	s_tx_skip = tx_skip;
	s_logp = logp;

	if (dmarx == 0 && dmatx == 0 && tx_repeat_mode == 0) {
		fprintf(stderr, "Usage: DMATX and/or DMARX must be enabled (or TX REPEAT is set)!\n");
		//exit(EXIT_FAILURE);
		multidev = 1;
		device = NULL;
	}
	if (loopback) {
		txsamplerate = rxsamplerate;
	}

	if (dmarx && rxsamplerate == 0) {
		fprintf(stderr, "Usage: DMARX requested but RXRATE == 0!\n");
		exit(EXIT_FAILURE);
	}
	if (dmatx && txsamplerate == 0) {
		fprintf(stderr, "Usage: DMATX requested but TXRATE == 0!\n");
		exit(EXIT_FAILURE);
	}

	if (!dmarx) {
		rxsamplerate = 0;
	}
	if (!dmatx && !tx_repeat_mode) {
		txsamplerate = 0;
	}

	xtrx_log_setlevel(loglevel, NULL);
	if (bufs_cnt > BUF_TO_FLUSH)
		bufs_cnt = BUF_TO_FLUSH;
	else if (bufs_cnt <= 0)
		bufs_cnt = 1;

	const bool rx_flush_data = (out_name != NULL);
	const unsigned rx_buffer_count = (rx_flush_data) ? bufs_cnt : 1;
	const bool tx_flush_data = (in_name != NULL);
	const unsigned tx_buffer_count = (tx_flush_data) ? bufs_cnt : 1;


	//
	// Open XTRX Device
	//
	unsigned dev_count;
	int res;
	if (multidev) {
		if (device == NULL || strlen(device) < 1) {
			// Do discovery
			xtrx_device_info_t di[MAX_DEVS];
			int cnt = xtrx_discovery(di, MAX_DEVS);
			if (cnt < 0) {
				fprintf(stderr, "Failed xtrx_discovery: %d\n", cnt);
				return -1;
			}
			for (int i = 0; i < cnt; i++) {
				printf("[%02d] %-32s %-16s %-16s %-16s\n", i,
					   di[i].uniqname, di[i].devid, di[i].proto, di[i].speed);
			}

			return 0;
		}

		res = xtrx_open_string(device, &dev);
		if (res < 0) {
			fprintf(stderr, "Failed xtrx_open: %d\n", res);
			goto falied_open;
		}

		dev_count = res;
	} else {
		res = xtrx_open(device, loglevel | XTRX_O_RESET, &dev);
		if (res) {
			fprintf(stderr, "Failed xtrx_open: %d\n", res);
			goto falied_open;
		}

		dev_count = 1;
	}

	//
	// Initialize RX file output stream
	//
	pthread_t rx_write_thread;
	struct rx_flush_data rxd;
	char* out_buffs = NULL;
	if (dmarx) {
		out_buffs = alloc_flush_data_bufs(&rxd,
										  rx_buffer_count,
										  dev_count * (mimomode ? 2 : 1),
										  rx_sample_host_size * samples / (mimomode ? 2 : 1),
										  "RX",
										  out_name,
										  "wb",
										  true);
		if (out_buffs == NULL) {
			goto failed_rx;
		}
		if (rx_flush_data) {
			res = pthread_create(&rx_write_thread, NULL, thread_rx_to_file, &rxd);
			if (res) {
				goto failed_rx;
			}
		}
	}

	//
	// Initialize TX file input stream
	//
	pthread_t tx_read_thread;
	struct rx_flush_data txd;
	char* in_buffs = NULL;
	if (dmatx) {
		in_buffs = alloc_flush_data_bufs(&txd,
										 tx_buffer_count,
										 dev_count * (mimomode ? 2 : 1),
										 tx_sample_host_size * samples / (mimomode ? 2 : 1),
										 "TX",
										 in_name,
										 "rb",
										 false);

		if (in_buffs == NULL) {
			goto failed_tx;
		}
		if (tx_flush_data) {
			res = pthread_create(&tx_read_thread, NULL, thread_file_to_tx, &txd);
			if (res) {
				goto failed_tx;
			}
		}
	}

	//
	// Set XTRX parameters
	//
	if (refclk || extclk) {
		res = xtrx_set_ref_clk(dev, refclk, (extclk) ? XTRX_CLKSRC_EXT : XTRX_CLKSRC_INT);
		if (res) {
			fprintf(stderr, "Failed xtrx_set_ref_clk: %d\n", res);
			goto falied_samplerate;
		}
	}

	double master;
	res = xtrx_set_samplerate(dev, master_in, rxsamplerate, txsamplerate, samples_flag,
							  &master, &actual_rxsample_rate, &actual_txsample_rate);
	if (res) {
		fprintf(stderr, "Failed xtrx_set_samplerate: %d\n", res);
		//goto falied_samplerate;
	}
	fprintf(stderr, "Master: %.3f MHz; RX rate: %.3f MHz; TX rate: %.3f MHz\n",
			master / 1e6,
			actual_rxsample_rate / 1e6,
			actual_txsample_rate / 1e6);

	if (vio) {
		xtrx_val_set(dev, XTRX_TRX, XTRX_CH_ALL, XTRX_LMS7_VIO, vio);
	}

	if (dmarx) {
		xtrx_set_antenna(dev, XTRX_RX_AUTO);

		res = xtrx_tune(dev, XTRX_TUNE_RX_FDD, rxfreq, &rxactualfreq);
		if (res) {
			fprintf(stderr, "Failed xtrx_tune: %d\n", res);
			goto falied_tune;
		}
		fprintf(stderr, "RX tunned: %f\n", rxactualfreq);

#if 0
		if (rxfreq < 900e6) {
			xtrx_set_antenna(dev, XTRX_RX_L);
		} else if (rxfreq > 2300e6) {
			xtrx_set_antenna(dev, XTRX_RX_H);
		} else {
			xtrx_set_antenna(dev, XTRX_RX_W);
		}
#endif

		res = xtrx_tune_rx_bandwidth(dev, ch, rxbandwidth, &actual_rxbandwidth);
		if (res) {
			fprintf(stderr, "Failed xtrx_tune_rx_bandwidth: %d\n", res);
			//goto falied_tune;
		}
		fprintf(stderr, "RX bandwidth: %f\n", actual_rxbandwidth);


		res = xtrx_set_gain(dev, ch, XTRX_RX_LNA_GAIN, rxgain_lna, &actuallnagain);
		if (res) {
			fprintf(stderr, "Failed xtrx_set_gain(LNA): %d\n", res);
			goto falied_tune;
		}
		fprintf(stderr, "RX LNA gain: %f\n", actuallnagain);

		res = xtrx_set_gain(dev, ch, XTRX_RX_PGA_GAIN, rxgain_pga, &actuallnagain);
		if (res) {
			fprintf(stderr, "Failed xtrx_set_gain(PGA): %d\n", res);
			goto falied_tune;
		}
		fprintf(stderr, "RX PGA gain: %f\n", actuallnagain);

		res = xtrx_set_gain(dev, ch, XTRX_RX_TIA_GAIN, rxgain_tia, &actuallnagain);
		if (res) {
			fprintf(stderr, "Failed xtrx_set_gain(TIA): %d\n", res);
			goto falied_tune;
		}
		fprintf(stderr, "RX TIA gain: %f\n", actuallnagain);
	}

	int do_tx_rf = (dmatx || tx_repeat_mode) && !loopback;
	if (do_tx_rf) {
		res = xtrx_tune(dev, XTRX_TUNE_TX_FDD, txfreq, &txactualfreq);
		if (res) {
			fprintf(stderr, "Failed xtrx_tune (TX): %d\n", res);
			goto falied_tune;
		}
		fprintf(stderr, "TX tunned: %f\n", txactualfreq);

		if (txfreq > 2300e6) {
			xtrx_set_antenna(dev, XTRX_TX_H);
		} else {
			xtrx_set_antenna(dev, XTRX_TX_W);
		}

		res = xtrx_tune_tx_bandwidth(dev, ch, txbandwidth, &actual_txbandwidth);
		if (res) {
			fprintf(stderr, "Failed xtrx_tune_tx_bandwidth: %d\n", res);
		}
		fprintf(stderr, "TX bandwidth: %f\n", actual_txbandwidth);

		res = xtrx_set_gain(dev, ch, XTRX_TX_PAD_GAIN, txgain, &actuallnagain);
		if (res) {
			fprintf(stderr, "Failed xtrx_set_gain(PAD): %d\n", res);
			goto falied_tune;
		}
		fprintf(stderr, "TX PAD gain: %f\n", actuallnagain);
	}

	if (gmode > 0) {
		rx_rep_gtime = 1;

		gtime_data_t in = {0,0};
		gtime_data_t out;
		res = xtrx_gtime_op(dev, -1, XTRX_GTIME_DISABLE, in, &out);
		if (res) {
			fprintf(stderr, "Failed xtrx_gtime_op(0): %d\n", res);
			goto falied_tune;
		}

		unsigned cmd = (gmode == 6) ? XTRX_GTIME_ENABLE_EXT :
						(gmode == 5) ? XTRX_GTIME_ENABLE_EXTNFW :
						(gmode == 4) ? XTRX_GTIME_ENABLE_INT_WEXTENFW :
						(gmode == 3) ? XTRX_GTIME_ENABLE_INT_WEXTE :
						(gmode == 2) ? XTRX_GTIME_ENABLE_INT_WEXT :
									 XTRX_GTIME_ENABLE_INT;

		in.sec = (gmode == 5) ? 4 : 7;
		res = xtrx_gtime_op(dev, -1, cmd, in, &out);
		if (res) {
			fprintf(stderr, "Failed xtrx_gtime_op(3): %d\n", res);
			goto falied_tune;
		}

		for (unsigned i = 0; i < 8; i++) {
			usleep(450000);
			for (unsigned j = 0; j < dev_count; j++) {
				res = xtrx_gtime_op(dev, j, XTRX_GTIME_GET_CUR, in, &out);
				if (res) {
					fprintf(stderr, "Failed xtrx_gtime_op(4): %d\n", res);
					goto falied_tune;
				}
				fprintf(stderr, "Current time[%d]: %08d.%09d\n", j, out.sec, out.nsec);
			}
		}
	}

	s_tx_start_ts = s_tx_skip / (tx_siso ? 1 : 2);
	s_tx_nodiscard = tx_nodiscard;

	xtrx_stop(dev, XTRX_TRX);
	xtrx_stop(dev, XTRX_TRX);

	xtrx_direction_t dir = 0;

	if (dmarx) {
		dir |= XTRX_RX;
	}
	if (dmatx || tx_repeat_mode) {
		dir |= XTRX_TX;
	}

	xtrx_run_params_t params;
	params.dir = dir;
	params.nflags = (loopback) ? XTRX_RUN_DIGLOOPBACK :
								 (rxlfsr)   ? XTRX_RUN_RXLFSR : 0;
	params.rx.wfmt = rx_wire_fmt;
	params.rx.hfmt = rx_host_fmt;
	params.rx.chs = ch;
	params.rx.flags = (rx_tst_a    ? XTRX_RSP_TEST_SIGNAL_A : 0) |
			(rx_tst_b    ? XTRX_RSP_TEST_SIGNAL_B : 0) |
			(rx_siso     ? XTRX_RSP_SISO_MODE     : 0) |
			(rx_swap_ab  ? XTRX_RSP_SWAP_AB       : 0) |
			(rx_swap_iq  ? XTRX_RSP_SWAP_IQ       : 0);
	params.rx.paketsize = rx_packet_size / ((rx_siso) ? 1 : 2);
	params.tx.wfmt = tx_wire_fmt;
	params.tx.hfmt = tx_host_fmt;
	params.tx.chs = ch;
	params.tx.flags = (tx_tst_a    ? XTRX_RSP_TEST_SIGNAL_A : 0) |
			(tx_tst_b    ? XTRX_RSP_TEST_SIGNAL_B : 0) |
			(tx_siso     ? XTRX_RSP_SISO_MODE     : 0) |
			(tx_swap_ab  ? XTRX_RSP_SWAP_AB       : 0) |
			(tx_swap_iq  ? XTRX_RSP_SWAP_IQ       : 0);
	params.tx.paketsize = tx_packet_size / ((tx_siso) ? 1 : 2);
	params.rx_stream_start = rx_skip;
	params.tx_repeat_buf = NULL; //(tx_repeat_mode) ? BUF : NULL;
	params.gtime.sec = 11;
	params.gtime.nsec = 750000000; //250 ms delay
	if (gmode > 0) {
		params.nflags |= XTRX_RUN_GTIME;
	}

	res = xtrx_run_ex(dev, &params);
	if (res) {
		fprintf(stderr, "Failed xtrx_run: %d\n", res);
		goto falied_tune;
	}

	if (gmode > 0) {
		gtime_data_t in = {0,0};
		gtime_data_t out;
		res = xtrx_gtime_op(dev, -1, XTRX_GTIME_GET_CUR, in, &out);
		if (res) {
			fprintf(stderr, "Failed xtrx_gtime_op(4): %d\n", res);
			goto falied_tune;
		}
		fprintf(stderr, "Current time: %08d.%09d\n", out.sec, out.nsec);
	}

	//
	// Streaming
	//
	uint64_t rx_processed = 0;
	struct stream_data sd_rx;
	sd_rx.dev = dev;
	sd_rx.cycles = cycles;
	sd_rx.samples_per_cyc = samples;
	sd_rx.slice_sz = rx_slice;
	sd_rx.ramplerate = actual_rxsample_rate;
	sd_rx.mux_demux = mimomode;
	sd_rx.siso = rx_siso;
	sd_rx.flush_data = rx_flush_data;
	sd_rx.sample_host_size = rx_sample_host_size;

	uint64_t tx_processed = 0;
	struct stream_data sd_tx;
	sd_tx.dev = dev;
	sd_tx.cycles = cycles;
	sd_tx.samples_per_cyc = samples;
	sd_tx.slice_sz = tx_slice;
	sd_tx.ramplerate = actual_txsample_rate;
	sd_tx.mux_demux = mimomode;
	sd_tx.siso = tx_siso;
	sd_tx.flush_data = tx_flush_data;
	sd_tx.sample_host_size = tx_sample_host_size;

	if (dmarx) {
		stream_rx(&sd_rx, &rxd);
		rx_tm = sd_rx.out_tm_diff;
		rx_processed = sd_rx.out_samples_per_dev;

		fprintf(stderr, "RX STAT Overruns:%d\n", sd_rx.out_overruns);

		if (rx_flush_data) {
			rxd.stop = true;
			sem_post(&rxd.sem_read_wr);
			pthread_join(rx_write_thread, NULL);
		}
	} else if (dmatx && !dmarx) {
		stream_tx(&sd_tx, &txd);
		tx_tm = sd_tx.out_tm_diff;
		tx_processed = sd_tx.out_samples_per_dev;

		fprintf(stderr, "TX STAT Underruns:%d\n", sd_tx.out_overruns);

		if (tx_flush_data) {
			txd.stop = true;
			sem_post(&txd.sem_read_rx);
			pthread_join(tx_read_thread, NULL);
		}
	} else if (dmatx && dmarx) {
		// TODO TX & RX

		abort();
	}

	xtrx_stop(dev, dir);
	xtrx_stop(dev, dir);
	fprintf(stderr, "Success!\n");


	unsigned rxchs = (rx_siso ? 1 : 2);
	unsigned txchs = (tx_siso ? 1 : 2);

	double rx_t = (dmarx) ? (double)rx_processed*1000/rx_tm : 0;
	double rx_w = (dmarx) ? (double)rx_processed*(rx_wire_fmt+1)*1000/rx_tm : 0;

	double tx_t = (dmatx) ? (double)tx_processed*1000/tx_tm : 0;
	double tx_w = (dmatx) ? (double)tx_processed*(tx_wire_fmt+1)*1000/tx_tm : 0;

	fprintf(stderr, "Processed %d devs, each: RX %d x %.3f = %.3f MSPS (WIRE: %f)    TX %d x %.3f = %.3f MSPS (WIRE: %f MB/s) \n",
			dev_count,
			rxchs, rx_t / rxchs, rx_t, rx_w,
			txchs, tx_t / txchs, tx_t, tx_w);

	xtrx_close(dev);
	return 0;


	xtrx_stop(dev, XTRX_TRX);
falied_tune:
falied_samplerate:
failed_rx:
failed_tx:
	xtrx_close(dev);
	free(out_buffs);
falied_open:
	return res;
}
