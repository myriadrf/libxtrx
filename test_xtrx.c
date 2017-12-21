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

static int create_stream(const char* filename, int flags)
{
	int fd = open(filename, flags);
	if (fd < 0) {
		perror("Can't open file");
		exit(EXIT_FAILURE);
	}
	return fd;
}

static int create_out_stream(const char* filename)
{
	int g_out_stream = create_stream(filename, O_WRONLY);
	return g_out_stream;
}
#if 0
static int create_in_stream(const char* filename)
{
	int g_in_stream = create_stream(filename, O_RDONLY);
	return g_in_stream;
}
#endif
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

static const int16_t testbuf[] =
{ 0x0000, 0xffff,
  0xaaaa, 0x5555,
  0x0000, 0x0000,
  0x0000, 0x0000,
  0xffff, 0xffff,
  0xffff, 0xffff,
  0xaaaa, 0xaaaa,
  0xaaaa, 0xaaaa};

#define V(x) ((unsigned)(x) << 4)

int16_t testbuf2[256];

void init_buf()
{
	int i;
	for (i = 0; i < 256; ++i) {
		testbuf2[i] = V(i+1);
	}
}

#define BUF testbuf2
static volatile int s_stopflag = 0;
static uint32_t s_tx_slice = 8192;
static uint32_t s_tx_skip = 0;
static uint32_t s_tx_pause = 0;
static int s_tx_siso = 0;
static double s_actual_txsample_rate;
static uint64_t s_tx_tm = 0;
static uint64_t s_tx_cycles = 0;
static unsigned s_logp = 0;

static void serialize_b12(uint64_t v, uint16_t* out)
{
	*out++ = v << 4;
	*out++ = (v >> 12) << 4;
	*out++ = (v >> 24) << 4;
	*out = (v >> 36) << 4;
}

static uint64_t deserialize_b12(const uint16_t* out)
{
	uint64_t v;
	v = *out++ >> 4;
	v |= (uint64_t)(*out++ >> 4) << 12;
	v |= (uint64_t)(*out++ >> 4) << 24;
	v |= (uint64_t)(*out++ >> 4) << 36;
	return v;
}


void* send_thread_func1(void* obj)
{
	static uint16_t buf[16384*16];
	int j;
	for (j = 0; j < (2*2*2*s_tx_slice)/64; j++) {
		memcpy(&buf[64*j], testbuf2, 64*2);
	}
	// Fill remaining buffer with dead value
	for (int k = j*64; k < 16384*8; k++) {
		buf[k] = 0x1234; //V(66);
	}


	fprintf(stderr, "send_thread_func1 started!!!\n");
	int res;
	uint32_t ts = s_tx_skip; //2 * s_send_samples;
	struct xtrx_dev *dev = (struct xtrx_dev *)obj;
	xtrx_send_ex_info_t nfo;
	const void *buffers[1] = { (void*)&buf[0] };

	nfo.buffers = buffers;
	nfo.buffer_count = 1;
	nfo.flags = XTRX_TX_DONT_BUFFER;
	nfo.samples = s_tx_slice;

	uint64_t sp = grtime();
	uint64_t abpkt = sp;
	uint64_t st, tm;
	st = sp;

	uint64_t underruns = 0;
	uint64_t p = 0;
	uint64_t h = 0;

	while (!s_stopflag) {
		nfo.ts = ts;

		uint64_t sa = grtime();
		uint64_t da = sa - sp;

		if (1) {
			//((uint64_t*)buf)[0] = sa;
			serialize_b12(sa, &buf[0]);
		}

		res = xtrx_send_sync_ex(dev, &nfo);
		sp = grtime();
		uint64_t sb = sp - sa;

		//ts += nfo.out_samples * nfo.buffer_count / (s_tx_siso ? 1 : 2) + s_skip;
		ts += nfo.samples * nfo.buffer_count / (s_tx_siso ? 1 : 2) + s_tx_pause;

		abpkt += 1e9 * nfo.out_samples * nfo.buffer_count / s_actual_txsample_rate / (s_tx_siso ? 1 : 2);

		if (s_logp == 0 || p % s_logp == 0)
			fprintf(stderr, "PROCESSED TX SLICE %" PRIu64 "/%" PRIu64 ": res %d TS:%8" PRIu64 " %c%c  %6" PRId64 " us DELTA %6" PRId64 " us LATE %6" PRId64 " us"
							" %d samples T=%16" PRIx64 "\n",
					p, h, res, nfo.out_txlatets,
					(nfo.out_flags & XTRX_TX_DISCARDED_TO)    ? 'D' : ' ',
					' ',
					sb / 1000, da / 1000, (int64_t)(sp - abpkt) / 1000, nfo.out_samples * nfo.buffer_count,
					sa);
		if (res) {
			fprintf(stderr, "send_thread_func1: xtrx_send_burst_sync err %d\n", res);
			return NULL;
		}
		if (nfo.out_flags) {
			underruns ++;
		}

		p++;
	}

	tm = grtime() - st;
	fprintf(stderr, "TX Underruns:%" PRIu64 "\n", underruns);

	s_tx_cycles = p;
	s_tx_tm = tm;

	fprintf(stderr, "send_thread_func1 finished!!!\n");
	return NULL;
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

int main(int argc, char** argv)
{
	struct xtrx_dev *dev;
	//uint32_t result;
	//uint32_t i;
	int opt;
	uint64_t st, tm;
	const char* device = "/dev/xtrx0";

	pthread_t sendthread;
	int dump_regs = 0;
	double rxsamplerate = 4.0e6, actual_rxsample_rate = 0;
	double txsamplerate = 4.0e6, actual_txsample_rate = 0;
	double rxfreq = 900e6, rxactualfreq = 0;
	double txfreq = 450e6, txactualfreq = 0;
	double rxbandwidth = 5e6, actual_rxbandwidth = 0;
	double txbandwidth = 2e6, actual_txbandwidth = 0;
	double lnagain = 15, actuallnagain = 0;
	xtrx_channel_t ch = XTRX_CH_AB;
	xtrx_wire_format_t rx_wire_fmt = XTRX_WF_16;
	xtrx_host_format_t rx_host_fmt = XTRX_IQ_INT16;
	xtrx_wire_format_t tx_wire_fmt = XTRX_WF_16;
	xtrx_host_format_t tx_host_fmt = XTRX_IQ_INT16;
	uint64_t samples = 524288;
	unsigned rx_slice = samples;
	int outstream;
	//int outstreamb;
	unsigned rx_sample_host_size = sizeof(float) * 2;
	unsigned tx_sample_host_size = sizeof(float) * 2;

	int loglevel = 4;
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

	unsigned samples_flag = 0;
	unsigned refclk = 0;
	unsigned cycles = 1;
	unsigned rx_skip = 0;
	unsigned logp = 0;
	double master_in = 0;
	int extclk = 0;
	init_buf();


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
	{"dumpregs",no_argument,       0,   'd' },
	{"device",  required_argument, 0,   'D' },
	{"samples", required_argument,  0,   'N' },
	{"out",     required_argument,  0,   'O' },
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
		case 'P':	loopback = 1;				break;
		case 'R':	tx_repeat_mode = 1;			break;
		case 'r': rxlfsr = 1;					break;
		case 'M':	mimomode = 1;				break;
		case 'm':	extclk = 1;					break;
		case 'l':	loglevel = (atoi(optarg));	break;
		case 'L': logp = (atoi(optarg));		break;
		case 'd':	dump_regs = 1;				break;
		case 'D': device = optarg;				break;
		case 'x':
			switch (atoi(optarg)) {
			case 8: rx_wire_fmt = XTRX_WF_8; break;
			case 12: rx_wire_fmt = XTRX_WF_12; break;
			default: rx_wire_fmt = XTRX_WF_16; break;
			}
			break;
		case 'N':	samples = atoll(optarg);		break;
		case 'O':
			outstream = create_out_stream(optarg);
			if (outstream == -1) {
				perror("Can't open out strem:");
				exit(EXIT_FAILURE);
			}
			break;
		case 'Z': tx_packet_size = atoi(optarg); break;
		case 'z': rx_packet_size = atoi(optarg); break;

		case 'K':	s_tx_skip = atoi(optarg);		break;
		case 'k':	rx_skip = atoi(optarg);		break;

		case 'E':	s_tx_slice = atoi(optarg);	break;
		case 'e': rx_slice = atoi(optarg);	break;

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

		case 'y': master_in = atof(optarg); break;
		case 'Y': vio = atoi(optarg); break;
		default: /* '?' */
			fprintf(stderr, "Usage: %s <options>\n", argv[0]);
			generate_help(long_options);
			exit(EXIT_FAILURE);
		}
	}

	if (samples < rx_slice) {
		rx_slice = samples;
	}

	s_tx_siso = tx_siso;
	s_logp = logp;

	if (dmarx == 0 && dmatx == 0 && tx_repeat_mode == 0) {
		fprintf(stderr, "Usage: DMATX and/or DMARX must be enabled (or TX REPEAT is set)!\n");
		exit(EXIT_FAILURE);
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

	char* data ;
	char* datab = NULL;
	data = (char*)malloc(rx_sample_host_size * samples); // IQ * float * samples
	memset(data, 0, rx_sample_host_size * samples);

	if (mimomode) {
		datab = (data + rx_sample_host_size * samples / 2);
	}

	int res = xtrx_open(device, loglevel, &dev);
	if (res) {
		fprintf(stderr, "Failed xtrx_open: %d\n", res);
		goto falied_open;
	}

	if (refclk || extclk) {
		xtrx_set_ref_clk(dev, refclk, (extclk) ? XTRX_CLKSRC_EXT : XTRX_CLKSRC_INT);
	}

	double master;
	res = xtrx_set_samplerate(dev, master_in, rxsamplerate, txsamplerate, samples_flag,
							  &master, &actual_rxsample_rate, &actual_txsample_rate);
	if (res) {
		fprintf(stderr, "Failed xtrx_set_samplerate: %d\n", res);
		goto falied_samplerate;
	}
	fprintf(stderr, "Master: %f; RX rate: %f; TX rate: %f\n",
			master, actual_rxsample_rate, actual_txsample_rate);

	s_actual_txsample_rate = actual_txsample_rate;

	if (vio) {
		xtrx_val_set(dev, XTRX_TRX, XTRX_CH_AB, XTRX_LMS7_VIO, vio);
	}

	if (dmarx) {
		res = xtrx_tune(dev, XTRX_TUNE_RX_FDD, rxfreq, &rxactualfreq);
		if (res) {
			fprintf(stderr, "Failed xtrx_tune: %d\n", res);
			goto falied_tune;
		}
		fprintf(stderr, "RX tunned: %f\n", rxactualfreq);

		if (rxfreq < 200e6) {
			xtrx_set_antenna(dev, XTRX_RX_L);
		} else if (rxfreq > 1800e6) {
			xtrx_set_antenna(dev, XTRX_RX_H);
		} else {
			xtrx_set_antenna(dev, XTRX_RX_W);
		}

		res = xtrx_tune_rx_bandwidth(dev, ch, rxbandwidth, &actual_rxbandwidth);
		if (res) {
			fprintf(stderr, "Failed xtrx_tune_rx_bandwidth: %d\n", res);
			//goto falied_tune;
		}
		fprintf(stderr, "RX bandwidth: %f\n", actual_rxbandwidth);

		res = xtrx_set_gain(dev, ch, XTRX_RX_LNA_GAIN, lnagain, &actuallnagain);
		if (res) {
			fprintf(stderr, "Failed xtrx_set_gain: %d\n", res);
			goto falied_tune;
		}
		fprintf(stderr, "RX LNA gain: %f\n", actuallnagain);
	}

	int do_tx_rf = (dmatx || tx_repeat_mode) && !loopback;
	if (do_tx_rf) {
		res = xtrx_tune(dev, XTRX_TUNE_TX_FDD, txfreq, &txactualfreq);
		if (res) {
			fprintf(stderr, "Failed xtrx_tune (TX): %d\n", res);
			goto falied_tune;
		}
		fprintf(stderr, "TX tunned: %f\n", txactualfreq);

		if (rxfreq < 1000e6) {
			xtrx_set_antenna(dev, XTRX_TX_L);
		} else {
			xtrx_set_antenna(dev, XTRX_TX_W);
		}

		res = xtrx_tune_tx_bandwidth(dev, ch, txbandwidth, &actual_txbandwidth);
		if (res) {
			fprintf(stderr, "Failed xtrx_tune_tx_bandwidth: %d\n", res);
			//goto falied_tune;
		}
		fprintf(stderr, "TX bandwidth: %f\n", actual_txbandwidth);

		// TODO set GAIN
	}

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
	params.tx.paketsize = (tx_repeat_mode) ? (sizeof(BUF)/2) : tx_packet_size / ((tx_siso) ? 1 : 2);
	params.rx_stream_start = rx_skip;
	params.tx_repeat_buf = (tx_repeat_mode) ? BUF : NULL;

	res = xtrx_run_ex(dev, &params);
	if (res) {
		fprintf(stderr, "Failed xtrx_run: %d\n", res);
		goto falied_tune;
	}

	if (dmatx && dmarx) {
		res = pthread_create(&sendthread, NULL, send_thread_func1, dev);
		if (res) {
			fprintf(stderr, "Failed start TX thread: %d\n", res);
			goto falied_tune;
		}
	}

	if (dmarx) {
		uint64_t zero_inserted = 0;
		int overruns = 0;

		uint64_t sp = grtime();
		uint64_t abpkt = sp;
		fprintf(stderr, "RX SAMPLES=%" PRIu64 " SLICE=%u PARTS=%" PRIu64 "\n", samples, rx_slice, samples / rx_slice);
		st = grtime();
		for (uint64_t p = 0; p < cycles; p++) {
			for (uint64_t h = 0; h < samples / rx_slice; h++) {
				xtrx_recv_ex_info_t ri;
				size_t rem = samples - h * rx_slice;
				if (rem > rx_slice)
					rem = rx_slice;

				void* buffers[2] = {(datab) ? ((char*)data) + h * rx_slice * rx_sample_host_size / 2 :   ((char*)data) + h * rx_slice * rx_sample_host_size,
									(datab) ? ((char*)datab) + h * rx_slice * rx_sample_host_size / 2 : NULL };
				ri.samples = rem / ((datab) ? 2 : 1);
				ri.buffer_count = (datab) ? 2 : 1;
				ri.buffers = buffers;
				ri.flags = 0;

				uint64_t sa = grtime();
				uint64_t da = sa - sp;
				res = xtrx_recv_sync_ex(dev, &ri);
				sp = grtime();
				uint64_t sb = sp - sa;

				abpkt += 1e9 * ri.samples * ri.buffer_count / actual_rxsample_rate  / (rx_siso ? 1 : 2);

				uint64_t dt = 0;
				if (1) {
					dt = deserialize_b12((const uint16_t*)((char*)data + 0x80));
				}

				if (logp == 0 || p % s_logp == 0)
					fprintf(stderr, "PROCESSED RX SLICE %" PRIu64 " /%" PRIu64 ": res %d TS:%8" PRIu64 " %c%c  %6" PRId64 " us DELTA %6" PRId64 " us LATE %6" PRId64 " us"
									" %d samples R=%16" PRIx64 " DELTA=%d\n",
							p, h, res, ri.out_first_sample,
							(ri.out_events & RCVEX_EVENT_OVERFLOW)    ? 'O' : ' ',
							(ri.out_events & RCVEX_EVENT_FILLED_ZERO) ? 'Z' : ' ',
							sb / 1000, da / 1000, (int64_t)(sp - abpkt) / 1000, ri.out_samples,
							dt, (int)((sa - dt) / 1000));
				if (res) {
					fprintf(stderr, "Failed xtrx_recv_sync: %d\n", res);
					goto falied_stop_rx;
				}

				if (ri.out_events & RCVEX_EVENT_OVERFLOW) {
					overruns++;
					zero_inserted += (ri.out_resumed_at - ri.out_overrun_at);
				}
			}
		}
falied_stop_rx:
		tm = grtime() - st;
		fprintf(stderr, "XXXXX Overruns:%d Zeros:%" PRIu64 "\n", overruns, zero_inserted);

		if (rx_sample_host_size == 4) {
			uint16_t *d = (uint16_t*)data;
			uint16_t vor[4] = {0,0,0,0};
			uint16_t vand[4] = {0xffff,0xffff,0xffff,0xffff};

			for (unsigned i = 0; i < samples / 2;) {
				vor[0] |= *d; vand[0] &= *d;  ++i, ++d;
				vor[1] |= *d; vand[1] &= *d;  ++i, ++d;
				vor[2] |= *d; vand[2] &= *d;  ++i, ++d;
				vor[3] |= *d; vand[3] &= *d;  ++i, ++d;
			}

			fprintf(stderr, "xAND=[0x%04x,0x%04x,0x%04x,0x%04x] xOR=[0x%04x,0x%04x,0x%04x,0x%04x]\n",
					vand[0], vand[1], vand[2], vand[3],
					vor[0], vor[1], vor[2], vor[3]);
		} else if (rx_sample_host_size == 2) {
			uint8_t *d = (uint8_t*)data;
			uint8_t vor[4] = {0,0,0,0};
			uint8_t vand[4] = {0xff,0xff,0xff,0xff};

			for (unsigned i = 0; i < samples / 2;) {
				vor[0] |= *d; vand[0] &= *d;  ++i, ++d;
				vor[1] |= *d; vand[1] &= *d;  ++i, ++d;
				vor[2] |= *d; vand[2] &= *d;  ++i, ++d;
				vor[3] |= *d; vand[3] &= *d;  ++i, ++d;
			}

			fprintf(stderr, "xAND=[0x%02x,0x%02x,0x%02x,0x%02x] xOR=[0x%02x,0x%02x,0x%02x,0x%02x]\n",
					vand[0], vand[1], vand[2], vand[3],
					vor[0], vor[1], vor[2], vor[3]);
		}
	} else if (dmatx && !dmarx) {
		static char buf[32768*16];
		int j;
		for (j = 0; j < (sizeof(buf))/(sizeof(testbuf2)); j++) {
			memcpy(&buf[sizeof(testbuf2)*j], testbuf2, sizeof(testbuf2));
		}

		//unsigned txchs = (tx_siso ? 1 : 2);
		uint64_t underruns = 0;
		uint64_t tx_start_ts = s_tx_skip / (tx_siso ? 1 : 2);
		uint64_t tx_sent_samples = 0;

		uint64_t sp = grtime();
		uint64_t abpkt = sp;
		uint64_t p;
		fprintf(stderr, "TX SAMPLES=%" PRIu64 " SLICE=%u PARTS=%" PRIu64 "\n", samples, s_tx_slice, samples / s_tx_slice);
		st = grtime();
		for (p = 0; p < cycles; p++) {
			for (uint64_t h = 0; h < samples / s_tx_slice; h++) {
				size_t rem = samples - h * s_tx_slice;
				if (rem > s_tx_slice)
					rem = s_tx_slice;

				xtrx_send_ex_info_t nfo;
				const void *buffers[2] = { (void*)&buf[0], (void*)&buf[0] };
				nfo.samples = rem / ((mimomode) ? 2 : 1);
				nfo.flags = XTRX_TX_DONT_BUFFER;
				nfo.ts = tx_start_ts + tx_sent_samples;
				nfo.buffers = buffers;
				nfo.buffer_count = (mimomode) ? 2 : 1;

				uint64_t sa = grtime();
				uint64_t da = sa - sp;
				res = xtrx_send_sync_ex(dev, &nfo);
				sp = grtime();
				uint64_t sb = sp - sa;

				abpkt += 1e9 * nfo.samples * nfo.buffer_count / actual_txsample_rate / (tx_siso ? 1 : 2);

				if (logp == 0 || p % s_logp == 0)
					fprintf(stderr, "PROCESSED TX SLICE %" PRIu64 "/%" PRIu64 ": res %d TS:%8" PRIu64 " %c%c  %6" PRId64 " us DELTA %6" PRId64 " us LATE %6" PRId64 " us  %d samples\n",
							p, h, res, nfo.out_txlatets,
							(nfo.out_flags & XTRX_TX_DISCARDED_TO)    ? 'D' : ' ',
							' ',
							sb / 1000, da / 1000, (int64_t)(sp - abpkt) / 1000, nfo.out_samples * nfo.buffer_count);
				if (res) {
					fprintf(stderr, "Failed xtrx_send_sync_ex: %d\n", res);
					goto falied_stop_tx;
				}

				//tx_sent_samples += nfo.out_samples * nfo.buffer_count / (tx_siso ? 1 : 2);
				tx_sent_samples += nfo.samples * nfo.buffer_count / (tx_siso ? 1 : 2);
				if (nfo.out_flags) {
					underruns ++;
				}
			}
		}
falied_stop_tx:
		tm = grtime() - st;

		s_tx_cycles = p * (samples / s_tx_slice);
		s_tx_tm = tm;
		fprintf(stderr, "TX Underruns:%" PRIu64 "\n", underruns);
	}

	if (dump_regs) {
		xtrx_debug_dump_lms(dev, "outlms.ini");
	}

	s_stopflag = 1;
	if (dmatx && dmarx) {
		pthread_join(sendthread, NULL);
	}


	xtrx_stop(dev, dir);
	xtrx_stop(dev, dir);
	fprintf(stderr, "Success!\n");


	unsigned rxchs = (rx_siso ? 1 : 2);
	unsigned txchs = (tx_siso ? 1 : 2);

	double rx_t = (dmarx) ? cycles*(double)samples*1000/tm : 0;
	double rx_w = (dmarx) ? cycles*(double)(rx_wire_fmt+1)*samples*1000/tm : 0;
	double tx_t = (dmatx) ? s_tx_cycles*(double)s_tx_slice*1000/s_tx_tm : 0;
	double tx_w = (dmatx) ? s_tx_cycles*(double)s_tx_slice*(double)(tx_wire_fmt+1)*1000/s_tx_tm : 0;

	fprintf(stderr, "Processed RX %d x %.3f = %.3f MSPS (WIRE: %f)    TX %d x %.3f = %.3f MSPS (WIRE: %f MB/s) \n",
			rxchs, rx_t / rxchs, rx_t, rx_w,
			txchs, tx_t / txchs, tx_t, tx_w);

	if (outstream && dmarx) {
		if (mimomode) {
			write(outstream, data, rx_sample_host_size * samples / 2);
			write(outstream, datab, rx_sample_host_size * samples / 2);
		} else {
			write(outstream, data, rx_sample_host_size * samples);
		}
	}

	xtrx_close(dev);
	return 0;

//falied_stop:
	xtrx_stop(dev, XTRX_TRX);
falied_tune:
falied_samplerate:
	if (dump_regs) {
		xtrx_debug_dump_lms(dev, "outerrlms.ini");
	}
	xtrx_close(dev);
falied_open:
	return res;
}
