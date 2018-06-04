/*
 * xtrx high level source file
 * Copyright (c) 2018 Sergey Kostanbaev <sergey.kostanbaev@fairwaves.co>
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
#include "xtrx_api.h"
#include <xtrxll_api.h>
#include <xtrxll_mmcm.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <inttypes.h>
#include <xtrxdsp.h>
#include <xtrxdsp_filters.h>
#include <xtrxll_log.h>
#include "xtrx_fe.h"
#include "xtrx_debug.h"

#define MAX(x,y) (((x) > (y)) ? (x) : (y))

enum {
	MAX_LMS = 1,
	DEF_BUFSIZE = 32768,
	DEF_RX_BUFSIZE = DEF_BUFSIZE,
	DEF_TX_BUFSIZE = DEF_BUFSIZE,
};

typedef double clock_val_t;

struct xtrx_multill_stream {
	bool                run;

	void*               buf;
	size_t              buf_total;
	size_t              buf_processed;    /** Number of processed samples (total for all channels) in the cached buffer */
	uint64_t            buf_processed_ts; /** Number of samples (total for all channels) in the cached buffer */
	uint64_t            buf_ts;           /** First sample number in the cached buffer */
	uint64_t            buf_conv_state;
	unsigned            buf_len_iq_symbol;
	unsigned            buf_len_iq_host_sym;
	unsigned            chans_in_stream; /**< Number of logical channels multiplexed into the device stream */

	float               scale_16;
	xtrx_host_format_t  hostfmt;
	xtrx_wire_format_t  busfmt;

	xtrxll_fe_t         fefmt;
};

struct xtrx_dev {
	struct xtrxll_dev* lldev;
	struct xtrx_fe_obj* fe;
	xtrx_debug_ctx_t* debugif;

	clock_val_t         refclock;
	xtrx_clock_source_t clock_source;

	char                rxinit;
	xtrx_host_format_t  rx_hostfmt;
	xtrx_wire_format_t  rx_busfmt;
	xtrxll_fe_t         rx_fefmt;
	uint64_t            rx_samples; /* num of RX samples for both channels in MIMO or signle channel is SISO */
	float               rx_scale_16;

	void*               rxbuf;
	unsigned            rxbuf_total;
	unsigned            rxbuf_processed;    /** Number of processed bytes (total for all channels) in the cached buffer */
	uint64_t            rxbuf_processed_ts; /** Number of samples (in each device channel) in the cached buffer */
	uint64_t            rxbuf_ts;           /** First sample number in the cached buffer */
	uint64_t            rxbuf_conv_state;
	unsigned            rxbuf_len_iq_symbol;
	unsigned            rxbuf_len_iq_host_sym;
	bool                rx_run;
	unsigned            rx_chans_in_stream; /**< Number of logical channels multiplexed into the device stream */

	char                txinit;
	xtrx_host_format_t  tx_hostfmt;
	xtrx_wire_format_t  tx_busfmt;
	xtrxll_fe_t         tx_fefmt;
	float               tx_scale_16;

	void*               txbuf;
	unsigned            txbuf_total;
	unsigned            txbuf_processed;    /** Number of processed bytes (total for all channels) in the cached buffer */
	uint64_t            txbuf_processed_ts; /** Number of samples (in each device channel) in the cached buffer */
	uint64_t            txbuf_conv_state;
	unsigned            txbuf_len_iq_symbol;
	unsigned            txbuf_len_iq_host_sym;
	unsigned            txburt_late_prev;
	uint64_t            txskip_time;
	bool                tx_run;
	unsigned            tx_chans_in_stream;

	unsigned            tx_pkt_samples;

	double              tx_bandwidth;
	double              rx_bandwidth;

	xtrxdsp_filter_state_t rx_host_filter[2]; /* For A & B channels  */
	xtrxdsp_filter_state_t tx_host_filter[2]; /* For A & B channels  */
	unsigned               tx_host_inter;   /* Interpolation on HOST */
	unsigned               rx_host_decim;   /* Decimation on HOST */
};

static int _debug_param_io(void* obj, unsigned param, uint64_t val, uint64_t* oval)
{
	int res;
	int tmp;
	struct xtrx_dev* dev = (struct xtrx_dev*)obj;

	switch (param) {
	case DEBUG_RFIC_SPI_WR:
		return xtrx_val_set(dev, XTRX_TRX, (xtrx_channel_t)(val >> 32),
							(xtrx_val_t)(XTRX_RFIC_REG_0 + ((val >> 16) & 0x7fff)),
							val & 0xffff);

	case DEBUG_RFIC_SPI_RD:
		return xtrx_val_get(dev, XTRX_TRX, (xtrx_channel_t)(val >> 32),
							(xtrx_val_t)(XTRX_RFIC_REG_0 + ((val >> 16) & 0x7fff)),
							oval);
	case DEBUG_GET_REFCLK:
		*oval = (unsigned)dev->refclock;
		return 0;

	case DEBUG_BOARD_TEMP:
		res = xtrxll_get_sensor(dev->lldev, 0, &tmp);
		XTRXLL_LOG(XTRXLL_INFO, "Temp %.1f C\n", (double)tmp/256);

		*oval = (unsigned)tmp;
		return res;

	case DEBUG_BOARD_DAC:
		res = -EINVAL;
		if (oval) {
			res = xtrxll_get_sensor(dev->lldev, XTRXLL_DAC_REG, &tmp);
			if (res)
				return res;
			*oval = (unsigned)tmp;

			XTRXLL_LOG(XTRXLL_INFO, "DAC: %d\n", tmp);
		}
		if (val > 0) {
			res = xtrxll_set_param(dev->lldev, XTRXLL_PARAM_REF_DAC, val);
		}
		return res;

	case DEBUG_GET_ANT_RX:
		return dev->fe->ops->get_reg(dev->fe, 0, XTRX_TRX, XTRX_FE_CUSTOM_0, oval);
	case DEBUG_GET_ANT_TX:
		return dev->fe->ops->get_reg(dev->fe, 0, XTRX_TRX, XTRX_FE_CUSTOM_0 + 1, oval);

	case DEBUG_SET_ANT_RX:
		return dev->fe->ops->set_reg(dev->fe, 0, XTRX_TRX, XTRX_FE_CUSTOM_0, val);
	case DEBUG_SET_ANT_TX:
		return dev->fe->ops->set_reg(dev->fe, 0, XTRX_TRX, XTRX_FE_CUSTOM_0 + 1, val);
	}
	return -EINVAL;
}

static const struct xtrx_debug_ops _debug_ops = {
	_debug_param_io,
};

int xtrx_discovery(xtrx_device_info_t* devs, size_t maxbuf)
{
	int res, i;
	xtrxll_device_info_t lldevs[maxbuf];
	res = xtrxll_discovery(lldevs, maxbuf);
	if (res < 0)
		return res;

	for (i = 0; i < res; i++) {
		strncpy(devs[i].uniqname, lldevs[i].uniqname, sizeof(devs[i].uniqname));
		strncpy(devs[i].proto, lldevs[i].proto, sizeof(lldevs[i].proto));
		strncpy(devs[i].speed, lldevs[i].busspeed, sizeof(lldevs[i].busspeed));
		strncpy(devs[i].serial, "", sizeof(devs[i].serial)); // TODO
		strncpy(devs[i].devid, "", sizeof(devs[i].devid));   // TODO
	}
	return res;
}


int xtrx_open(const char* device, unsigned flags, struct xtrx_dev** outdev)
{
	struct xtrxll_dev* lldev;
	struct xtrx_dev*   dev;
	int res;
	int loglevel;

	loglevel = flags & XTRX_O_LOGLVL_MASK;
	xtrxll_set_loglevel(loglevel);

	res = xtrxll_open(device, 0, &lldev);
	if (res)
		goto failed_openll;

	dev = (struct xtrx_dev*)malloc(sizeof(struct xtrx_dev));
	if (dev == NULL) {
		res = -errno;
		goto failed_mem;
	}
	memset(dev, 0, sizeof(struct xtrx_dev));

	dev->lldev = lldev;
	dev->refclock = 0;
	dev->clock_source = XTRX_CLKSRC_INT;

	xtrxdsp_init();

	res = xtrx_fe_init(lldev, flags, &dev->fe);
	if (res) {
		XTRXLL_LOG(XTRXLL_ERROR, "Failed to initialize frontend: err=%d\n", res);
		goto failed_fe;
	}

	res = xtrx_debug_init(NULL, &_debug_ops, dev, &dev->debugif);
	if (res) {
		XTRXLL_LOG(XTRXLL_ERROR, "Failed to initialize debug service: err=%d\n", res);
		goto failed_fe;
	}

	*outdev = dev;
	return 0;


failed_fe:
	free(dev);
failed_mem:
	xtrxll_close(lldev);
failed_openll:
	return res;
}

void xtrx_close(struct xtrx_dev* dev)
{
	if (dev->debugif) {
		xtrx_debug_free(dev->debugif);
	}

	dev->fe->ops->fe_deinit(dev->fe);

	xtrxll_close(dev->lldev);

	free(dev);
}

void xtrx_set_ref_clk(struct xtrx_dev* dev, unsigned refclkhz, xtrx_clock_source_t clksrc)
{
	dev->clock_source = clksrc;
	dev->refclock = refclkhz;
}


int xtrx_set_samplerate(struct xtrx_dev* dev,
						double cgen_rate,
						double rxrate,
						double txrate,
						unsigned flags,
						double *actualcgen,
						double* actualrx,
						double* actualtx)
{
	int res = 0, hwid;
	res = xtrxll_get_sensor(dev->lldev, XTRXLL_HWID, &hwid);
	if (res) {
		XTRXLL_LOG(XTRXLL_ERROR, "xtrx_set_samplerate: unable to get HWID\n");
		return res;
	}

	res = xtrxll_set_param(dev->lldev, XTRXLL_PARAM_EXT_CLK, (dev->clock_source) ? 1 : 0);
	if (res) {
		XTRXLL_LOG(XTRXLL_ERROR, "xtrx_set_samplerate: unable to set clock source\n");
		return res;
	}

	if (dev->refclock < 1) {
		// Determine refclk
		int osc;
		unsigned i;
		static const int base_refclk[] = { 10000000, 19200000, 26000000, 30720000, 38400000, 40000000 };
		res = xtrxll_get_sensor(dev->lldev, XTRXLL_REFCLK_CLK, &osc);
		if (res) {
			return res;
		}

		for (i = 0; i < sizeof(base_refclk) / sizeof(base_refclk[0]); i++) {
			int diff = base_refclk[i] - osc;
			if (abs(diff) * (int64_t)100 / base_refclk[i] < 1) {
				dev->refclock = base_refclk[i];
				XTRXLL_LOG(XTRXLL_INFO, "xtrx_set_samplerate: set RefClk to %d based on %d measurement\n",
						   (int)dev->refclock, osc);
				break;
			}
		}

		if (dev->refclock < 1) {
			XTRXLL_LOG(XTRXLL_INFO, "xtrx_set_samplerate: wierd RefClk %d! set RefClk manually\n", osc);
			return -ENOENT;
		}
	}

	struct xtrx_fe_samplerate inrates, outrates;
	memset(&inrates, 0, sizeof(inrates));
	memset(&inrates, 0, sizeof(outrates));

	inrates.adc.rate = rxrate;
	inrates.adc.hwrate = (rxrate > 0) ? cgen_rate / 4 : 0;
	inrates.adc.refclk = dev->refclock;

	inrates.dac.rate = txrate;
	inrates.dac.hwrate = (txrate > 0) ? cgen_rate / 4 : 0;
	inrates.dac.refclk = dev->refclock;

	inrates.flags = flags;
	inrates.hwid = hwid;
	inrates.refclk_source = dev->clock_source;

	res = dev->fe->ops->dd_set_samplerate(dev->fe, &inrates, &outrates);
	if (res)
		return res;

	dev->rx_host_decim = outrates.adc.host_di;
	dev->tx_host_inter = outrates.dac.host_di;

	if (actualcgen) {
		*actualcgen = MAX(outrates.adc.hwrate  * 4, outrates.dac.hwrate  * 4);
	}
	if (actualrx) {
		*actualrx = outrates.adc.rate;
	}
	if (actualtx) {
		*actualtx = outrates.dac.rate;
	}
	return res;
}

int xtrx_tune(struct xtrx_dev* dev, xtrx_tune_t type, double freq,
			  double *actualfreq)
{
	return xtrx_tune_ex(dev, type, XTRX_CH_AB, freq, actualfreq);
}

int xtrx_tune_ex(struct xtrx_dev* dev, xtrx_tune_t type, xtrx_channel_t ch,
				 double freq, double *actualfreq)
{
	switch (type) {
	case XTRX_TUNE_RX_FDD:
	case XTRX_TUNE_TX_FDD:
	case XTRX_TUNE_TX_AND_RX_TDD:
		return dev->fe->ops->fe_set_freq(dev->fe, ch, type, freq, actualfreq);

	case XTRX_TUNE_BB_RX:
	case XTRX_TUNE_BB_TX:
		return dev->fe->ops->bb_set_freq(dev->fe, ch, type, freq, actualfreq);

	default: return -EINVAL;
	}
}

static int xtrx_tune_bandwidth(struct xtrx_dev* dev, xtrx_channel_t xch,
							   int rbb, double bw, double *actualbw)
{
	unsigned type = (rbb) ? XTRX_TUNE_BB_RX : XTRX_TUNE_BB_TX;
	return dev->fe->ops->bb_set_badwidth(dev->fe, xch, type, bw, actualbw);
}

int xtrx_tune_tx_bandwidth(struct xtrx_dev* dev, xtrx_channel_t xch, double bw,
						   double *actualbw)
{
	int res = xtrx_tune_bandwidth(dev, xch, 0, bw, &dev->tx_bandwidth);
	if (actualbw) {
		*actualbw = dev->tx_bandwidth;
	}
	return res;
}

int xtrx_tune_rx_bandwidth(struct xtrx_dev* dev, xtrx_channel_t xch, double bw,
						   double *actualbw)
{
	int res = xtrx_tune_bandwidth(dev, xch, 1, bw, &dev->rx_bandwidth);
	if (actualbw) {
		*actualbw = dev->rx_bandwidth;
	}
	return res;
}


int xtrx_set_gain(struct xtrx_dev* dev, xtrx_channel_t xch, xtrx_gain_type_t gt,
				  double gain, double *actualgain)
{
	// TODO BB/FE multiplexing
	return dev->fe->ops->bb_set_gain(dev->fe, xch, gt, gain, actualgain);
}


int xtrx_set_antenna(struct xtrx_dev* dev, xtrx_antenna_t antenna)
{
	return dev->fe->ops->fe_set_lna(dev->fe, XTRX_CH_AB, 0, antenna);
}

/**
 * @brief xtrx_wire_format_get_iq_size Get size of symbol IQ pair on the wire
 * @param stream Stream
 */
static unsigned xtrx_wire_format_get_iq_size(xtrx_wire_format_t fmt)
{
	switch (fmt) {
	case XTRX_WF_8:  return 2;
	case XTRX_WF_12: return 3;
	case XTRX_WF_16: return 4;
	}

	return 0;
}

/**
 * @brief xtrx_wire_format_get_iq_size Get size of symbol IQ pair on the wire
 * @param stream Stream
 */
static unsigned xtrx_host_format_get_iq_size(xtrx_host_format_t fmt)
{
	switch (fmt) {
	case XTRX_IQ_INT8:    return 2;
	case XTRX_IQ_INT16:   return 4;
	case XTRX_IQ_FLOAT32: return 8;
	}

	return 0;
}

static bool xtrx_run_params_stream_is_mimo(const xtrx_run_stream_params_t* stream)
{
	return (stream->chs == XTRX_CH_AB &&
			!(stream->flags & XTRX_RSP_SISO_MODE));
}

static unsigned calculate_pkt_size(const xtrx_run_stream_params_t* stream)
{
	return xtrx_wire_format_get_iq_size(stream->wfmt) *
			stream->paketsize *
			((xtrx_run_params_stream_is_mimo(stream)) ? 2 : 1);
}

static void xtrx_run_params_stream_init(xtrx_run_stream_params_t* stream)
{
	stream->wfmt = XTRX_WF_16;
	stream->hfmt = XTRX_IQ_FLOAT32;
	stream->chs  = XTRX_CH_AB;
	stream->paketsize = (DEF_BUFSIZE / 2) / 2; /* IQ and 16 bit each */
}

void xtrx_run_params_init(xtrx_run_params_t* params)
{
	params->dir = XTRX_TRX;
	params->nflags = 0;
	xtrx_run_params_stream_init(&params->rx);
	xtrx_run_params_stream_init(&params->tx);
	params->rx_stream_start = 0;
	params->tx_repeat_buf = NULL;
}

#define MAX_TX_MSPS  8192

int xtrx_run_ex(struct xtrx_dev* dev, const xtrx_run_params_t* params)
{
	int res = -EINVAL;
	const int chan = 0; //TODO

	xtrxll_mode_t   tx_mode = XTRXLL_FE_MODE_MIMO;
	xtrxll_fe_t     tx_fe_fmt = XTRXLL_FE_DONTTOUCH;

	xtrxll_mode_t   rx_mode = XTRXLL_FE_MODE_MIMO;
	unsigned        rx_mode_flags = 0;
	xtrxll_fe_t     rx_fe_fmt = XTRXLL_FE_DONTTOUCH;
	unsigned        rx_bpkt_size = 0;

	wts_long_t      rx_start_ts = 0;

	unsigned        rx_stream_count = 0;
	unsigned        tx_stream_count = 0;

	/* at least one direction must be specified */
	if (!(params->dir & XTRX_TRX)) {
		return -EINVAL;
	}

	if (params->dir & XTRX_RX) {
		if ((params->rx.wfmt == XTRX_WF_8 && params->rx.hfmt == XTRX_IQ_INT16) ||
				(params->rx.wfmt == XTRX_WF_16 && params->rx.hfmt == XTRX_IQ_INT8) ||
				(params->rx.wfmt == XTRX_WF_12 && params->rx.hfmt != XTRX_IQ_FLOAT32)) {

			XTRXLL_LOG(XTRXLL_ERROR, "Specified combination of host and wire formats isn't supported for RX stream\n");
			return -EINVAL;
		}

		rx_bpkt_size = calculate_pkt_size(&params->rx) << dev->rx_host_decim;
		if (params->rx.hfmt != XTRX_IQ_FLOAT32 && (params->rx.flags & XTRX_RSP_SCALE)) {
			XTRXLL_LOG(XTRXLL_ERROR, "XTRX_RSP_SCALE is supported only for XTRX_IQ_FLOAT32 host format in RX stream\n");
			return -EINVAL;
		}

		if (params->rx.flags & XTRX_STREAMDSP_1)
			rx_mode_flags |= XTRXLL_FE_MODE_RXDSP_MODE1;
		if (params->rx.flags & XTRX_STREAMDSP_2)
			rx_mode_flags |= XTRXLL_FE_MODE_RXDSP_MODE2;

		rx_mode = (xtrx_run_params_stream_is_mimo(&params->rx)) ? XTRXLL_FE_MODE_MIMO
																: XTRXLL_FE_MODE_SISO;
		rx_fe_fmt = (params->rx.wfmt == XTRX_WF_8) ? XTRXLL_FE_8BIT :
					(params->rx.wfmt == XTRX_WF_12) ? XTRXLL_FE_12BIT : XTRXLL_FE_16BIT;
		rx_stream_count = (rx_mode == XTRXLL_FE_MODE_SISO) ? 1 : 2;
	}

	if (params->dir & XTRX_TX) {
		if ((params->tx.wfmt != XTRX_WF_16 || params->tx.hfmt == XTRX_IQ_INT8)) {
			XTRXLL_LOG(XTRXLL_ERROR, "Specified combination of host and wire formats isn't supported for TX stream\n");
			return -EINVAL;
		}

		if (params->tx.hfmt != XTRX_IQ_FLOAT32 && (params->tx.flags & XTRX_RSP_SCALE)) {
			XTRXLL_LOG(XTRXLL_ERROR, "XTRX_RSP_SCALE is supported only for XTRX_IQ_FLOAT32 host format in TX stream\n");
			return -EINVAL;
		}

		if (params->tx.flags & XTRX_STREAMDSP_1)
			return -EINVAL;
		if (params->tx.flags & XTRX_STREAMDSP_2)
			return -EINVAL;

		tx_mode = (xtrx_run_params_stream_is_mimo(&params->tx)) ? XTRXLL_FE_MODE_MIMO
																: XTRXLL_FE_MODE_SISO;
		tx_fe_fmt = (params->tx.wfmt == XTRX_WF_8) ? XTRXLL_FE_8BIT :
					(params->tx.wfmt == XTRX_WF_12) ? XTRXLL_FE_12BIT : XTRXLL_FE_16BIT;
		tx_stream_count = (tx_mode == XTRXLL_FE_MODE_SISO) ? 1 : 2;
	}

	struct xtrx_dd_params fe_params;
	fe_params.dir = params->dir;
	fe_params.nflags = params->nflags;
	fe_params.rx.chs = params->rx.chs;
	fe_params.rx.flags = params->rx.flags;
	fe_params.tx.chs = params->tx.chs;
	fe_params.tx.flags = params->tx.flags;

	res = dev->fe->ops->dd_set_modes(dev->fe, XTRX_FEDD_CONFIGURE, &fe_params);
	if (res)
		return res;

	/* Validation of input parameters done, do the things */
	if (params->dir & XTRX_RX) {
		rx_start_ts = dev->rx_samples = params->rx_stream_start << dev->rx_host_decim;

		if (!dev->rxinit) {
			res = xtrxll_dma_rx_init(dev->lldev, chan, rx_bpkt_size, &rx_bpkt_size);
			if (res) {
				goto failed_fe;
			}
			XTRXLL_LOG(XTRXLL_INFO, "RX ititialized to %d bytes paket size\n", rx_bpkt_size);
			dev->rxinit = 1;
		}

		if (dev->rx_host_decim) {
			unsigned i;
			if (dev->rx_host_decim != 1) {
				res = -EINVAL;
				goto failed_fe;
			}

			if (params->rx.hfmt == XTRX_IQ_INT16) {
				for (res = 0, i = 0; i < rx_stream_count; i++) {
					res |=  xtrxdsp_filter_initi(g_filter_int16_taps_64_2x,
												 FILTER_TAPS_64,
												 dev->rx_host_decim,
												 0,
												 rx_bpkt_size / xtrx_wire_format_get_iq_size(params->rx.wfmt),
												 &dev->rx_host_filter[i]);
				}
			} else if (params->rx.hfmt == XTRX_IQ_FLOAT32) {
				for (res = 0, i = 0; i < rx_stream_count; i++) {
					res |=  xtrxdsp_filter_init(g_filter_float_taps_64_2x,
												FILTER_TAPS_64,
												dev->rx_host_decim,
												0,
												rx_bpkt_size / xtrx_wire_format_get_iq_size(params->rx.wfmt),
												&dev->rx_host_filter[i]);
				}
			} else {
				XTRXLL_LOG(XTRXLL_ERROR, "This RX host format is usnsupported for host filtering\n");
				res = -EINVAL;
				goto failed_fe;
			}

			if (res) {
				XTRXLL_LOG(XTRXLL_ERROR, "Unable to initialize host decimation/filtering err=%d\n", res);
				goto filter_cleanup;
			}
		}

		// Accumulation isn't supported yet, so hardwire it
		dev->rx_scale_16 = 1.0f / 2048;
		if (params->rx.flags & XTRX_RSP_SCALE) {
			dev->rx_scale_16 *= params->rx.scale;
		}

		dev->rxbuf = NULL;
		dev->rxbuf_total = 0;
		dev->rxbuf_processed = 0;
		dev->rxbuf_processed_ts = 0;
		dev->rxbuf_conv_state = 0;
		dev->rxbuf_len_iq_symbol = rx_stream_count * xtrx_wire_format_get_iq_size(params->rx.wfmt);
		dev->rxbuf_len_iq_host_sym = rx_stream_count * xtrx_host_format_get_iq_size(params->rx.hfmt);
		dev->rx_chans_in_stream = rx_stream_count;
		dev->rx_run = true;
		dev->rx_hostfmt = params->rx.hfmt;
		dev->rx_busfmt = params->rx.wfmt;
		dev->rx_fefmt = rx_fe_fmt;
	}

	if (params->dir & XTRX_TX) {
		if (!dev->txinit) {
			res = xtrxll_dma_tx_init(dev->lldev, chan, DEF_TX_BUFSIZE);
			if (res) {
				goto failed_tx_init;
			}
			dev->txinit = 1;
		}

		if (params->tx_repeat_buf) {
			if (params->tx.paketsize < 4) {
				return -EINVAL;
			}
			unsigned ssz = (tx_mode == XTRXLL_FE_MODE_MIMO) ? params->tx.paketsize * 2 :
															  params->tx.paketsize;
			res = xtrxll_repeat_tx_buf(dev->lldev,
									   chan,
									   tx_fe_fmt,
									   params->tx_repeat_buf,
									   ssz,
									   tx_mode);
			if (res) {
				goto failed_tx_init;
			}
		}

		dev->tx_pkt_samples = params->tx.paketsize << dev->tx_host_inter;
		if (dev->tx_pkt_samples == 0) {
			//dev->tx_pkt_samples = (MAX_TX_MSPS / tx_stream_count) >> dev->tx_host_inter;
			dev->tx_pkt_samples = (MAX_TX_MSPS / tx_stream_count);

		} else if (((dev->tx_pkt_samples * tx_stream_count)) > MAX_TX_MSPS ) {
			XTRXLL_LOG(XTRXLL_WARNING, "hardware TX burst size is too high: PKT:%d, lowering to %d\n",
					   dev->tx_pkt_samples, MAX_TX_MSPS / tx_stream_count);
			//res = -EINVAL;
			//goto failed_tx_init;

			dev->tx_pkt_samples = (MAX_TX_MSPS / tx_stream_count);
		}

		dev->tx_scale_16 = 32767;
		if (params->tx.flags & XTRX_RSP_SCALE) {
			dev->tx_scale_16 /= params->tx.scale;
		}

		if (dev->tx_host_inter) {
			unsigned i;

			if (params->tx_repeat_buf) {
				XTRXLL_LOG(XTRXLL_ERROR, "Host filtering doesn't work with repeat buffer\n");
				res = -EINVAL;
				goto failed_tx_init;
			}
			if (dev->tx_host_inter > 1) {
				res = -EINVAL;
				goto failed_tx_init;
			}

			if (params->tx.hfmt == XTRX_IQ_INT16) {
				for (res = 0, i = 0; i < tx_stream_count; i++) {
					res |= xtrxdsp_filter_initi(g_filter_int16_taps_64_2x,
												FILTER_TAPS_64,
												0,
												dev->tx_host_inter,
												dev->tx_pkt_samples,
												&dev->tx_host_filter[i]);
				}
			} else if (params->tx.hfmt == XTRX_IQ_FLOAT32) {
				for (res = 0, i = 0; i < tx_stream_count; i++) {
					res |= xtrxdsp_filter_init(g_filter_float_taps_64_2x,
											   FILTER_TAPS_64,
											   0,
											   dev->tx_host_inter,
											   dev->tx_pkt_samples,
											   &dev->tx_host_filter[i]);
				}
			} else {
				XTRXLL_LOG(XTRXLL_ERROR, "This TX host format is usnsupported for host filtering\n");
				res = -EINVAL;
				goto failed_tx_init;
			}
			if (res) {
				XTRXLL_LOG(XTRXLL_ERROR, "Unable to initialize host interpolation/filtering err=%d\n", res);
			}
		}

		dev->txskip_time = 0;
		dev->txburt_late_prev = 0;
		dev->txbuf = NULL;
		dev->txbuf_total = 0;
		dev->txbuf_conv_state = 0;
		dev->tx_chans_in_stream = tx_stream_count;
		dev->txbuf_len_iq_symbol = dev->tx_chans_in_stream * xtrx_wire_format_get_iq_size(params->tx.wfmt);
		dev->txbuf_len_iq_host_sym = dev->tx_chans_in_stream * xtrx_host_format_get_iq_size(params->tx.hfmt);

		dev->tx_run = true;
		dev->tx_hostfmt = params->tx.hfmt;
		dev->tx_busfmt = params->tx.wfmt;
		dev->tx_fefmt = tx_fe_fmt;
	}

	res = xtrxll_dma_start(dev->lldev,
						   chan,
						   rx_fe_fmt,
						   (xtrxll_mode_t)(rx_mode | rx_mode_flags),
						   rx_start_ts,
						   tx_fe_fmt,
						   tx_mode);
	if (res) {
		XTRXLL_LOG(XTRXLL_ERROR, "Unable to start DMA err=%d\n", res);
		goto fail_dma_start;
	}

	return 0;

fail_dma_start:
failed_tx_init:
	dev->rx_run = false;
	dev->tx_run = false;

filter_cleanup:
	xtrxdsp_filter_free(&dev->rx_host_filter[0]);
	xtrxdsp_filter_free(&dev->rx_host_filter[1]);
	xtrxdsp_filter_free(&dev->tx_host_filter[0]);
	xtrxdsp_filter_free(&dev->tx_host_filter[1]);

failed_fe:
	dev->fe->ops->dd_set_modes(dev->fe, XTRX_FEDD_RESET, &fe_params);
	return res;
}

int xtrx_stop(struct xtrx_dev* dev, xtrx_direction_t dir)
{
	int res;
	const int chan = 0; //TODO

	if (dir & XTRX_RX) {
		res = xtrxll_dma_rx_start(dev->lldev, 0, XTRXLL_FE_STOP);

		dev->rxbuf = NULL;
		dev->rxbuf_total = 0;
		dev->rxbuf_processed = 0;
		dev->rxbuf_processed_ts = 0;
		dev->rx_run = false;

		xtrxdsp_filter_free(&dev->rx_host_filter[0]);
		xtrxdsp_filter_free(&dev->rx_host_filter[1]);
	}

	if (dir & XTRX_TX) {
		res = xtrxll_dma_tx_start(dev->lldev, chan, XTRXLL_FE_STOP, XTRXLL_FE_MODE_MIMO);

		dev->txbuf = NULL;
		dev->txbuf_total = 0;
		dev->tx_run = false;

		xtrxdsp_filter_free(&dev->tx_host_filter[0]);
		xtrxdsp_filter_free(&dev->tx_host_filter[1]);
	}

	struct xtrx_dd_params fe_params;
	fe_params.dir = dir;
	fe_params.nflags = 0;
	fe_params.rx.chs = XTRX_CH_AB;
	fe_params.rx.flags = 0;
	fe_params.tx.chs = XTRX_CH_AB;
	fe_params.tx.flags = 0;

	res = dev->fe->ops->dd_set_modes(dev->fe, XTRX_FEDD_RESET, &fe_params);
	if (res)
		return res;

	res = xtrxll_dma_start(dev->lldev, chan,
						   (dir & XTRX_RX) ? XTRXLL_FE_STOP : XTRXLL_FE_DONTTOUCH, XTRXLL_FE_MODE_MIMO,
						   0,
						   (dir & XTRX_TX) ? XTRXLL_FE_STOP : XTRXLL_FE_DONTTOUCH, XTRXLL_FE_MODE_MIMO);
	return res;
}

#define MIN(x,y) (((x) < (y)) ? (x) : (y))
#define MAKE_FORMAT(wire, format) (((wire) << 8) | (format))
enum processing_type {
	WIRE_I8_HOST_I8    = MAKE_FORMAT(XTRX_WF_8,  XTRX_IQ_INT8),
	WIRE_I8_HOST_I16   = MAKE_FORMAT(XTRX_WF_8,  XTRX_IQ_INT16),
	WIRE_I8_HOST_F32   = MAKE_FORMAT(XTRX_WF_8,  XTRX_IQ_FLOAT32),
	WIRE_I12_HOST_F32  = MAKE_FORMAT(XTRX_WF_12, XTRX_IQ_FLOAT32),
	WIRE_I16_HOST_F32  = MAKE_FORMAT(XTRX_WF_16, XTRX_IQ_FLOAT32),
	WIRE_I16_HOST_I16  = MAKE_FORMAT(XTRX_WF_16, XTRX_IQ_INT16),
};

int xtrx_send_sync_ex(struct xtrx_dev* dev, xtrx_send_ex_info_t *info)
{
	if (!dev->tx_run) {
		XTRXLL_LOG(XTRXLL_ERROR, "xtrx_send_sync_ex: TX stream is not configured\n");
		return -ENOSTR;
	}

	const int chan = 0;
	/* Total numer of sample in all channels */
	size_t total_samples = info->samples * info->buffer_count;
	/* Total bytes user requested */
	size_t user_total = total_samples * dev->txbuf_len_iq_host_sym / dev->tx_chans_in_stream;
	if (user_total == 0 || info->buffer_count > 2)
		return -EINVAL;

	/* Total bytes satisfied from user */
	size_t user_processed = 0;
	int res;

	/* expanding from host encoding to the wire format times 2, we need this
	   to present 12-bit wire format in integer */
	unsigned encode_amplification_x2 = 2 * dev->txbuf_len_iq_host_sym / dev->txbuf_len_iq_symbol;

	uint16_t cur_late;//, upd_late;
	wts_long_t cur_wts = info->ts << dev->tx_host_inter;

	info->out_flags = 0;
	info->out_samples = 0;

	unsigned timeout_ms = (info->flags & XTRX_TX_TIMEOUT) ? info->timeout : ~0UL;

	for (;;) {
		void* wire_buffer_ptr;
		size_t wire_buffer_size; /* maximum DMA buffer size in bytes */

		if (dev->txbuf == NULL) {
			for (;;) {
				res = xtrxll_dma_tx_getfree_ex(dev->lldev, chan, &wire_buffer_ptr, &cur_late, timeout_ms);
				switch (res) {
				//case -EPIPE:
				default:
					abort();
				case -EIO:
				case -EBUSY:
					return res;
				case 0:
					dev->txbuf = wire_buffer_ptr;
					goto got_buffer;
				}
			}
got_buffer:
			dev->txbuf = wire_buffer_ptr;
			dev->txbuf_total = wire_buffer_size = dev->tx_pkt_samples * dev->txbuf_len_iq_symbol;
			dev->txbuf_processed = 0;
			dev->txbuf_processed_ts = 0;

			if ((info->flags & XTRX_TX_NO_DISCARD) == 0) {
				uint16_t upd_late = cur_late - dev->txburt_late_prev;
				if (dev->txskip_time > cur_wts) {
					XTRXLL_LOG(XTRXLL_ERROR, "xtrx_send_burst_sync: TX DMA Current skip due to TO buffers: %d, prev: %d TS:%" PRId64 " STPS:%" PRId64 "\n",
							   cur_late, dev->txburt_late_prev, cur_wts, dev->txskip_time);
					dev->txburt_late_prev = cur_late;

					info->out_flags |= XTRX_TX_DISCARDED_TO;
					return 0;
				}

				if (upd_late > 8) {
					XTRXLL_LOG(XTRXLL_ERROR, "xtrx_send_burst_sync: TX DMA Current delayed buffers: %d, prev: %d TS:%" PRId64 "\n",
							   cur_late, dev->txburt_late_prev, cur_wts);
					dev->txburt_late_prev = cur_late;
					dev->txskip_time = cur_wts + 4*info->samples; // Discard  2x packets

					info->out_flags |= XTRX_TX_DISCARDED_TO;
					return 0;
				}
			}
		} else {
			cur_late = dev->txburt_late_prev;

			/* Use cached buffer, just move pointers to non processed data */
			wire_buffer_ptr = (void*)((char*)dev->txbuf + dev->txbuf_processed);
			wire_buffer_size = dev->txbuf_total - dev->txbuf_processed;
		}

		XTRXLL_LOG(XTRXLL_DEBUG, "xtrx_send_burst_sync: Total=%u Processed=%u PTS=%" PRId64
				   " cwts=%" PRId64 " UserTotal=%u UserProcessed=%u\n",
				   dev->txbuf_total, dev->txbuf_processed, dev->txbuf_processed_ts, cur_wts,
				   (unsigned)user_total, (unsigned)user_processed);

		/* bytes need to fill from user */
		size_t remaining = user_total - user_processed;

		// TODO Move this buffer away from stack
#define TMP_SIZE	32768
		static float tmp_buffer[TMP_SIZE];
		float* tmp_buffer_p[2] = { tmp_buffer, tmp_buffer + TMP_SIZE/2 };

		const void* usr_buf[2] = { (info->buffer_count == 1) ?
								   info->buffers[0] + user_processed : info->buffers[0] + user_processed / 2,
								   (info->buffer_count == 1) ?
								   NULL : info->buffers[1] + user_processed / 2 };
		const void* enc_buf[2];
		unsigned i;
		/* processed data in the current cycle */
		unsigned user_cur_proceesed;
		/* Number of bytes consumed from the device buffer */
		size_t wire_bytes_consumed;
		/* Number of full samples consumed from the buffer (in each device channel) */
		size_t wire_samples_consumed;

		/* Host filtration if needed */
		if (dev->tx_host_filter[0].filter_taps || (info->flags & XTRX_TX_SEND_ZEROS)) {
			enc_buf[0] = (void*)tmp_buffer_p[0];
			enc_buf[1] = (void*)tmp_buffer_p[1];
		} else {
			enc_buf[0] = usr_buf[0];
			enc_buf[1] = usr_buf[1];
		}

		/* total number of bytes that we'll consume from DMA buffer in the
		 * current cycle (for all channels) */
		wire_bytes_consumed = MIN(wire_buffer_size,
								  (remaining << (dev->tx_host_inter + 1)) / encode_amplification_x2);

		/* number of bytes to be filled for the user */
		user_cur_proceesed = encode_amplification_x2 * wire_bytes_consumed >> (dev->tx_host_inter + 1);

		/* number of sample consumed in each channel in the device stream */
		wire_samples_consumed = wire_bytes_consumed / dev->txbuf_len_iq_symbol;

		if (info->flags & XTRX_TX_SEND_ZEROS) {
			for (i = 0; i < info->buffer_count; i++) {
				memset(tmp_buffer_p[i], 0, wire_bytes_consumed / info->buffer_count);
			}
		}

		/* host filtration & interpolation */
		if (dev->tx_host_filter[0].filter_taps) {
			unsigned num_fsamp = ((2 * dev->tx_chans_in_stream * wire_samples_consumed) >> dev->tx_host_inter) / info->buffer_count;
			switch (dev->tx_hostfmt) {
			case XTRX_IQ_FLOAT32:
				for (i = 0; i < info->buffer_count; i++) {
					res = xtrxdsp_filter_work(&dev->tx_host_filter[i],
											  (const float*)usr_buf[i],
											  (float*)enc_buf[i],
											  num_fsamp);
				}
				break;
			case XTRX_IQ_INT16:
				for (i = 0; i < info->buffer_count; i++) {
					res = xtrxdsp_filter_worki(&dev->tx_host_filter[i],
											   (const int16_t *)usr_buf[i],
											   (int16_t*)enc_buf[i],
											   num_fsamp);
				}
				break;
			default:
				abort();
			}
		}


		switch (MAKE_FORMAT(dev->tx_busfmt, dev->tx_hostfmt)) {
		case WIRE_I16_HOST_I16:
			if (info->buffer_count == 1) {
				memcpy(wire_buffer_ptr,
					   enc_buf[0],
						wire_bytes_consumed);
			} else {
				xtrxdsp_ic16i_iq16(enc_buf[0],
						enc_buf[1],
						wire_buffer_ptr,
						wire_bytes_consumed);
			}
			break;
		case WIRE_I16_HOST_F32:
			if (info->buffer_count == 1) {
				xtrxdsp_sc32_iq16(enc_buf[0],
						wire_buffer_ptr,
						dev->tx_scale_16,
						wire_bytes_consumed);
			} else {
				xtrxdsp_sc32i_iq16(enc_buf[0],
						enc_buf[1],
						wire_buffer_ptr,
						dev->tx_scale_16,
						wire_bytes_consumed);
			}
			break;
		default:
			abort();
		}

		dev->txbuf_processed_ts += wire_samples_consumed;
		dev->txbuf_processed += wire_bytes_consumed;

		user_processed += user_cur_proceesed;
		info->out_samples += (dev->tx_chans_in_stream * wire_samples_consumed >> dev->tx_host_inter) / info->buffer_count;

		if (dev->txbuf_processed == dev->txbuf_total ||
				((user_processed == user_total) && (info->flags & XTRX_TX_DONT_BUFFER))) {
			res = xtrxll_dma_tx_post(dev->lldev,
									 chan,
									 dev->txbuf,
									 (dev->tx_chans_in_stream == 1) ? cur_wts / 2 : cur_wts,
									 dev->txbuf_processed / 8);
			dev->txbuf = NULL;
			if (res) {
				XTRXLL_LOG(XTRXLL_ERROR, "xtrxll_dma_tx_post res=%d (wts=%" PRIu64 " samples=%u)\n",
						   res, cur_wts, (unsigned)wire_samples_consumed);
				return res;
			}
		}

		/* User request satisfied */
		if (user_processed == user_total) {
			return 0;
		}

		cur_wts += dev->txbuf_processed_ts;
	}

	return -EINVAL;
}

int xtrx_recv_sync_ex(struct xtrx_dev* dev, xtrx_recv_ex_info_t* info)
{
	if (!dev->rx_run) {
		XTRXLL_LOG(XTRXLL_ERROR, "xtrx_recv_sync_ex: RX stream is not configured\n");
		return -ENOSTR;
	}

	const int chan = 0;
	/* One sample in all channels in bytes */
	size_t total_samples = info->samples * info->buffer_count;
	/* Total bytes user requested */
	size_t user_total = total_samples * dev->rxbuf_len_iq_host_sym  / dev->rx_chans_in_stream;
	if (user_total == 0 || info->buffer_count > 2)
		return -EINVAL;

	/* Total bytes satisfied for user */
	size_t user_processed = 0;

	xtrxll_dma_rx_flags_t flags = 0;
	int res;

	/* expanding from wire decoding to the host format times 2 */
	unsigned decode_amplification_x2 = 2 * dev->rxbuf_len_iq_host_sym / dev->rxbuf_len_iq_symbol;

	info->out_samples = 0;
	info->out_events = 0;

	if (info->flags & RCVEX_DONT_WAIT)
		flags |= XTRXLL_RX_DONTWAIT;

	if (info->flags & RCVEX_EXTRA_LOG)
		flags |= XTRXLL_RX_FORCELOG;

	if (info->flags & RCVEX_TIMOUT)
		flags |= XTRXLL_RX_REPORT_TIMEOUT;

	for (;;) {
		void* wire_buffer_ptr;
		unsigned wire_buffer_size; /* in bytes */

		if (!dev->rxbuf) {
			/* Don't have any cached buffer, need to obtain one */
			for (;;) {
				res = xtrxll_dma_rx_getnext(dev->lldev,
											chan,
											&wire_buffer_ptr,
											&dev->rxbuf_ts,
											&wire_buffer_size,
											flags,
											info->timeout);
				switch (res) {
				case -EOVERFLOW:
					info->out_events |= RCVEX_EVENT_OVERFLOW;
					info->out_resumed_at = dev->rxbuf_ts >> dev->rx_host_decim;
					info->out_overrun_at = dev->rx_samples >> dev->rx_host_decim;

					if (info->flags & RCVEX_STOP_ON_OVERRUN) {
						return -EOVERFLOW;
					}

					xtrxll_dma_rx_resume_at(dev->lldev, chan, dev->rxbuf_ts);
					continue;
				case -EINTR: /* operation was cancelled */
				case -EAGAIN:
				case -EPIPE: /* logic error! */
					return res;
				case 0:
					goto got_buffer;

				default:
					// Other non-handles error
					abort();
				}
			}
got_buffer:

			dev->rxbuf = wire_buffer_ptr;
			dev->rxbuf_total = wire_buffer_size;
			dev->rxbuf_processed = 0;
			dev->rxbuf_processed_ts = 0;
		} else {
			/* Use cached buffer, just move pointers to non processed data */
			wire_buffer_ptr = (void*)((char*)dev->rxbuf + dev->rxbuf_processed);
			wire_buffer_size = dev->rxbuf_total - dev->rxbuf_processed;
		}

		if (user_processed == 0) {
			info->out_first_sample = dev->rx_samples >> dev->rx_host_decim;
		}

		XTRXLL_LOG((dev->rxbuf_ts + dev->rxbuf_processed_ts != dev->rx_samples) ? XTRXLL_WARNING : XTRXLL_DEBUG,
				   "xtrx_recv_sync: Total=%u Processed=%u UserTotal=%u UserProcessed=%u BUFTS=%" PRIu64 "+%" PRIu64 " OURTS=%" PRIu64 "\n",
				   dev->rxbuf_total, dev->rxbuf_processed, (unsigned)user_total, (unsigned)user_processed,
				   dev->rxbuf_ts, dev->rxbuf_processed_ts, dev->rx_samples);

		/* bytes need to fill in user */
		size_t remaining = user_total - user_processed;

		// TODO Move this buffer away from stack
#define TMP_SIZE	32768
		static float tmp_buffer[TMP_SIZE];
		float* tmp_buffer_p[2] = { tmp_buffer, tmp_buffer + TMP_SIZE/2 };

		void* usr_buf[2] = { (info->buffer_count == 1) ?
							 info->buffers[0] + user_processed : info->buffers[0] + user_processed / 2,
							 (info->buffer_count == 1) ?
							 NULL : info->buffers[1] + user_processed / 2 };
		void* dst_buf[2];
		unsigned i;
		/* processed data in the current cycle */
		unsigned user_cur_proceesed;
		/* Number of bytes consumed from the device buffer */
		size_t wire_bytes_consumed;
		/* Number of full samples consumed from the buffer */
		size_t wire_samples_consumed;

		/* Host filtration if needed */
		if (dev->rx_host_filter[0].filter_taps) {
			dst_buf[0] = (void*)tmp_buffer_p[0];
			dst_buf[1] = (void*)tmp_buffer_p[1];
		} else {
			dst_buf[0] = usr_buf[0];
			dst_buf[1] = usr_buf[1];
		}

		if (dev->rxbuf_ts + dev->rxbuf_processed_ts > dev->rx_samples) {
			if (info->flags & RCVEX_DONT_INSER_ZEROS) {
				/* User asked to ignore lost packets, so we just fast forward next
				 * packet. Well in case of host filtering it's a bad idea
				 */
				dev->rx_samples = dev->rxbuf_ts + dev->rxbuf_processed_ts;
				if (user_processed == 0) {
					info->out_first_sample = dev->rx_samples >> dev->rx_host_decim;
				} else {
					/* Lost data in the middle of filling, no concatenation */
					return 0;
				}
			} else {
				/* Fill with zeroes */
				uint64_t gap_ts = dev->rxbuf_ts + dev->rxbuf_processed_ts - dev->rx_samples;
				/* set as if we received this number of zeros from wire */
				wire_buffer_size = gap_ts * dev->rxbuf_len_iq_host_sym;
			}
		}

		/* total number of bytes that we'll consume from DMA buffer in the
		 * current cycle */
		wire_bytes_consumed = MIN(wire_buffer_size,
								  (remaining << (dev->rx_host_decim + 1)) / decode_amplification_x2);

		/* number of bytes to be filled for the user */
		user_cur_proceesed = decode_amplification_x2 * wire_bytes_consumed >> (dev->rx_host_decim + 1);

		/* number of sample consumed in each logical channel in the device stream */
		wire_samples_consumed = wire_bytes_consumed / dev->rxbuf_len_iq_symbol;


		if (dev->rxbuf_ts + dev->rxbuf_processed_ts > dev->rx_samples) {
			for (i = 0; i < info->buffer_count; i++) {
				memset(dst_buf[i], 0, user_cur_proceesed / info->buffer_count);
			}
			info->out_events |= RCVEX_EVENT_FILLED_ZERO;
		} else {
			int datafmt = MAKE_FORMAT(dev->rx_busfmt, dev->rx_hostfmt);
			switch (datafmt) {
			case WIRE_I8_HOST_I8:
			case WIRE_I16_HOST_I16:
				if (info->buffer_count == 1) {
					memcpy(dst_buf[0],
							wire_buffer_ptr,
							wire_bytes_consumed);
				} else if (datafmt == WIRE_I16_HOST_I16) {
					xtrxdsp_iq16_ic16i(wire_buffer_ptr,
									   dst_buf[0],
							dst_buf[1],
							wire_bytes_consumed);
				} else if (datafmt == WIRE_I8_HOST_I8) {
					xtrxdsp_iq8_ic8i(wire_buffer_ptr,
									   dst_buf[0],
							dst_buf[1],
							wire_bytes_consumed);
				}
				break;
			case WIRE_I8_HOST_I16:
				if (info->buffer_count == 1) {
					xtrxdsp_iq8_ic16(wire_buffer_ptr,
									 dst_buf[0],
							wire_bytes_consumed);
				} else if (datafmt == WIRE_I16_HOST_I16) {
					xtrxdsp_iq8_ic16i(wire_buffer_ptr,
									   dst_buf[0],
							dst_buf[1],
							wire_bytes_consumed);
				}
				break;
			case WIRE_I8_HOST_F32:
				if (info->buffer_count == 1) {
					xtrxdsp_iq8_sc32(wire_buffer_ptr,
									 dst_buf[0],
							wire_bytes_consumed);
				} else {
					xtrxdsp_iq8_sc32i(wire_buffer_ptr,
									  dst_buf[0],
							dst_buf[1],
							wire_bytes_consumed);
				}
				break;
			case WIRE_I16_HOST_F32:
				if (info->buffer_count == 1) {
					xtrxdsp_iq16_sc32(wire_buffer_ptr,
									  dst_buf[0],
							dev->rx_scale_16,
							wire_bytes_consumed);
				} else {
					xtrxdsp_iq16_sc32i(wire_buffer_ptr,
									   dst_buf[0],
							dst_buf[1],
							dev->rx_scale_16,
							wire_bytes_consumed);
				}
				break;
			case WIRE_I12_HOST_F32:
			default:
				// TODO generelize call to 12bit convertion to not cross DMA
				// buffer block boundary, removed old code for now
				abort();
			}

			dev->rxbuf_processed_ts += wire_samples_consumed;
			dev->rxbuf_processed += wire_bytes_consumed;

			if (dev->rxbuf_processed == dev->rxbuf_total) {
				xtrxll_dma_rx_release(dev->lldev, chan, dev->rxbuf);
				dev->rxbuf = NULL;
			}
		}

		dev->rx_samples += wire_samples_consumed;
		info->out_samples += (dev->rx_chans_in_stream * wire_samples_consumed >> dev->rx_host_decim) / info->buffer_count;


		if (dev->rx_host_filter[0].filter_taps) {
			unsigned num_fsamp = 2 * dev->rx_chans_in_stream * wire_samples_consumed / info->buffer_count;
			switch (dev->rx_hostfmt) {
			case XTRX_IQ_FLOAT32:
				for (i = 0; i < info->buffer_count; i++) {
					res = xtrxdsp_filter_work(&dev->rx_host_filter[i],
											  (const float*)dst_buf[i],
											  (float*)usr_buf[i],
											  num_fsamp);
				}
				break;
			case XTRX_IQ_INT16:
				for (i = 0; i < info->buffer_count; i++) {
					res = xtrxdsp_filter_worki(&dev->rx_host_filter[i],
											   (const int16_t *)dst_buf[i],
											   (int16_t*)usr_buf[i],
											   num_fsamp);
				}
				break;
			default:
				abort();
			}
		}

		user_processed += user_cur_proceesed;

		/* User request satisfied */
		if (user_processed == user_total) {
			return 0;
		}
	}

	return -EINVAL;
}

void xtrx_set_logfunction(xtrx_logfunc_t func)
{
	xtrxll_set_logfunc(func);
}


int xtrx_val_set(struct xtrx_dev* dev, xtrx_direction_t dir,
				 xtrx_channel_t chan, xtrx_val_t type, uint64_t val)
{
	if (type >= XTRX_RFIC_REG_0 && type <= XTRX_RFIC_REG_0 + 65535)	{
		return dev->fe->ops->set_reg(dev->fe, chan, dir, type, val);
	}

	switch (type) {
	case XTRX_LML_PHY_PHASE:
		XTRXLL_LOG(XTRXLL_INFO, "Set LMS7 LML FCLK Phase to %d\n", (int)val);
		return xtrxll_mmcm_fphase_corr(dev->lldev, true, val, false);
	case XTRX_LML_PHY_FBPHASE:
		XTRXLL_LOG(XTRXLL_INFO, "Set LMS7 LML FB Phase to %d\n", (int)val);
		return xtrxll_mmcm_fphase_corr(dev->lldev, true, val, true);
	case XTRX_LMS7_PWR_MODE:
		XTRXLL_LOG(XTRXLL_INFO, "Set LMS7 power mode to %d\n", (int)val);
		return xtrxll_set_param(dev->lldev, XTRXLL_PARAM_PWR_MODE, val);
	case XTRX_LMS7_VIO:
		XTRXLL_LOG(XTRXLL_INFO, "Set LMS7 VIO to %d\n", (int)val);
		return xtrxll_set_param(dev->lldev, XTRXLL_PARAM_PWR_VIO, val);
	case XTRX_LMS7_XSP_DC_IQ:
		return dev->fe->ops->set_reg(dev->fe, chan, dir, type, val);
	case XTRX_VCTCXO_DAC_VAL:
		XTRXLL_LOG(XTRXLL_INFO, "Set XTRX DAC %d\n", (int)val);
		return xtrxll_set_param(dev->lldev, XTRXLL_PARAM_REF_DAC, val);
	case XTRX_DSPFE_CMD:
		return xtrxll_set_param(dev->lldev, XTRXLL_PARAM_DSPFE_CMD, val);
	default:
		return -EINVAL;
	}
}

XTRX_API int xtrx_val_get(struct xtrx_dev* dev, xtrx_direction_t dir,
						  xtrx_channel_t chan, xtrx_val_t type, uint64_t* oval)
{
	int res, val;

	if (type >= XTRX_RFIC_REG_0 && type <= XTRX_RFIC_REG_0 + 65535)	{
		return dev->fe->ops->get_reg(dev->fe, chan, dir, type, oval);
	};

	switch (type) {
	case XTRX_UNDERLYING_LL:
		*oval = (intptr_t)dev->lldev;
		return 0;
	case XTRX_BOARD_TEMP:
		res = xtrxll_get_sensor(dev->lldev, XTRXLL_TEMP_SENSOR_CUR, &val);
		if (!res) {
			*oval = val;
		}
		return res;
	case XTRX_OSC_LATCH_1PPS:
		res = xtrxll_get_sensor(dev->lldev, XTRXLL_OSC_LATCHED, &val);
		if (!res) {
			*oval = val;
		}
		return res;
	case XTRX_WAIT_1PPS:
		res = xtrxll_get_sensor(dev->lldev, XTRXLL_ONEPPS_CAPTURED, &val);
		if (!res) {
			*oval = val;
		}
		return res;

	case XTRX_LMS7_DATA_RATE:
	case XTRX_LMS7_RSSI:
		return dev->fe->ops->get_reg(dev->fe, chan, dir, type, oval);
	case XTRX_PERF_LLFIFO:
		if (dir == XTRX_RX) {
			res = xtrxll_get_sensor(dev->lldev, XTRXLL_DMABUF_RXST64K, &val);
		} else if (dir == XTRX_TX) {
			res = xtrxll_get_sensor(dev->lldev, XTRXLL_DMABUF_TXST64K, &val);
		} else {
			return -EINVAL;
		}
		if (!res) {
			*oval = val;
		}
		return res;
	default:
		return -EINVAL;
	}
}
