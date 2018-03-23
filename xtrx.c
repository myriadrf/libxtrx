/*
 * xtrx high level source file
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
#include "xtrx_api.h"
#include <xtrxll_api.h>
#include <xtrxll_mmcm.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <inttypes.h>
#include <LMS7002M/LMS7002M.h>
#include <LMS7002M/LMS7002M_logger.h>
#include <xtrxdsp.h>
#include <xtrxdsp_filters.h>
#include <xtrxll_log.h>

#ifdef USE_EXT_CALIBRATION
#include <liblms7_calibrations.h>
#endif

enum {
	MAX_LMS = 4,
	DEF_BUFSIZE = 32768,
	DEF_RX_BUFSIZE = DEF_BUFSIZE,
	DEF_TX_BUFSIZE = DEF_BUFSIZE,

	MIN_TX_RATE = 2100000, /* 2.1e6 Minimum samplerate supported by the XTRX hardware */
};

typedef double clock_val_t;

struct lms7_struct {
	unsigned         lmsno;
	int              lasterr;
	LMS7002M_t*      lms7;
	struct xtrx_dev* head;
};

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
	unsigned            ant;
	unsigned            chans_in_stream; /**< Number of logical channels multiplexed into the device stream */

	float               scale_16;
	xtrx_host_format_t  hostfmt;
	xtrx_wire_format_t  busfmt;

	xtrxll_fe_t         fefmt;
};

struct xtrx_dev {
	struct xtrxll_dev* lldev;
	bool               fuselms7_callib;
	bool               rx_no_siso_map;
	bool               tx_no_siso_map;

	int                lmsnum;
	struct lms7_struct lmsdrv[MAX_LMS];

	clock_val_t         masterclock;
	clock_val_t         refclock;
	xtrx_clock_source_t clock_source;

	unsigned            rxcgen_div;
	unsigned            txcgen_div;
	unsigned            rxtsp_div;     /* Div ratio at LML */
	unsigned            rxtsp_decim;   /* Decimation in TSP */
	unsigned            txtsp_div;
	unsigned            txtsp_interp;  /* Interpolation in TSP */

	uint64_t            master_ts;

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
	unsigned            rxant;
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
	unsigned            txant;
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

	uint8_t             rx_mmcm_div;
	uint8_t             tx_mmcm_div;
	uint8_t             rx_port_cfg;
	uint8_t             tx_port_cfg;
};

static void xtrx_lms7_logging(const LMS7_log_level_t logLevel, struct LMS7002M_struct *obj, const char *message)
{
	struct lms7_struct* lmss = (struct lms7_struct*)LMS7002M_get_spi_handle(obj);
	struct xtrx_dev* dev = lmss->head;

	enum xtrxll_loglevel log_lvlmap = XTRXLL_NONE;
	switch (logLevel) {
	case LMS7_FATAL:
	case LMS7_CRITICAL:
	case LMS7_ERROR:
		log_lvlmap = XTRXLL_ERROR;
		break;
	case LMS7_WARNING:
		log_lvlmap = XTRXLL_WARNING;
		break;
	case LMS7_NOTICE:
		log_lvlmap = XTRXLL_INFO;
		break;
	case LMS7_INFO:
		log_lvlmap = XTRXLL_INFO_LMS;
		break;
	case LMS7_DEBUG:
		log_lvlmap = XTRXLL_INFO_LMS; //XTRXLL_DEBUG;
		break;
	case LMS7_TRACE:
		log_lvlmap = XTRXLL_INFO_LMS; //XTRXLL_PARANOIC;
		break;
	}

	xtrxll_log(log_lvlmap, "RFIC_LMS7", logLevel, "LMS7 %s: %s\n",
			   xtrxll_get_name(dev->lldev), message);
}

static uint32_t xtrx_spi_transact(void *handle, const uint32_t data, const bool readback)
{
	struct lms7_struct* lmss = (struct lms7_struct*)handle;
	struct xtrx_dev* dev = lmss->head;
	uint32_t read = 0;
	lmss->lasterr = xtrxll_lms7_spi_bulk(dev->lldev, 1 << lmss->lmsno, &data, &read, 1);
	return read;
}

static int xtrx_init_lms(struct lms7_struct* lms)
{
	LMS7002M_reset(lms->lms7);

	LMS7002M_ldo_enable(lms->lms7, true, LMS7002M_LDO_ALL);

	LMS7002M_xbuf_share_tx(lms->lms7, true);

	return lms->lasterr;
}


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
	int i;
	int loglevel;

	loglevel = flags & XTRX_O_LOGLVL_MASK;
	xtrxll_set_loglevel(loglevel);

	res = xtrxll_open(device, 0, &lldev);
	if (res)
		goto failed_openll;

	//loglevel = flags & XTRX_O_LOGLVL_MASK;
	LMS7_set_log_level(LMS7_TRACE);

	dev = (struct xtrx_dev*)malloc(sizeof(struct xtrx_dev));
	if (dev == NULL) {
		res = -errno;
		goto failed_mem;
	}
	memset(dev, 0, sizeof(struct xtrx_dev));

	dev->lldev = lldev;

	res = xtrxll_get_cfg(lldev, XTRXLL_CFG_NUM_LMS7, &dev->lmsnum);
	if (res || (dev->lmsnum <= 0)) {
		goto failed_no_lms;
	}
	if (dev->lmsnum > MAX_LMS) {
		dev->lmsnum = MAX_LMS;
	}

	for (i = 0; i < dev->lmsnum; ++i) {
		dev->lmsdrv[i].lmsno = i;
		dev->lmsdrv[i].lasterr = 0;
		dev->lmsdrv[i].lms7 = LMS7002M_create(xtrx_spi_transact, &dev->lmsdrv[i]);
		if (dev->lmsdrv[i].lms7 == NULL) {
			res = -ENOMEM;
			goto failed_lms7;
		}
		dev->lmsdrv[i].head = dev;
	}

	if (XTRX_O_RESET & flags) {
		res = xtrxll_lms7_pwr_ctrl(dev->lldev, (1 << dev->lmsnum) - 1, 0);
		usleep(10000);
	}

	// Power on all LMS device (but stay TX and RX turned off)
	res = xtrxll_lms7_pwr_ctrl(dev->lldev, (1 << dev->lmsnum) - 1, XTRXLL_LMS7_GPWR_PIN | XTRXLL_LMS7_RESET_PIN |
							   XTRXLL_LMS7_RXEN_PIN | XTRXLL_LMS7_TXEN_PIN);
	if (res) {
		goto failed_lms7;
	}

	for (i = 0; i < dev->lmsnum; ++i) {
		res = xtrx_init_lms(&dev->lmsdrv[i]);
		if (res) {
			goto failed_lms7;
		}
	}

	LMS7_set_log_handler(xtrx_lms7_logging);

	xtrxdsp_init();

	dev->refclock = 0;
	dev->clock_source = XTRX_CLKSRC_INT;
	*outdev = dev;
	return 0;

failed_lms7:
	for (--i; i >= 0; --i) {
		LMS7002M_destroy(dev->lmsdrv[i].lms7);
	}
failed_no_lms:
	free(dev);
failed_mem:
	xtrxll_close(lldev);
failed_openll:
	return res;
}

void xtrx_close(struct xtrx_dev* dev)
{
	int i;
	for (i = 0; i < dev->lmsnum; ++i) {
		LMS7002M_destroy(dev->lmsdrv[i].lms7);
	}

	// Put device in power down mode
	xtrxll_lms7_pwr_ctrl(dev->lldev, XTRXLL_LMS7_ALL, 0);

	usleep(1000);

	xtrxll_mmcm_onoff(dev->lldev, true, false);
	xtrxll_mmcm_onoff(dev->lldev, false, false);

	xtrxll_close(dev->lldev);

	free(dev);
}

void xtrx_set_ref_clk(struct xtrx_dev* dev, unsigned refclkhz, xtrx_clock_source_t clksrc)
{
	dev->clock_source = clksrc;
	dev->refclock = refclkhz;
}


#define MAX(x,y) (((x) > (y)) ? (x) : (y))

// FIXME define this value
#define CGEN_MIN 10e6

enum {
	LMS7_DECIM_MAX = 32,
	LMS7_INTER_MAX = 32,
};

static bool check_lime_decimation(unsigned decim)
{
	switch (decim) {
	case 1:  /* 2^0 */
	case 2:  /* 2^1 */
	case 4:  /* 2^2 */
	case 8:  /* 2^3 */
	case 16: /* 2^4 */
	case 32: /* 2^5 */
		return true;
	}
	return false;
}

// FIXME B is inverted here for tests only!!!!
//static const int diqmap[4]              = { LMS7002M_LML_BQ, LMS7002M_LML_AI, LMS7002M_LML_BI, LMS7002M_LML_AQ };

static const int diqmap[4]              = { LMS7002M_LML_BI, LMS7002M_LML_AI, LMS7002M_LML_BQ, LMS7002M_LML_AQ };
static const int diqmap_qi[4]           = { LMS7002M_LML_BQ, LMS7002M_LML_AQ, LMS7002M_LML_BI, LMS7002M_LML_AI };
static const int diqmap_swap[4]         = { LMS7002M_LML_AI, LMS7002M_LML_BI, LMS7002M_LML_AQ, LMS7002M_LML_BQ };
static const int diqmap_swap_qi[4]      = { LMS7002M_LML_AQ, LMS7002M_LML_BQ, LMS7002M_LML_AI, LMS7002M_LML_BI };

static const int diqmap_siso[4]         = { LMS7002M_LML_AI, LMS7002M_LML_AI, LMS7002M_LML_AQ, LMS7002M_LML_AQ };
static const int diqmap_siso_qi[4]      = { LMS7002M_LML_AQ, LMS7002M_LML_AQ, LMS7002M_LML_AI, LMS7002M_LML_AI };
static const int diqmap_siso_swap[4]    = { LMS7002M_LML_BI, LMS7002M_LML_BI, LMS7002M_LML_BQ, LMS7002M_LML_BQ };
static const int diqmap_siso_swap_qi[4] = { LMS7002M_LML_BQ, LMS7002M_LML_BQ, LMS7002M_LML_BI, LMS7002M_LML_BI };


int xtrx_set_samplerate(struct xtrx_dev* dev,
						double cgen_rate,
						double rxrate,
						double txrate,
						unsigned flags,
						double *actualcgen,
						double* actualrx,
						double* actualtx)
{
	int res = 0, i, hwid;
	int rxdiv = 1;
	int txdiv = 1;
	double actualmaster;
	const unsigned adcdiv_fixed = 4;
	const unsigned dacdiv       = 4;

	res = xtrxll_get_sensor(dev->lldev, XTRXLL_HWID, &hwid);
	if (res) {
		XTRXLL_LOG(XTRXLL_ERROR, "xtrx_set_samplerate: unable to get HWID\n");
		return res;
	}

	const unsigned lml1_port_id = hwid & 0x7;
	const unsigned lml2_port_id = (hwid >> 4) & 0x7;

	const int rx_tdis =  0;
	const bool no_vio_set = (flags & XTRX_SAMPLERATE_DEBUG_NO_VIO_SET) ? true : false;
	const int rx_delay = ((flags >> 6) & 0x3ff) | (((flags >> 20) & 0x3ff) << 10); // 1024 max
	const bool no_rx_fwd_clk = (flags & XTRX_SAMPLERATE_DEBUG_NO_RX_FCLK_GEN) ? true : false;

	const bool opt_decim_inter = (flags & XTRX_SAMPLERATE_AUTO_DECIM) ? true : false;

	dev->rx_no_siso_map = (flags & XTRX_SAMPLERATE_DEBUG_NO_RX_SISO_LML) ? true : false;
	dev->tx_no_siso_map = (flags & XTRX_SAMPLERATE_DEBUG_NO_TX_SISO_LML) ? true : false;

	unsigned tx_host_inter = 0;
	unsigned rx_host_decim = 0;
	unsigned tx_host_mul = 1;
	unsigned rx_host_div = 1;

	const bool lml1_rx_valid = (lml1_port_id == 3 || lml1_port_id == 4 || lml1_port_id == 5 || lml1_port_id == 6)     || lml1_port_id == 7;
	const bool lml2_rx_valid = (lml2_port_id == 3 || lml2_port_id == 4 || lml2_port_id == 5 || lml2_port_id == 6)     || lml2_port_id == 7;

	const bool lml1_tx_valid = (lml1_port_id == 1 || lml1_port_id == 2 || lml1_port_id == 4 || lml1_port_id == 6);
	const bool lml2_tx_valid = (lml2_port_id == 1 || lml2_port_id == 2 || lml2_port_id == 4 || lml2_port_id == 6);

	const bool lml1_use_mmcm = (lml1_port_id == 2 || lml1_port_id == 5 || lml1_port_id == 6)    || lml1_port_id == 7;
	const bool lml2_use_mmcm = (lml2_port_id == 2 || lml2_port_id == 5 || lml2_port_id == 6)    || lml2_port_id == 7;

	const LMS7002M_port_t tx_port = (lml1_tx_valid && !lml2_tx_valid) ? LMS_PORT1 : LMS_PORT2;
	const LMS7002M_port_t rx_port = (lml2_rx_valid && !lml1_rx_valid) ? LMS_PORT2 : LMS_PORT1;

	const int tx_gen = (tx_port == LMS_PORT1) ? lml1_use_mmcm : lml2_use_mmcm;
	const int rx_gen = (rx_port == LMS_PORT2) ? lml2_use_mmcm : lml1_use_mmcm;

	if (lml2_port_id == 0 && lml1_port_id == 0) {
		XTRXLL_LOG(XTRXLL_ERROR, "Incorrect FPGA port configuration HWID=%08x => TX=%d RX=%d\n",
				   hwid, tx_port, rx_port);
		return -EFAULT;
	}
	if ((rxrate > 1) && (!lml1_rx_valid && !lml2_rx_valid)) {
		XTRXLL_LOG(XTRXLL_ERROR, "Current FPGA configuration doesn't support RX, HWID=%08x\n",
				   hwid);
		return -EFAULT;
	}
	if ((txrate > 1) && (!lml1_tx_valid && !lml2_tx_valid)) {
		XTRXLL_LOG(XTRXLL_ERROR, "Current FPGA configuration doesn't support TX, HWID=%08x\n",
				   hwid);
		return -EFAULT;
	}

	dev->rx_port_cfg = rx_gen;
	dev->tx_port_cfg = tx_gen;
	const bool rx_no_decim = (((flags & XTRX_SAMPLERATE_DEBUG_NO_RX_DECIM) ? true : false) && rx_gen);
	const bool tx_no_decim = (((flags & XTRX_SAMPLERATE_DEBUG_NO_TX_INTR) ? true : false) && tx_gen);

	const bool no_8ma = (flags & XTRX_SAMPLERATE_DEBUG_NO_8MA_LML) ? true : false;
	const bool slow_mclk_rx_x2 = (((flags & XTRX_SAMPLERATE_DEBUG_SLOW_MCLK) ? true : false) && rx_gen);
	const bool slow_mclk_tx_x2 = (((flags & XTRX_SAMPLERATE_DEBUG_SLOW_MCLK) ? true : false) && tx_gen);

	const bool x2_int_clk = (lml1_rx_valid && (lml1_port_id == 7)) || (lml2_rx_valid && (lml2_port_id == 7));
	const unsigned slow_factor = 2;

	if (dev->clock_source) {
		XTRXLL_LOG(XTRXLL_ERROR, "CLK TYPE: %d\n", dev->clock_source);
		res = xtrxll_lms7_pwr_ctrl(dev->lldev, (1 << dev->lmsnum) - 1,
								   XTRXLL_LMS7_GPWR_PIN | XTRXLL_LMS7_RESET_PIN |
								   ((dev->clock_source == XTRX_CLKSRC_EXT) ? XTRXLL_EXT_CLK : 0) |
								   ((1) ? XTRXLL_LMS7_RXEN_PIN : 0) |
								   ((1) ? XTRXLL_LMS7_TXEN_PIN : 0));

		usleep(1000000);
	}

	if (dev->refclock == 0) {
		// Determine refclk
		int osc, i;
		static const int base_refclk[] = { 10000000, 19200000, 26000000, 30720000, 38400000, 40000000 };
		res = xtrxll_get_sensor(dev->lldev, XTRXLL_REFCLK_CLK, &osc);
		if (res) {
			return res;
		}

		for (i = 0; i < sizeof(base_refclk) / sizeof(base_refclk[0]); i++) {
			int diff = base_refclk[i] - osc;
			if (abs(diff) * (int64_t)1000 / base_refclk[i] < 1) {
				dev->refclock = base_refclk[i];
				XTRXLL_LOG(XTRXLL_INFO, "xtrx_set_samplerate: set RefClk to %d based on %d measurement\n",
						   (int)dev->refclock, osc);
				break;
			}
		}

		if (dev->refclock == 0) {
			XTRXLL_LOG(XTRXLL_INFO, "xtrx_set_samplerate: wierd RefClk %d! set RefClk manually\n", osc);
			return -ENOENT;
		}
	}

	if ((txrate > 1) && ((txrate < MIN_TX_RATE) || (flags & XTRX_SAMPLERATE_FORCE_TX_INTR))) {
		do {
			tx_host_mul <<= 1;
			tx_host_inter++;
		} while ((txrate * tx_host_mul) < MIN_TX_RATE);

		/* check what we can deliver */
		if (tx_host_mul > 2) {
			XTRXLL_LOG(XTRXLL_ERROR, "xtrx_set_samplerate: %d extra TX "
									 "interpolation; however, it's not supported yet...\n",
					   tx_host_mul);
			return -EINVAL;
		}
	}

	if ((rxrate > 1) && (flags & XTRX_SAMPLERATE_FORCE_RX_DECIM)) {
		rx_host_div <<= 1;
		rx_host_decim++;
	}

	// TODO: determine best possible combination of adcdiv / dacdiv, just
	// for now we use fixed 4 divider for both
	if (cgen_rate == 0) {
		// If we activate host decimation or interpolation than we need more samplerate
		cgen_rate = MAX((rx_no_decim ? 1 : 2) * rxrate * rx_host_div * adcdiv_fixed,
						(tx_no_decim ? 1 : ((tx_gen) ? 2 : 4)) * txrate * tx_host_mul * dacdiv);

		// For low sample rate increase DAC/ADC due to frequency aliasing
		if ((rxrate > 1 && rxrate < 2e6) || (txrate > 1 && txrate < 2e6) || opt_decim_inter) {
			for (; cgen_rate <= 320e6; cgen_rate *= 2) {
				if ((rxrate > 1) && (((cgen_rate * 2 / (rxrate * rx_host_div)) / adcdiv_fixed) >= LMS7_DECIM_MAX))
					break;

				if ((txrate > 1) && (((cgen_rate * 2 / (txrate * tx_host_mul)) / dacdiv) >= LMS7_INTER_MAX))
					break;

				XTRXLL_LOG(XTRXLL_INFO, "Increase CGEN %.3f Mhz", cgen_rate * 2);
			}
		}
	}

	if (rxrate > 1) {
		rxdiv = (cgen_rate / (rxrate * rx_host_div)) / adcdiv_fixed;
	}
	if (txrate > 1) {
		txdiv = (cgen_rate / (txrate * tx_host_mul)) / dacdiv;
	}

	if (rxrate > 1 && !check_lime_decimation(rxdiv)) {
		XTRXLL_LOG(XTRXLL_ERROR, "xtrx_set_samplerate: can't deliver decimation: %d of %.3f MHz CGEN and %.3f MHz samplerate\n",
				   rxdiv, cgen_rate / 1e6, rxrate / 1e6);
		return -EINVAL;
	}

	if (txrate > 1 && !check_lime_decimation(txdiv)) {
		XTRXLL_LOG(XTRXLL_ERROR, "xtrx_set_samplerate: can't deliver interpolation: %d of %.3f MHz CGEN and %.3f MHz samplerate\n",
				   txdiv, cgen_rate / 1e6, txrate / 1e6);
		return -EINVAL;
	}


	if (((dev->rx_run) && (dev->rx_host_decim != rx_host_decim)) ||
		((dev->tx_run) && (dev->tx_host_inter != tx_host_inter))) {
		XTRXLL_LOG(XTRXLL_ERROR, "xtrx_set_samplerate: can't change extra host decimation when stream is running\n");
		return -EINVAL;
	}

	// Store all data for correct NCO calculations
	dev->rxcgen_div = adcdiv_fixed;
	dev->txcgen_div = dacdiv;
	dev->rxtsp_div = rxdiv;
	dev->rxtsp_decim = rxdiv;
	dev->txtsp_div = txdiv;
	dev->txtsp_interp = txdiv;

	dev->tx_host_inter = tx_host_inter;
	dev->rx_host_decim = rx_host_decim;

	for (i = 0; i < dev->lmsnum; ++i) {
		// For clock speed more 100Mhz we need to increase drive strength
		bool h = (!no_8ma) && ((rxrate > 40e6) || (txrate > 40e6) || (tx_gen == 0 && txrate > 20e6));
		LMS7002M_set_drive_strength(dev->lmsdrv[i].lms7, h, h);
	}

	// 1. Set CGEN frequency
	for (i = 0; i < dev->lmsnum; ++i) {
		unsigned j;
		for (j = 0; j < 10; j++) {
			res = LMS7002M_set_data_clock_div(dev->lmsdrv[i].lms7,
											  dev->refclock,
											  false,
											  2,
											  cgen_rate,
											  &actualmaster);
			if (res == -1) {
				return -EINVAL;
			} else if (res == -3) {
				continue;
			} else if (res != 0) {
				abort();
			} else {
				break;
			}
		}
		if (res == -3) {
			XTRXLL_LOG(XTRXLL_ERROR, "xtrx_set_samplerate: can't tune VCO for data clock\n");
			return -ERANGE;
		}

		if (actualcgen)
			*actualcgen = actualmaster;

		dev->masterclock = actualmaster;
	}

	// TODO FIXME
	bool update_running = false;
	if (dev->rx_run || dev->tx_run) {
		if (~(flags & XTRX_SAMPLERATE_FORCE_UPDATE))
			return res;

		update_running = true;
	}

	if (!update_running) {
		LMS7002M_reset(dev->lmsdrv[0].lms7);
		usleep(10000);
		LMS7002M_lml_en(dev->lmsdrv[0].lms7);
	}

	// 2. Set LML RX
	unsigned rxtsp_div = 1;
	if (rxrate > 0) {
		rxtsp_div = ((slow_mclk_rx_x2) ? slow_factor : 1) * ((rxdiv > 1) ? (rxdiv / 2) : 1);
		for (i = 0; i < dev->lmsnum; ++i) {
			LMS7002M_configure_lml_port(dev->lmsdrv[i].lms7,
										rx_port/*LML_RX_PORT*/,
										LMS_RX,
										rxtsp_div);

			if (~dev->rx_run) {
				LMS7002M_set_diq_mux(dev->lmsdrv[i].lms7, LMS_RX, diqmap);

				LMS7002M_rbb_enable(dev->lmsdrv[i].lms7, LMS_CHAB, true);
			}
		}
		if (actualrx) {
			*actualrx =  actualmaster / rxdiv / adcdiv_fixed;
		}

		for (unsigned q = 0; q <= 12; q++) {
			xtrxll_set_param(dev->lldev, XTRXLL_PARAM_RX_DLY, (q) | (0 << 4));
		}
	}

	// 3. Set LML TX
	unsigned txtsp_div = 1;
	if (txrate > 1) {
		txtsp_div = ((slow_mclk_tx_x2) ? slow_factor : 1) * ((txdiv > 1) ? (txdiv / 2) : 1);
		for (i = 0; i < dev->lmsnum; ++i) {
			LMS7002M_configure_lml_port(dev->lmsdrv[i].lms7,
										tx_port/*LML_TX_PORT*/,
										LMS_TX,
										(tx_gen) ? txtsp_div : txtsp_div / 2);
			LMS7002M_set_diq_mux(dev->lmsdrv[i].lms7, LMS_TX, diqmap);

			if (~dev->tx_run) {
				LMS7002M_tbb_enable(dev->lmsdrv[i].lms7, LMS_CHAB, true);
			}
		}

		if (actualtx) {
			*actualtx = actualmaster / txdiv / dacdiv;
		}
	}

	// 4.
	//rxrate = txrate = 1; //FIXME TODO Need tx for filer tunning
	res = xtrxll_lms7_pwr_ctrl(dev->lldev, (1 << dev->lmsnum) - 1,
							   XTRXLL_LMS7_GPWR_PIN | XTRXLL_LMS7_RESET_PIN |
							   ((dev->clock_source == XTRX_CLKSRC_EXT) ? XTRXLL_EXT_CLK : 0) |
							   ((1) ? XTRXLL_LMS7_RXEN_PIN : 0) |
							   ((1) ? XTRXLL_LMS7_TXEN_PIN : 0) |
							   ((rx_tdis) ? XTRXLL_LMS7_RX_TERM_D : 0) |
							   ((rx_gen && !no_rx_fwd_clk) ?  XTRXLL_LMS7_RX_GEN : 0));

	if (!no_vio_set && (rxrate > 1 || txrate > 1)) {
		res = xtrxll_set_param(dev->lldev, XTRXLL_PARAM_PWR_VIO, 1800);
		if (res)
			return res;
	}

	// 5. TX MMCM
	double tx_mclk = 0;
	if (txrate > 1 && tx_gen) {
		// MCLK is generated only when AFE is enabled
		LMS7002M_afe_enable(dev->lmsdrv[0].lms7, LMS_TX, LMS_CHAB, true);

		tx_mclk = actualmaster / dacdiv / txtsp_div;
		int mclk = tx_mclk;

		XTRXLL_LOG(XTRXLL_ERROR, "TX MCLK=%.3f (extra %d) MHz\n", mclk / 1.0e6, (((slow_mclk_tx_x2) ? slow_factor : 1)));
		res = xtrxll_mmcm_onoff(dev->lldev, true, true);
		if (res) {
			XTRXLL_LOG(XTRXLL_ERROR, "xtrx_set_samplerate: can't turn on TX MMCM\n");
			return -EFAULT;
		}

		res = xtrxll_mmcm_setfreq(dev->lldev,
								  tx_port == LMS_PORT1,
								  mclk,
								  ((tx_no_decim && txdiv == 1) ? LML_CLOCK_X2 : LML_CLOCK_NORM) | LML_CLOCK_FWD_90,
								  0,
								  &dev->tx_mmcm_div,
								  (slow_mclk_tx_x2) ? 2 * slow_factor : 2);
		if (res != 0) {
			XTRXLL_LOG(XTRXLL_ERROR, "Unable to configure TX MMCM to %d res = %d\n", mclk, res);
			return -ERANGE;
		}
	} else {
		dev->tx_mmcm_div = 0;
	}

	// 6. RX MMCM
	double rx_mclk = 0;
	if (rxrate > 1 && rx_gen) {
		// MCLK is generated only when AFE is enabled
		LMS7002M_afe_enable(dev->lmsdrv[0].lms7, LMS_RX, LMS_CHAB, true);
		//usleep(1000*1000);

		//int mclk = 2 * actualmaster / rxdiv / adcdiv_fixed / ((rx_no_decim  && rxdiv == 1) ? 2 : 1);
		rx_mclk = actualmaster / adcdiv_fixed / rxtsp_div;
		int mclk = rx_mclk;
		XTRXLL_LOG(XTRXLL_ERROR, "RX MCLK=%.3f (%d extra) MHz\n", mclk / 1.0e6, (((slow_mclk_rx_x2) ? slow_factor : 1)));
		res = xtrxll_mmcm_onoff(dev->lldev, false, true);
		if (res) {
			XTRXLL_LOG(XTRXLL_ERROR, "xtrx_set_samplerate: can't turn on RX MMCM\n");
			return -EFAULT;
		}
		usleep(10*1000);

		res = xtrxll_mmcm_setfreq(dev->lldev,
								  rx_port == LMS_PORT1,
								  mclk,
								  ((rx_no_decim && rxdiv == 1) ? LML_CLOCK_X2 : ((no_rx_fwd_clk) ? LML_CLOCK_RX_SELF : LML_CLOCK_NORM)) | (x2_int_clk ? LML_CLOCK_INT_X2 : 0),
								  rx_delay,
								  &dev->rx_mmcm_div,
								  (slow_mclk_rx_x2) ? 2 * slow_factor : 2);
		if (res != 0) {
			XTRXLL_LOG(XTRXLL_ERROR, "Unable to configure RX MMCM to %d res = %d\n", mclk, res);
			return -ERANGE;
		}

		if (!no_rx_fwd_clk) {
			LMS7002M_configure_lml_port_rdfclk(dev->lmsdrv[0].lms7, rx_port/*LML_RX_PORT*/);
		}
	} else {
		dev->rx_mmcm_div = 0;
	}

	XTRXLL_LOG(XTRXLL_INFO, "xtrx_set_samplerate: rxrate=%.3fMHz txrate=%.3fMHz"
							" actual_master=%.3fMHz rxdecim=%d(h_%d) txinterp=%d(h_%d)"
							" RX_ADC=%.3fMHz TX_DAC=%.3fMHz hintr=%d hdecim=%d delay=%d NRXFWD=%d LML1HID=%d LML2HID=%d"
							" RX_div=%d TX_div=%d RX_TSP_div=%d TX_TSP_div=%d FclkRX=%.3f (PHS=%d)"
							" RXx2=%d\n",
			   rxrate / 1e6, txrate / 1e6,
			   actualmaster / 1e6, rxdiv, rx_host_div, txdiv, tx_host_mul,
			   cgen_rate / adcdiv_fixed / 1e6, cgen_rate / dacdiv / 1e6,
			   tx_host_inter, rx_host_decim,
			   rx_delay, no_rx_fwd_clk,
			   lml1_port_id, lml2_port_id,
			   dev->rx_mmcm_div, dev->tx_mmcm_div,
			   rxtsp_div, txtsp_div,
			   (dev->rx_mmcm_div == 0) ? 0 : dev->rx_mmcm_div * rx_mclk / (dev->rx_mmcm_div / ((slow_mclk_rx_x2) ? 2 * slow_factor : 2)) / 1e6,
			   (dev->rx_mmcm_div == 0) ? 0 : (dev->rx_mmcm_div / ((slow_mclk_rx_x2) ? 2 * slow_factor : 2)),
			   x2_int_clk);
	return 0;
}

static int xtrx_channel_to_lms7(xtrx_channel_t xch, LMS7002M_chan_t* out)
{
	switch (xch) {
	case XTRX_CH_A:  *out = LMS_CHA; break;
	case XTRX_CH_B:  *out = LMS_CHB; break;
	case XTRX_CH_AB: *out = LMS_CHAB; break;
	default: return -EINVAL;
	}
	return 0;
}

int xtrx_tune(struct xtrx_dev* dev, xtrx_tune_t type, double freq, double *actualfreq)
{
	return xtrx_tune_ex(dev, type, XTRX_CH_AB, freq, actualfreq);
}

int xtrx_tune_ex(struct xtrx_dev* dev, xtrx_tune_t type, xtrx_channel_t ch, double freq, double *actualfreq)
{
	int i = 0, res;
	LMS7002M_dir_t dir;
	bool nco = false;

	switch (type) {
	case XTRX_TUNE_RX_FDD:
		dir = LMS_RX;
		break;
	case XTRX_TUNE_TX_FDD:
	case XTRX_TUNE_TX_AND_RX_TDD:
		dir = LMS_TX;
		break;
	case XTRX_TUNE_BB_RX:
		dir = LMS_RX;
		nco = true;
		break;
	case XTRX_TUNE_BB_TX:
		dir = LMS_TX;
		nco = true;
		break;
	default: return -EINVAL;
	}

	if (nco) {
		double rel_freq;
		LMS7002M_chan_t lmsch;
		res = xtrx_channel_to_lms7(ch, &lmsch);
		if (res)
			return res;

		if (dir == LMS_TX) {
			double tx_dac_freq = dev->masterclock / dev->txcgen_div;
			rel_freq = freq / tx_dac_freq;
			if (rel_freq > 0.5 || rel_freq < -0.5) {
				XTRXLL_LOG(XTRXLL_WARNING,
						   "NCO TX ouf of range, requested %.3f while DAC %.3f\n",
						   rel_freq / 1000, tx_dac_freq / 1000);
				return -EINVAL;
			}
			LMS7002M_txtsp_set_freq(dev->lmsdrv[i].lms7, lmsch, rel_freq);
		} else {
			double rx_dac_freq = dev->masterclock / dev->rxcgen_div;
			rel_freq = freq / rx_dac_freq;
			if (rel_freq > 0.5 || rel_freq < -0.5) {
				XTRXLL_LOG(XTRXLL_WARNING,
						   "NCO RX ouf of range, requested %.3f (%.3f kHz) while ADC %.3f kHz\n",
						   rel_freq, freq / 1000, rx_dac_freq / 1000);
				return -EINVAL;
			}
			LMS7002M_rxtsp_set_freq(dev->lmsdrv[i].lms7, lmsch, -rel_freq);
		}
		if (actualfreq)
			*actualfreq = rel_freq;
		return 0;
	}

	if (freq == 0.0) {
		LMS7002M_sxx_enable(dev->lmsdrv[i].lms7, dir, false);
		if (actualfreq)
			*actualfreq = 0.0;
		return 0;
	}

	LMS7002M_sxx_enable(dev->lmsdrv[i].lms7, dir, true);

	if (type == XTRX_TUNE_TX_AND_RX_TDD) {
		LMS7002M_sxx_enable(dev->lmsdrv[i].lms7, LMS_RX, false);
	}

	res = LMS7002M_set_lo_freq(dev->lmsdrv[i].lms7, dir, dev->refclock, freq, actualfreq);

	if (type == XTRX_TUNE_TX_AND_RX_TDD) {
		LMS7002M_sxt_to_sxr(dev->lmsdrv[i].lms7, true);
	}

	switch (res) {
	case -1: return -EINVAL;
	case -2: XTRXLL_LOG(XTRXLL_ERROR, "No freq\n"); return -ERANGE;
	case -3: return -ENAVAIL;
	case 0: return 0;
	default: return -EFAULT;
	}
}

static int xtrx_tune_bandwidth(struct xtrx_dev* dev, xtrx_channel_t xch, int rbb, double bw, double *actualbw)
{
	int i = 0, j;
	int res;
	LMS7002M_chan_t ch;

	res = xtrx_channel_to_lms7(xch, &ch);
	if (res)
		return res;

	for (j = LMS_CHA; j <= LMS_CHB; j++) {
		if (ch == LMS_CHA && j == LMS_CHB)
			continue;
		else if (ch == LMS_CHB && j == LMS_CHA)
			continue;

		res = (rbb) ? LMS7002M_rbb_set_filter_bw(dev->lmsdrv[i].lms7, (LMS7002M_chan_t)j, bw, actualbw) :
					  LMS7002M_tbb_set_filter_bw(dev->lmsdrv[i].lms7, (LMS7002M_chan_t)j, bw, actualbw);
	}

	switch (res) {
	case -1: return -EINVAL;  // Incorrect DIV
	case -2: return -ERANGE;  // VCO out of range
	case -3: return -ENAVAIL; // Can't deliver VCO
	case 0: return 0;
	default: return -EFAULT;
	}
}

int xtrx_tune_tx_bandwidth(struct xtrx_dev* dev, xtrx_channel_t xch, double bw, double *actualbw)
{
	int res;

	res = xtrx_tune_bandwidth(dev, xch, 0, bw, &dev->tx_bandwidth);
	if (actualbw) {
		*actualbw = dev->tx_bandwidth;
	}
	return res;
}

int xtrx_tune_rx_bandwidth(struct xtrx_dev* dev, xtrx_channel_t xch, double bw, double *actualbw)
{
	int res;
#if 1
	res = xtrx_tune_bandwidth(dev, xch, 1, bw, &dev->rx_bandwidth);
	if (actualbw) {
		*actualbw = dev->rx_bandwidth;
	}
#else
	const int chan = 0;
	LMS7002M_set_mac_ch(dev->lmsdrv[chan].lms7, LMS_CHA);
	res = lms7_tune_rx(xtrx_spi_transact, &dev->lmsdrv[chan], dev->refclock, bw);
	dev->rx_bandwidth = bw;

	if (res == 0) {
		*actualbw = dev->rx_bandwidth;
	} else {
		XTRXLL_LOG(XTRXLL_ERROR, "Tune RX error: %d\n", res);
	}
#endif
	return res;
}


int xtrx_set_gain(struct xtrx_dev* dev, xtrx_channel_t xch, xtrx_gain_type_t gt, double gain, double *actualgain)
{
	int i = 0;
	double actual;
	LMS7002M_chan_t ch;

	int res = xtrx_channel_to_lms7(xch, &ch);
	if (res)
		return res;

	XTRXLL_LOG(XTRXLL_INFO, "Set gain %.1f to %d on %d channel\n", gain, gt, xch);

	switch (gt) {
	case XTRX_RX_LNA_GAIN: actual = LMS7002M_rfe_set_lna(dev->lmsdrv[i].lms7, ch, gain); break;
	case XTRX_RX_TIA_GAIN: actual = LMS7002M_rfe_set_tia(dev->lmsdrv[i].lms7, ch, gain); break;
	case XTRX_RX_PGA_GAIN: actual = LMS7002M_rbb_set_pga(dev->lmsdrv[i].lms7, ch, gain); break;
	case XTRX_RX_LB_GAIN: actual = LMS7002M_rfe_set_loopback_lna(dev->lmsdrv[i].lms7, ch, gain); break;

	case XTRX_TX_PAD_GAIN: actual = LMS7002M_trf_set_pad(dev->lmsdrv[i].lms7, ch, gain); break;
	default: return -EINVAL;
	}

	if (actualgain)
		*actualgain = actual;
	return 0;
}


int xtrx_set_antenna(struct xtrx_dev* dev, xtrx_antenna_t antenna)
{
	int i = 0;
	int band;
	int tx;
	LMS7002M_chan_t ch = LMS_CHAB;

	switch (antenna) {
	case XTRX_RX_L: band = LMS7002M_RFE_LNAL; tx = 0; break;
	case XTRX_RX_H: band = LMS7002M_RFE_LNAH; tx = 0; break;
	case XTRX_RX_W: band = LMS7002M_RFE_LNAW; tx = 0; break;

	case XTRX_RX_L_LB: band = LMS7002M_RFE_LB2; tx = 0; break;
	case XTRX_RX_W_LB: band = LMS7002M_RFE_LB1; tx = 0; break;


	case XTRX_TX_L: band = 1; tx = 1; break; // FIXME!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	case XTRX_TX_W: band = 2; tx = 1; break; // FIXME!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	default: return -EINVAL;
	}

	if (tx) {
		LMS7002M_trf_select_band(dev->lmsdrv[i].lms7, ch, band);
		dev->txant = (band == 1) ? 1 : 0;
	} else {
		LMS7002M_rfe_set_path(dev->lmsdrv[i].lms7, ch, band);
		if (0) {
			dev->rxant = (band == LMS7002M_RFE_LNAW) ? 0 :
						 (band == LMS7002M_RFE_LNAH) ? 1 :
						 (band == LMS7002M_RFE_LNAL) ? 2 : 3;
		} else {
			dev->rxant = (band == LMS7002M_RFE_LNAW) ? 0 :
						 (band == LMS7002M_RFE_LNAH) ? 2 :
						 (band == LMS7002M_RFE_LNAL) ? 1 : 3;
		}
	}

	return xtrxll_lms7_ant(dev->lldev, dev->rxant, dev->txant);
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

static bool xtrx_run_params_stream_is_swap(const xtrx_run_stream_params_t* stream)
{
	return (stream->chs == XTRX_CH_AB && (stream->flags & XTRX_RSP_SWAP_AB)) ||
			stream->chs == XTRX_CH_A;
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

	LMS7002M_chan_t tx_lmschan = 0;
	xtrxll_mode_t   tx_mode = XTRXLL_FE_MODE_MIMO;
	xtrxll_fe_t     tx_fe_fmt = XTRXLL_FE_DONTTOUCH;

	LMS7002M_chan_t rx_lmschan = 0;
	xtrxll_mode_t   rx_mode = XTRXLL_FE_MODE_MIMO;
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
		if (xtrx_channel_to_lms7(params->rx.chs, &rx_lmschan)) {
			return -EINVAL;
		}

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

		rx_mode = (xtrx_run_params_stream_is_mimo(&params->rx)) ? XTRXLL_FE_MODE_MIMO
																: XTRXLL_FE_MODE_SISO;
		rx_fe_fmt = (params->rx.wfmt == XTRX_WF_8) ? XTRXLL_FE_8BIT :
					(params->rx.wfmt == XTRX_WF_12) ? XTRXLL_FE_12BIT : XTRXLL_FE_16BIT;
		rx_stream_count = (rx_mode == XTRXLL_FE_MODE_SISO) ? 1 : 2;
	}

	if (params->dir & XTRX_TX) {
		if (xtrx_channel_to_lms7(params->tx.chs, &tx_lmschan)) {
			return -EINVAL;
		}

		if ((params->tx.wfmt != XTRX_WF_16 || params->tx.hfmt == XTRX_IQ_INT8)) {
			XTRXLL_LOG(XTRXLL_ERROR, "Specified combination of host and wire formats isn't supported for TX stream\n");
			return -EINVAL;
		}

		if (params->tx.hfmt != XTRX_IQ_FLOAT32 && (params->tx.flags & XTRX_RSP_SCALE)) {
			XTRXLL_LOG(XTRXLL_ERROR, "XTRX_RSP_SCALE is supported only for XTRX_IQ_FLOAT32 host format in TX stream\n");
			return -EINVAL;
		}

		tx_mode = (xtrx_run_params_stream_is_mimo(&params->tx)) ? XTRXLL_FE_MODE_MIMO
																: XTRXLL_FE_MODE_SISO;
		tx_fe_fmt = (params->tx.wfmt == XTRX_WF_8) ? XTRXLL_FE_8BIT :
					(params->tx.wfmt == XTRX_WF_12) ? XTRXLL_FE_12BIT : XTRXLL_FE_16BIT;
		tx_stream_count = (tx_mode == XTRXLL_FE_MODE_SISO) ? 1 : 2;
	}

	/* Validation of input parameters done, do the things */
	if (params->dir & XTRX_RX) {
		rx_start_ts = dev->rx_samples = params->rx_stream_start << dev->rx_host_decim;

		if (!dev->rxinit) {
			res = xtrxll_dma_rx_init(dev->lldev, chan, rx_bpkt_size, &rx_bpkt_size);
			if (res) {
				return res;
			}
			XTRXLL_LOG(XTRXLL_INFO, "RX ititialized to %d bytes paket size\n", rx_bpkt_size);
			dev->rxinit = 1;
		}

		if (dev->rx_host_decim) {
			unsigned i;
			if (dev->rx_host_decim != 1) {
				return -EINVAL;
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
				return -EINVAL;
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

#if 0
		if (xtrx_run_params_stream_is_swap(&params->rx)) {
			if (params->rx.flags & XTRX_RSP_SWAP_IQ) {
				LMS7002M_set_diq_mux(dev->lmsdrv[chan].lms7, LMS_RX, diqmap_swap_qi);
			} else {
				LMS7002M_set_diq_mux(dev->lmsdrv[chan].lms7, LMS_RX, diqmap_swap);
			}
		} else if (params->rx.flags & XTRX_RSP_SWAP_IQ) {
			LMS7002M_set_diq_mux(dev->lmsdrv[chan].lms7, LMS_RX, diqmap_qi);
		} else {
			if (!xtrx_run_params_stream_is_mimo(&params->rx)) {
				LMS7002M_set_diq_mux(dev->lmsdrv[chan].lms7, LMS_RX, diqmap_siso);
			}
		}
#else
		const int *const diqarray[8] = {
			diqmap, diqmap_qi, diqmap_swap, diqmap_swap_qi,
			diqmap_siso, diqmap_siso_qi, diqmap_siso_swap, diqmap_siso_swap_qi,
		};

		unsigned diqidx = 0;
		if (params->rx.flags & XTRX_RSP_SWAP_IQ)
			diqidx |= 1;
		if (xtrx_run_params_stream_is_swap(&params->rx))
			diqidx |= 2;
		if (!dev->rx_no_siso_map && !xtrx_run_params_stream_is_mimo(&params->rx))
			diqidx |= 4;

		assert(diqidx < 8);
		if (diqidx != 0) {
			LMS7002M_set_diq_mux(dev->lmsdrv[chan].lms7, LMS_RX, diqarray[diqidx]);
		}
#endif

		LMS7002M_rxtsp_enable(dev->lmsdrv[chan].lms7, rx_lmschan, true);
		LMS7002M_rxtsp_set_decim(dev->lmsdrv[chan].lms7, rx_lmschan, dev->rxtsp_decim);
		LMS7002M_rfe_enable(dev->lmsdrv[chan].lms7, rx_lmschan, true);
		LMS7002M_afe_enable(dev->lmsdrv[chan].lms7, LMS_RX, rx_lmschan, true);

		/*
		res = LMS7002M_set_gfir_taps(dev->lmsdrv[chan].lms7,
							   LMS_RX,
							   rx_lmschan,
							   3,
							   g_filter_taps_120_4x,
							   FILTER_TAPS_120);
		assert(res == 0);
		*/
		/*
		res = LMS7002M_set_gfir_taps(dev->lmsdrv[chan].lms7,
							   LMS_RX,
							   rx_lmschan,
							   2,
							   g_filter_taps_40_long_2x_4x,
							   FILTER_TAPS_40);
		assert(res == 0);
*/
		/*
		res = LMS7002M_set_gfir_taps(dev->lmsdrv[chan].lms7,
							   LMS_RX,
							   rx_lmschan,
							   1,
							   g_filter_taps_40_long_2x_4x,
							   FILTER_TAPS_40);
		assert(res == 0);
		*/

		dev->rxbuf = NULL;
		dev->rxbuf_total = 0;
		dev->rxbuf_processed = 0;
		dev->rxbuf_processed_ts = 0;
		dev->rxbuf_conv_state = 0;
		dev->rxbuf_len_iq_symbol = rx_stream_count * xtrx_wire_format_get_iq_size(params->rx.wfmt);
		dev->rxbuf_len_iq_host_sym = rx_stream_count * xtrx_host_format_get_iq_size(params->rx.hfmt);

		LMS7002M_rxtsp_set_dc_correction(dev->lmsdrv[chan].lms7, rx_lmschan, true, 15);
		if (params->rx.flags & XTRX_RSP_TEST_SIGNAL_A) {
			LMS7002M_rxtsp_tsg_tone(dev->lmsdrv[chan].lms7, LMS_CHA);
		}
		if (params->rx.flags & XTRX_RSP_TEST_SIGNAL_B) {
			LMS7002M_rxtsp_tsg_tone(dev->lmsdrv[chan].lms7, LMS_CHB);
		}

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

#if 0
		if (xtrx_run_params_stream_is_swap(&params->tx)) {
			if (params->tx.flags & XTRX_RSP_SWAP_IQ) {
				LMS7002M_set_diq_mux(dev->lmsdrv[chan].lms7, LMS_TX, diqmaptx_swap_qi);
			} else {
				LMS7002M_set_diq_mux(dev->lmsdrv[chan].lms7, LMS_TX, diqmaptx_swap);
			}
		} else if (params->tx.flags & XTRX_RSP_SWAP_IQ) {
			LMS7002M_set_diq_mux(dev->lmsdrv[chan].lms7, LMS_TX, diqmaptx_qi);
		}
#else
		const int *const diqarray[8] = {
			diqmap, diqmap_qi, diqmap_swap, diqmap_swap_qi,
			diqmap_siso, diqmap_siso_qi, diqmap_siso_swap, diqmap_siso_swap_qi,
		};

		unsigned diqidx = 0;
		if (params->tx.flags & XTRX_RSP_SWAP_IQ)
			diqidx |= 1;
		if (xtrx_run_params_stream_is_swap(&params->tx))
			diqidx |= 2;
		if (!dev->tx_no_siso_map && !xtrx_run_params_stream_is_mimo(&params->tx))
			diqidx |= 4;

		assert(diqidx < 8);
		if (diqidx != 0) {
			LMS7002M_set_diq_mux(dev->lmsdrv[chan].lms7, LMS_TX, diqarray[diqidx]);
		}
#endif
		LMS7002M_txtsp_enable(dev->lmsdrv[chan].lms7, tx_lmschan, true);
		LMS7002M_txtsp_set_interp(dev->lmsdrv[chan].lms7, tx_lmschan, dev->txtsp_interp);
		LMS7002M_trf_enable(dev->lmsdrv[chan].lms7, tx_lmschan, true);
		LMS7002M_afe_enable(dev->lmsdrv[chan].lms7, LMS_TX, tx_lmschan, true);
		if (params->tx.flags & XTRX_RSP_TEST_SIGNAL_A) {
			LMS7002M_txtsp_tsg_tone(dev->lmsdrv[chan].lms7, LMS_CHA);
		}
		if (params->tx.flags & XTRX_RSP_TEST_SIGNAL_B) {
			LMS7002M_txtsp_tsg_tone(dev->lmsdrv[chan].lms7, LMS_CHB);
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

	if (params->nflags & XTRX_RUN_DIGLOOPBACK) {
		XTRXLL_LOG(XTRXLL_INFO, "Enable digital loopback\n");
		LMS7002M_setup_digital_loopback(dev->lmsdrv[chan].lms7);
	}
	if (params->nflags & XTRX_RUN_RXLFSR) {
		XTRXLL_LOG(XTRXLL_INFO, "Enable RX LFSR\n");
		LMS7002M_setup_rx_lfsr(dev->lmsdrv[chan].lms7);
	}

	res = xtrxll_dma_start(dev->lldev,
						   chan,
						   rx_fe_fmt,
						   rx_mode,
						   rx_start_ts,
						   tx_fe_fmt,
						   tx_mode);
	if (res) {
		XTRXLL_LOG(XTRXLL_ERROR, "Unable to start DMA err=%d\n", res);
		goto fail_dma_start;
	}
#ifdef USE_EXT_CALIBRATION
	/* Do the calibration */
	// TODO: that bandwidth is set
	if (params->dir & XTRX_RX) {
		//xtrx_set_antenna(dev, XTRX_RX_W);
		if (params->rx.chs & XTRX_CH_A) {
			LMS7002M_set_mac_ch(dev->lmsdrv[chan].lms7, LMS_CHA);
			res = lms7_calibrate_rx(xtrx_spi_transact, &dev->lmsdrv[chan], dev->refclock);
			if (res) {
				XTRXLL_LOG(XTRXLL_INFO, "LMS7_RX CHA calibration result: %d\n", res);
			}
		}
		if (params->rx.chs & XTRX_CH_B) {
			LMS7002M_set_mac_ch(dev->lmsdrv[chan].lms7, LMS_CHB);
			res = lms7_calibrate_rx(xtrx_spi_transact, &dev->lmsdrv[chan], dev->refclock);
			if (res) {
				XTRXLL_LOG(XTRXLL_INFO, "LMS7_RX CHB calibration result: %d\n", res);
			}
		}
	}

	if (params->dir & XTRX_TX) {
		if (params->tx.chs & XTRX_CH_A) {
			LMS7002M_set_mac_ch(dev->lmsdrv[chan].lms7, LMS_CHA);
			res = lms7_calibrate_tx(xtrx_spi_transact, &dev->lmsdrv[chan], dev->refclock);
			if (res) {
				XTRXLL_LOG(XTRXLL_INFO, "LMS7_TX CHA calibration result: %d\n", res);
			}
		}
		if (params->tx.chs & XTRX_CH_B) {
			LMS7002M_set_mac_ch(dev->lmsdrv[chan].lms7, LMS_CHB);
			res = lms7_calibrate_tx(xtrx_spi_transact, &dev->lmsdrv[chan], dev->refclock);
			if (res) {
				XTRXLL_LOG(XTRXLL_INFO, "LMS7_TX CHB calibration result: %d\n", res);
			}
		}
	}
#endif

	//xtrxll_set_param(dev->lldev, XTRXLL_PARAM_RX_DLY, (0) | (1 << 4));
	//xtrxll_set_param(dev->lldev, XTRXLL_PARAM_RX_DLY, (15));

	return 0;

fail_dma_start:
	dev->rx_run = false;
	dev->tx_run = false;

	if (params->dir & XTRX_TX) {
		LMS7002M_txtsp_enable(dev->lmsdrv[chan].lms7, tx_lmschan, false);
		LMS7002M_trf_enable(dev->lmsdrv[chan].lms7, tx_lmschan, false);
		LMS7002M_afe_enable(dev->lmsdrv[chan].lms7, LMS_TX, tx_lmschan, false);
	}
failed_tx_init:
	if (params->dir & XTRX_RX) {
		LMS7002M_rxtsp_enable(dev->lmsdrv[chan].lms7, rx_lmschan, false);
		LMS7002M_rfe_enable(dev->lmsdrv[chan].lms7, rx_lmschan, false);
		LMS7002M_afe_enable(dev->lmsdrv[chan].lms7, LMS_RX, rx_lmschan, false);
	}
filter_cleanup:
	xtrxdsp_filter_free(&dev->rx_host_filter[0]);
	xtrxdsp_filter_free(&dev->rx_host_filter[1]);
	xtrxdsp_filter_free(&dev->tx_host_filter[0]);
	xtrxdsp_filter_free(&dev->tx_host_filter[1]);
	return res;
}

int xtrx_stop(struct xtrx_dev* dev, xtrx_direction_t dir)
{
	int res;
	const int chan = 0; //TODO
	LMS7002M_chan_t lmschan = LMS_CHAB;

	if (dir & XTRX_RX) {
		res = xtrxll_dma_rx_start(dev->lldev, 0, XTRXLL_FE_STOP);

		dev->rxbuf = NULL;
		dev->rxbuf_total = 0;
		dev->rxbuf_processed = 0;
		dev->rxbuf_processed_ts = 0;

		LMS7002M_rxtsp_enable(dev->lmsdrv[chan].lms7, lmschan, false);
		LMS7002M_rfe_enable(dev->lmsdrv[chan].lms7, lmschan, false);
		LMS7002M_afe_enable(dev->lmsdrv[chan].lms7, LMS_RX, lmschan, false);

		dev->rx_run = false;

		xtrxdsp_filter_free(&dev->rx_host_filter[0]);
		xtrxdsp_filter_free(&dev->rx_host_filter[1]);
	}

	if (dir & XTRX_TX) {
		res = xtrxll_dma_tx_start(dev->lldev, chan, XTRXLL_FE_STOP, XTRXLL_FE_MODE_MIMO);

		dev->txbuf = NULL;
		dev->txbuf_total = 0;
		//dev->txbuf_processed = 0;

		LMS7002M_txtsp_enable(dev->lmsdrv[chan].lms7, lmschan, false);
		LMS7002M_trf_enable(dev->lmsdrv[chan].lms7, lmschan, false);
		LMS7002M_afe_enable(dev->lmsdrv[chan].lms7, LMS_TX, lmschan, false);

		dev->tx_run = false;

		xtrxdsp_filter_free(&dev->tx_host_filter[0]);
		xtrxdsp_filter_free(&dev->tx_host_filter[1]);
	}

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
	WIRE_I8_HOST_F32   = MAKE_FORMAT(XTRX_WF_8,  XTRX_IQ_FLOAT32),
	WIRE_I12_HOST_F32  = MAKE_FORMAT(XTRX_WF_12, XTRX_IQ_FLOAT32),
	WIRE_I16_HOST_F32  = MAKE_FORMAT(XTRX_WF_16, XTRX_IQ_FLOAT32),
	WIRE_I16_HOST_I16  = MAKE_FORMAT(XTRX_WF_16, XTRX_IQ_INT16),
};

int xtrx_send_sync_ex(struct xtrx_dev* dev, xtrx_send_ex_info_t *info)
{
	const int chan = 0;
	/* Total numer of sample in all channels */
	size_t total_samples = info->samples * info->buffer_count;
	/* Total bytes user requested */
	size_t user_total = total_samples * dev->txbuf_len_iq_host_sym / dev->tx_chans_in_stream;
	/* Total bytes satisfied from user */
	size_t user_processed = 0;

	int res;

	if (user_total == 0 || info->buffer_count > 2)
		return -EINVAL;

	/* expanding from host encoding to the wire format times 2, we need this
	   to present 12-bit wire format in integer */
	unsigned encode_amplification_x2 = 2 * dev->txbuf_len_iq_host_sym / dev->txbuf_len_iq_symbol;

	uint16_t cur_late, upd_late;
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

			upd_late = cur_late - dev->txburt_late_prev;
#if 0
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
				dev->txskip_time = cur_wts + 2*info->samples; // Discard  2x packets

				info->out_flags |= XTRX_TX_DISCARDED_TO;
				return 0;
			}
#endif
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
				// TODO add dsp function for the stream merging
				abort();
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
	const int chan = 0;
	/* One sample in all channels in bytes */
	size_t total_samples = info->samples * info->buffer_count;
	/* Total bytes user requested */
	size_t user_total = total_samples * dev->rxbuf_len_iq_host_sym  / dev->rx_chans_in_stream;
	/* Total bytes satisfied for user */
	size_t user_processed = 0;

	xtrxll_dma_rx_flags_t flags = 0;
	int res;

	/* expanding from wire decoding to the host format times 2 */
	unsigned decode_amplification_x2 = 2 * dev->rxbuf_len_iq_host_sym / dev->rxbuf_len_iq_symbol;

	if (user_total == 0 || info->buffer_count > 2)
		return -EINVAL;

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
			switch (MAKE_FORMAT(dev->rx_busfmt, dev->rx_hostfmt)) {
			case WIRE_I8_HOST_I8:
			case WIRE_I16_HOST_I16:
				if (info->buffer_count == 1) {
					memcpy(dst_buf[0],
							wire_buffer_ptr,
							wire_bytes_consumed);
				} else {
					// TODO add dsp function for the stream separation
					abort();
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

int xtrx_debug_dump_lms(struct xtrx_dev* dev, const char* path)
{
	int i = 0;
	int res;
	res = LMS7002M_dump_ini(dev->lmsdrv[i].lms7, path);
	if (res)
		return -EINVAL;
	return 0;
}


void xtrx_set_logfunction(xtrx_logfunc_t func)
{
	xtrxll_set_logfunc(func);
}


int xtrx_val_set(struct xtrx_dev* dev, xtrx_direction_t dir,
				 xtrx_channel_t chan, xtrx_val_t type, uint64_t val)
{
	const int i = 0;
	LMS7002M_chan_t lmsch;
	int res;

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
		res = xtrx_channel_to_lms7(chan, &lmsch);
		if (res)
			return res;

		if (dir & XTRX_TX)
			LMS7002M_txtsp_tsg_const(dev->lmsdrv[i].lms7, lmsch,
									 val & 0xffff, (val >> 16) & 0xffff);
		if (dir & XTRX_RX)
			LMS7002M_rxtsp_tsg_const(dev->lmsdrv[i].lms7, lmsch,
									 val & 0xffff, (val >> 16) & 0xffff);
		return 0;
	case XTRX_VCTCXO_DAC_VAL:
		return xtrxll_set_osc_dac(dev->lldev, val);
	default:
		return -EINVAL;
	}
}

XTRX_API int xtrx_val_get(struct xtrx_dev* dev, xtrx_direction_t dir,
						  xtrx_channel_t chan, xtrx_val_t type, uint64_t* oval)
{
	int res, val;
	const int i = 0;
	LMS7002M_chan_t lmsch;

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
	case XTRX_LMS7_RSSI:
		res = xtrx_channel_to_lms7(chan, &lmsch);
		if (res)
			return res;

		*oval = LMS7002M_rxtsp_read_rssi(dev->lmsdrv[i].lms7, lmsch);
		return 0;
	case XTRX_LMS7_DATA_RATE:
		if (dir == XTRX_RX) {
			*oval = (dev->rxcgen_div == 0) ? 0 : (uint64_t)dev->masterclock / dev->rxcgen_div;
		} else if (dir == XTRX_TX) {
			*oval = (dev->txcgen_div == 0) ? 0 : (uint64_t)dev->masterclock / dev->txcgen_div;
		} else {
			return -EINVAL;
		}
		return 0;
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
