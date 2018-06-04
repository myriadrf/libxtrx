/*
 * xtrx frontend source file
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
#include "xtrx_fe.h"
#include <xtrxll_api.h>
#include <xtrxll_log.h>
#include <xtrxll_mmcm.h>

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <LMS7002M/LMS7002M.h>
#include <LMS7002M/LMS7002M_logger.h>

#include "xtrx_api.h"

enum {
	MIN_TX_RATE = 2100000, /* 2.1e6 Minimum samplerate supported by the XTRX hardware */
};

struct xtrx_fe_lms7
{
	struct xtrx_fe_obj base;

	int              lasterr;
	LMS7002M_t*      lms7;

	struct xtrxll_dev* lldev;

	double cgen_clk;

	unsigned lmsnum;
	unsigned refclock;
	unsigned refclk_source;

	bool rx_no_siso_map;
	bool tx_no_siso_map;

	bool tx_run;
	bool rx_run;

	uint8_t             rx_mmcm_div;
	uint8_t             tx_mmcm_div;
	uint8_t             rx_port_cfg;
	uint8_t             tx_port_cfg;

	unsigned rx_host_decim;
	unsigned tx_host_inter;

	unsigned            rxcgen_div;
	unsigned            txcgen_div;
	unsigned            rxtsp_div;     /* Div ratio at LML */
	unsigned            rxtsp_decim;   /* Decimation in TSP */
	unsigned            txtsp_div;
	unsigned            txtsp_interp;  /* Interpolation in TSP */

	bool                rx_lna_auto;
	bool                tx_lna_auto;
	unsigned            txant;
	unsigned            rxant;

	double              rx_lo;
	double              tx_lo;
};


enum xtrxll_lms7_pwr {
	XTRXLL_LMS7_RESET_PIN = 1<<1,
	XTRXLL_LMS7_GPWR_PIN  = 1<<2,
	XTRXLL_LMS7_RXEN_PIN  = 1<<3,
	XTRXLL_LMS7_TXEN_PIN  = 1<<4,

	XTRXLL_LMS7_RX_GEN    = 1<<6,
	XTRXLL_LMS7_RX_TERM_D = 1<<7,
};



static void _xtrx_lms7_logging(const LMS7_log_level_t logLevel,
							   struct LMS7002M_struct *obj,
							   const char *message)
{
	struct xtrx_fe_lms7* lmss = (struct xtrx_fe_lms7*)LMS7002M_get_spi_handle(obj);
	struct xtrxll_dev* dev = lmss->lldev;

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
			   xtrxll_get_name(dev), message);
}

static uint32_t _xtrx_spi_transact(void *handle, const uint32_t data, const bool readback)
{
	struct xtrx_fe_lms7* lmss = (struct xtrx_fe_lms7*)handle;
	struct xtrxll_dev* dev = lmss->lldev;
	uint32_t read = 0;
	lmss->lasterr = xtrxll_lms7_spi_bulk(dev, 1, &data, &read, 1);
	return read;
}

static int _xtrx_init_lms(struct xtrx_fe_lms7* lms)
{
	LMS7002M_reset(lms->lms7);
	LMS7002M_ldo_enable(lms->lms7, true, LMS7002M_LDO_ALL);
	LMS7002M_xbuf_share_tx(lms->lms7, true);

	return lms->lasterr;
}

static int _xtrx_set_lna_rx(struct xtrx_fe_lms7 *dev, int band)
{
	LMS7002M_chan_t ch = LMS_CHAB;
	LMS7002M_rfe_set_path(dev->lms7, ch, band);
	dev->rxant = (band == LMS7002M_RFE_LNAW) ? 0 :
				 (band == LMS7002M_RFE_LNAH) ? 2 :
				 (band == LMS7002M_RFE_LNAL) ? 1 : 3;

	return xtrxll_set_param(dev->lldev, XTRXLL_PARAM_SWITCH_RX_ANT, dev->rxant);
}

static int _xtrx_set_lna_tx(struct xtrx_fe_lms7 *dev, int band)
{
	LMS7002M_chan_t ch = LMS_CHAB;
	LMS7002M_trf_select_band(dev->lms7, ch, band);
	dev->txant = (band == 1) ? 1 : 0;

	return xtrxll_set_param(dev->lldev, XTRXLL_PARAM_SWITCH_TX_ANT, dev->rxant);
}

enum sigtype {
	XTRX_TX_LO_CHANGED,
	XTRX_RX_LO_CHANGED,
	XTRX_TX_LNA_CHANGED,
	XTRX_RX_LNA_CHANGED,
};

static int _xtrx_signal_event(struct xtrx_fe_lms7 *dev, enum sigtype t)
{
	switch (t) {
	case XTRX_RX_LO_CHANGED:
	case XTRX_RX_LNA_CHANGED:
		if (dev->rx_lna_auto) {
			int band = (dev->rx_lo > 2200e6) ? LMS7002M_RFE_LNAH :
					   (dev->rx_lo > 1500e6) ? LMS7002M_RFE_LNAW : LMS7002M_RFE_LNAL;
			XTRXLL_LOG(XTRXLL_INFO, "Auto RX band selection: %c\n", band);
			return _xtrx_set_lna_rx(dev, band);
		}
		break;
	case XTRX_TX_LO_CHANGED:
	case XTRX_TX_LNA_CHANGED:
		if (dev->tx_lna_auto) {
			int band = (dev->tx_lo > 2200e6) ? 2 : 1;
			XTRXLL_LOG(XTRXLL_INFO, "Auto TX band selection: %d\n", band);
			return _xtrx_set_lna_tx(dev, band);
		}
		break;
	}
	return 0;
}

#define MAX(x,y) (((x) > (y)) ? (x) : (y))

// FIXME define this value
#define CGEN_MIN 10e6


enum {
	LMS7_DECIM_MAX = 32,
	LMS7_INTER_MAX = 32,
};

static bool _check_lime_decimation(unsigned decim)
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

static void _lms7fe_init_base(struct xtrx_fe_lms7 *dev);

static int _xtrx_channel_to_lms7(unsigned xch, LMS7002M_chan_t* out)
{
	switch (xch) {
	case XTRX_CH_A:  *out = LMS_CHA; break;
	case XTRX_CH_B:  *out = LMS_CHB; break;
	case XTRX_CH_AB: *out = LMS_CHAB; break;
	default: return -EINVAL;
	}
	return 0;
}

int lms7fe_init(struct xtrxll_dev* lldev,
				unsigned flags,
				struct xtrx_fe_obj** obj)
{
	LMS7_set_log_level(LMS7_TRACE);

	struct xtrx_fe_lms7 *dev;
	int lmscnt = 0;
	int res = xtrxll_get_sensor(lldev, XTRXLL_CFG_NUM_RFIC, &lmscnt);
	if (res || (lmscnt != 1)) {
		goto failed_no_lms;
	}

	dev = (struct xtrx_fe_lms7*)malloc(sizeof(struct xtrx_fe_lms7));
	if (dev == NULL) {
		res = -errno;
		goto failed_mem;
	}
	memset(dev, 0, sizeof(struct xtrx_fe_lms7));
	_lms7fe_init_base(dev);
	dev->lmsnum = 1;
	dev->lasterr = 0;
	dev->lms7 = LMS7002M_create(_xtrx_spi_transact, dev);
	if (dev->lms7 == NULL) {
		res = -ENOMEM;
		goto failed_lms7;
	}
	dev->lldev = lldev;

	res = xtrxll_set_param(dev->lldev, XTRXLL_PARAM_PWR_CTRL, PWR_CTRL_ON);
	if (res) {
		goto failed_lms7;
	}

	if (XTRX_O_RESET & flags) {
		res = xtrxll_set_param(dev->lldev, XTRXLL_PARAM_FE_CTRL, 0);
		if (res) {
			goto failed_lms7;
		}

		usleep(10000);
	}

	// Power on all LMS device (but stay TX and RX turned off)
	res = xtrxll_set_param(dev->lldev, XTRXLL_PARAM_FE_CTRL,
							   XTRXLL_LMS7_GPWR_PIN | XTRXLL_LMS7_RESET_PIN |
							   XTRXLL_LMS7_RXEN_PIN | XTRXLL_LMS7_TXEN_PIN);
	if (res) {
		goto failed_lms7;
	}

	res = _xtrx_init_lms(dev);
	if (res) {
		goto failed_lms7;
	}

	LMS7_set_log_handler(_xtrx_lms7_logging);

	dev->rx_lna_auto = true;
	dev->tx_lna_auto = true;
	dev->rx_lo = 0;
	dev->tx_lo = 0;

	*obj = (struct xtrx_fe_obj*)dev;
	return 0;

failed_lms7:
	free(dev);
failed_mem:
failed_no_lms:
	return res;
}

int lms7fe_deinit(struct xtrx_fe_obj* obj)
{
	struct xtrx_fe_lms7 *dev = (struct xtrx_fe_lms7 *)obj;
	LMS7002M_destroy(dev->lms7);

	// Put device in power down mode
	xtrxll_set_param(dev->lldev, XTRXLL_PARAM_FE_CTRL, 0);
	xtrxll_set_param(dev->lldev, XTRXLL_PARAM_PWR_CTRL, PWR_CTRL_PDOWN);

	usleep(1000);

	xtrxll_mmcm_onoff(dev->lldev, true, false);
	xtrxll_mmcm_onoff(dev->lldev, false, false);

	return 0;
}

static unsigned get_dacdiv(double rxrate, double txrate, int tx_gen)
{
	if (rxrate < 1)
		return 1;

	double tx_gain = (txrate + 1) / rxrate;
	if (tx_gain >= 4)
		return 1;
	else if (tx_gain >= 2)
		return 2;
	else if (tx_gain >= 1)
		return 4;

	return 8;
}

int lms7fe_dd_set_samplerate(struct xtrx_fe_obj* obj,
							 const struct xtrx_fe_samplerate* inrates,
							 struct xtrx_fe_samplerate* outrates)
{
	int res;
	struct xtrx_fe_lms7 *dev = (struct xtrx_fe_lms7 *)obj;
	double rxrate = inrates->adc.rate;
	double txrate = inrates->dac.rate;
	unsigned flags = inrates->flags;
	unsigned hwid = inrates->hwid;
	int rxdiv = 1;
	int txdiv = 1;
	double actualmaster;
	double cgen_rate;
	const unsigned l1_pid = hwid & 0x7;
	const unsigned l2_pid = (hwid >> 4) & 0x7;
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

	const bool lml1_rx_valid = (l1_pid == 3 || l1_pid == 4 || l1_pid == 5 || l1_pid == 6)     || l1_pid == 7;
	const bool lml2_rx_valid = (l2_pid == 3 || l2_pid == 4 || l2_pid == 5 || l2_pid == 6)     || l2_pid == 7;

	const bool lml1_tx_valid = (l1_pid == 1 || l1_pid == 2 || l1_pid == 4 || l1_pid == 6);
	const bool lml2_tx_valid = (l2_pid == 1 || l2_pid == 2 || l2_pid == 4 || l2_pid == 6);

	const bool lml1_use_mmcm = (l1_pid == 2 || l1_pid == 5 || l1_pid == 6)    || l1_pid == 7;
	const bool lml2_use_mmcm = (l2_pid == 2 || l2_pid == 5 || l2_pid == 6)    || l2_pid == 7;

	const LMS7002M_port_t tx_port = (lml1_tx_valid && !lml2_tx_valid) ? LMS_PORT1 : LMS_PORT2;
	const LMS7002M_port_t rx_port = (lml2_rx_valid && !lml1_rx_valid) ? LMS_PORT2 : LMS_PORT1;

	const int tx_gen = (tx_port == LMS_PORT1) ? lml1_use_mmcm : lml2_use_mmcm;
	const int rx_gen = (rx_port == LMS_PORT2) ? lml2_use_mmcm : lml1_use_mmcm;

	if (l2_pid == 0 && l1_pid == 0) {
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

	const unsigned adcdiv_fixed = 4;
	const unsigned dacdiv       = get_dacdiv(inrates->adc.rate, inrates->dac.rate, tx_gen);

	dev->rx_port_cfg = rx_gen;
	dev->tx_port_cfg = tx_gen;
	const bool rx_no_decim = (((flags & XTRX_SAMPLERATE_DEBUG_NO_RX_DECIM) ? true : false) && rx_gen);
	const bool tx_no_decim = (((flags & XTRX_SAMPLERATE_DEBUG_NO_TX_INTR) ? true : false) && tx_gen);

	const bool no_8ma = (flags & XTRX_SAMPLERATE_DEBUG_NO_8MA_LML) ? true : false;
	const bool slow_mclk_rx_x2 = (((flags & XTRX_SAMPLERATE_DEBUG_SLOW_MCLK) ? true : false) && rx_gen);
	const bool slow_mclk_tx_x2 = (((flags & XTRX_SAMPLERATE_DEBUG_SLOW_MCLK) ? true : false) && tx_gen);

	const bool x2_int_clk = (lml1_rx_valid && (l1_pid == 7)) || (lml2_rx_valid && (l2_pid == 7));
	const unsigned slow_factor = 2;

	dev->refclk_source = inrates->refclk_source;

	// Add host interpolation for slow rates if needed
	if ((tx_gen) && ((txrate > 1) && ((txrate < MIN_TX_RATE) || (flags & XTRX_SAMPLERATE_FORCE_TX_INTR)))) {
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

	// Extra RX decimation
	if ((rxrate > 1) && (flags & XTRX_SAMPLERATE_FORCE_RX_DECIM)) {
		rx_host_div <<= 1;
		rx_host_decim++;
	}

	double txmaster_min = adcdiv_fixed * inrates->dac.hwrate;
	double rxmaster_min = dacdiv * inrates->adc.hwrate;
	cgen_rate = MAX(txmaster_min, rxmaster_min);

	// TODO: determine best possible combination of adcdiv / dacdiv, just
	// for now we use fixed 4 divider for both
	if (cgen_rate == 0) {
		// If we activate host decimation or interpolation than we need more samplerate
		cgen_rate = MAX((rx_no_decim ? 1 : 2) * rxrate * rx_host_div * adcdiv_fixed,
						(tx_no_decim ? 1 : ((tx_gen) ? 2 : 4)) * txrate * tx_host_mul * dacdiv);

		// For low sample rate increase DAC/ADC due to frequency aliasing
		if ((rxrate > 1 && rxrate < 2e6) || (txrate > 1 && txrate < 2e6) || opt_decim_inter) {
			for (; cgen_rate <= 320e6; cgen_rate *= 2) {
				unsigned rx_ndiv = (rxrate > 1) ? ((cgen_rate * 2 / (rxrate * rx_host_div)) / adcdiv_fixed) : 0;
				unsigned tx_ndiv = (txrate > 1) ? ((cgen_rate * 2 / (txrate * tx_host_mul)) / dacdiv) : 0;

				if (rx_ndiv > LMS7_DECIM_MAX || tx_ndiv > LMS7_INTER_MAX)
					break;

				XTRXLL_LOG(XTRXLL_INFO, "Increase RXdiv=%2d TXdiv=%2d => CGEN %03.1f Mhz\n",
						   rx_ndiv, tx_ndiv, cgen_rate * 2 / 1.0e6);
			}
		}
	}

	if (rxrate > 1) {
		rxdiv = (cgen_rate / (rxrate * rx_host_div)) / adcdiv_fixed;
	}
	if (txrate > 1) {
		txdiv = (cgen_rate / (txrate * tx_host_mul)) / dacdiv;
	}

	if (rxrate > 1 && !_check_lime_decimation(rxdiv)) {
		XTRXLL_LOG(XTRXLL_ERROR, "xtrx_set_samplerate: can't deliver decimation: %d of %.3f MHz CGEN and %.3f MHz samplerate\n",
				   rxdiv, cgen_rate / 1e6, rxrate / 1e6);
		return -EINVAL;
	}

	if (txrate > 1 && !_check_lime_decimation(txdiv)) {
		XTRXLL_LOG(XTRXLL_ERROR, "xtrx_set_samplerate: can't deliver interpolation: %d of %.3f MHz CGEN and %.3f MHz samplerate\n",
				   txdiv, cgen_rate / 1e6, txrate / 1e6);
		return -EINVAL;
	}


	if (((dev->rx_run) && (dev->rx_host_decim != rx_host_decim)) ||
		((dev->tx_run) && (dev->tx_host_inter != tx_host_inter))) {
		XTRXLL_LOG(XTRXLL_ERROR, "xtrx_set_samplerate: can't change extra host decimation when stream is running\n");
		return -EINVAL;
	}

	// TODO FIXME
	//bool update_running = false;
	if (dev->rx_run || dev->tx_run) {
		if (!(flags & XTRX_SAMPLERATE_FORCE_UPDATE))
			return -EPERM;

		//update_running = true;
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


	bool h = (!no_8ma) && ((rxrate > 40e6) || (txrate > 40e6) || (tx_gen == 0 && txrate > 20e6));
	LMS7002M_set_drive_strength(dev->lms7, h, h);

	dev->refclock = inrates->dac.refclk;
	// 1. Set CGEN frequency
	// --------------------------------------------------------------
	for (unsigned j = 0; j < 10; j++) {
		unsigned clkdiv = (dacdiv == 1) ? 0 :
						  (dacdiv == 2) ? 1 :
						  (dacdiv == 4) ? 2 : 3;
		res = LMS7002M_set_data_clock_div(dev->lms7,
										  dev->refclock,
										  false,
										  clkdiv,
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

	dev->cgen_clk = actualmaster;

	//if (!update_running) {
	LMS7002M_reset(dev->lms7);
	usleep(10000);
	LMS7002M_lml_en(dev->lms7);
	//}

	// 2. Set LML RX
	unsigned rxtsp_div = 1;
	if (rxrate > 0) {
		rxtsp_div = ((slow_mclk_rx_x2) ? slow_factor : 1) * ((rxdiv > 1) ? (rxdiv / 2) : 1);
		LMS7002M_configure_lml_port(dev->lms7,
									rx_port/*LML_RX_PORT*/,
									LMS_RX,
									rxtsp_div);

		if (~dev->rx_run) {
			LMS7002M_rbb_enable(dev->lms7, LMS_CHAB, true);
		}
		if (outrates) {
			outrates->adc.rate =  actualmaster / rxdiv / adcdiv_fixed / rx_host_div;
			outrates->adc.hwrate = actualmaster / adcdiv_fixed;
			outrates->adc.host_di = rx_host_decim;
		}

		for (unsigned q = 0; q <= 12; q++) {
			xtrxll_set_param(dev->lldev, XTRXLL_PARAM_RX_DLY, (q) | (0 << 4));
		}
	}

	// 3. Set LML TX
	unsigned txtsp_div = 1;
	if (txrate > 1) {
		txtsp_div = ((slow_mclk_tx_x2) ? slow_factor : 1) * ((txdiv > 1) ? (txdiv / 2) : 1);
		LMS7002M_configure_lml_port(dev->lms7,
									tx_port/*LML_TX_PORT*/,
									LMS_TX,
									(tx_gen) ? txtsp_div : txtsp_div / 2);
		if (~dev->tx_run) {
			LMS7002M_tbb_enable(dev->lms7, LMS_CHAB, true);
		}

		if (outrates) {
			outrates->dac.rate = actualmaster / txdiv / dacdiv / tx_host_mul;
			outrates->dac.hwrate = actualmaster / dacdiv;
			outrates->dac.host_di = tx_host_inter;
		}
	}

	// 4.
	//rxrate = txrate = 1; //FIXME TODO Need tx for filer tunning
	res = xtrxll_set_param(dev->lldev, XTRXLL_PARAM_FE_CTRL,
						   XTRXLL_LMS7_GPWR_PIN | XTRXLL_LMS7_RESET_PIN |
						   XTRXLL_LMS7_RXEN_PIN | XTRXLL_LMS7_TXEN_PIN |
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
		LMS7002M_afe_enable(dev->lms7, LMS_TX, LMS_CHAB, true);

		tx_mclk = actualmaster / dacdiv / txtsp_div;
		int mclk = tx_mclk;

		XTRXLL_LOG(XTRXLL_ERROR, "TX MCLK=%.3f (extra %d) MHz\n",
				   mclk / 1.0e6, (((slow_mclk_tx_x2) ? slow_factor : 1)));
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
		LMS7002M_afe_enable(dev->lms7, LMS_RX, LMS_CHAB, true);
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
			LMS7002M_configure_lml_port_rdfclk(dev->lms7, rx_port/*LML_RX_PORT*/);
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
			   l1_pid, l2_pid,
			   dev->rx_mmcm_div, dev->tx_mmcm_div,
			   rxtsp_div, txtsp_div,
			   (dev->rx_mmcm_div == 0) ? 0 : dev->rx_mmcm_div * rx_mclk / (dev->rx_mmcm_div / ((slow_mclk_rx_x2) ? 2 * slow_factor : 2)) / 1e6,
			   (dev->rx_mmcm_div == 0) ? 0 : (dev->rx_mmcm_div / ((slow_mclk_rx_x2) ? 2 * slow_factor : 2)),
			   x2_int_clk);

	return 0;
}

static bool _xtrx_run_params_stream_is_swap(const struct xtrx_dd_chpar* stream)
{
	return (stream->chs == XTRX_CH_AB && (stream->flags & XTRX_RSP_SWAP_AB)) ||
			stream->chs == XTRX_CH_A;
}
static bool _xtrx_run_params_stream_is_mimo(const struct xtrx_dd_chpar* stream)
{
	return (stream->chs == XTRX_CH_AB &&
			!(stream->flags & XTRX_RSP_SISO_MODE));
}

static const int *const _get_lml_portcfg(const struct xtrx_dd_chpar* par,
										 bool no_siso_map)
{
	static const int diqarray[12][4] = {
		// MIMO modes
		{ LMS7002M_LML_BI, LMS7002M_LML_AI, LMS7002M_LML_BQ, LMS7002M_LML_AQ },
		{ LMS7002M_LML_BQ, LMS7002M_LML_AQ, LMS7002M_LML_BI, LMS7002M_LML_AI },
		{ LMS7002M_LML_AI, LMS7002M_LML_BI, LMS7002M_LML_AQ, LMS7002M_LML_BQ },
		{ LMS7002M_LML_AQ, LMS7002M_LML_BQ, LMS7002M_LML_AI, LMS7002M_LML_BI },
		// SISO modes
		{ LMS7002M_LML_AI, LMS7002M_LML_AI, LMS7002M_LML_AQ, LMS7002M_LML_AQ },
		{ LMS7002M_LML_AQ, LMS7002M_LML_AQ, LMS7002M_LML_AI, LMS7002M_LML_AI },
		{ LMS7002M_LML_BI, LMS7002M_LML_BI, LMS7002M_LML_BQ, LMS7002M_LML_BQ },
		{ LMS7002M_LML_BQ, LMS7002M_LML_BQ, LMS7002M_LML_BI, LMS7002M_LML_BI },
		// MIMO test modes
		{ LMS7002M_LML_BQ, LMS7002M_LML_AI, LMS7002M_LML_BI, LMS7002M_LML_AQ },
		{ LMS7002M_LML_BI, LMS7002M_LML_AQ, LMS7002M_LML_BQ, LMS7002M_LML_AI },
		{ LMS7002M_LML_AQ, LMS7002M_LML_BI, LMS7002M_LML_AI, LMS7002M_LML_BQ },
		{ LMS7002M_LML_AI, LMS7002M_LML_BQ, LMS7002M_LML_AQ, LMS7002M_LML_BI },
	};

	unsigned diqidx = 0;
	if (par->flags & XTRX_RSP_SWAP_IQ)
		diqidx |= 1;

	if (_xtrx_run_params_stream_is_swap(par))
		diqidx |= 2;

	if (!no_siso_map && !_xtrx_run_params_stream_is_mimo(par))
		diqidx |= 4;
	else if (par->flags & XTRX_RSP_SWAP_IQB)
		diqidx |= 8;

	assert(diqidx < (sizeof(diqarray)/sizeof(diqarray[0])));
	return diqarray[diqidx];
}

int lms7fe_dd_configure(struct xtrx_fe_lms7* dev,
						const struct xtrx_dd_params *params)
{
	LMS7002M_chan_t rx_lmschan = 0;
	LMS7002M_chan_t tx_lmschan = 0;
	unsigned dir = params->dir;

	if (dir & XTRX_RX) {
		if (_xtrx_channel_to_lms7(params->rx.chs, &rx_lmschan)) {
			return -EINVAL;
		}

	}
	if (dir & XTRX_TX) {
		if (_xtrx_channel_to_lms7(params->tx.chs, &tx_lmschan)) {
			return -EINVAL;
		}
	}

	if (dir & XTRX_RX) {
		LMS7002M_set_diq_mux(dev->lms7, LMS_RX,
							 _get_lml_portcfg(&params->rx, dev->rx_no_siso_map));
		LMS7002M_rxtsp_enable(dev->lms7, rx_lmschan, true);
		LMS7002M_rxtsp_set_decim(dev->lms7, rx_lmschan, dev->rxtsp_decim);
		LMS7002M_rfe_enable(dev->lms7, rx_lmschan, true);
		LMS7002M_afe_enable(dev->lms7, LMS_RX, rx_lmschan, true);
		if (params->rx.flags & XTRX_RSP_TEST_SIGNAL_A) {
			LMS7002M_rxtsp_tsg_tone(dev->lms7, LMS_CHA);
		}
		if (params->rx.flags & XTRX_RSP_TEST_SIGNAL_B) {
			LMS7002M_rxtsp_tsg_tone(dev->lms7, LMS_CHB);
		}
		if (~(params->rx.flags & XTRX_RSP_NO_DC_CORR)) {
			LMS7002M_rxtsp_set_dc_correction(dev->lms7, rx_lmschan, true, 15);
		}
		dev->rx_run = true;
	}
	if (dir & XTRX_TX) {
		LMS7002M_set_diq_mux(dev->lms7, LMS_TX,
							 _get_lml_portcfg(&params->tx, dev->tx_no_siso_map));

		LMS7002M_txtsp_enable(dev->lms7, tx_lmschan, true);
		LMS7002M_txtsp_set_interp(dev->lms7, tx_lmschan, dev->txtsp_interp);
		LMS7002M_trf_enable(dev->lms7, tx_lmschan, true);
		LMS7002M_afe_enable(dev->lms7, LMS_TX, tx_lmschan, true);
		if (params->tx.flags & XTRX_RSP_TEST_SIGNAL_A) {
			LMS7002M_txtsp_tsg_tone(dev->lms7, LMS_CHA);
		}
		if (params->tx.flags & XTRX_RSP_TEST_SIGNAL_B) {
			LMS7002M_txtsp_tsg_tone(dev->lms7, LMS_CHB);
		}
		dev->tx_run = false;
	}

	if (params->nflags & XTRX_RUN_DIGLOOPBACK) {
		XTRXLL_LOG(XTRXLL_INFO, "Enable digital loopback\n");
		LMS7002M_setup_digital_loopback(dev->lms7);
	}
	if (params->nflags & XTRX_RUN_RXLFSR) {
		XTRXLL_LOG(XTRXLL_INFO, "Enable RX LFSR\n");
		LMS7002M_setup_rx_lfsr(dev->lms7);
	}

	return 0;
}

int lms7fe_dd_reset(struct xtrx_fe_lms7* dev,
						const struct xtrx_dd_params *params)
{
	LMS7002M_chan_t rx_lmschan = 0;
	LMS7002M_chan_t tx_lmschan = 0;
	unsigned dir = params->dir;
	if (dir & XTRX_RX) {
		if (_xtrx_channel_to_lms7(params->rx.chs, &rx_lmschan)) {
			return -EINVAL;
		}

	}
	if (dir & XTRX_TX) {
		if (_xtrx_channel_to_lms7(params->tx.chs, &tx_lmschan)) {
			return -EINVAL;
		}
	}

	if (dir & XTRX_RX) {
		LMS7002M_rxtsp_enable(dev->lms7, rx_lmschan, false);
		LMS7002M_rfe_enable(dev->lms7, rx_lmschan, false);
		LMS7002M_afe_enable(dev->lms7, LMS_RX, rx_lmschan, false);
		dev->rx_run = false;
	}

	if (dir & XTRX_TX) {
		LMS7002M_txtsp_enable(dev->lms7, tx_lmschan, false);
		LMS7002M_trf_enable(dev->lms7, tx_lmschan, false);
		LMS7002M_afe_enable(dev->lms7, LMS_TX, tx_lmschan, false);
		dev->tx_run = false;
	}

	return 0;
}

int lms7fe_dd_set_modes(struct xtrx_fe_obj* obj,
						unsigned op,
						const struct xtrx_dd_params *params)
{
	struct xtrx_fe_lms7 *dev = (struct xtrx_fe_lms7 *)obj;

	switch (op) {
	case XTRX_FEDD_CONFIGURE: return lms7fe_dd_configure(dev, params);
	case XTRX_FEDD_RESET: return lms7fe_dd_reset(dev, params);
	}

	return -EINVAL;
}


int lms7fe_bb_set_freq(struct xtrx_fe_obj* obj,
					   unsigned channel,
					   unsigned type,
					   double freq,
					   double* actualfreq)
{
	int res;
	LMS7002M_dir_t dir;
	switch (type) {
	case XTRX_TUNE_BB_RX:
		dir = LMS_RX;
		break;
	case XTRX_TUNE_BB_TX:
		dir = LMS_TX;
		break;
	default: return -EINVAL;
	}

	double rel_freq;
	LMS7002M_chan_t lmsch;
	struct xtrx_fe_lms7 *dev = (struct xtrx_fe_lms7 *)obj;
	res = _xtrx_channel_to_lms7(channel, &lmsch);
	if (res)
		return res;

	if (dir == LMS_TX) {
		double tx_dac_freq = dev->cgen_clk / dev->txcgen_div;
		rel_freq = freq / tx_dac_freq;
		if (rel_freq > 0.5 || rel_freq < -0.5) {
			XTRXLL_LOG(XTRXLL_WARNING,
					   "NCO TX ouf of range, requested %.3f while DAC %.3f\n",
					   rel_freq / 1000, tx_dac_freq / 1000);
			return -EINVAL;
		}
		XTRXLL_LOG(XTRXLL_WARNING, "TX_DAC=%.3f RelF=%.4f\n",
				   tx_dac_freq / 1e6, rel_freq);
		LMS7002M_txtsp_set_freq(dev->lms7, lmsch, rel_freq);
	} else {
		double rx_dac_freq = dev->cgen_clk / dev->rxcgen_div;
		rel_freq = freq / rx_dac_freq;
		if (rel_freq > 0.5 || rel_freq < -0.5) {
			XTRXLL_LOG(XTRXLL_WARNING,
					   "NCO RX ouf of range, requested %.3f (%.3f kHz) while ADC %.3f kHz\n",
					   rel_freq, freq / 1000, rx_dac_freq / 1000);
			return -EINVAL;
		}
		LMS7002M_rxtsp_set_freq(dev->lms7, lmsch, -rel_freq);
	}
	if (actualfreq)
		*actualfreq = rel_freq;
	return 0;
}

int lms7fe_bb_set_badwidth(struct xtrx_fe_obj* obj,
						   unsigned channel,
						   unsigned dir,
						   double bw,
						   double* actualbw)
{
	int res;
	LMS7002M_chan_t ch;
	struct xtrx_fe_lms7 *dev = (struct xtrx_fe_lms7 *)obj;

	res = _xtrx_channel_to_lms7(channel, &ch);
	if (res)
		return res;

	for (int j = LMS_CHA; j <= LMS_CHB; j++) {
		if (ch == LMS_CHA && j == LMS_CHB)
			continue;
		else if (ch == LMS_CHB && j == LMS_CHA)
			continue;

		res = (dir == XTRX_TUNE_BB_RX) ? LMS7002M_rbb_set_filter_bw(dev->lms7, (LMS7002M_chan_t)j, bw, actualbw) :
					  LMS7002M_tbb_set_filter_bw(dev->lms7, (LMS7002M_chan_t)j, bw, actualbw);
	}

	switch (res) {
	case -1: return -EINVAL;  // Incorrect DIV
	case -2: return -ERANGE;  // VCO out of range
	case -3: return -ENAVAIL; // Can't deliver VCO
	case 0: return 0;
	default: return -EFAULT;
	}
}

int lms7fe_set_gain(struct xtrx_fe_obj* obj,
					unsigned channel,
					unsigned gain_type,
					double gain,
					double *actualgain)
{
	double actual;
	LMS7002M_chan_t ch;
	struct xtrx_fe_lms7 *dev = (struct xtrx_fe_lms7 *)obj;

	int res = _xtrx_channel_to_lms7(channel, &ch);
	if (res)
		return res;

	XTRXLL_LOG(XTRXLL_INFO, "Set gain %.1f to %d on %d channel\n", gain, gain_type, channel);

	switch (gain_type) {
	case XTRX_RX_LNA_GAIN: actual = LMS7002M_rfe_set_lna(dev->lms7, ch, gain); break;
	case XTRX_RX_TIA_GAIN: actual = LMS7002M_rfe_set_tia(dev->lms7, ch, gain); break;
	case XTRX_RX_PGA_GAIN: actual = LMS7002M_rbb_set_pga(dev->lms7, ch, gain); break;
	case XTRX_RX_LB_GAIN: actual = LMS7002M_rfe_set_loopback_lna(dev->lms7, ch, gain); break;

	case XTRX_TX_PAD_GAIN: actual = LMS7002M_trf_set_pad(dev->lms7, ch, gain); break;
	default: return -EINVAL;
	}

	if (actualgain)
		*actualgain = actual;
	return 0;
}

int lms7fe_fe_set_freq(struct xtrx_fe_obj* obj,
					   unsigned channel,
					   unsigned type,
					   double freq,
					   double *actualfreq)
{
	int res;
	double res_freq = 0;
	LMS7002M_dir_t dir;
	struct xtrx_fe_lms7 *dev = (struct xtrx_fe_lms7 *)obj;

	switch (type) {
	case XTRX_TUNE_RX_FDD:
		dir = LMS_RX;
		break;
	case XTRX_TUNE_TX_FDD:
	case XTRX_TUNE_TX_AND_RX_TDD:
		dir = LMS_TX;
		break;
	default: return -EINVAL;
	}

	if (freq == 0.0) {
		LMS7002M_sxx_enable(dev->lms7, dir, false);
		if (actualfreq)
			*actualfreq = 0.0;
		return 0;
	}

	LMS7002M_sxx_enable(dev->lms7, dir, true);

	if (type == XTRX_TUNE_TX_AND_RX_TDD) {
		LMS7002M_sxx_enable(dev->lms7, LMS_RX, false);
	}

	res = LMS7002M_set_lo_freq(dev->lms7, dir, dev->refclock, freq, &res_freq);
	if (res == 0) {
		if (actualfreq)
			*actualfreq = res_freq;

		if (type == XTRX_TUNE_TX_AND_RX_TDD) {
			dev->rx_lo = dev->tx_lo = res_freq;
		} else {
			if (dir == LMS_TX) {
				dev->tx_lo = res_freq;
			} else {
				dev->rx_lo = res_freq;
			}
		}

		if (type == XTRX_TUNE_TX_AND_RX_TDD || type == XTRX_TUNE_RX_FDD) {
			res = _xtrx_signal_event(dev, XTRX_RX_LO_CHANGED);
			if (res)
				return res;
		}
		if (type == XTRX_TUNE_TX_AND_RX_TDD || type == XTRX_TUNE_TX_FDD) {
			res = _xtrx_signal_event(dev, XTRX_TX_LO_CHANGED);
			if (res)
				return res;
		}

		if (type == XTRX_TUNE_TX_AND_RX_TDD) {
			LMS7002M_sxt_to_sxr(dev->lms7, true);
		}

		return 0;
	}

	switch (res) {
	case -1: return -EINVAL;
	case -2: XTRXLL_LOG(XTRXLL_ERROR, "No freq\n"); return -ERANGE;
	case -3: return -ENAVAIL;
	case 0: return 0;
	}
	return -EFAULT;
}

int lms7fe_fe_set_lna(struct xtrx_fe_obj* obj,
					  unsigned channel,
					  unsigned dir,
					  unsigned lna)
{
	struct xtrx_fe_lms7 *dev = (struct xtrx_fe_lms7 *)obj;
	int band;
	int tx;

	switch (lna) {
	case XTRX_RX_L: band = LMS7002M_RFE_LNAL; tx = 0; break;
	case XTRX_RX_H: band = LMS7002M_RFE_LNAH; tx = 0; break;
	case XTRX_RX_W: band = LMS7002M_RFE_LNAW; tx = 0; break;

	case XTRX_RX_L_LB: band = LMS7002M_RFE_LB2; tx = 0; break;
	case XTRX_RX_W_LB: band = LMS7002M_RFE_LB1; tx = 0; break;


	case XTRX_TX_L: band = 1; tx = 1; break; // FIXME!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	case XTRX_TX_W: band = 2; tx = 1; break; // FIXME!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	case XTRX_RX_AUTO: dev->rx_lna_auto = false; return _xtrx_signal_event(dev, XTRX_RX_LNA_CHANGED);
	case XTRX_TX_AUTO: dev->tx_lna_auto = false; return _xtrx_signal_event(dev, XTRX_TX_LNA_CHANGED);
	default: return -EINVAL;
	}

	if (tx) {
		dev->tx_lna_auto = false;
		return _xtrx_set_lna_tx(dev, band);
	} else {
		dev->rx_lna_auto = false;
		return _xtrx_set_lna_rx(dev, band);
	}
}

int lms7fe_get_reg(struct xtrx_fe_obj* obj,
			   unsigned channel,
			   unsigned dir,
			   unsigned type,
			   uint64_t* outval)
{
	struct xtrx_fe_lms7 *dev = (struct xtrx_fe_lms7 *)obj;
	LMS7002M_chan_t lmsch;
	if (channel) {
		int res = _xtrx_channel_to_lms7(channel, &lmsch);
		if (res)
			return res;
	} else {
		lmsch = LMS_CHAB;
	}

	switch (type) {
	case XTRX_LMS7_RSSI:
		*outval = LMS7002M_rxtsp_read_rssi(dev->lms7, lmsch);
		return 0;
	case XTRX_LMS7_DATA_RATE:
		if (dir == XTRX_RX) {
			*outval = (dev->rxcgen_div == 0) ? 0 : (uint64_t)dev->cgen_clk / dev->rxcgen_div;
		} else if (dir == XTRX_TX) {
			*outval = (dev->txcgen_div == 0) ? 0 : (uint64_t)dev->cgen_clk / dev->txcgen_div;
		} else {
			return -EINVAL;
		}
		return 0;
	default:
		if (type >= XTRX_RFIC_REG_0 && type <= XTRX_RFIC_REG_0 + 65535)	{
			LMS7002M_set_mac_ch(dev->lms7, lmsch);
			uint32_t tmp;
			uint32_t wr = ((0x7fff & (type - XTRX_RFIC_REG_0)) << 16);
			int res = xtrxll_lms7_spi_bulk(dev->lldev, 1, &wr, &tmp, 1);
			*outval = tmp & 0xffff;

			XTRXLL_LOG(XTRXLL_ERROR, "RD %c %08x => %08x\n",
					   lmsch, wr, tmp);
			return res;
		}
		return -EINVAL;
	}
}

int lms7fe_set_reg(struct xtrx_fe_obj* obj,
			   unsigned channel,
			   unsigned dir,
			   unsigned type,
			   uint64_t val)
{
	struct xtrx_fe_lms7 *dev = (struct xtrx_fe_lms7 *)obj;
	LMS7002M_chan_t lmsch;
	if (channel) {
		int res = _xtrx_channel_to_lms7(channel, &lmsch);
		if (res)
			return res;
	} else {
		lmsch = LMS_CHAB;
	}

	switch (type) {
	case XTRX_LMS7_XSP_DC_IQ:
		if (dir & XTRX_TX)
			LMS7002M_txtsp_tsg_const(dev->lms7, lmsch,
									 val & 0xffff, (val >> 16) & 0xffff);
		if (dir & XTRX_RX)
			LMS7002M_rxtsp_tsg_const(dev->lms7, lmsch,
									 val & 0xffff, (val >> 16) & 0xffff);
		return 0;
	default:
		if (type >= XTRX_RFIC_REG_0 && type <= XTRX_RFIC_REG_0 + 65535)	{
			uint32_t rd;
			uint32_t wr = ((0x8000 | (type - XTRX_RFIC_REG_0)) << 16) | (val & 0xffff);
			LMS7002M_set_mac_ch(dev->lms7, lmsch);

			return xtrxll_lms7_spi_bulk(dev->lldev, 1, &wr, &rd, 1);
		}
		return -EINVAL;
	}
}

static const struct xtrx_fe_ops _lms7fe_ops = {
	lms7fe_dd_set_modes,
	lms7fe_dd_set_samplerate,

	lms7fe_bb_set_freq,
	lms7fe_bb_set_badwidth,
	lms7fe_set_gain,

	lms7fe_fe_set_freq,
	lms7fe_fe_set_lna,
	lms7fe_set_gain,

	lms7fe_get_reg,
	lms7fe_set_reg,

	lms7fe_deinit,
};

void _lms7fe_init_base(struct xtrx_fe_lms7 *dev)
{
	dev->base.ops = &_lms7fe_ops;
}





