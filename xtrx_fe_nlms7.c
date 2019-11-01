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
#include <xtrxll_api.h>
#include <xtrxll_log.h>
#include <xtrxll_mmcm.h>

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdarg.h>

#include "xtrx_api.h"
#include "xtrx_fe_nlms7.h"


enum {
	MIN_TX_RATE = 2100000, /* 2.1e6 Minimum samplerate supported by the XTRX hardware */
};

static void bparam_set_null(xtrx_bparam_t* p)
{
	p->set = false;
	p->value = 0;
}
static void bparam_set_val(xtrx_bparam_t* p, unsigned val)
{
	p->set = true;
	p->value = val;
}

#if 0
typedef struct xtrx_bparamu8
{
	bool set;
	uint8_t value;
} xtrx_bparamu8_t;

static void bparamu8_set_null(xtrx_bparam_t* p)
{
	p->set = false;
	p->value = 0;
}
static void bparamu8_set_val(xtrx_bparam_t* p, unsigned val)
{
	p->set = true;
	p->value = val;
}
#endif


enum xtrxll_lms7_pwr {
	XTRXLL_LMS7_RESET_PIN = 1<<1,
	XTRXLL_LMS7_GPWR_PIN  = 1<<2,
	XTRXLL_LMS7_RXEN_PIN  = 1<<3,
	XTRXLL_LMS7_TXEN_PIN  = 1<<4,

	XTRXLL_LMS7_RX_GEN    = 1<<6,
	XTRXLL_LMS7_RX_TERM_D = 1<<7,
};

static struct xtrx_nfe_lms7* get_nfe(struct lms7_state* s)
{
	return (struct xtrx_nfe_lms7*)((char*)s - offsetof(struct xtrx_nfe_lms7, lms_state));
}

void lms7_log_ex(struct lms7_state* s,
				 const char* function,
				 const char* file,
				 int line_no,
				 const char* fmt, ...)
{
	struct xtrx_nfe_lms7* nfe = get_nfe(s);
	char logbuffer[1024];
	int len;
	va_list ap;
	va_start(ap, fmt);

	len = vsnprintf(logbuffer, sizeof(logbuffer) - 1, fmt, ap);
	if (len > sizeof(logbuffer) - 1)
		logbuffer[sizeof(logbuffer) - 1] = 0;
	else if (len < 0)
		logbuffer[0] = 0;
	va_end(ap);

	xtrxll_log(XTRXLL_INFO, "LSM7", function, file, line_no,
			   "%s: %s\n", xtrxll_get_name(nfe->lldev), logbuffer);
}

int lms7_spi_transact(struct lms7_state* s, uint16_t ival, uint32_t* oval)
{
	uint32_t v = (uint32_t)ival<<16;
	struct xtrx_nfe_lms7* nfe = get_nfe(s);
	return xtrxll_lms7_spi_bulk(nfe->lldev, nfe->lmsnum, &v, oval, 1);
}

int lms7_spi_post(struct lms7_state* s, const unsigned count, const uint32_t* regs)
{
	uint32_t dummy[count];

	struct xtrx_nfe_lms7* nfe = get_nfe(s);
	return xtrxll_lms7_spi_bulk(nfe->lldev, nfe->lmsnum, regs, dummy, count);
}



static int _xtrx_set_lna_rx(struct xtrx_nfe_lms7 *dev, int band)
{
	XTRXLLS_LOG("LMSF", XTRXLL_INFO, "%s: Set RX band to %d (%c)\n",
				xtrxll_get_name(dev->lldev), band,
				band == RFE_LNAW ? 'W' : band == RFE_LNAH ? 'H' : band == RFE_LNAL ? 'L' : '-');

	int res = lms7_rfe_set_path(&dev->lms_state, band,
								dev->rx_run_a, dev->rx_run_b);
	if (res)
		return res;

	dev->rxant = (band == RFE_LNAW) ? 0 :
				 (band == RFE_LNAH) ? 2 :
				 (band == RFE_LNAL) ? 1 : 3;
	return xtrxll_set_param(dev->lldev, XTRXLL_PARAM_SWITCH_RX_ANT, dev->rxant);
}


static int _xtrx_set_lna_tx(struct xtrx_nfe_lms7 *dev, int band)
{
	XTRXLLS_LOG("LMSF", XTRXLL_INFO, "%s: Set TX band to %d (%c)\n",
				xtrxll_get_name(dev->lldev), band, (band == 1) ? 'H' : 'W');

	int res = lms7_trf_set_path(&dev->lms_state, band);
	if (res)
		return res;

	dev->txant = (band == 1) ? 1 : 0;
	return xtrxll_set_param(dev->lldev, XTRXLL_PARAM_SWITCH_TX_ANT, dev->txant);
}


enum sigtype {
	XTRX_TX_LO_CHANGED,
	XTRX_RX_LO_CHANGED,
	XTRX_TX_LNA_CHANGED,
	XTRX_RX_LNA_CHANGED,
};

const char* get_band_name(unsigned l)
{
	switch (l) {
	case RFE_NONE: return "NONE";
	case RFE_LNAH: return "LNAH";
	case RFE_LNAL: return "LNAL";
	case RFE_LNAW: return "LNAW";
	case RFE_LBW:  return "LBW";
	case RFE_LBL:  return "LBL";
	}
	return "<unknown>";
}

static int _xtrx_signal_event(struct xtrx_nfe_lms7 *dev, enum sigtype t)
{
	int res;

	switch (t) {
	case XTRX_RX_LO_CHANGED:
	case XTRX_RX_LNA_CHANGED:
		if (dev->rx_lna_auto) {
			int band = (dev->rx_lo > 2200e6) ? RFE_LNAH :
					   (dev->rx_lo > 1500e6) ? RFE_LNAW : RFE_LNAL;
			XTRXLLS_LOG("LMSF", XTRXLL_INFO, "%s: Auto RX band selection: %s\n",
						xtrxll_get_name(dev->lldev), get_band_name(band));
			res = lms7_mac_set(&dev->lms_state, LMS7_CH_AB);
			if (res)
				return res;
			res = _xtrx_set_lna_rx(dev, band);
		}
		break;
	case XTRX_TX_LO_CHANGED:
	case XTRX_TX_LNA_CHANGED:
		if (dev->tx_lna_auto) {
			int band = (dev->tx_lo > 2200e6) ? 1 : 2;
			XTRXLLS_LOG("LMSF", XTRXLL_INFO, "%s: Auto TX band selection: %s\n",
						xtrxll_get_name(dev->lldev),
						band == 1 ? "H (Band1)" : "W (Band2)");
			res = lms7_mac_set(&dev->lms_state, LMS7_CH_AB);
			if (res)
				return res;
			res = _xtrx_set_lna_tx(dev, band);
		}
		break;
	}

	XTRXLLS_LOG("LMSF", XTRXLL_INFO, "%s: DC START\n",
				xtrxll_get_name(dev->lldev));
	res = lms7_dc_start(&dev->lms_state,
						dev->rx_run_a, dev->rx_run_b,
						dev->tx_run_a, dev->tx_run_b);
	if (res)
		return res;

	//usleep(1000);
	//lms7_cal_rxdc(&dev->lms_state);

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

static void _lms7nfe_init_base(struct xtrx_nfe_lms7 *dev);

static int _xtrx_channel_to_lms7(unsigned xch, enum lms7_mac_mode* out)
{
	switch (xch) {
	case XTRX_CH_A:  *out = LMS7_CH_A; break;
	case XTRX_CH_B:  *out = LMS7_CH_B; break;
	case XTRX_CH_AB: *out = LMS7_CH_AB; break;
	default: return -EINVAL;
	}
	return 0;
}

int lms7nfe_init(struct xtrxll_dev* lldev,
				 unsigned flags,
				 const char *fename,
				 struct xtrx_fe_obj** obj)
{
	struct xtrx_nfe_lms7 *dev;
	int lmscnt = 0;
	int res = xtrxll_get_sensor(lldev, XTRXLL_CFG_NUM_RFIC, &lmscnt);
	if (res || (lmscnt != 1)) {
		goto failed_no_lms;
	}

	dev = (struct xtrx_nfe_lms7*)malloc(sizeof(struct xtrx_nfe_lms7));
	if (dev == NULL) {
		res = -errno;
		goto failed_mem;
	}
	memset(dev, 0, sizeof(struct xtrx_nfe_lms7));

	_lms7nfe_init_base(dev);
	dev->lmsnum = 1;
	dev->lldev = lldev;
	dev->rx_lna_auto = true;
	dev->tx_lna_auto = true;
	dev->rx_lo = 0;
	dev->tx_lo = 0;


	usleep(10000);

	res = xtrxll_set_param(dev->lldev, XTRXLL_PARAM_PWR_CTRL, PWR_CTRL_BUSONLY);
	if (res) {
		goto failed_lms7;
	}

	usleep(100000);

	// Power on all LMS device and put it in reset
	res = xtrxll_set_param(dev->lldev, XTRXLL_PARAM_FE_CTRL,
						   XTRXLL_LMS7_GPWR_PIN);
	if (res) {
		goto failed_lms7;
	}

	usleep(100000);

	res = xtrxll_set_param(dev->lldev, XTRXLL_PARAM_PWR_CTRL, PWR_CTRL_ON);
	if (res) {
		goto failed_lms7;
	}

	usleep(10000);

	res = xtrxll_set_param(dev->lldev, XTRXLL_PARAM_FE_CTRL,
							   XTRXLL_LMS7_GPWR_PIN | XTRXLL_LMS7_RESET_PIN);
	if (res) {
		goto failed_lms7;
	}

	usleep(10000);

	res = lms7_enable(&dev->lms_state);
	if (res) {
		goto failed_lms7;
	}

	res = xtrxll_set_param(dev->lldev, XTRXLL_PARAM_FE_CTRL,
							   XTRXLL_LMS7_GPWR_PIN | XTRXLL_LMS7_RESET_PIN |
							   XTRXLL_LMS7_RXEN_PIN | XTRXLL_LMS7_TXEN_PIN);

	bparam_set_null(&dev->tx_bw[0]);
	bparam_set_null(&dev->tx_bw[1]);
	bparam_set_null(&dev->rx_bw[0]);
	bparam_set_null(&dev->rx_bw[1]);
	bparam_set_null(&dev->tx_dsp[0]);
	bparam_set_null(&dev->tx_dsp[1]);
	bparam_set_null(&dev->rx_dsp[0]);
	bparam_set_null(&dev->rx_dsp[1]);

	*obj = (struct xtrx_fe_obj*)dev;
	return 0;

failed_lms7:
	free(dev);
failed_mem:
failed_no_lms:
	return res;
}

int lms7nfe_deinit(struct xtrx_fe_obj* obj)
{
	struct xtrx_nfe_lms7 *dev = (struct xtrx_nfe_lms7 *)obj;
	lms7_disable(&dev->lms_state);

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

int lms7nfe_dd_set_samplerate(struct xtrx_fe_obj* obj,
							 const struct xtrx_fe_samplerate* inrates,
							 struct xtrx_fe_samplerate* outrates)
{
	int res;
	struct xtrx_nfe_lms7 *dev = (struct xtrx_nfe_lms7 *)obj;
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

	const unsigned tx_port = (lml1_tx_valid && !lml2_tx_valid) ? 1 : 2;
	const unsigned rx_port = (lml2_rx_valid && !lml1_rx_valid) ? 2 : 1;

	const int tx_gen = (tx_port == 1) ? lml1_use_mmcm : lml2_use_mmcm;
	const int rx_gen = (rx_port == 2) ? lml2_use_mmcm : lml1_use_mmcm;

	if (dev->refclock == 0 || dev->lms_state.fref == 0) {
		XTRXLLS_LOG("LMSF", XTRXLL_ERROR, "%s: refclock is not set, can't set samplerate\n",
		            xtrxll_get_name(dev->lldev));
		return -EINVAL;
	}

	if (l2_pid == 0 && l1_pid == 0) {
		XTRXLLS_LOG("LMSF", XTRXLL_ERROR, "%s: Incorrect FPGA port configuration HWID=%08x => TX=%d RX=%d\n",
				   xtrxll_get_name(dev->lldev), hwid, tx_port, rx_port);
		return -EFAULT;
	}
	if ((rxrate > 1) && (!lml1_rx_valid && !lml2_rx_valid)) {
		XTRXLLS_LOG("LMSF", XTRXLL_ERROR, "%s: Current FPGA configuration doesn't support RX, HWID=%08x\n",
				   xtrxll_get_name(dev->lldev), hwid);
		return -EFAULT;
	}
	if ((txrate > 1) && (!lml1_tx_valid && !lml2_tx_valid)) {
		XTRXLLS_LOG("LMSF", XTRXLL_ERROR, "%s: Current FPGA configuration doesn't support TX, HWID=%08x\n",
				   xtrxll_get_name(dev->lldev), hwid);
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
	dev->rx_port_1 = (rx_port == 1);

	// Add host interpolation for slow rates if needed
	if ((tx_gen) && ((txrate > 1) && ((txrate < MIN_TX_RATE) || (flags & XTRX_SAMPLERATE_FORCE_TX_INTR)))) {
		do {
			tx_host_mul <<= 1;
			tx_host_inter++;
		} while ((txrate * tx_host_mul) < MIN_TX_RATE);

		/* check what we can deliver */
		if (tx_host_mul > 2) {
			XTRXLLS_LOG("LMSF", XTRXLL_ERROR, "%s: %d extra TX "
									 "interpolation; however, it's not supported yet...\n",
					   xtrxll_get_name(dev->lldev), tx_host_mul);
			return -EINVAL;
		}
	}

	// Extra RX decimation
	if ((rxrate > 1) && (flags & XTRX_SAMPLERATE_FORCE_RX_DECIM)) {
		rx_host_div <<= 1;
		rx_host_decim++;
	}

	double txmaster_min = adcdiv_fixed * inrates->adc.hwrate;
	double rxmaster_min = dacdiv * inrates->dac.hwrate;
	cgen_rate = MAX(txmaster_min, rxmaster_min);

	XTRXLLS_LOG("LMSF", XTRXLL_DEBUG, "%s: TXm = %.3f RXm = %.3f cgen_rate= %.3f\n",
			   xtrxll_get_name(dev->lldev), txmaster_min / 1e6, rxmaster_min / 1e6, cgen_rate / 1e6);

	// TODO: determine best possible combination of adcdiv / dacdiv, just
	// for now we use fixed 4 divider for both
	if (cgen_rate < 1) {
		// If we activate host decimation or interpolation than we need more samplerate
		cgen_rate = MAX((rx_no_decim ? 1 : 2) * rxrate * rx_host_div * adcdiv_fixed,
						(tx_no_decim ? 1 : ((tx_gen) ? 2 : 4)) * txrate * tx_host_mul * dacdiv);

		XTRXLLS_LOG("LMSF", XTRXLL_DEBUG, "%s: Initial CGEN set to %03.1f Mhz\n",
					xtrxll_get_name(dev->lldev), cgen_rate / 1.0e6);

		// For low sample rate increase DAC/ADC due to frequency aliasing
		if ((rxrate > 1 && rxrate < 2e6) || (txrate > 1 && txrate < 2e6) || opt_decim_inter) {
			for (; cgen_rate <= 320e6; cgen_rate *= 2) {
				unsigned rx_ndiv = (rxrate > 1) ? ((cgen_rate * 2 / (rxrate * rx_host_div)) / adcdiv_fixed) : 0;
				unsigned tx_ndiv = (txrate > 1) ? ((cgen_rate * 2 / (txrate * tx_host_mul)) / dacdiv) : 0;

				if (rx_ndiv > LMS7_DECIM_MAX || tx_ndiv > LMS7_INTER_MAX)
					break;

				XTRXLLS_LOG("LMSF", XTRXLL_INFO, "%s: Increase RXdiv=%2d TXdiv=%2d => CGEN %03.1f Mhz\n",
						   xtrxll_get_name(dev->lldev), rx_ndiv, tx_ndiv, cgen_rate * 2 / 1.0e6);
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
		XTRXLLS_LOG("LMSF", XTRXLL_ERROR, "%s: can't deliver "
								 "decimation: %d of %.3f MHz CGEN and %.3f MHz samplerate; TXm = %.3f RXm = %.3f\n",
				   xtrxll_get_name(dev->lldev), rxdiv, cgen_rate / 1e6, rxrate / 1e6,
				   txmaster_min / 1e6, rxmaster_min / 1e6);
		return -EINVAL;
	}

	if (txrate > 1 && !_check_lime_decimation(txdiv)) {
		XTRXLLS_LOG("LMSF", XTRXLL_ERROR, "%s: can't deliver "
								 "interpolation: %d of %.3f MHz CGEN and %.3f MHz samplerate; TXm = %.3f RXm = %.3f\n",
				   xtrxll_get_name(dev->lldev), txdiv, cgen_rate / 1e6, txrate / 1e6,
				   txmaster_min / 1e6, rxmaster_min / 1e6);
		return -EINVAL;
	}


	if (((dev->rx_run_a || dev->rx_run_b) && (dev->rx_host_decim != rx_host_decim)) ||
		((dev->tx_run_b || dev->tx_run_b) && (dev->tx_host_inter != tx_host_inter))) {
		XTRXLLS_LOG("LMSF", XTRXLL_ERROR, "%s: can't change extra "
				   "host decimation when stream is running\n",
					xtrxll_get_name(dev->lldev));
		return -EINVAL;
	}

	// TODO FIXME
	bool update_running = false;
	if (dev->rx_run_a || dev->rx_run_b || dev->tx_run_a || dev->tx_run_b) {
		if (!(flags & XTRX_SAMPLERATE_FORCE_UPDATE))
			return -EPERM;

		update_running = true;
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

	// 1. Set CGEN frequency
	// --------------------------------------------------------------
	for (unsigned j = 0; j < 40; j++) {
		unsigned clkdiv = (dacdiv == 1) ? 0 :
						  (dacdiv == 2) ? 1 :
						  (dacdiv == 4) ? 2 : 3;

		res = lms7_cgen_tune_sync(&dev->lms_state,
								  cgen_rate,
								  clkdiv);
		if (res == 0) {
			// TODO FIXME!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
			actualmaster = cgen_rate;
			break;
		}
	}
	if (res != 0) {
		XTRXLLS_LOG("LMSF", XTRXLL_ERROR, "%s: can't tune VCO for data clock\n",
					xtrxll_get_name(dev->lldev));
		return -ERANGE;
	}
	dev->cgen_clk = cgen_rate;

	// 2. Set LML RX
	unsigned rxtsp_div = 1;
	if (rxrate > 0) {
		rxtsp_div = ((slow_mclk_rx_x2) ? slow_factor : 1) * ((rxdiv > 1) ? (rxdiv / 2) : 1);
		//if (~dev->rx_run) {
		//	TODO: Enable RBB?
		//}
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
		//if (~dev->tx_run) {
		//	TODO: Enable TBB?
		//}

		if (outrates) {
			outrates->dac.rate = actualmaster / txdiv / dacdiv / tx_host_mul;
			outrates->dac.hwrate = actualmaster / dacdiv;
			outrates->dac.host_di = tx_host_inter;
		}
	}

	unsigned lml_txtsp_div = (tx_gen) ? txtsp_div : txtsp_div / 2;
	enum lml_mode mode = LML_NORMAL;
	if ((!no_8ma) && ((rxrate > 40e6) || (txrate > 40e6) || (tx_gen == 0 && txrate > 20e6))) {
		mode |= LML_DS_HIGH;
	}
	if (rxrate > 1 && rx_gen && !no_rx_fwd_clk) {
		mode |= LML_RD_FCLK;
	}

	res = lms7_lml_configure(&dev->lms_state,
							 dev->rx_port_1,
							 lml_txtsp_div,
							 rxtsp_div,
							 mode);
	if (res)
		return res;

	dev->lml_txdiv = lml_txtsp_div;
	dev->lml_rxdiv = rxtsp_div;
	dev->lml_mode = mode;

	if (!update_running) {
		res = lms7_reset(&dev->lms_state);
		if (res)
			return res;
	}

	// 4.
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

	bool rxafen = rxrate > 1 && rx_gen;
	bool txafen = txrate > 1 && tx_gen;

	// 5. TX MMCM
	double tx_mclk = 0;
	if (txrate > 1 && tx_gen) {
		// MCLK is generated only when AFE is enabled
		res = lms7_afe_ctrl(&dev->lms_state, rxafen, rxafen, txafen, txafen);

		tx_mclk = actualmaster / dacdiv / txtsp_div;
		int mclk = tx_mclk;

		XTRXLLS_LOG("LMSF", XTRXLL_ERROR, "%s: TX MCLK=%.3f (extra %d) MHz\n",
				   xtrxll_get_name(dev->lldev),
					mclk / 1.0e6, (((slow_mclk_tx_x2) ? slow_factor : 1)));
		res = xtrxll_mmcm_onoff(dev->lldev, true, true);
		if (res) {
			XTRXLLS_LOG("LMSF", XTRXLL_ERROR, "%s: can't turn on TX MMCM\n",
						xtrxll_get_name(dev->lldev));
			return -EFAULT;
		}

		res = xtrxll_mmcm_setfreq(dev->lldev,
								  tx_port == 1,
								  mclk,
								  ((tx_no_decim && txdiv == 1) ? LML_CLOCK_X2 : LML_CLOCK_NORM) | LML_CLOCK_FWD_90,
								  0,
								  &dev->tx_mmcm_div,
								  (slow_mclk_tx_x2) ? 2 * slow_factor : 2);
		if (res != 0) {
			XTRXLLS_LOG("LMSF", XTRXLL_ERROR, "%s: Unable to configure TX MMCM to %d res = %d\n",
						xtrxll_get_name(dev->lldev), mclk, res);
			return -ERANGE;
		}
	} else {
		dev->tx_mmcm_div = 0;
	}

	// 6. RX MMCM
	double rx_mclk = 0;
	if (rxrate > 1 && rx_gen) {
		// MCLK is generated only when AFE is enabled
		res = lms7_afe_ctrl(&dev->lms_state, rxafen, rxafen, txafen, txafen);

		//int mclk = 2 * actualmaster / rxdiv / adcdiv_fixed / ((rx_no_decim  && rxdiv == 1) ? 2 : 1);
		rx_mclk = actualmaster / adcdiv_fixed / rxtsp_div;
		int mclk = rx_mclk;
		XTRXLLS_LOG("LMSF", XTRXLL_ERROR, "%s: RX MCLK=%.3f (%d extra) MHz\n",
					xtrxll_get_name(dev->lldev), mclk / 1.0e6, (((slow_mclk_rx_x2) ? slow_factor : 1)));
		res = xtrxll_mmcm_onoff(dev->lldev, false, true);
		if (res) {
			XTRXLLS_LOG("LMSF", XTRXLL_ERROR, "%s: can't turn on RX MMCM\n",
						xtrxll_get_name(dev->lldev));
			return -EFAULT;
		}
		usleep(10*1000);

		res = xtrxll_mmcm_setfreq(dev->lldev,
								  rx_port == 1,
								  mclk,
								  ((rx_no_decim && rxdiv == 1) ? LML_CLOCK_X2 : ((no_rx_fwd_clk) ? LML_CLOCK_RX_SELF : LML_CLOCK_NORM)) | (x2_int_clk ? LML_CLOCK_INT_X2 : 0),
								  rx_delay,
								  &dev->rx_mmcm_div,
								  (slow_mclk_rx_x2) ? 2 * slow_factor : 2);
		if (res != 0) {
			XTRXLLS_LOG("LMSF", XTRXLL_ERROR, "%s: Unable to configure RX MMCM to %d res = %d\n",
						xtrxll_get_name(dev->lldev), mclk, res);
			return -ERANGE;
		}
	} else {
		dev->rx_mmcm_div = 0;
	}


	XTRXLLS_LOG("LMSF", XTRXLL_INFO, "%s: rxrate=%.3fMHz txrate=%.3fMHz"
							" actual_master=%.3fMHz rxdecim=%d(h_%d) txinterp=%d(h_%d)"
							" RX_ADC=%.3fMHz TX_DAC=%.3fMHz hintr=%d hdecim=%d delay=%d NRXFWD=%d LML1HID=%d LML2HID=%d"
							" RX_div=%d TX_div=%d RX_TSP_div=%d TX_TSP_div=%d FclkRX=%.3f (PHS=%d)"
							" RXx2=%d\n",
				xtrxll_get_name(dev->lldev),
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

const struct lml_map lms7nfe_get_lml_portcfg(const struct xtrx_dd_chpar* par,
											 bool no_siso_map)
{
	static const struct lml_map diqarray[16] = {
		// MIMO modes
		{{ LML_BI, LML_AI, LML_BQ, LML_AQ }},
		{{ LML_BQ, LML_AQ, LML_BI, LML_AI }},
		{{ LML_AI, LML_BI, LML_AQ, LML_BQ }},
		{{ LML_AQ, LML_BQ, LML_AI, LML_BI }},
		// SISO modes
		{{ LML_AI, LML_AI, LML_AQ, LML_AQ }},
		{{ LML_AQ, LML_AQ, LML_AI, LML_AI }},
		{{ LML_BI, LML_BI, LML_BQ, LML_BQ }},
		{{ LML_BQ, LML_BQ, LML_BI, LML_BI }},
		// MIMO test modes (swap IQ_B)
		{{ LML_BQ, LML_AI, LML_BI, LML_AQ }},
		{{ LML_BI, LML_AQ, LML_BQ, LML_AI }},
		{{ LML_AQ, LML_BI, LML_AI, LML_BQ }},
		{{ LML_AI, LML_BQ, LML_AQ, LML_BI }},
		// MIMO test modes (swap IQ_A)
		{{ LML_BI, LML_AQ, LML_BQ, LML_AI }},
		{{ LML_BQ, LML_AI, LML_BI, LML_AQ }},
		{{ LML_AI, LML_BQ, LML_AQ, LML_BI }},
		{{ LML_AQ, LML_BI, LML_AI, LML_BQ }},
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
	else if (par->flags & XTRX_RSP_SWAP_IQA)
		diqidx |= 12;

	assert(diqidx < (sizeof(diqarray)/sizeof(diqarray[0])));
	return diqarray[diqidx];
}

static unsigned _ulog(unsigned d)
{
	switch (d) {
	case 1: return 0;
	case 2: return 1;
	case 4: return 2;
	case 8: return 3;
	case 16: return 4;
	}
	return 5;
}

static enum lms7_mac_mode _corr_ch(enum lms7_mac_mode mode,
								   unsigned flags)
{
	if (mode == LMS7_CH_AB && (flags & XTRX_RSP_SISO_MODE) && (!(flags & XTRX_RSP_SISO_SWITCH))) {
		if (flags & XTRX_RSP_SWAP_AB) {
			mode = LMS7_CH_B;
		} else {
			mode = LMS7_CH_A;
		}
	}

	return mode;
}

int lms7nfe_dd_configure(struct xtrx_nfe_lms7* dev,
						const struct xtrx_dd_params *params)
{
	enum lms7_mac_mode rx_lmschan = LMS7_CH_NONE;
	enum lms7_mac_mode tx_lmschan = LMS7_CH_NONE;
	unsigned dir = params->dir;
	bool rxafen_a = dev->rx_run_a;
	bool rxafen_b = dev->rx_run_b;

	bool txafen_a = dev->tx_run_a;
	bool txafen_b = dev->tx_run_b;
	int res;
	unsigned ich;

	if (dir & XTRX_RX) {
		if (_xtrx_channel_to_lms7(params->rx.chs, &rx_lmschan)) {
			return -EINVAL;
		}
		rx_lmschan = _corr_ch(rx_lmschan, params->rx.flags);
		dev->chprx = params->rx;
		dev->maprx = lms7nfe_get_lml_portcfg(&dev->chprx, dev->rx_no_siso_map);

		rxafen_a = rx_lmschan != LMS7_CH_B;
		rxafen_b = rx_lmschan != LMS7_CH_A;
	}
	if (dir & XTRX_TX) {
		if (_xtrx_channel_to_lms7(params->tx.chs, &tx_lmschan)) {
			return -EINVAL;
		}
		tx_lmschan = _corr_ch(tx_lmschan, params->tx.flags);
		dev->chptx = params->tx;
		dev->maptx = lms7nfe_get_lml_portcfg(&dev->chptx, dev->tx_no_siso_map);

		txafen_a = tx_lmschan != LMS7_CH_B;
		txafen_b = tx_lmschan != LMS7_CH_A;
	}

	res = lms7_lml_set_map(&dev->lms_state,
						   dev->rx_port_1 ? dev->maprx : dev->maptx,
						   dev->rx_port_1 ? dev->maptx : dev->maprx);
	if (res)
		return res;

	res = lms7_afe_ctrl(&dev->lms_state,
						rxafen_a || rxafen_b, rxafen_b,
						txafen_a || txafen_b, txafen_b);
	if (res)
		return res;

	res = lms7_dc_init(&dev->lms_state,
					   rxafen_a, rxafen_b, txafen_a, txafen_b);
	if (res)
		return res;

	XTRXLLS_LOG("LMSF", XTRXLL_INFO, "%s: AFE TX=[%d;%d] RX=[%d;%d]\n",
			   xtrxll_get_name(dev->lldev), txafen_a, txafen_b, rxafen_a, rxafen_b);

	if (dir & XTRX_RX) {
		res = lms7_mac_set(&dev->lms_state, rx_lmschan);
		if (res)
			return res;
		res = lms7_rxtsp_init(&dev->lms_state, _ulog(dev->rxtsp_decim));
		if (res)
			return res;
		res = lms7_rbb_set_path(&dev->lms_state, RBB_LBF);
		if (res)
			return res;

		res = lms7_rfe_set_path(&dev->lms_state,
								(dev->rxant == 0) ? RFE_LNAW :
								(dev->rxant == 1) ? RFE_LNAL :
								(dev->rxant == 2) ? RFE_LNAH : RFE_NONE,
								rxafen_a, rxafen_b);
		if (res)
			return res;
		if (~(params->rx.flags & XTRX_RSP_NO_DC_CORR)) {
			res = lms7_rxtsp_dc_corr(&dev->lms_state, 7);
			if (res)
				return res;
		}

		// Restore settings
		for (ich = 0; ich < 2; ich++) {
			enum lms7_mac_mode lch = (ich == 0) ? LMS7_CH_A : LMS7_CH_B;
			if (!(rx_lmschan & lch))
				continue;

			res = lms7_mac_set(&dev->lms_state, lch);
			if (res)
				return res;

			unsigned tsf = (ich == 0) ? XTRX_RSP_TEST_SIGNAL_A : XTRX_RSP_TEST_SIGNAL_B;
			if (params->rx.flags & tsf) {
				XTRXLLS_LOG("LMSF", XTRXLL_INFO,"%s: RBB Applying test tone on %c\n",
							xtrxll_get_name(dev->lldev), (ich == 0) ? 'A' : 'B');
				res = lms7_rxtsp_tsg_tone(&dev->lms_state, false, false);
				if (res)
					return res;
			}
			if (dev->rx_bw[ich].set) {
				XTRXLLS_LOG("LMSF", XTRXLL_INFO, "%s: RBB Restore BW[%d]=%d\n",
							xtrxll_get_name(dev->lldev), ich, dev->rx_bw[ich].value);
				res = lms7_rbb_set_bandwidth(&dev->lms_state, dev->rx_bw[ich].value);
				if (res)
					return res;
			}
			if (dev->rx_dsp[ich].set) {
				XTRXLLS_LOG("LMSF", XTRXLL_INFO, "%s: RBB Restore DSP[%d]=%d\n",
							xtrxll_get_name(dev->lldev), ich, dev->rx_dsp[ich].value);
				res = lms7_rxtsp_cmix(&dev->lms_state, dev->rx_dsp[ich].value);
				if (res)
					return res;
			}
		}

		dev->rx_run_a = rxafen_a;
		dev->rx_run_b = rxafen_b;
	}
	if (dir & XTRX_TX) {
		res = lms7_mac_set(&dev->lms_state, tx_lmschan);
		if (res)
			return res;
		res = lms7_txtsp_init(&dev->lms_state, _ulog(dev->txtsp_interp));
		if (res)
			return res;
		res = lms7_tbb_set_path(&dev->lms_state, TBB_BYP);
		if (res)
			return res;
		res = lms7_trf_enable(&dev->lms_state, txafen_a, txafen_b);
		if (res)
			return res;

		// Restore settings
		for (ich = 0; ich < 2; ich++) {
			enum lms7_mac_mode lch = (ich == 0) ? LMS7_CH_A : LMS7_CH_B;
			if (!(tx_lmschan & lch))
				continue;

			res = lms7_mac_set(&dev->lms_state, lch);
			if (res)
				return res;

			unsigned tsf = (ich == 0) ? XTRX_RSP_TEST_SIGNAL_A : XTRX_RSP_TEST_SIGNAL_B;
			if (params->tx.flags & tsf) {
				XTRXLLS_LOG("LMSF", XTRXLL_INFO,"%s: TBB Applying test tone on %c\n",
							xtrxll_get_name(dev->lldev), (ich == 0) ? 'A' : 'B');
				res = lms7_txtsp_tsg_tone(&dev->lms_state, false, false);
				if (res)
					return res;
			}
			if (dev->tx_bw[ich].set) {
				XTRXLLS_LOG("LMSF", XTRXLL_INFO, "%s: TBB Restore BW[%d]=%d\n",
							xtrxll_get_name(dev->lldev), ich, dev->tx_bw[ich].value);
				res = lms7_tbb_set_bandwidth(&dev->lms_state, dev->tx_bw[ich].value);
				if (res)
					return res;
			}
			if (dev->tx_dsp[ich].set) {
				XTRXLLS_LOG("LMSF", XTRXLL_INFO, "%s: TBB Restore DSP[%d]=%d\n",
							xtrxll_get_name(dev->lldev), ich, dev->tx_dsp[ich].value);
				res = lms7_txtsp_cmix(&dev->lms_state, dev->tx_dsp[ich].value);
				if (res)
					return res;
			}
		}

		dev->tx_run_a = txafen_a;
		dev->tx_run_b = txafen_b;
	}

	unsigned nlml_mode = dev->lml_mode;
	if (params->nflags & XTRX_RUN_DIGLOOPBACK) {
		XTRXLLS_LOG("LMSF", XTRXLL_INFO, "%s: Enable digital loopback\n",
					xtrxll_get_name(dev->lldev));
		nlml_mode |= LML_LOOPBACK;
	}
	if (params->nflags & XTRX_RUN_RXLFSR) {
		XTRXLLS_LOG("LMSF", XTRXLL_INFO, "%s: Enable RX LFSR\n",
					xtrxll_get_name(dev->lldev));
		nlml_mode |= LML_RXLFSR;
	}

	if (nlml_mode != dev->lml_mode) {
		res = lms7_lml_configure(&dev->lms_state,
								 dev->rx_port_1,
								 dev->lml_txdiv,
								 dev->lml_rxdiv,
								 dev->lml_mode);
		if (res)
			return res;

		dev->lml_mode = nlml_mode;
	}

	XTRXLLS_LOG("LMSF", XTRXLL_INFO_LMS, "%s: configure done\n",
			   xtrxll_get_name(dev->lldev));
	return 0;
}

int lms7nfe_dd_reset(struct xtrx_nfe_lms7* dev,
					 const struct xtrx_dd_params *params)
{
	int res;
	enum lms7_mac_mode rx_lmschan = LMS7_CH_NONE;
	enum lms7_mac_mode tx_lmschan = LMS7_CH_NONE;
	unsigned dir = params->dir;

	if ((dir & XTRX_RX) && _xtrx_channel_to_lms7(params->rx.chs, &rx_lmschan)) {
		return -EINVAL;
	}
	if ((dir & XTRX_TX) && _xtrx_channel_to_lms7(params->tx.chs, &tx_lmschan)) {
		return -EINVAL;
	}

	if (dir & XTRX_RX) {
		lms7_rxtsp_disable(&dev->lms_state);
		lms7_rfe_disable(&dev->lms_state);
		dev->rx_run_a = false;
		dev->rx_run_b = false;
	}

	if (dir & XTRX_TX) {
		lms7_txtsp_disable(&dev->lms_state);
		lms7_trf_disable(&dev->lms_state);
		dev->tx_run_a = false;
		dev->tx_run_b = false;
	}

	res = lms7_afe_ctrl(&dev->lms_state,
						dev->rx_run_a, dev->rx_run_b,
						dev->tx_run_a, dev->tx_run_b);
	if (res)
		return res;

	return 0;
}

int lms7nfe_dd_set_modes(struct xtrx_fe_obj* obj,
						unsigned op,
						const struct xtrx_dd_params *params)
{
	struct xtrx_nfe_lms7 *dev = (struct xtrx_nfe_lms7 *)obj;
	switch (op) {
	case XTRX_FEDD_CONFIGURE:
		return lms7nfe_dd_configure(dev, params);
	case XTRX_FEDD_RESET:
		return lms7nfe_dd_reset(dev, params);
	}

	return -EINVAL;
}

int lms7nfe_bb_set_freq(struct xtrx_fe_obj* obj,
					   unsigned channel,
					   unsigned type,
					   double freq,
					   double* actualfreq)
{
	int res;
	enum lms7_mac_mode ch;
	struct xtrx_nfe_lms7 *dev = (struct xtrx_nfe_lms7 *)obj;
	double rel_freq;

	res = _xtrx_channel_to_lms7(channel, &ch);
	if (res)
		return res;

	if (type == XTRX_TUNE_BB_TX) {
		double tx_dac_freq = dev->cgen_clk / dev->txcgen_div;
		rel_freq = freq / tx_dac_freq;
		if (rel_freq > 0.5 || rel_freq < -0.5) {
			XTRXLLS_LOG("LMSF", XTRXLL_WARNING,
					   "%s: NCO TX ouf of range, requested %.3f while DAC %.3f\n",
						xtrxll_get_name(dev->lldev),
					   rel_freq / 1000, tx_dac_freq / 1000);
			return -EINVAL;
		}
		int pfreq = rel_freq * 4294967296;
		if (ch & LMS7_CH_A)
			bparam_set_val(&dev->tx_dsp[0], pfreq);
		if (ch & LMS7_CH_B)
			bparam_set_val(&dev->tx_dsp[1], pfreq);

		if (dev->tx_run_a || dev->tx_run_b) {
			res = lms7_mac_set(&dev->lms_state, ch);
			if (res)
				return res;
			res = lms7_txtsp_cmix(&dev->lms_state,
								  ch == LMS7_CH_B ? dev->tx_dsp[1].value : dev->tx_dsp[0].value);
		}
	} else {
		double rx_dac_freq = dev->cgen_clk / dev->rxcgen_div;
		rel_freq = freq / rx_dac_freq;
		if (rel_freq > 0.5 || rel_freq < -0.5) {
			XTRXLLS_LOG("LMSF", XTRXLL_WARNING,
					   "%s: NCO RX ouf of range, requested %.3f (%.3f kHz) while ADC %.3f kHz\n",
						xtrxll_get_name(dev->lldev),
					   rel_freq, freq / 1000, rx_dac_freq / 1000);
			return -EINVAL;
		}
		int pfreq = rel_freq * 4294967296;
		if (ch & LMS7_CH_A)
			bparam_set_val(&dev->rx_dsp[0], pfreq);
		if (ch & LMS7_CH_B)
			bparam_set_val(&dev->rx_dsp[1], pfreq);

		if (dev->rx_run_a || dev->rx_run_b) {
			res = lms7_mac_set(&dev->lms_state, ch);
			if (res)
				return res;
			res = lms7_rxtsp_cmix(&dev->lms_state,
								  ch == LMS7_CH_B ? dev->rx_dsp[1].value : dev->rx_dsp[0].value);
		}
	}
	if (res)
		return res;

	XTRXLLS_LOG("LMSF", XTRXLL_INFO, "%s: NCO ch=%d type=%d freq=%.f\n",
			   xtrxll_get_name(dev->lldev), channel, type, freq);

	if (actualfreq)
		*actualfreq = rel_freq;
	return 0;
}

int lms7nfe_bb_set_badwidth(struct xtrx_fe_obj* obj,
						   unsigned channel,
						   unsigned dir,
						   double bw,
						   double* actualbw)
{
	int res;
	enum lms7_mac_mode ch;
	struct xtrx_nfe_lms7 *dev = (struct xtrx_nfe_lms7 *)obj;

	res = _xtrx_channel_to_lms7(channel, &ch);
	if (res)
		return res;

	for (int j = LMS7_CH_A; j <= LMS7_CH_B; j++) {
		if (ch == LMS7_CH_A && j == LMS7_CH_B)
			continue;
		else if (ch == LMS7_CH_B && j == LMS7_CH_A)
			continue;

		res = lms7_mac_set(&dev->lms_state, j);
		if (res)
			return res;

		if (dir == XTRX_TUNE_BB_RX) {
			bparam_set_val(&dev->rx_bw[(j == LMS7_CH_A) ? 0 : 1], bw);

//			res = lms7_rbb_set_ext(&dev->lms_state);
#if 1

			///////////////// FIXMEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!
			res = lms7_rbb_set_path(&dev->lms_state, RBB_LBF);
			if (res)
				return res;

			res = lms7_rbb_set_bandwidth(&dev->lms_state, bw);
			if (actualbw)
				*actualbw = bw;
#endif
		} else if (dir == XTRX_TUNE_BB_TX) {
			bparam_set_val(&dev->tx_bw[(j == LMS7_CH_A) ? 0 : 1], bw);

			res = lms7_tbb_set_bandwidth(&dev->lms_state, bw);
			if (actualbw)
				*actualbw = bw;
		}
	}

	return res;
}

static double clamp(double i, double min, double max)
{
	if (i < min)
		return min;
	else if (i > max)
		return max;
	return i;
}

int lms7nfe_set_gain(struct xtrx_fe_obj* obj,
					unsigned channel,
					unsigned gain_type,
					double gain,
					double *actualgain)
{
	double actual;
	unsigned aret;
	enum lms7_mac_mode ch;
	struct xtrx_nfe_lms7 *dev = (struct xtrx_nfe_lms7 *)obj;

	int res = _xtrx_channel_to_lms7(channel, &ch);
	if (res)
		return res;

	XTRXLLS_LOG("LMSF", XTRXLL_INFO, "%s: Set gain %.1f to %d on %d channel\n",
				xtrxll_get_name(dev->lldev), gain, gain_type, channel);
	res = lms7_mac_set(&dev->lms_state, ch);
	if (res)
		return res;

	switch (gain_type) {
	case XTRX_RX_LNA_GAIN:
		gain = clamp(gain, 0, 30);
		res = lms7_rfe_set_lna(&dev->lms_state, 30 - gain, &aret);
		actual = 30 - aret;
		break;
	case XTRX_RX_TIA_GAIN:
		actual = gain;
		break;
	case XTRX_RX_PGA_GAIN:
		actual = gain;
		res = lms7_rbb_set_pga(&dev->lms_state, gain + 12.5);
		break;
	case XTRX_RX_LB_GAIN:
		actual = gain;
		res = lms7_rfe_set_lblna(&dev->lms_state, 160 - gain * 4, NULL);
		break;

	case XTRX_TX_PAD_GAIN:
		if (gain > 0)
			gain = 0;
		actual = gain;
		res = lms7_trf_set_pad(&dev->lms_state, -gain);
		break;

	default:
		return -EINVAL;
	}

	if (actualgain)
		*actualgain = actual;
	return res;
}

int lms7nfe_fe_set_refclock(struct xtrx_fe_obj* obj,
					   double refclock)
{
	struct xtrx_nfe_lms7 *dev = (struct xtrx_nfe_lms7 *)obj;
	dev->refclock = dev->lms_state.fref = refclock;
	return 0;
}

int lms7nfe_fe_set_freq(struct xtrx_fe_obj* obj,
					   unsigned channel,
					   unsigned type,
					   double freq,
					   double *actualfreq)
{
	int res;
	double res_freq = 0;
	bool rx;
	struct xtrx_nfe_lms7 *dev = (struct xtrx_nfe_lms7 *)obj;

	if (dev->refclock == 0 || dev->lms_state.fref == 0) {
		XTRXLLS_LOG("LMSF", XTRXLL_ERROR, "%s: refclock is not set, can't tune\n", xtrxll_get_name(dev->lldev));
		return -EINVAL;
	}

	switch (type) {
	case XTRX_TUNE_RX_FDD:
		rx = true;
		break;
	case XTRX_TUNE_TX_FDD:
	case XTRX_TUNE_TX_AND_RX_TDD:
		rx = false;
		break;
	default: return -EINVAL;
	}

	if (freq == 0.0) {
		lms7_sxx_disable(&dev->lms_state, rx);
		if (actualfreq)
			*actualfreq = 0.0;
		return 0;
	}

	if (type == XTRX_TUNE_TX_AND_RX_TDD) {
		lms7_sxx_disable(&dev->lms_state, true);
	}

	XTRXLLS_LOG("LMSF", XTRXLL_INFO, "%s: FE_FREQ rx=%d type=%d freq=%f ch=%d\n",
				xtrxll_get_name(dev->lldev), rx, type, freq, channel);

	res = lms7_sxx_tune_sync(&dev->lms_state, rx, (unsigned)freq,
							 type == XTRX_TUNE_TX_AND_RX_TDD);
	res_freq = freq; //TODO !!!!!
	if (res) {
		return res;
	}

	if (actualfreq)
		*actualfreq = res_freq;

	if (type == XTRX_TUNE_TX_AND_RX_TDD) {
		dev->rx_lo = dev->tx_lo = res_freq;
	} else {
		if (!rx) {
			dev->tx_lo = res_freq;
		} else {
			dev->rx_lo = res_freq;
		}
	}

	if ((type == XTRX_TUNE_TX_AND_RX_TDD || type == XTRX_TUNE_RX_FDD) &&
			(dev->rx_run_a || dev->rx_run_b)) {
		res = _xtrx_signal_event(dev, XTRX_RX_LO_CHANGED);
		if (res)
			return res;
	}
	if ((type == XTRX_TUNE_TX_AND_RX_TDD || type == XTRX_TUNE_TX_FDD) &&
			(dev->tx_run_a || dev->tx_run_b)) {
		res = _xtrx_signal_event(dev, XTRX_TX_LO_CHANGED);
		if (res)
			return res;
	}

	return 0;
}

int lms7nfe_fe_set_lna(struct xtrx_fe_obj* obj,
					  unsigned channel,
					  unsigned dir,
					  unsigned lna)
{
	struct xtrx_nfe_lms7 *dev = (struct xtrx_nfe_lms7 *)obj;
	int band;
	int tx;

	enum lms7_mac_mode ch;
	int res = _xtrx_channel_to_lms7(channel, &ch);
	if (res)
		return res;

	switch (lna) {
	case XTRX_RX_L: band = RFE_LNAL; tx = 0; break;
	case XTRX_RX_H: band = RFE_LNAH; tx = 0; break;
	case XTRX_RX_W: band = RFE_LNAW; tx = 0; break;

	case XTRX_RX_L_LB: band = RFE_LBL; tx = 0; break;
	case XTRX_RX_W_LB: band = RFE_LBW; tx = 0; break;

	case XTRX_TX_H: band = 1; tx = 1; break;
	case XTRX_TX_W: band = 2; tx = 1; break;

	case XTRX_RX_AUTO: dev->rx_lna_auto = true; return _xtrx_signal_event(dev, XTRX_RX_LNA_CHANGED);
	case XTRX_TX_AUTO: dev->tx_lna_auto = true; return _xtrx_signal_event(dev, XTRX_TX_LNA_CHANGED);
	default: return -EINVAL;
	}

	res = lms7_mac_set(&dev->lms_state, ch);
	if (res)
		return res;

	if (tx) {
		dev->tx_lna_auto = false;
		return _xtrx_set_lna_tx(dev, band);
	} else {
		dev->rx_lna_auto = false;
		return _xtrx_set_lna_rx(dev, band);
	}
}

int lms7nfe_get_reg(struct xtrx_fe_obj* obj,
					unsigned channel,
					unsigned dir,
					unsigned type,
					uint64_t* outval)
{
	int res;
	uint32_t tmp;
	struct xtrx_nfe_lms7 *dev = (struct xtrx_nfe_lms7 *)obj;
	enum lms7_mac_mode lmsch = LMS7_CH_NONE;
	if (channel != 0) {
		int res = _xtrx_channel_to_lms7(channel, &lmsch);
		if (res)
			return res;
	}

	switch (type) {
	case XTRX_LMS7_RSSI:
		res = lms7_rxtsp_get_rssi(&dev->lms_state, 0, &tmp);
		if (res)
			return res;

		*outval = tmp;
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
	case XTRX_FE_CUSTOM_0:
		*outval = (dev->rxant);
		return 0;
	case XTRX_FE_CUSTOM_0 + 1:
		*outval = (dev->txant);
		return 0;
	default:
		if (type >= XTRX_RFIC_REG_0 && type <= XTRX_RFIC_REG_0 + 65535)	{
			if (lmsch != LMS7_CH_NONE) {
				res = lms7_mac_set(&dev->lms_state, lmsch);
				if (res)
					return res;
			}

			uint32_t wr = ((0x7fff & (type - XTRX_RFIC_REG_0)) << 16);
			res = xtrxll_lms7_spi_bulk(dev->lldev, 1, &wr, &tmp, 1);
			*outval = tmp & 0xffff;

			return res;
		}
		return -EINVAL;
	}
}

int lms7nfe_set_reg(struct xtrx_fe_obj* obj,
			   unsigned channel,
			   unsigned dir,
			   unsigned type,
			   uint64_t val)
{
	int res;
	struct xtrx_nfe_lms7 *dev = (struct xtrx_nfe_lms7 *)obj;
	enum lms7_mac_mode lmsch = LMS7_CH_NONE;
	if (channel != 0) {
		int res = _xtrx_channel_to_lms7(channel, &lmsch);
		if (res)
			return res;
	}

	switch (type) {
	case XTRX_LMS7_XSP_DC_IQ:
		if (dir & XTRX_TX) {
			res = lms7_txtsp_tsg_const(&dev->lms_state,
									   val & 0xffff, (val >> 16) & 0xffff);
			if (res)
				return res;
		}
		if (dir & XTRX_RX) {
			res = lms7_rxtsp_tsg_const(&dev->lms_state,
									   val & 0xffff, (val >> 16) & 0xffff);
			if (res)
				return res;
		}
		return 0;
	case XTRX_FE_CUSTOM_0:
		dev->rxant = val & 3;
		return xtrxll_set_param(dev->lldev, XTRXLL_PARAM_SWITCH_RX_ANT, dev->rxant);
	case XTRX_FE_CUSTOM_0 + 1:
		dev->txant = val & 1;
		return xtrxll_set_param(dev->lldev, XTRXLL_PARAM_SWITCH_TX_ANT, dev->txant);

	case XTRX_FE_CUSTOM_0 + 2:
		if (val) {
			dev->chprx.flags |= XTRX_RSP_SWAP_AB;
		} else {
			dev->chprx.flags &= ~XTRX_RSP_SWAP_AB;
		}
		dev->maprx = lms7nfe_get_lml_portcfg(&dev->chprx, dev->rx_no_siso_map);
		return lms7_lml_set_map(&dev->lms_state,
								dev->rx_port_1 ? dev->maprx : dev->maptx,
								dev->rx_port_1 ? dev->maptx : dev->maprx);

	case XTRX_FE_CUSTOM_0 + 3:
		if (val) {
			dev->chptx.flags |= XTRX_RSP_SWAP_AB;
		} else {
			dev->chptx.flags &= ~XTRX_RSP_SWAP_AB;
		}
		dev->maptx = lms7nfe_get_lml_portcfg(&dev->chptx, dev->tx_no_siso_map);
		return lms7_lml_set_map(&dev->lms_state,
								dev->rx_port_1 ? dev->maprx : dev->maptx,
								dev->rx_port_1 ? dev->maptx : dev->maprx);

	default:
		if (type >= XTRX_RFIC_REG_0 && type <= XTRX_RFIC_REG_0 + 65535)	{
			uint32_t rd;
			uint32_t wr = ((0x8000 | (type - XTRX_RFIC_REG_0)) << 16) | (val & 0xffff);

			if (lmsch != LMS7_CH_NONE) {
				res = lms7_mac_set(&dev->lms_state, lmsch);
				if (res)
					return res;
			}

			// TODO update internal stated in driver!!!
			return xtrxll_lms7_spi_bulk(dev->lldev, 1, &wr, &rd, 1);
		}
		return -EINVAL;
	}
}

static const struct xtrx_fe_ops _lms7nfe_ops = {
	lms7nfe_dd_set_modes,
	lms7nfe_dd_set_samplerate,

	lms7nfe_bb_set_freq,
	lms7nfe_bb_set_badwidth,
	lms7nfe_set_gain,

	lms7nfe_fe_set_refclock,
	lms7nfe_fe_set_freq,
	lms7nfe_fe_set_lna,
	lms7nfe_set_gain,

	lms7nfe_get_reg,
	lms7nfe_set_reg,

	lms7nfe_deinit,
};

void _lms7nfe_init_base(struct xtrx_nfe_lms7 *dev)
{
	dev->base.ops = &_lms7nfe_ops;
}

