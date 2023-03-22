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

	DEF_TX_BUFSIZE = DEF_BUFSIZE,
};

struct xtrx_multill_stream {
	bool                run;

	void*               buf;
	size_t              buf_total;
	size_t              buf_processed;    /**< Number of processed samples (total for all channels) in the cached buffer */
	uint64_t            buf_processed_ts; /**< Number of samples (total for all channels) in the cached buffer */
	uint64_t            buf_ts;           /**< First sample number in the cached buffer */
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
	unsigned dev_idx; //Index of the DEVICE structure in an array
	unsigned dev_max; //Total number of linked XTRX devices

	struct xtrxll_dev* lldev;
	struct xtrx_fe_obj* fe;
	xtrx_debug_ctx_t* debugif;

	unsigned            refclock;
	xtrx_clock_source_t clock_source;

	bool                refclock_checked;
	char                rxinit;
	xtrx_host_format_t  rx_hostfmt;
	xtrx_wire_format_t  rx_busfmt;
	xtrxll_fe_t         rx_fefmt;
	uint64_t            rx_samples; /* num of RX samples for both channels in MIMO or signle channel is SISO */
	float               rx_scale_16;

	void*               rxbuf;
	unsigned            rxbuf_total;
	unsigned            rxbuf_processed;    /**< Number of processed bytes (total for all channels) in the cached buffer */
	uint64_t            rxbuf_processed_ts; /**< Number of samples (in each device channel) in the cached buffer */
	uint64_t            rxbuf_ts;           /**< First sample number in the cached buffer */
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

	unsigned               gpio_cfg_funcs;
	unsigned               gpio_cfg_dir;

	master_ts              gtime_start;
};

static const char* _devname(struct xtrx_dev* dev)
{
	return xtrxll_get_name(dev->lldev);
}


enum xtrx_reg_idxs {
	OSCLATCH = 0,
	RXTIME = 1,
	TXTIME = 2,
	GTIME = 3,
	GT_OFF = 4,
};

static int _debug_param_io(void* obj, unsigned param, unsigned chno, uint64_t val, uint64_t* oval)
{
	int res = -EINVAL;
	int tmp;
	struct xtrx_dev* dev = (struct xtrx_dev*)obj;
	xtrx_channel_t xch = (XTRX_CH_A << chno);

	unsigned devno = (chno >> 1);
	if (devno >= dev->dev_max) {
		XTRXLLS_LOG("XTRX", XTRXLL_WARNING, "Incorrect device channel: %d\n", chno);
		return -EINVAL;
	}

	XTRXLLS_LOG("XTRX", XTRXLL_WARNING, "%s: DEBUG: %x %x %lx\n",
				_devname(&dev[devno]), param, chno, val);

	switch (param) {
	case DEBUG_RFIC_SPI_WR:
		return xtrx_val_set(dev, XTRX_TRX, xch,
							(xtrx_val_t)(XTRX_RFIC_REG_0 + ((val >> 16) & 0x7fff)),
							val & 0xffff);

	case DEBUG_RFIC_SPI_RD:
		return xtrx_val_get(dev, XTRX_TRX, xch,
							(xtrx_val_t)(XTRX_RFIC_REG_0 + ((val >> 16) & 0x7fff)),
							oval);

	case DEBUG_GET_REFCLK:
		*oval = (unsigned)dev[devno].refclock;
		return 0;

	case DEBUG_BOARD_TEMP:
		res = xtrxll_get_sensor(dev[devno].lldev, 0, &tmp);
		XTRXLLS_LOG("XTRX", XTRXLL_INFO, "%s: Temp %.1f C\n",
					_devname(&dev[devno]), (double)tmp/256);

		*oval = (unsigned)tmp;
		return res;

	case DEBUG_BOARD_DAC:
		if (oval) {
			res = xtrxll_get_sensor(dev[devno].lldev, XTRXLL_DAC_REG, &tmp);
			if (res)
				return res;
			*oval = (unsigned)tmp;

			XTRXLLS_LOG("XTRX", XTRXLL_INFO, "%s: DAC: %d\n",
						_devname(&dev[devno]), tmp);
		}
		if (val > 0) {
			res = xtrxll_set_param(dev[devno].lldev, XTRXLL_PARAM_REF_DAC, val);
		}
		return res;

	case DEBUG_ANT_RX:
		if (val != UINT_MAX) {
			return dev[devno].fe->ops->set_reg(dev[devno].fe, 0, XTRX_TRX, XTRX_FE_CUSTOM_0, val);
		} else {
			return dev[devno].fe->ops->get_reg(dev[devno].fe, 0, XTRX_TRX, XTRX_FE_CUSTOM_0, oval);
		}
	case DEBUG_ANT_TX:
		if (val != UINT_MAX) {
			return dev[devno].fe->ops->set_reg(dev[devno].fe, 0, XTRX_TRX, XTRX_FE_CUSTOM_0 + 1, val);
		} else {
			return dev[devno].fe->ops->get_reg(dev[devno].fe, 0, XTRX_TRX, XTRX_FE_CUSTOM_0 + 1, oval);
		}
	case DEBUG_GET_DEVICES:
		*oval = dev->dev_max; return 0;

	case DEBUG_GET_RXIQ_ODD:
		res = xtrxll_get_sensor(dev[devno].lldev, XTRXLL_TEST_CNT_RXIQ_MALGN, &tmp);
		*oval = tmp;
		return res;

	case DEBUG_GET_RXIQ_MISS:
		res = xtrxll_get_sensor(dev[devno].lldev, XTRXLL_TEST_CNT_RXIQ_MALGN, &tmp);
		*oval = tmp;
		return res;

	case DEBUG_VIO:
		if (oval) {
			res = xtrxll_get_sensor(dev[devno].lldev, XTRXLL_XTRX_VIO, &tmp);
			if (res)
				return res;
			*oval = (unsigned)tmp;
		}
		if (val > 0) {
			res = xtrxll_set_param(dev[devno].lldev, XTRXLL_PARAM_PWR_VIO, val);
		}
		return res;

	case DEBUG_V33:
		if (oval) {
			res = xtrxll_get_sensor(dev[devno].lldev, XTRXLL_XTRX_VGPIO, &tmp);
			if (res)
				return res;
			*oval = (unsigned)tmp;
		}
		if (val > 0) {
			res = xtrxll_set_param(dev[devno].lldev, XTRXLL_PARAM_PWR_VGPIO, val);
		}
		return res;

	case DEBUG_FGP_CTRL:
		if (oval) {
			res = xtrxll_get_sensor(dev[devno].lldev, XTRXLL_EXT_CLK, &tmp);
			if (res)
				return res;
			*oval = (unsigned)tmp;
		}
		if (val != UINT_MAX) {
			res = xtrxll_set_param(dev[devno].lldev, XTRXLL_PARAM_EXT_CLK, val);
		}
		return res;
	case DEBUG_XTRX_GET_REG: {
		uint32_t d[2];
		unsigned llreg;

		switch (val) {
		case OSCLATCH: llreg = XTRXLL_OSC_LATCHED; break;
		case RXTIME: llreg = XTRXLL_RX_TIME; break;
		case TXTIME: llreg = XTRXLL_TX_TIME; break;
		case GTIME: llreg = XTRXLL_GTIME_SECFRAC; break;
		case GT_OFF: llreg = XTRXLL_GTIME_OFF; break;
		default:
			return -EINVAL;
		}
		res = xtrxll_get_sensor(dev[devno].lldev, llreg, (int*)&d[0]);
		if (res)
			return res;

		if (val == GTIME) {
			uint64_t v = (uint64_t)d[0] * 1000000000UL + (uint64_t)d[1];
			*oval = v;
		} else {
			*oval = d[0];
		}
		return res;
	}
	default:
		XTRXLLS_LOG("XTRX", XTRXLL_WARNING, "%s: Unknown CMDIDX:%x\n",
					_devname(&dev[devno]), param);
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
		strncpy(devs[i].devid, lldevs[i].addr, sizeof(devs[i].devid));
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

	dev->dev_idx = 0;
	dev->dev_max = 1;
	dev->lldev = lldev;
	dev->refclock = 0;
	dev->refclock_checked = false;
	dev->clock_source = XTRX_CLKSRC_INT;

	xtrxdsp_init();

	res = xtrx_fe_init(dev, lldev, flags, NULL, &dev->fe);
	if (res) {
		XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: Failed to initialize frontend: err=%d\n",
					_devname(dev), res);
		goto failed_fe;
	}

	res = xtrx_debug_init(NULL, &_debug_ops, dev, &dev->debugif);
	if (res) {
		XTRXLLS_LOG("XTRX", XTRXLL_WARNING, "%s: Failed to initialize debug service: err=%d, debug service is disabled\n",
					_devname(dev), res);
		dev->debugif = NULL;
	}

	*outdev = dev;

	/* We need to set a samplerate, otherwise if we set frequency first, it will crash due to a divide of fref by zero.*/
    unsigned int MIN_TX_RATE = 2100000;
    double master_clock;
    double _actual_rx_rate;
    double _actual_tx_rate;
    int ret = xtrx_set_samplerate(dev, 0, MIN_TX_RATE, MIN_TX_RATE,
                                  0, //XTRX_SAMPLERATE_FORCE_UPDATE,
                                  &master_clock, &_actual_rx_rate, &_actual_tx_rate);
	return 0;


failed_fe:
	free(dev);
failed_mem:
	xtrxll_close(lldev);
failed_openll:
	return res;
}

enum {
	XTRX_DEVS_MAX = 32
};
int xtrx_open_multi(const xtrx_open_multi_info_t *dinfo, struct xtrx_dev** outdev)
{
	int res;
	int loglevel = dinfo->loglevel;
	if (loglevel >= 0) {
		xtrxll_set_loglevel(loglevel);
	}

	if (dinfo->devcount > XTRX_DEVS_MAX || dinfo->devcount == 0) {
		XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "Incorrect number of XTRXes in the multidevice: %d!\n",
					dinfo->devcount);
		return -EINVAL;
	}

	struct xtrxll_dev* lldev[XTRX_DEVS_MAX];
	for (unsigned num = 0; num < dinfo->devcount; num++) {
		res = xtrxll_open(dinfo->devices[num], XTRXLL_FULL_DEV_MATCH, &lldev[num]);
		if (res) {
			for (; num > 0; num--) {
				xtrxll_close(lldev[num - 1]);
			}
			return res;
		}
	}

	xtrxdsp_init();

	//All devices are claimed
	struct xtrx_dev* dev = (struct xtrx_dev*)malloc(sizeof(struct xtrx_dev) * dinfo->devcount);
	if (dev == NULL) {
		res = -errno;
		goto failed_mem;
	}
	memset(dev, 0, sizeof(struct xtrx_dev) * dinfo->devcount);

	for (unsigned num = 0; num < dinfo->devcount; num++) {
		dev[num].dev_idx = num;
		dev[num].dev_max = dinfo->devcount;
		dev[num].lldev = lldev[num];
		dev[num].refclock = 0;
		dev[num].clock_source = XTRX_CLKSRC_INT;
		dev[num].fe = dev[0].fe;

		res = xtrx_fe_init(&dev[num], lldev[num],
						   (num == 0 ? XTRX_FE_MASTER : 0) | num,
						   (dinfo->flags & XTRX_OMI_FE_SET) ? dinfo->frontend : NULL,
						   &dev[num].fe);
		if (res) {
			XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: Failed to initialize frontend: err=%d on dev %d/%d\n",
					   _devname(dev), res, num, dinfo->devcount);
			for (; num > 0; num--) {
				dev[num - 1].fe->ops->fe_deinit(dev[num - 1].fe);
			}
			goto failed_fe;
		}
	}

	if (dinfo->flags & XTRX_OMI_DEBUGIF) {
		res = xtrx_debug_init(NULL, &_debug_ops, dev, &dev->debugif);
		if (res) {
			XTRXLLS_LOG("XTRX", XTRXLL_WARNING, "%s: Failed to initialize debug service: err=%d\n",
						_devname(dev), res);
			goto failed_fe;
		}
	}

	*outdev = dev;
	return 0;

failed_fe:
	free(dev);
failed_mem:
	for (unsigned num = 0; num < dinfo->devcount; num++) {
		xtrxll_close(lldev[num]);
	}
	return res;
}

int xtrx_open_string(const char* paramstring, struct xtrx_dev** dev)
{
	int res;
	char copypstr[4096];
	char* str;
	char* saveptr;
	char* ldevices[XTRX_DEVS_MAX] = {0};

	char* devices = NULL;
	char* flags = NULL;

	xtrxll_log_initialize(NULL);

	xtrx_open_multi_info_t params;
	memset(&params, 0, sizeof(params));

	params.loglevel = -1;
	params.devcount = 1;
	params.devices = (const char**)ldevices;

	if (paramstring) {
		strncpy(copypstr, paramstring, sizeof(copypstr) - 1);
		copypstr[sizeof(copypstr) - 1] = '\0';
		devices = copypstr;

		char* separator = strstr(copypstr, ";;");
		if (devices == separator) {
			devices = NULL;
		}
		if (separator) {
			*separator = 0;
			separator += 2;
			if (*separator != 0) {
				flags = separator;
			}
		}
	}

	if (flags) {
		for (str = flags; ; str = NULL) {
			char* token = strtok_r(str, ";", &saveptr);
			if (token == NULL)
				break;

			char* eq = strchr(token, '=');
			char* val = NULL;
			if (eq) {
				*eq = 0;
				val = eq + 1;
				if (*val == 0)
					val = NULL;
			}
			if (strcmp(token, "loglevel") == 0) {
				if (val != NULL) {
					params.loglevel = atoi(val) & XTRX_O_LOGLVL_MASK;

					xtrxll_set_loglevel(params.loglevel);
				}
			} else if (strcmp(token, "fe") == 0) {
				params.frontend = val;
				params.flags |= XTRX_OMI_FE_SET;
			} else if (strcmp(token, "debug") == 0) {
				params.flags |= XTRX_OMI_DEBUGIF;
			} else {
				XTRXLLS_LOG("XTRX", XTRXLL_ERROR,
							"xtrx_open(): unknown flag '%s' with value '%s'\n",
							token, val);
			}
		}
	}

	if (devices) {
		int j;
		for (j = 0, str = devices; j < XTRX_DEVS_MAX; j++, str = NULL) {
			char* token = strtok_r(str, ";", &saveptr);
			if (token == NULL)
				break;
			ldevices[j] = token;
			XTRXLLS_LOG("XTRX", XTRXLL_INFO, "xtrx_open(): dev[%d]='%s'\n",
						j, ldevices[j]);
		}
		if (j == 0) {
			XTRXLLS_LOG("XTRX", XTRXLL_INFO, "xtrx_open(): no devices were found\n");
			return -ENOENT;
		}
		params.devcount = j;
	}

	res = xtrx_open_multi(&params, dev);
	if (res)
		return res;

	return params.devcount;
}

void xtrx_close(struct xtrx_dev* dev)
{
	if (dev->debugif) {
		xtrx_debug_free(dev->debugif);
	}

	for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		dev[devnum].fe->ops->fe_deinit(dev[devnum].fe);
		xtrxll_close(dev[devnum].lldev);
	}

	free(dev);
}


int xtrx_set_ref_clk(struct xtrx_dev* dev, unsigned refclkhz, xtrx_clock_source_t clksrc)
{
	int res;
	static const unsigned base_refclk_ch[] = { 10000000, 19200000, 26000000, 30720000, 38400000, 40000000 };
	static const unsigned base_refclk_ch_cnt = (unsigned)(sizeof(base_refclk_ch) / sizeof(base_refclk_ch[0]));

	enum {
		REF_MIN = 10000000,
		REF_MAX = 52000000,

		PPM_LIMIT = 10000,
	};

	if ((refclkhz < REF_MIN && refclkhz != 0) || refclkhz > REF_MAX) {
		XTRXLLS_LOG("XTRX", XTRXLL_WARNING, "%s: RefClk %d is out of range [%d;%d]!\n",
				   _devname(dev), refclkhz, REF_MIN, REF_MAX);
		return -EINVAL;
	}

	for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		dev[devnum].clock_source = clksrc;

		res = xtrxll_set_param(dev[devnum].lldev,
							   XTRXLL_PARAM_EXT_CLK,
							   (dev[devnum].clock_source) ? XTRXLL_CLK_EXT_NOPD : XTRXLL_CLK_INT);
		if (res) {
			XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: Unable to set clock source\n",
						_devname(&dev[devnum]));
			return res;
		}
	}

	if (dev->refclock < 1) {
		const unsigned* base_refclk = (refclkhz > 0) ? &refclkhz : base_refclk_ch;
		unsigned base_refclk_cnt = (refclkhz > 0) ? 1 : base_refclk_ch_cnt;

		int osc;
		res = xtrxll_get_sensor(dev->lldev, XTRXLL_REFCLK_CLK, &osc);
		if (res) {
			return res;
		}

		dev->refclock_checked = false;
		for (unsigned i = 0; i < base_refclk_cnt; i++) {
			int diff = (int)base_refclk[i] - osc;
			if (abs(diff) * (int64_t)(1000000/PPM_LIMIT) / base_refclk[i] < 1) {
				dev->refclock = base_refclk[i];
				dev->refclock_checked = true;
				XTRXLLS_LOG("XTRX", XTRXLL_INFO, "%s: Set %s RefClk to %d based on %d measurement\n",
						   _devname(dev),
							(dev->clock_source) ? "EXT" : "INT",
							(int)dev->refclock, osc);
				// Pass it down to the RF FE
				res = dev->fe->ops->fe_set_refclock(dev->fe, dev->refclock);
				if (res)
					return res;
				break;
			}
		}

		if (!dev->refclock_checked) {
			XTRXLLS_LOG("XTRX", XTRXLL_INFO, "%s: Wierd RefClk %d! set RefClk manually\n",
						_devname(dev), osc);
			return -ENOENT;
		}
	}

	for (unsigned devnum = 1; devnum < dev->dev_max; devnum++) {
		int osc;
		res = xtrxll_get_sensor(dev[devnum].lldev, XTRXLL_REFCLK_CLK, &osc);
		if (res) {
			XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: Unable to get OSC VAL (%d)\n",
						_devname(&dev[devnum]), res);
			return res;
		}

		if (abs((int)dev->refclock - osc) * (int64_t)(1000000/PPM_LIMIT) / dev->refclock > 1) {
			XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: RefClk %d doesn't look like %d on master!\n",
					   _devname(&dev[devnum]), osc, (int)dev->refclock);

			dev->refclock = 0;
			dev->refclock_checked = false;
			return -EIO;
		}

		dev[devnum].refclock = dev->refclock;
		dev[devnum].refclock_checked = dev->refclock_checked;

		// Pass it down to the RF FE
		res = dev[devnum].fe->ops->fe_set_refclock(dev[devnum].fe, dev->refclock);
		if (res)
			return res;
	}

	XTRXLLS_LOG("XTRX", XTRXLL_DEBUG, "%s: Set RefClk to %d Hz %s\n",
				_devname(dev),
			   (int)dev->refclock, (dev->clock_source == XTRX_CLKSRC_INT) ? "internal" : "extarnal");
	return 0;
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
	int res = 0;
	if (!dev->refclock_checked) {
		res = xtrx_set_ref_clk(dev, 0, dev->clock_source);
		if (res)
			return res;
	}

	int hwid;
	res = xtrxll_get_sensor(dev->lldev, XTRXLL_HWID, &hwid);
	if (res) {
		XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: unable to get HWID\n", _devname(dev));
		return res;
	}

	// Check that other boards have the same settings
	for (unsigned devnum = 1; devnum < dev->dev_max; devnum++) {
		int nhwid;
		res = xtrxll_get_sensor(dev[devnum].lldev, XTRXLL_HWID, &nhwid);
		if (res) {
			XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: unable to get HWID\n",
						_devname(&dev[devnum]));
			return res;
		}
		if (nhwid != hwid) {
			XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: board HWID: %08x != %08x on master board\n",
					   _devname(&dev[devnum]), nhwid, hwid);
			return -EIO;
		}
	}

	struct xtrx_fe_samplerate inrates, outrates;
	memset(&inrates, 0, sizeof(inrates));
	memset(&outrates, 0, sizeof(outrates));

	inrates.adc.rate = rxrate;
	inrates.adc.hwrate = (rxrate > 0) ? cgen_rate / 4 : 0;
	inrates.adc.refclk = dev->refclock;

	inrates.dac.rate = txrate;
	inrates.dac.hwrate = (txrate > 0) ? cgen_rate / 4 : 0;
	inrates.dac.refclk = dev->refclock;

	inrates.flags = flags;
	inrates.hwid = hwid;
	inrates.refclk_source = dev->clock_source;

	for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		res = dev[devnum].fe->ops->dd_set_samplerate(dev[devnum].fe, &inrates, &outrates);
		if (res)
			return res;

		dev[devnum].rx_host_decim = outrates.adc.host_di;
		dev[devnum].tx_host_inter = outrates.dac.host_di;
	}

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
	return xtrx_tune_ex(dev, type, XTRX_CH_ALL, freq, actualfreq);
}

int xtrx_tune_ex(struct xtrx_dev* dev, xtrx_tune_t type, xtrx_channel_t ch,
				 double freq, double *actualfreq)
{
	int res;
	switch (type) {
	case XTRX_TUNE_RX_FDD:
	case XTRX_TUNE_TX_FDD:
	case XTRX_TUNE_TX_AND_RX_TDD:
	case XTRX_TUNE_EXT_FE:
		if (!dev->refclock_checked) {
			res = xtrx_set_ref_clk(dev, 0, dev->clock_source);
			if (res)
				return res;
		}

		for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
			unsigned fe_ch = (ch >> (2 * devnum)) & XTRX_CH_AB;
			if (fe_ch == 0)
				continue;

			res = dev[devnum].fe->ops->fe_set_freq(dev[devnum].fe, fe_ch, type, freq, actualfreq);
			if (res)
				return res;
		}
		return 0;

	case XTRX_TUNE_BB_RX:
	case XTRX_TUNE_BB_TX:
		for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
			unsigned fe_ch = (ch >> (2 * devnum)) & XTRX_CH_AB;
			if (fe_ch == 0)
				continue;

			res = dev[devnum].fe->ops->bb_set_freq(dev[devnum].fe, fe_ch, type, freq, actualfreq);
			if (res)
				return res;
		}
		return 0;
	}

	return -EINVAL;
}

static int xtrx_tune_bandwidth(struct xtrx_dev* dev, xtrx_channel_t xch,
							   int rbb, double bw, double *actualbw)
{
	int res;
	unsigned type = (rbb) ? XTRX_TUNE_BB_RX : XTRX_TUNE_BB_TX;
	for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		unsigned fe_ch = (xch >> (2 * devnum)) & XTRX_CH_AB;
		if (fe_ch == 0)
			continue;

		res = dev[devnum].fe->ops->bb_set_badwidth(dev[devnum].fe, fe_ch, type, bw, actualbw);
		if (res)
			return res;
	}
	return 0;
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
	int res;
	for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		unsigned fe_ch = (xch >> (2 * devnum)) & XTRX_CH_AB;
		if (fe_ch == 0)
			continue;

		res = dev[devnum].fe->ops->bb_set_gain(dev[devnum].fe, fe_ch, gt, gain, actualgain);
		if (res)
			return res;
	}
	return 0;
}


int xtrx_set_antenna(struct xtrx_dev* dev, xtrx_antenna_t antenna)
{
	return xtrx_set_antenna_ex(dev, XTRX_CH_ALL, antenna);
}

int xtrx_set_antenna_ex(struct xtrx_dev* dev, xtrx_channel_t ch, xtrx_antenna_t antenna)
{
	int res;
	for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		unsigned fe_ch = (ch >> (2 * devnum)) & XTRX_CH_AB;
		if (fe_ch == 0)
			continue;

		res = dev[devnum].fe->ops->fe_set_lna(dev[devnum].fe, fe_ch, 0, antenna);
		if (res)
			return res;
	}
	return 0;
}

static unsigned _xtrx_ticks_to_ns(unsigned refclk, unsigned val)
{
	return ((uint64_t)1000000000U) * val / refclk;
}

static unsigned _xtrx_ns_to_ticks(unsigned refclk, unsigned ns)
{
	return (unsigned)(((uint64_t)ns) * refclk / 1000000000);
}

static int _xtrx_ns_to_ticks_i(int refclk, int ns)
{
	return (int)(((int64_t)ns) * refclk / 1000000000);
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
	return ((stream->chs & XTRX_CH_AB) == XTRX_CH_AB &&
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
	stream->paketsize = 0;
	stream->flags = 0;
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

			XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: Specified combination of host and wire formats isn't supported for RX stream\n",
						_devname(dev));
			return -EINVAL;
		}

		rx_bpkt_size = calculate_pkt_size(&params->rx) << dev->rx_host_decim;
		if (params->rx.hfmt != XTRX_IQ_FLOAT32 && (params->rx.flags & XTRX_RSP_SCALE)) {
			XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: XTRX_RSP_SCALE is supported only for XTRX_IQ_FLOAT32 host format in RX stream\n",
						_devname(dev));
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
			XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: Specified combination of host and wire formats isn't supported for TX stream\n",
						_devname(dev));
			return -EINVAL;
		}

		if (params->tx.hfmt != XTRX_IQ_FLOAT32 && (params->tx.flags & XTRX_RSP_SCALE)) {
			XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: XTRX_RSP_SCALE is supported only for XTRX_IQ_FLOAT32 host format in TX stream\n",
						_devname(dev));
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
	fe_params.rx.flags = params->rx.flags;
	fe_params.tx.flags = params->tx.flags;

	for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		fe_params.rx.chs = (params->rx.chs >> (2 * devnum)) & XTRX_CH_AB;
		fe_params.tx.chs = (params->tx.chs >> (2 * devnum)) & XTRX_CH_AB;
		if (fe_params.rx.chs == 0 && fe_params.tx.chs == 0) {
			XTRXLLS_LOG("XTRX", XTRXLL_INFO, "%s: Skipping RX/TX configuration\n",
						_devname(&dev[devnum]));
			continue;
		}

		res = dev[devnum].fe->ops->dd_set_modes(dev[devnum].fe, XTRX_FEDD_CONFIGURE, &fe_params);
		if (res)
			return res;
	}

	/* Validation of input parameters done, do the things */
	if (params->dir & XTRX_RX) for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		rx_start_ts = dev[devnum].rx_samples = params->rx_stream_start << dev[devnum].rx_host_decim;

		if (!dev[devnum].rxinit) {
			res = xtrxll_dma_rx_init(dev[devnum].lldev, chan, rx_bpkt_size, &rx_bpkt_size);
			if (res) {
				goto failed_fe;
			}
			XTRXLLS_LOG("XTRX", XTRXLL_INFO, "%s: RX initialized to %d bytes packet size\n",
						_devname(&dev[devnum]), rx_bpkt_size);
			dev[devnum].rxinit = 1;
		}

		if (dev[devnum].rx_host_decim) {
			unsigned i;
			if (dev[devnum].rx_host_decim != 1) {
				res = -EINVAL;
				goto failed_fe;
			}

			if (params->rx.hfmt == XTRX_IQ_INT16) {
				for (res = 0, i = 0; i < rx_stream_count; i++) {
					res |=  xtrxdsp_filter_initi(g_filter_int16_taps_64_2x,
												 FILTER_TAPS_64,
												 dev[devnum].rx_host_decim,
												 0,
												 rx_bpkt_size / xtrx_wire_format_get_iq_size(params->rx.wfmt),
												 &dev[devnum].rx_host_filter[i]);
				}
			} else if (params->rx.hfmt == XTRX_IQ_FLOAT32) {
				for (res = 0, i = 0; i < rx_stream_count; i++) {
					res |=  xtrxdsp_filter_init(g_filter_float_taps_64_2x,
												FILTER_TAPS_64,
												dev[devnum].rx_host_decim,
												0,
												rx_bpkt_size / xtrx_wire_format_get_iq_size(params->rx.wfmt),
												&dev[devnum].rx_host_filter[i]);
				}
			} else {
				XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: This RX host format is usnsupported for host filtering\n",
							_devname(&dev[devnum]));
				res = -EINVAL;
				goto failed_fe;
			}

			if (res) {
				XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: Unable to initialize host decimation/filtering err=%d\n",
							_devname(&dev[devnum]), res);
				goto filter_cleanup;
			}
		}

		// Accumulation isn't supported yet, so hardwire it
		dev[devnum].rx_scale_16 = 1.0f / 2048;
		if (params->rx.flags & XTRX_RSP_SCALE) {
			dev[devnum].rx_scale_16 *= params->rx.scale;
		}

		dev[devnum].rxbuf = NULL;
		dev[devnum].rxbuf_total = 0;
		dev[devnum].rxbuf_processed = 0;
		dev[devnum].rxbuf_processed_ts = 0;
		dev[devnum].rxbuf_conv_state = 0;
		dev[devnum].rxbuf_len_iq_symbol = rx_stream_count * xtrx_wire_format_get_iq_size(params->rx.wfmt);
		dev[devnum].rxbuf_len_iq_host_sym = rx_stream_count * xtrx_host_format_get_iq_size(params->rx.hfmt);
		dev[devnum].rx_chans_in_stream = rx_stream_count;
		dev[devnum].rx_run = true;
		dev[devnum].rx_hostfmt = params->rx.hfmt;
		dev[devnum].rx_busfmt = params->rx.wfmt;
		dev[devnum].rx_fefmt = rx_fe_fmt;
	}

	if (params->dir & XTRX_TX) for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		if (!dev[devnum].txinit) {
			res = xtrxll_dma_tx_init(dev[devnum].lldev, chan, DEF_TX_BUFSIZE);
			if (res) {
				goto failed_tx_init;
			}
			dev[devnum].txinit = 1;
		}

		if (params->tx_repeat_buf) {
			if (params->tx.paketsize < 4) {
				return -EINVAL;
			}
			unsigned ssz = (tx_mode == XTRXLL_FE_MODE_MIMO) ? params->tx.paketsize * 2 :
															  params->tx.paketsize;
			res = xtrxll_repeat_tx_buf(dev[devnum].lldev,
									   chan,
									   tx_fe_fmt,
									   params->tx_repeat_buf,
									   ssz,
									   tx_mode);
			if (res) {
				goto failed_tx_init;
			}
		}

		dev[devnum].tx_pkt_samples = params->tx.paketsize << dev[devnum].tx_host_inter;
		if (dev[devnum].tx_pkt_samples == 0) {
			//dev[devnum].tx_pkt_samples = (MAX_TX_MSPS / tx_stream_count) >> dev[devnum].tx_host_inter;
			dev[devnum].tx_pkt_samples = (MAX_TX_MSPS / tx_stream_count);

		} else if (((dev[devnum].tx_pkt_samples * tx_stream_count)) > MAX_TX_MSPS ) {
			XTRXLLS_LOG("XTRX", XTRXLL_WARNING, "%s: hardware TX burst size is too high: PKT:%d, lowering to %d\n",
						_devname(&dev[devnum]),
						dev[devnum].tx_pkt_samples, MAX_TX_MSPS / tx_stream_count);
			//res = -EINVAL;
			//goto failed_tx_init;

			dev[devnum].tx_pkt_samples = (MAX_TX_MSPS / tx_stream_count);
		}

		dev[devnum].tx_scale_16 = 32767;
		if (params->tx.flags & XTRX_RSP_SCALE) {
			dev[devnum].tx_scale_16 /= params->tx.scale;
		}

		if (dev[devnum].tx_host_inter) {
			unsigned i;

			if (params->tx_repeat_buf) {
				XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: Host filtering doesn't work with repeat buffer\n",
							_devname(&dev[devnum]));
				res = -EINVAL;
				goto failed_tx_init;
			}
			if (dev[devnum].tx_host_inter > 1) {
				res = -EINVAL;
				goto failed_tx_init;
			}

			if (params->tx.hfmt == XTRX_IQ_INT16) {
				for (res = 0, i = 0; i < tx_stream_count; i++) {
					res |= xtrxdsp_filter_initi(g_filter_int16_taps_64_2x,
												FILTER_TAPS_64,
												0,
												dev[devnum].tx_host_inter,
												dev[devnum].tx_pkt_samples,
												&dev[devnum].tx_host_filter[i]);
				}
			} else if (params->tx.hfmt == XTRX_IQ_FLOAT32) {
				for (res = 0, i = 0; i < tx_stream_count; i++) {
					res |= xtrxdsp_filter_init(g_filter_float_taps_64_2x,
											   FILTER_TAPS_64,
											   0,
											   dev[devnum].tx_host_inter,
											   dev[devnum].tx_pkt_samples,
											   &dev[devnum].tx_host_filter[i]);
				}
			} else {
				XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: This TX host format is usnsupported for host filtering\n",
							_devname(&dev[devnum]));
				res = -EINVAL;
				goto failed_tx_init;
			}
			if (res) {
				XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: Unable to initialize host interpolation/filtering err=%d\n",
							_devname(&dev[devnum]), res);
			}
		}

		dev[devnum].txskip_time = 0;
		dev[devnum].txburt_late_prev = 0;
		dev[devnum].txbuf = NULL;
		dev[devnum].txbuf_total = 0;
		dev[devnum].txbuf_conv_state = 0;
		dev[devnum].tx_chans_in_stream = tx_stream_count;
		dev[devnum].txbuf_len_iq_symbol = dev[devnum].tx_chans_in_stream * xtrx_wire_format_get_iq_size(params->tx.wfmt);
		dev[devnum].txbuf_len_iq_host_sym = dev[devnum].tx_chans_in_stream * xtrx_host_format_get_iq_size(params->tx.hfmt);

		dev[devnum].tx_run = true;
		dev[devnum].tx_hostfmt = params->tx.hfmt;
		dev[devnum].tx_busfmt = params->tx.wfmt;
		dev[devnum].tx_fefmt = tx_fe_fmt;
	}

	dev->gtime_start = ~0UL;
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		struct xtrxll_dmaop op;
		op.rxfe = rx_fe_fmt;
		op.rxmode = (xtrxll_mode_t)(rx_mode | rx_mode_flags);
		op.txfe = tx_fe_fmt;
		op.txmode = tx_mode;
		op.rx_start_sample = rx_start_ts;

		if (params->nflags & XTRX_RUN_GTIME) {
			op.gtime_sec = params->gtime.sec;
			op.gtime_frac = _xtrx_ns_to_ticks(dev->refclock, params->gtime.nsec);
		} else {
			op.gtime_sec = 0;
			op.gtime_frac = 0;
		}

		op.gidx = 0;

		res = xtrxll_dma_start(dev[devnum].lldev, chan, &op);
		if (res) {
			XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: Unable to start DMA err=%d\n",
						_devname(&dev[devnum]), res);
			goto fail_dma_start;
		}

		if (params->nflags & XTRX_RUN_GTIME) {
			dev->gtime_start = params->gtime.sec * 1000000000UL + params->gtime.nsec;
			XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: STIME=%ld\n",
						_devname(&dev[devnum]), dev->gtime_start);
		}
	}

	return 0;

fail_dma_start:
failed_tx_init:
	for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		dev[devnum].rx_run = false;
		dev[devnum].tx_run = false;
	}

filter_cleanup:
	for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		xtrxdsp_filter_free(&dev[devnum].rx_host_filter[0]);
		xtrxdsp_filter_free(&dev[devnum].rx_host_filter[1]);
		xtrxdsp_filter_free(&dev[devnum].tx_host_filter[0]);
		xtrxdsp_filter_free(&dev[devnum].tx_host_filter[1]);
	}
failed_fe:
	for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		fe_params.rx.chs = (params->rx.chs >> (2 * devnum)) & XTRX_CH_AB;
		fe_params.tx.chs = (params->tx.chs >> (2 * devnum)) & XTRX_CH_AB;

		dev[devnum].fe->ops->dd_set_modes(dev[devnum].fe, XTRX_FEDD_RESET, &fe_params);
	}
	return res;
}

int xtrx_stop(struct xtrx_dev* dev, xtrx_direction_t dir)
{
	int res = 0;
	const int chan = 0; //TODO

	if (dir & XTRX_RX) for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		res = xtrxll_dma_rx_start(dev[devnum].lldev, 0, XTRXLL_FE_STOP);

		dev[devnum].rxbuf = NULL;
		dev[devnum].rxbuf_total = 0;
		dev[devnum].rxbuf_processed = 0;
		dev[devnum].rxbuf_processed_ts = 0;
		dev[devnum].rx_run = false;

		xtrxdsp_filter_free(&dev[devnum].rx_host_filter[0]);
		xtrxdsp_filter_free(&dev[devnum].rx_host_filter[1]);
	}

	if (dir & XTRX_TX) for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		res = xtrxll_dma_tx_start(dev[devnum].lldev, chan, XTRXLL_FE_STOP, XTRXLL_FE_MODE_MIMO);

		dev[devnum].txbuf = NULL;
		dev[devnum].txbuf_total = 0;
		dev[devnum].tx_run = false;

		xtrxdsp_filter_free(&dev[devnum].tx_host_filter[0]);
		xtrxdsp_filter_free(&dev[devnum].tx_host_filter[1]);
	}

	struct xtrx_dd_params fe_params;
	fe_params.dir = dir;
	fe_params.nflags = 0;
	fe_params.rx.chs = XTRX_CH_AB;
	fe_params.rx.flags = 0;
	fe_params.tx.chs = XTRX_CH_AB;
	fe_params.tx.flags = 0;

	for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		res = dev[devnum].fe->ops->dd_set_modes(dev[devnum].fe, XTRX_FEDD_RESET, &fe_params);

		struct xtrxll_dmaop op;
		op.rxfe = (dir & XTRX_RX) ? XTRXLL_FE_STOP : XTRXLL_FE_DONTTOUCH;
		op.rxmode = XTRXLL_FE_MODE_MIMO;
		op.txfe = (dir & XTRX_TX) ? XTRXLL_FE_STOP : XTRXLL_FE_DONTTOUCH;
		op.txmode = XTRXLL_FE_MODE_MIMO;
		op.rx_start_sample = 0;

		op.gtime_sec = 0;
		op.gtime_frac = 0;

		res = xtrxll_dma_start(dev[devnum].lldev, chan, &op);
	}
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

int xtrx_send_sync_ex(struct xtrx_dev* mdev, xtrx_send_ex_info_t *info)
{
	if (!mdev->tx_run) {
		XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: TX stream is not configured\n",
					_devname(mdev));
		return -ENOSTR;
	}

	const int chan = 0;
	/* Total numer of sample in all channels */
	size_t total_samples = info->samples * info->buffer_count / mdev->dev_max;
	/* Total bytes user requested */
	size_t user_total = total_samples * mdev->txbuf_len_iq_host_sym / mdev->tx_chans_in_stream;

	if (user_total == 0 || (info->buffer_count != mdev->dev_max && info->buffer_count != 2 * mdev->dev_max))
		return -EINVAL;

	/* Total bytes satisfied from user */
	size_t user_processed = 0;
	int res = -EINVAL;

	/* expanding from host encoding to the wire format times 2, we need this
	   to present 12-bit wire format in integer */
	unsigned encode_amplification_x2 = 2 * mdev->txbuf_len_iq_host_sym / mdev->txbuf_len_iq_symbol;

	bool single_ch_streaming = (info->buffer_count == mdev->dev_max);

	uint16_t cur_late;//, upd_late;
	wts_long_t cur_wts = info->ts << mdev->tx_host_inter;

	info->out_flags = 0;
	info->out_samples = 0;

	unsigned timeout_ms = (info->flags & XTRX_TX_TIMEOUT) ? info->timeout : ~0UL;

	for (unsigned devnum = 0; devnum < mdev->dev_max;) {
		struct xtrx_dev* dev = &mdev[devnum];
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
				case -ETIMEDOUT:
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
					XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: TX DMA Current skip due to TO buffers: %d, prev: %d TS:%" PRId64 " STPS:%" PRId64 "\n",
							   _devname(dev), cur_late, dev->txburt_late_prev, cur_wts, dev->txskip_time);
					dev->txburt_late_prev = cur_late;

					info->out_flags |= XTRX_TX_DISCARDED_TO;
					return 0;
				}

				if (upd_late > 8) {
					XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: TX DMA Current delayed buffers: %d, prev: %d TS:%" PRId64 "\n",
							   _devname(dev), cur_late, dev->txburt_late_prev, cur_wts);
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

		XTRXLLS_LOG("XTRX", XTRXLL_DEBUG, "%s: Total=%u Processed=%u PTS=%" PRId64
				   " cwts=%" PRId64 " UserTotal=%u UserProcessed=%u\n", _devname(dev),
				   dev->txbuf_total, dev->txbuf_processed, dev->txbuf_processed_ts, cur_wts,
				   (unsigned)user_total, (unsigned)user_processed);

		/* bytes need to fill from user */
		size_t remaining = user_total - user_processed;
		unsigned bcnt = single_ch_streaming ? 1 : 2;

		// TODO Move this buffer away from stack
#define TMP_SIZE	32768
		static float tmp_buffer[TMP_SIZE];
		float* tmp_buffer_p[2] = { tmp_buffer, tmp_buffer + TMP_SIZE/2 };

		const void* usr_buf[2] = { (single_ch_streaming) ?
								   info->buffers[devnum] + user_processed : info->buffers[2*devnum + 0] + user_processed / 2,
								   (single_ch_streaming) ?
								   NULL : info->buffers[2*devnum + 1] + user_processed / 2 };
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
			for (i = 0; i < bcnt; i++) {
				memset(tmp_buffer_p[i], 0, wire_bytes_consumed / bcnt);
			}
		}

		/* host filtration & interpolation */
		if (dev->tx_host_filter[0].filter_taps) {
			unsigned num_fsamp = ((2 * dev->tx_chans_in_stream * wire_samples_consumed) >> dev->tx_host_inter) / bcnt;
			switch (dev->tx_hostfmt) {
			case XTRX_IQ_FLOAT32:
				for (i = 0; i < bcnt; i++) {
					res = xtrxdsp_filter_work(&dev->tx_host_filter[i],
											  (const float*)usr_buf[i],
											  (float*)enc_buf[i],
											  num_fsamp);
				}
				break;
			case XTRX_IQ_INT16:
				for (i = 0; i < bcnt; i++) {
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
			if (single_ch_streaming) {
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
			if (single_ch_streaming) {
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
		if (devnum == 0) {
			info->out_samples += (dev->tx_chans_in_stream * wire_samples_consumed >> dev->tx_host_inter) / bcnt;
		}

		if (dev->txbuf_processed == dev->txbuf_total ||
				((user_processed == user_total) && (info->flags & XTRX_TX_DONT_BUFFER))) {
			res = xtrxll_dma_tx_post(dev->lldev,
									 chan,
									 dev->txbuf,
									 (dev->tx_chans_in_stream == 1) ? cur_wts / 2 : cur_wts,
									 dev->txbuf_processed / 8);
			dev->txbuf = NULL;
			if (res) {
				XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: xtrxll_dma_tx_post res=%d (wts=%" PRIu64 " samples=%u)\n",
						   _devname(dev), res, cur_wts, (unsigned)wire_samples_consumed);
				return res;
			}
		}

		/* User request satisfied */
		if (user_processed == user_total) {
			// Switch to next dev
			devnum++;
			user_processed = 0;
			res = 0;

			cur_wts = info->ts << mdev->tx_host_inter;
		} else {
			cur_wts += dev->txbuf_processed_ts;
		}
	}

	return res;
}

int xtrx_recv_sync_ex(struct xtrx_dev* mdev, xtrx_recv_ex_info_t* info)
{
	if (!mdev->rx_run) {
		XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: RX stream is not configured\n",
					_devname(mdev));
		return -ENOSTR;
	}
	if ((info->flags & RCVEX_REPORT_GTIME) && (mdev->gtime_start == ~0UL)) {
		return -EINVAL;
	}

	const int chan = 0;
	/* One sample in all channels in bytes */
	const size_t total_samples = info->samples * info->buffer_count / mdev->dev_max;
	/* Total bytes user requested */
	const size_t user_total = total_samples * mdev->rxbuf_len_iq_host_sym  / mdev->rx_chans_in_stream;
	if (user_total == 0 || (info->buffer_count != mdev->dev_max && info->buffer_count != 2 * mdev->dev_max))
		return -EINVAL;

	/* Total bytes satisfied for user */
	size_t user_processed = 0;

	xtrxll_dma_rx_flags_t flags = 0;
	int res = -EINVAL;

	/* expanding from wire decoding to the host format times 2 */
	const unsigned decode_amplification_x2 = 2 * mdev->rxbuf_len_iq_host_sym / mdev->rxbuf_len_iq_symbol;

	const bool single_ch_streaming = (info->buffer_count == mdev->dev_max);

	master_ts gstarttm = mdev->gtime_start;
	info->out_samples = 0;
	info->out_events = 0;

	if (info->flags & RCVEX_DONT_WAIT)
		flags |= XTRXLL_RX_DONTWAIT;

	if (info->flags & RCVEX_EXTRA_LOG)
		flags |= XTRXLL_RX_FORCELOG;

	if (info->flags & RCVEX_TIMOUT)
		flags |= XTRXLL_RX_REPORT_TIMEOUT;

	for (unsigned devnum = 0; devnum < mdev->dev_max;) {
		struct xtrx_dev* dev = &mdev[devnum];
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
				case -ETIMEDOUT:
					XTRXLLS_LOG("XTRX", XTRXLL_WARNING, "%s: Buffer timed out!\n", _devname(dev));
					return res;

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
			if (info->flags & RCVEX_REPORT_GTIME) {
				info->out_first_sample = gstarttm +
						info->out_first_sample * 1000000000UL / dev->refclock;
			}
		}
/*
		XTRXLLS_LOG("XTRX", (dev->rxbuf_ts + dev->rxbuf_processed_ts != dev->rx_samples) ? XTRXLL_WARNING : XTRXLL_DEBUG,
				   "%s: Total=%u Processed=%u UserTotal=%u UserProcessed=%u BUFTS=%" PRIu64 "+%" PRIu64 " OURTS=%" PRIu64 "\n",
				   _devname(dev), dev->rxbuf_total, dev->rxbuf_processed, (unsigned)user_total, (unsigned)user_processed,
				   dev->rxbuf_ts, dev->rxbuf_processed_ts, dev->rx_samples);
*/
		/* bytes need to fill in user */
		size_t remaining = user_total - user_processed;
		unsigned bcnt = single_ch_streaming ? 1 : 2;
		// TODO Move this buffer away from stack
#define TMP_SIZE	32768
		static float tmp_buffer[TMP_SIZE];
		float* tmp_buffer_p[2] = { tmp_buffer, tmp_buffer + TMP_SIZE/2 };

		void* usr_buf[2] = { (single_ch_streaming) ?
							 info->buffers[devnum] + user_processed : info->buffers[2*devnum + 0] + user_processed / 2,
							 (single_ch_streaming) ?
							 NULL : info->buffers[2*devnum + 1] + user_processed / 2 };
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
					if (info->flags & RCVEX_REPORT_GTIME) {
						info->out_first_sample = gstarttm +
								info->out_first_sample * 1000000000UL / dev->refclock;
					}
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
			for (i = 0; i < bcnt; i++) {
				memset(dst_buf[i], 0, user_cur_proceesed / bcnt);
			}
			info->out_events |= RCVEX_EVENT_FILLED_ZERO;
		} else {
			int datafmt = MAKE_FORMAT(dev->rx_busfmt, dev->rx_hostfmt);
			switch (datafmt) {
			case WIRE_I8_HOST_I8:
			case WIRE_I16_HOST_I16:
				if (single_ch_streaming) {
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
				if (single_ch_streaming) {
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
				if (single_ch_streaming) {
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
				if (single_ch_streaming) {
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
		if (devnum == 0) {
			info->out_samples += (dev->rx_chans_in_stream * wire_samples_consumed >> dev->rx_host_decim) / bcnt;
		}

		if (dev->rx_host_filter[0].filter_taps) {
			unsigned num_fsamp = 2 * dev->rx_chans_in_stream * wire_samples_consumed / bcnt;
			switch (dev->rx_hostfmt) {
			case XTRX_IQ_FLOAT32:
				for (i = 0; i < bcnt; i++) {
					res = xtrxdsp_filter_work(&dev->rx_host_filter[i],
											  (const float*)dst_buf[i],
											  (float*)usr_buf[i],
											  num_fsamp);
				}
				break;
			case XTRX_IQ_INT16:
				for (i = 0; i < bcnt; i++) {
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
			// Switch to next dev
			user_processed = 0;
			devnum++;
			res = 0;
		}
	}

	return res;
}

static int _xtrx_val_set_int(struct xtrx_dev* dev, xtrx_direction_t dir,
				 xtrx_channel_t chan, xtrx_val_t type, uint64_t val)
{
	if (type >= XTRX_RFIC_REG_0 && type < XTRX_DEBUG_0)	{
		XTRXLLS_LOG("XTRX", XTRXLL_INFO, "%s: FE REG %x %lx\n",
					_devname(dev), type, val);
		return dev->fe->ops->set_reg(dev->fe, chan, dir, type, val);
	}

	switch (type) {
	case XTRX_LML_PHY_PHASE:
		XTRXLLS_LOG("XTRX", XTRXLL_INFO, "%s: Set LMS7 LML FCLK Phase to %d\n", _devname(dev), (int)val);
		return xtrxll_mmcm_fphase_corr(dev->lldev, true, val, false);
	case XTRX_LML_PHY_FBPHASE:
		XTRXLLS_LOG("XTRX", XTRXLL_INFO, "%s: Set LMS7 LML FB Phase to %d\n", _devname(dev), (int)val);
		return xtrxll_mmcm_fphase_corr(dev->lldev, true, val, true);
	case XTRX_LMS7_PWR_MODE:
		XTRXLLS_LOG("XTRX", XTRXLL_INFO, "%s: Set LMS7 power mode to %d\n", _devname(dev), (int)val);
		return xtrxll_set_param(dev->lldev, XTRXLL_PARAM_PWR_MODE, val);
	case XTRX_LMS7_VIO:
		XTRXLLS_LOG("XTRX", XTRXLL_INFO, "%s: Set LMS7 VIO to %d\n", _devname(dev), (int)val);
		return xtrxll_set_param(dev->lldev, XTRXLL_PARAM_PWR_VIO, val);
	case XTRX_LMS7_XSP_DC_IQ:
		return dev->fe->ops->set_reg(dev->fe, chan, dir, type, val);
	case XTRX_VCTCXO_DAC_VAL:
		XTRXLLS_LOG("XTRX", XTRXLL_INFO, "%s: Set XTRX DAC %d\n", _devname(dev), (int)val);
		return xtrxll_set_param(dev->lldev, XTRXLL_PARAM_REF_DAC, val);
	case XTRX_DSPFE_CMD:
		return xtrxll_set_param(dev->lldev, XTRXLL_PARAM_DSPFE_CMD, val);
	default:
		return -EINVAL;
	}
}

XTRX_API int xtrx_val_set(struct xtrx_dev* dev, xtrx_direction_t dir,
				 xtrx_channel_t chan, xtrx_val_t type, uint64_t val)
{
	int res = -EINVAL;
	for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		unsigned fe_ch = (chan >> 2 * devnum) & XTRX_CH_AB;
		if (fe_ch) {
			res = _xtrx_val_set_int(&dev[devnum], dir, fe_ch, type, val);
			if (res) {
				break;
			}
		}
	}
	return res;
}

static int _xtrx_val_get_int(struct xtrx_dev* dev, xtrx_direction_t dir,
						  xtrx_channel_t chan, xtrx_val_t type, uint64_t* oval)
{
	int res, val;

	if (type >= XTRX_RFIC_REG_0 && type < XTRX_DEBUG_0)	{
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
	case XTRX_REF_REFCLK:
		if (!dev->refclock_checked) {
			res = xtrx_set_ref_clk(dev, 0, dev->clock_source);
			if (res)
				return res;
		}
		*oval = dev->refclock;
		return 0;
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
	case XTRX_TX_TIME:
		res = xtrxll_get_sensor(dev->lldev, XTRXLL_TX_TIME, &val);
		if (!res) {
			*oval = val;
		}
		return res;
	default:
		return -EINVAL;
	}
}

XTRX_API int xtrx_val_get(struct xtrx_dev* dev, xtrx_direction_t dir,
						  xtrx_channel_t chan, xtrx_val_t type, uint64_t* oval)
{
	int res = -EINVAL;
	for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		unsigned fe_ch = (chan >> 2 * devnum) & XTRX_CH_AB;
		if (fe_ch) {
			res = _xtrx_val_get_int(&dev[devnum], dir, fe_ch, type, oval);
			if (res) {
				break;
			}
		}
	}
	return res;
}

static int _xtrx_gpio_configure(struct xtrx_dev* dev,
								int gpio_num, xtrx_gpio_func_t function)
{
	unsigned gfunc = 0;
	unsigned dir = 0;
	int res;

	switch (function) {
	case XTRX_GPIO_FUNC_IN:  gfunc = 0; break;
	case XTRX_GPIO_FUNC_OUT:  gfunc = 0; dir = 1; break;
	case XTRX_GPIO_FUNC_ALT0: gfunc = 1; break;
	case XTRX_GPIO_FUNC_ALT1: gfunc = 2; break;
	case XTRX_GPIO_FUNC_ALT2: gfunc = 3; break;
	default:
		if (gpio_num == XTRX_GPIO_ALL) {
			return -EINVAL;
		}
		if (gpio_num >= XTRX_GPIOS_TOTAL) {
			return -EINVAL;
		}

		if ((function == XTRX_GPIO_FUNC_PPS_O && (gpio_num == XTRX_GPIO_PPS_O || gpio_num == XTRX_GPIO_EPPS_O)) ||
				((function == XTRX_GPIO_FUNC_PPS_I && gpio_num == XTRX_GPIO_PPS_I))) {
			gfunc = 1;
			break;
		}
		return -EINVAL;
	}

	if (gpio_num == XTRX_GPIO_ALL) {
		dev->gpio_cfg_funcs = 0;
		dev->gpio_cfg_dir = 0;
		for (unsigned i = 0; i < XTRX_GPIOS_TOTAL; i++) {
			dev->gpio_cfg_funcs |= gfunc << (2 * i);
			dev->gpio_cfg_dir |=  dir << (i);
		}
	} else {
		unsigned msk = 0x3U << (2 * gpio_num);
		unsigned msk_dir = 0x1U << (gpio_num);

		dev->gpio_cfg_funcs = (~msk & dev->gpio_cfg_funcs) | (gfunc << (2 * gpio_num));
		dev->gpio_cfg_dir = (~msk_dir & dev->gpio_cfg_dir) | (dir << (gpio_num));
	}

	res = xtrxll_set_param(dev->lldev, XTRXLL_PARAM_GPIO_DIR, dev->gpio_cfg_dir);
	if (res)
		return res;

	res = xtrxll_set_param(dev->lldev, XTRXLL_PARAM_GPIO_FUNC, dev->gpio_cfg_funcs);
	if (res)
		return res;

	return 0;
}

XTRX_API int xtrx_gpio_configure(struct xtrx_dev* dev, int devno,
								 int gpio_num, xtrx_gpio_func_t function)
{
	if (devno >= (int)dev->dev_max) {
		return -EINVAL;
	}
	if (devno >= 0) {
		return _xtrx_gpio_configure(&dev[devno], gpio_num, function);
	}

	for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		int res = _xtrx_gpio_configure(&dev[devnum], gpio_num, function);
		if (res)
			return res;
	}

	return 0;
}

static int _xtrx_gpio_out(struct xtrx_dev* dev, unsigned out)
{
	return xtrxll_set_param(dev->lldev, XTRXLL_PARAM_GPIO_OUT, out);
}

XTRX_API int xtrx_gpio_out(struct xtrx_dev* dev, int devno, unsigned out)
{
	if (devno >= (int)dev->dev_max) {
		return -EINVAL;
	}
	if (devno >= 0) {
		return _xtrx_gpio_out(&dev[devno], out);
	}

	for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		int res = _xtrx_gpio_out(&dev[devnum], out);
		if (res)
			return res;
	}

	return 0;
}

static int _xtrx_gpio_clear_set(struct xtrx_dev* dev,
								 unsigned clear_msk, unsigned set_msk)
{
	enum {
		MAX_MSK = (1U << XTRX_GPIOS_TOTAL) - 1,
	};
	if (clear_msk > MAX_MSK || set_msk > MAX_MSK)
		return -EINVAL;

	unsigned cmd = (clear_msk << XTRX_GPIOS_TOTAL) | set_msk;
	return xtrxll_set_param(dev->lldev, XTRXLL_PARAM_GPIO_CS, cmd);
}

XTRX_API int xtrx_gpio_clear_set(struct xtrx_dev* dev, int devno,
								 unsigned clear_msk, unsigned set_msk)
{
	if (devno >= (int)dev->dev_max) {
		return -EINVAL;
	}
	if (devno >= 0) {
		return _xtrx_gpio_clear_set(&dev[devno], clear_msk, set_msk);
	}

	for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		int res = _xtrx_gpio_clear_set(&dev[devnum], clear_msk, set_msk);
		if (res)
			return res;
	}

	return 0;
}

XTRX_API int xtrx_gpio_in(struct xtrx_dev* dev, int devno, unsigned* in)
{
	if ((unsigned)devno >= dev->dev_max) {
		return -EINVAL;
	}

	return xtrxll_get_sensor(dev->lldev, XTRXLL_GPIO_IN, (int*)in);
}

static int _xtrx_gtime_ctrl(struct xtrx_dev* dev,
							bool external,
							unsigned isec,
							bool fwen)
{
	int res;
	res = xtrxll_set_param(dev->lldev,
						   XTRXLL_PARAM_GTIME_RESET, 1);
	if (res)
		return res;

	res = xtrxll_set_param(dev->lldev,
						   XTRXLL_PARAM_GTIME_SETCMP,
						   dev->refclock - 1);
	if (res)
		return res;

	res = xtrxll_set_param(dev->lldev,
						   XTRXLL_PARAM_GTIME_CTRL,
						   (external) ?
						   ((fwen) ? XTRXLL_GTIME_EXT_PPSFW : XTRXLL_GTIME_EXT_PPS) : XTRXLL_GTIME_INT_ISO);
	if (res)
		return res;

	res = xtrxll_set_param(dev->lldev,
						   XTRXLL_PARAM_ISOPPS_CTRL,
						   /*(external) ? XTRXLL_GISO_DISABLE :*/ XTRXLL_GISO_PPSFW);
	if (res)
		return res;

	res = xtrxll_set_param(dev->lldev,
						   XTRXLL_PARAM_PPSDO_CTRL,
						   /*(external) ? XTRXLL_PPSDO_DISABLE :*/ XTRXLL_PPSDO_INT_GPS);
	if (res)
		return res;

	res = xtrxll_set_param(dev->lldev,
						   XTRXLL_PARAM_GTIME_RESET, 0);
	if (res)
		return res;

	res = xtrxll_set_param(dev->lldev,
						   (fwen) ? XTRXLL_PARAM_ISOPPS_SETTIME : XTRXLL_PARAM_CURPPS_SETTIME,
						   isec);
	if (res)
		return res;

	return res;
}

static int _xtrx_gtime_op(struct xtrx_dev* dev,
						  struct xtrx_dev* mdev,
						  xtrx_gtime_cmd_t cmd,
						  gtime_data_t in,
						  gtime_data_t *out)
{
	int res = -EINVAL;

	switch (cmd) {
	case XTRX_GTIME_ENABLE_INT: {
		return _xtrx_gtime_ctrl(dev, false, in.sec, true);
	}
	case XTRX_GTIME_ENABLE_INT_WEXT:
	case XTRX_GTIME_ENABLE_INT_WEXTE:
	case XTRX_GTIME_ENABLE_INT_WEXTENFW: {
		res = _xtrx_gpio_configure(dev,
								   cmd != XTRX_GTIME_ENABLE_INT_WEXT ? XTRX_GPIO_EPPS_O : XTRX_GPIO_PPS_O,
								   XTRX_GPIO_FUNC_PPS_O);
		if (res)
			return res;

		res = _xtrx_gpio_configure(dev, XTRX_GPIO_PPS_I, XTRX_GPIO_FUNC_PPS_I);
		if (res)
			return res;

		return _xtrx_gtime_ctrl(dev, true, in.sec,
								cmd != XTRX_GTIME_ENABLE_INT_WEXTENFW);
	}
	case XTRX_GTIME_ENABLE_EXT:
	case XTRX_GTIME_ENABLE_EXTNFW: {
		res = _xtrx_gpio_configure(dev, XTRX_GPIO_PPS_I, XTRX_GPIO_FUNC_PPS_I);
		if (res)
			return res;

		return _xtrx_gtime_ctrl(dev, true, in.sec,
								cmd != XTRX_GTIME_ENABLE_EXTNFW);
	}
	case XTRX_GTIME_DISABLE: {
		res = xtrxll_set_param(dev->lldev,
							   XTRXLL_PARAM_GTIME_RESET, 1);
		if (res)
			return res;

		res = xtrxll_set_param(dev->lldev,
							   XTRXLL_PARAM_GTIME_CTRL,
							   XTRXLL_GTIME_DISABLE);
		if (res)
			return res;

		res = xtrxll_set_param(dev->lldev,
							   XTRXLL_PARAM_ISOPPS_CTRL,
							   XTRXLL_GISO_DISABLE);
		if (res)
			return res;
		break;
	}
	case XTRX_GTIME_GET_RESOLUTION: {
		out->nsec = _xtrx_ticks_to_ns(1, dev->refclock);
		out->sec = 0;
		break;
	}
	case XTRX_GTIME_SET_GENSEC: {
		res = xtrxll_set_param(dev->lldev, XTRXLL_PARAM_ISOPPS_SETTIME, in.sec);
		break;
	}
	case XTRX_GTIME_SET_CURSEC: {
		res = xtrxll_set_param(dev->lldev, XTRXLL_PARAM_CURPPS_SETTIME, in.sec);
		break;
	}
	case XTRX_GTIME_GET_CUR: {
		uint32_t tm[2];
		res = xtrxll_get_sensor(dev->lldev, XTRXLL_GTIME_SECFRAC, (int*)&tm);
		if (res)
			return res;

		out->sec = tm[0];
		out->nsec = _xtrx_ticks_to_ns(dev->refclock, tm[1]);
		break;
	}
	case XTRX_GTIME_APPLY_CORRECTION: {
		if (in.sec > 0)
			return -E2BIG;

		int corr = _xtrx_ns_to_ticks_i((int)dev->refclock, (int)in.nsec);
		if (corr < -(int)dev->refclock/2 || corr > (int)dev->refclock/2)
			return -E2BIG;

		res = xtrxll_set_param(dev->lldev,
							   XTRXLL_PARAM_GTIME_TRIMOFF,
							   (unsigned)corr);
		break;
	}
	case XTRX_GTIME_GET_GPSPPS_DELTA: {
		int data;
		res = xtrxll_get_sensor(dev->lldev, XTRXLL_GTIME_OFF, &data);
		if (res)
			return res;

		out->sec = 0;
		out->nsec = _xtrx_ticks_to_ns(dev->refclock, (unsigned)data);
		break;
	}
	}

	return res;
}

XTRX_API int xtrx_gtime_op(struct xtrx_dev* dev, int devno,
						   xtrx_gtime_cmd_t cmd, gtime_data_t in,
						   gtime_data_t *out)
{
	if (devno >= (int)dev->dev_max) {
		return -EINVAL;
	}
	if (dev->refclock < 1) {
		XTRXLLS_LOG("XTRX", XTRXLL_ERROR, "%s: RefClock is not set!\n",
					_devname(dev));
		return -EFAULT;
	}

	if (devno >= 0) {
		return _xtrx_gtime_op(&dev[devno], dev, cmd, in, out);
	}
	for (unsigned devnum = 0; devnum < dev->dev_max; devnum++) {
		int res = _xtrx_gtime_op(&dev[devnum], dev, cmd, in, out);
		if (res)
			return res;
	}

	return 0;
}


void xtrx_log_setfunc(xtrx_logfunc_t func)
{
	xtrxll_set_logfunc(func);
}

void xtrx_log_setlevel(int sevirity, const char sybsystem[4])
{
	(void)sybsystem;
	xtrxll_set_loglevel(sevirity);
}

