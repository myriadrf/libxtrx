#include <xtrxll_api.h>
#include <xtrxll_log.h>

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdarg.h>

#include <liblms7002m.h>

#include "xtrx_api.h"
#include "xtrx_fe_nlms7.h"
#include "octo/adf4355.h"

enum octo_flags {
	LO_SET = 1 << 0,
	ADF_INIT = 1 << 1,

	RX_ACTIVE = 1 << 2, //Active streaming on channel

	BW_SET = 1 << 3,
	LMS_TDD = 1 << 4,

	RX_DPATH_LMS = 1 << 7,
	LO_PORTB_EN = 1 << 8,
};

#define MAX_FE_BOARDS   4
struct xtrx_lms7octo
{
	struct xtrx_fe_obj base;

	struct xtrx_nfe_lms7 *lms;
	struct xtrxll_dev *master;
	struct xtrx_lms7octo *mdev;

	unsigned flags[2];

	unsigned devno;
	unsigned en_devs;           // Valid only on master

	uint8_t trf37_bb_gain[2];
	uint8_t trf37_lpf[2];
	unsigned rx_path[2];

	double lo;
	unsigned gain[2];
	unsigned bw[2];

	struct xtrx_dd_chpar run_rx;
};

#define IS_OCTO_PATH(p) (((p) == XTRX_RX_AUTO) || ((p) == XTRX_RX_ADC_EXT))
static unsigned get_octo_chans(struct xtrx_lms7octo* dev)
{
	if (dev->flags[0] & RX_DPATH_LMS) {
		return ((dev->rx_path[0] == XTRX_RX_ADC_EXT) ? XTRX_CH_A : 0) |
						((dev->rx_path[1] == XTRX_RX_ADC_EXT) ? XTRX_CH_B : 0);
	}

	return ((IS_OCTO_PATH(dev->rx_path[0])) ? XTRX_CH_A : 0) |
					((IS_OCTO_PATH(dev->rx_path[1])) ? XTRX_CH_B : 0);
}

enum {
	REG_GPIO_G = 0,
	REG_TMP_L  = 1,
	REG_ADI_H  = 2,
	REG_TRF_H  = 3,
	REG_I2C_H  = 4,

	REG_RD_STAT = 8,
	REG_RD_I2CL = 9,
	REG_RD_I2CH = 10,
};

static void _lms7octo_init_base(struct xtrx_lms7octo *dev);



#define MAKE_OCTO_SPI(cmd, data) \
	((((cmd) & 0xf) << 28) | ((data) & 0x0fffffff))

static int adf4355_spi(struct xtrxll_dev *dev, uint32_t out)
{
	int res;
	res = xtrxll_set_param(dev, XTRXLL_PARAM_EXT_SPI, MAKE_OCTO_SPI(REG_TMP_L, out));
	if (res)
		return res;

	res = xtrxll_set_param(dev, XTRXLL_PARAM_EXT_SPI, MAKE_OCTO_SPI(REG_ADI_H, out >> 28));
	if (res)
		return res;

	usleep(2000);
	return 0;
}


#define MAKE_TRF37_DEV_SETUP(dcoffcalpd, bbg, lpf, fltb, fg, x2fg, odb3) \
	((((odb3) & 1) << 31) | \
	(((x2fg) & 1) << 28) | \
	(((fg) & 1) << 27) | \
	(((fltb) & 3) << 25) | \
	(((lpf) & 0xff) << 17) | \
	(((bbg) & 0x1f) << 12) | \
	(((dcoffcalpd) & 1) << 10) | \
	(((1) & 1) << 7) | \
	9)

#define MAKE_TRF37_DEV_SETUP2(osctrim, cclk, clkdiv, calsel, idet, qdac, idac, ena) \
	((((osctrim) & 7) << 29) | \
	(((cclk) & 1) << 28) | \
	(((clkdiv) & 7) << 25) | \
	(((calsel) & 1) << 24) | \
	(((idet) & 3) << 22) | \
	(((qdac) & 0xff) << 14) | \
	(((idac) & 0xff) << 6) | \
	(((ena) & 1) << 5) | \
	12)

#define MAKE_TRF37_DEV_SETUP3(b, fc) \
		((((fc) & 3) << 30) | \
		(((b) & 1) << 29) | \
		13)

#define ROTATE2(x) \
	((((x) & 0x1) << 1) | \
	 (((x) & 0x2) >> 1))
#define XTRX_CH_TO_OCTO(devno, ch) \
	(ROTATE2(ch) << (2*(devno)))

static int trf37_spi(struct xtrxll_dev *dev, unsigned mask, uint32_t out)
{
	unsigned rotate = 0;
	for (unsigned i = 0; i < 32; i++) {
		if (out & (1<<i))
			rotate |= (1<<(31 - i));
	}
	out = rotate;

	int res;
	res = xtrxll_set_param(dev, XTRXLL_PARAM_EXT_SPI, MAKE_OCTO_SPI(REG_TMP_L, out));
	if (res)
		return res;

	res = xtrxll_set_param(dev, XTRXLL_PARAM_EXT_SPI,
						   MAKE_OCTO_SPI(REG_TRF_H, ((~mask) & 0xffff) << 4) | (out >> 28));
	if (res)
		return res;

	usleep(2000);
	return 0;
}

static int octo_read_spi(struct xtrxll_dev *dev, unsigned regno, unsigned* out)
{
	int res = xtrxll_set_param(dev, XTRXLL_PARAM_EXT_SPI, MAKE_OCTO_SPI(regno, 0));
	if (res)
		return res;

	usleep(1000);

	return xtrxll_get_sensor(dev, XTRXLL_EXT_SPI_RB, (int*)out);
}

enum octo_i2c_lut {
	OCTO_I2C_TMP = 0,
	OCTO_I2C_DAC_TCXO = 1,
	OCTO_I2C_DAC_VCOM = 2,
};
#define MAKE_I2C_CMD(RD, RDZSZ, WRSZ, DEVNO, DATA)  (\
	(((RD) & 1U) << 31) | \
	(((RDZSZ) & 7U) << 28) | \
	(((WRSZ) & 3U) << 26) | \
	(((DEVNO) & 3U) << 24) | \
	(((DATA) & 0xffffffu) << 0))

static int i2c_cmdwr(struct xtrxll_dev *dev, uint32_t cmd)
{
	int res;
	res = xtrxll_set_param(dev, XTRXLL_PARAM_EXT_SPI,
						   MAKE_OCTO_SPI(REG_TMP_L, cmd >> 4));
	if (res)
		return res;

	res = xtrxll_set_param(dev, XTRXLL_PARAM_EXT_SPI,
						   MAKE_OCTO_SPI(REG_I2C_H, cmd & 0xf));
	if (res)
		return res;

	usleep(2000);
	return 0;
}

static int i2c_wait_rb(struct xtrxll_dev *dev)
{
	unsigned oval;
	for (int i = 0; i < 200; i++) {
		int res = octo_read_spi(dev, REG_RD_STAT, &oval);
		if (res)
			return res;

		if (oval & (1 << 5))
			return 0;
	}

	XTRXLLS_LOG("OCTO", XTRXLL_ERROR, "Failed I2C transaction, status: %08x\n",
				oval);
	return -EIO;
}

static int tmp108_get(struct xtrxll_dev* dev, unsigned reg, int *outval)
{
	unsigned i2c_rb;
	int res = i2c_cmdwr(dev, MAKE_I2C_CMD(1, 1, 1, OCTO_I2C_TMP, reg));
	if (res)
		return res;

	res = i2c_wait_rb(dev);
	if (res)
		return res;

	res = octo_read_spi(dev, REG_RD_I2CL, &i2c_rb);
	if (res)
		return res;

	*outval = (int16_t)(htole16(i2c_rb & 0xffff));
	return 0;
}


static int dac_set(struct xtrxll_dev* dev, unsigned dno, unsigned val)
{
	uint32_t cmd = (((val >> 8) & 0x0f)) | (((val) & 0xff) << 8);
	return i2c_cmdwr(dev, MAKE_I2C_CMD(0, 0, 2, dno, cmd));
}

static int dac_get(struct xtrxll_dev* dev, unsigned dno, unsigned* oval)
{
	int res = i2c_cmdwr(dev, MAKE_I2C_CMD(1, 3, 0, dno, 0));
	if (res)
		return res;

	res = i2c_wait_rb(dev);
	if (res)
		return res;

	res = octo_read_spi(dev, REG_RD_I2CL, oval);
	return res;
}



static int lms7octo_update_trf(struct xtrx_lms7octo *dev, unsigned channel)
{
	int res;
	unsigned devmask = XTRX_CH_TO_OCTO(dev->devno, channel);
	unsigned octo_chans = get_octo_chans(dev);

	uint32_t trf_ref0 = MAKE_TRF37_DEV_SETUP(0, dev->trf37_bb_gain[0],
											dev->trf37_lpf[0], 2, 0, 0, 0);
	uint32_t trf_ref1 = MAKE_TRF37_DEV_SETUP(0, dev->trf37_bb_gain[1],
											dev->trf37_lpf[1], 2, 0, 0, 0);
	if (!(octo_chans & XTRX_CH_A) || !(dev->flags[0] & RX_ACTIVE)) {
		trf_ref0 |= (1 << 5) | (1 << 6) | (1 << 7) | (1 << 10);
	}
	if (!(octo_chans & XTRX_CH_B) || !(dev->flags[1] & RX_ACTIVE)) {
		trf_ref1 |= (1 << 5) | (1 << 6) | (1 << 7) | (1 << 10);
	}

	XTRXLLS_LOG("OCTO", XTRXLL_INFO_LMS, "OCTO Update TRF37: %02x:%08x:%08x:%02x:%02x\n",
				devmask, trf_ref0, trf_ref1, channel, octo_chans);

	if ((channel == XTRX_CH_AB) && (trf_ref0 == trf_ref1)) {
		return trf37_spi(dev->master, devmask, trf_ref0);
	}
	if (channel & XTRX_CH_A) {
		res = trf37_spi(dev->master, XTRX_CH_TO_OCTO(dev->devno, XTRX_CH_A), trf_ref0);
		if (res)
			return res;
	}
	if (channel & XTRX_CH_B) {
		res = trf37_spi(dev->master, XTRX_CH_TO_OCTO(dev->devno, XTRX_CH_B), trf_ref1);
		if (res)
			return res;
	}

	return 0;
}

static int trf_gain_to(unsigned gain, double *outgain, uint8_t *v)
{
	int ig = (int)gain + 12.5;
	if (ig < 0)
		ig = 0;
	else if (ig > 31)
		ig = 31;

	*outgain = ig - 12.5;
	*v = ig;
	return 0;
}

static int trf_bw_to(unsigned bw, double *outbw, uint8_t *ov)
{
	if (bw > 30e6)
		bw = 30e6;
	else if (bw < 1e6)
		bw = 1e6;
	*outbw = bw;

	// TODO
	int v = 255 * (30.000001e6 - bw) / (29e6);
	if (v < 0)
		v = 0;
	else if (v > 255)
		v = 255;

	*ov = v;
	return 0;
}

static int lms7octo_lo_tune(struct xtrx_lms7octo *dev)
{
	unsigned fref = 19200000; //TODO
	int res = adf4355_tune((spi_out_func_t)adf4355_spi, dev->master,
						   dev->mdev->lo, fref,
						   ((dev->mdev->flags[0] & ADF_INIT) ? 0 : ADF4355_EN_INIT) |
						   ((dev->mdev->flags[0] & LO_PORTB_EN) ? ADF4355_EN_B : 0) |
						   ADF4355_EN_A);
	if (res)
		return res;

	XTRXLLS_LOG("OCTO", XTRXLL_INFO, "LO tuned to %.3fMhz",
				dev->mdev->lo / 1.0e6);
	dev->mdev->flags[0] |= ADF_INIT;
	return 0;
}

static int lms7octo_lo_pd(struct xtrx_lms7octo *dev)
{
	XTRXLLS_LOG("OCTO", XTRXLL_INFO, "LO power down");

	dev->mdev->flags[0] &= ~ADF_INIT;
	return adf4355_pd((spi_out_func_t)adf4355_spi, dev->master);
}


enum {
	GPIO_DEF_FUNC =
		(0 << 26) | (1 << 24) |
		(0 << 22) | (0 << 20) | (0 << 18) | (0 << 16) |
		(0 << 14) | (1 << 12) | (1 << 10) | (1 << 8) |
		(0 << 6)  | (0 << 4)  | (0 << 2)  | (0 << 0),

	GPIO_EXSPI_FUNC =
		(1 << 20) | (0 << 18) | (1 << 16),
};

int lms7octo_init(struct xtrxll_dev* lldev,
				  unsigned flags,
				  const char* fename,
				  struct xtrx_fe_obj** obj)
{
	struct xtrx_lms7octo *dev;
	int res;
	int oval;
	unsigned devno = GET_DEV_FROM_FLAGS(flags);

	if (*obj != NULL) {
		// Check if we at the same FE on non master FE
		struct xtrx_lms7octo tmp_dev;
		_lms7octo_init_base(&tmp_dev);

		struct xtrx_lms7octo* master = *(struct xtrx_lms7octo**)obj;
		if (master->base.ops != tmp_dev.base.ops) {
			return -ENODEV;
		}

		if (devno >= MAX_FE_BOARDS) {
			//return -ENODEV;
			return lms7nfe_init(lldev, flags, fename, obj);
		}
	}

	dev = (struct xtrx_lms7octo*)malloc(sizeof(struct xtrx_lms7octo));
	if (dev == NULL) {
		res = -errno;
		goto failed_mem;
	}
	memset(dev, 0, sizeof(struct xtrx_lms7octo));
	if (*obj != NULL) {
		struct xtrx_lms7octo* master = *(struct xtrx_lms7octo**)obj;
		dev->mdev = master;
		dev->master = master->lms->lldev;
	}

	dev->flags[0] = dev->flags[1] = 0;
	dev->devno = devno;
	dev->en_devs = 0;

	res = lms7nfe_init(lldev, flags, fename, (struct xtrx_fe_obj**)&dev->lms);
	if (res)
		goto failed_lms7;

	if (dev->master == NULL) {
		unsigned stat;

		dev->mdev = dev;
		dev->master = lldev;

		res = xtrxll_set_param(lldev, XTRXLL_PARAM_GPIO_FUNC,
							   GPIO_DEF_FUNC | GPIO_EXSPI_FUNC);
		if (res)
			return res;

		// Init and check
		res = octo_read_spi(lldev, REG_RD_STAT, &stat);
		if (res)
			goto failed_check;

		if ((stat >> 16) != 0xf500) {
			bool automode = (fename == NULL) || (strcmp(fename, "auto") == 0);
			if (!automode || (stat != 0xffffffff && stat != 0x0)) {
				XTRXLLS_LOG("OCTO",
							(automode) ? XTRXLL_WARNING : XTRXLL_ERROR,
							"FE board reports STAT:%08x\n",
							stat);
			}
			res = -ENODEV;
			goto failed_check;
		}
		XTRXLLS_LOG("OCTO", XTRXLL_INFO_LMS, "OCTO FE STAT:%08x, Master IDX:%d\n",
					stat, dev->devno);

		res = xtrxll_set_param(lldev, XTRXLL_PARAM_EXT_SPI,
							   MAKE_OCTO_SPI(REG_GPIO_G, 0xff));
		if (res)
			goto failed_check;

		res = tmp108_get(dev->master, 0, &oval);
		XTRXLLS_LOG("OCTO", XTRXLL_INFO, "Board Temperature: %.2fC\n",
					oval / 256.0);
		if (res)
			goto failed_check;

		// Wait for power up
		usleep(10000);

		for (unsigned k = 0; k < 10; k++) {
			// Set VCM to 0.775V
			unsigned val = 775*4095/3300;
			res = dac_set(lldev, OCTO_I2C_DAC_VCOM, val);
			if (res)
				goto failed_check;

			usleep(5000);
			res = dac_get(lldev, OCTO_I2C_DAC_VCOM, (unsigned*)&oval);
			if (res)
				goto failed_check;

			unsigned z = (oval >> 12) & 0xfff;
			XTRXLLS_LOG("OCTO", XTRXLL_INFO_LMS, "Vcom = %08x (%d)\n", oval, z);
			if (z == val)
				break;
		}
	}

	_lms7octo_init_base(dev);
	dev->trf37_bb_gain[0] = dev->trf37_bb_gain[1] = 13;
	dev->trf37_lpf[0] = dev->trf37_lpf[1] = 128;
	dev->rx_path[0] = dev->rx_path[1] = XTRX_RX_ADC_EXT;

	if (fename && (strcmp(fename, "octoRFX6:clk") == 0)) {
		XTRXLLS_LOG("OCTO", XTRXLL_INFO, "AUTO defaults to LMS RX port, use ADC to enable OCTO frontend\n");
		dev->flags[0] |= RX_DPATH_LMS;
	}

	*obj = (struct xtrx_fe_obj*)dev;
	return 0;

failed_check:
	// Get GPIO state to default
	xtrxll_set_param(lldev, XTRXLL_PARAM_GPIO_FUNC, GPIO_DEF_FUNC);
	lms7nfe_deinit((struct xtrx_fe_obj*)dev->lms);
failed_lms7:
	free(dev);
failed_mem:
	return res;
}

int lms7octo_deinit(struct xtrx_fe_obj* obj)
{
	struct xtrx_lms7octo *dev = (struct xtrx_lms7octo *)obj;
	int res;

	//Turn off sync board
	if (dev->master == dev->lms->lldev) {
		res = xtrxll_set_param(dev->master, XTRXLL_PARAM_EXT_SPI,
							   MAKE_OCTO_SPI(REG_GPIO_G, 0));
		XTRXLLS_LOG("OCTO", XTRXLL_INFO_LMS, "Turn off");
	}
	res = lms7nfe_deinit((struct xtrx_fe_obj*)dev->lms);
	free(dev);

	return res;
}

// Switch RX path in BB between internal and extarnal path
static int octo_switch_fe(struct xtrx_lms7octo* dev,
						  unsigned mod_channel)
{
	int r = 0;
	unsigned channel;
	unsigned octo_chans = get_octo_chans(dev);
	XTRXLLS_LOG("OCTO", XTRXLL_INFO_LMS, "Dev %d: Reconfigure mod=%x octo=%x\n",
				dev->devno, mod_channel, octo_chans);

	if ((channel = (mod_channel & octo_chans))) {
		if (!(dev->mdev->en_devs & (1 << dev->devno))) {
			// Restore saved data
			if ((dev->mdev->en_devs == 0) && (dev->mdev->flags[0] & LO_SET)) {
				XTRXLLS_LOG("OCTO", XTRXLL_INFO, "Restore LO to %.3f\n", dev->mdev->lo / 1.0e6);
				r = (r) ? r : lms7octo_lo_tune(dev);
			}

			dev->mdev->en_devs |= (1 << dev->devno);
			r = (r) ? r : xtrxll_set_param(dev->master, XTRXLL_PARAM_EXT_SPI,
										   MAKE_OCTO_SPI(REG_GPIO_G, 0x000000ff | (dev->mdev->en_devs << 8)));
		}

		// Reconfigure for external ADC input
		r = (r) ? r : lms7_mac_set(&dev->lms->lms_state, channel);
		r = (r) ? r : lms7_rfe_disable(&dev->lms->lms_state);
		r = (r) ? r : lms7_rbb_set_ext(&dev->lms->lms_state);

		r = (r) ? r : lms7octo_update_trf(dev, channel);

		XTRXLLS_LOG("OCTO", XTRXLL_INFO, "Board:%d LMS7 ch:%d configured for extrnal ADC input; OctoEn: %08x\n",
					dev->devno, channel, dev->mdev->en_devs);
	}
	if ((channel = (mod_channel & ~octo_chans))) {
		// Reconfigure for internal ADC input
		r = (r) ? r : lms7nfe_set_gain((struct xtrx_fe_obj* )dev->lms,
									   channel, XTRX_RX_PGA_GAIN,
									   (channel == XTRX_CH_B) ? dev->gain[1] : dev->gain[0],
									   NULL);

		r = (r) ? r : lms7nfe_fe_set_lna((struct xtrx_fe_obj* )dev->lms, channel, 0,
										 (channel == XTRX_CH_B) ? dev->rx_path[1] : dev->rx_path[0]);

		if (dev->flags[0] & BW_SET) {
			r = (r) ? r : lms7nfe_bb_set_badwidth((struct xtrx_fe_obj* )dev->lms,
												  channel, XTRX_TUNE_BB_RX,
												 (channel == XTRX_CH_B) ? dev->bw[1] : dev->bw[0],
												 NULL);
		}
		if (dev->flags[0] & LO_SET) {
			r = (r) ? r : lms7nfe_fe_set_freq((struct xtrx_fe_obj* )dev->lms, channel,
											  (dev->flags[0] & LMS_TDD) ? XTRX_TUNE_TX_AND_RX_TDD : XTRX_TUNE_RX_FDD,
											  dev->lo, NULL);
		}

		if (octo_chans == 0) {
			dev->mdev->en_devs &= ~(1 << dev->devno);
			r = (r) ? r : xtrxll_set_param(dev->master, XTRXLL_PARAM_EXT_SPI,
										   MAKE_OCTO_SPI(REG_GPIO_G, 0x000000ff | (dev->mdev->en_devs << 8)));

			if (dev->mdev->en_devs == 0) {
				r = (r) ? r : lms7octo_lo_pd(dev->mdev);
			}
		}

		XTRXLLS_LOG("OCTO", XTRXLL_INFO, "Board:%d LMS7 ch:%d configured for internal path; OctoEn: %08x\n",
					dev->devno, channel, dev->mdev->en_devs);
	}

	if (r)
		return r;

	// Reconfigure LML port to fix I/Q mirror issue
	struct xtrx_dd_chpar crx = dev->run_rx;
	if ((octo_chans != 0 &&  octo_chans != XTRX_CH_AB) &&
			crx.chs == XTRX_CH_AB && !(crx.flags & XTRX_RSP_SISO_MODE)) {
		// Both LMS & OctoPath active on MIMO

		if (octo_chans & XTRX_CH_A) {
			crx.flags |= XTRX_RSP_SWAP_IQA;
		} else {
			crx.flags |= XTRX_RSP_SWAP_IQB;
		}
	} else if ((mod_channel & octo_chans)) {
		crx.flags ^= XTRX_RSP_SWAP_IQ;
	}

	dev->lms->maprx = lms7nfe_get_lml_portcfg(&crx, dev->lms->rx_no_siso_map);
	r = lms7_lml_set_map(&dev->lms->lms_state,
						   dev->lms->rx_port_1 ? dev->lms->maprx : dev->lms->maptx,
						   dev->lms->rx_port_1 ? dev->lms->maptx : dev->lms->maprx);

	return r;
}

int lms7octo_dd_set_samplerate(struct xtrx_fe_obj* obj,
							 const struct xtrx_fe_samplerate* inrates,
							 struct xtrx_fe_samplerate* outrates)
{
	struct xtrx_lms7octo *dev = (struct xtrx_lms7octo *)obj;
	return lms7nfe_dd_set_samplerate((struct xtrx_fe_obj*)dev->lms,
									 inrates,
									 outrates);
}

int lms7octo_dd_set_modes(struct xtrx_fe_obj* obj,
						  unsigned op,
						  const struct xtrx_dd_params *params)
{
	struct xtrx_lms7octo *dev = (struct xtrx_lms7octo *)obj;
	int res;

	switch (op) {
	case XTRX_FEDD_CONFIGURE:
		if (params->dir & XTRX_RX) {
			dev->run_rx = params->rx;
			res =  lms7nfe_dd_set_modes((struct xtrx_fe_obj*)dev->lms,
										op, params);
			if (res)
				return res;

			// TODO: FIXUP for old software
			unsigned enchans = params->rx.chs;
			if (enchans == LMS7_CH_AB && (params->rx.flags & XTRX_RSP_SISO_MODE)) {
				if (params->rx.flags & XTRX_RSP_SWAP_AB) {
					enchans = LMS7_CH_B;
				} else {
					enchans = LMS7_CH_A;
				}
			}
			if (enchans & LMS7_CH_A) {
				dev->flags[0] |= RX_ACTIVE;
			}
			if (enchans & LMS7_CH_B) {
				dev->flags[1] |= RX_ACTIVE;
			}

			//XTRXLLS_LOG("OCTO", XTRXLL_INFO_LMS, "ENCH %d\n", enchans);

			res = octo_switch_fe(dev, enchans);
			if (res)
				return res;

			// Update both TRF, PD one if not using
			res = lms7octo_update_trf(dev, XTRX_CH_AB);
			if (res)
				return res;
		} else {
			res =  lms7nfe_dd_set_modes((struct xtrx_fe_obj*)dev->lms,
										op, params);
		}
		return res;
	case XTRX_FEDD_RESET:
		if (params->dir & XTRX_RX) {
			if (dev->master == dev->lms->lldev) {
				xtrxll_set_param(dev->master, XTRXLL_PARAM_EXT_SPI,
								 MAKE_OCTO_SPI(REG_GPIO_G, 0x000000ff));

				lms7octo_lo_pd(dev->mdev);
			}
			dev->flags[0] &= ~RX_ACTIVE;
			dev->flags[1] &= ~RX_ACTIVE;
		}
		return lms7nfe_dd_set_modes((struct xtrx_fe_obj*)dev->lms,
								op, params);
	}

	return -EINVAL;
}

int lms7octo_bb_set_freq(struct xtrx_fe_obj* obj,
						 unsigned channel,
						 unsigned type,
						 double freq,
						 double* actualfreq)
{
	struct xtrx_lms7octo *dev = (struct xtrx_lms7octo *)obj;
	return lms7nfe_bb_set_freq((struct xtrx_fe_obj*)dev->lms,
							   channel,
							   type,
							   freq,
							   actualfreq);
}

int lms7octo_bb_set_badwidth(struct xtrx_fe_obj* obj,
							 unsigned channel,
							 unsigned type,
							 double bw,
							 double* actualbw)
{
	int res;
	unsigned ochmsk = 0;

	struct xtrx_lms7octo *dev = (struct xtrx_lms7octo *)obj;
	if (type == XTRX_TUNE_BB_RX) {
		if (channel & XTRX_CH_A) {
			dev->bw[0] = bw;
			dev->flags[0] |= BW_SET;
		}
		if (channel & XTRX_CH_B) {
			dev->bw[1] = bw;
			dev->flags[1] |= BW_SET;
		}

		/*
		bool bypass = (bw > 60e6);
		if (bypass) {
			uint32_t td3 = MAKE_TRF37_DEV_SETUP3(1, 0);
			return trf37_spi(dev->master,
							 XTRX_CH_TO_OCTO(dev->devno, channel),
							 td3);
		}
		*/
		uint8_t v;
		if (channel & XTRX_CH_A) {
			trf_bw_to(dev->bw[0], actualbw, &v);
			dev->trf37_lpf[0] = v;
		}
		if (channel & XTRX_CH_B) {
			trf_bw_to(dev->bw[1], actualbw, &v);
			dev->trf37_lpf[1] = v;
		}

		ochmsk = get_octo_chans(dev);
	}

	if (ochmsk & channel) {
		res = lms7octo_update_trf(dev, channel);
		if (res)
			return res;
	}
	if (~ochmsk & channel) {
		res = lms7nfe_bb_set_badwidth((struct xtrx_fe_obj*)dev->lms,
									  ~ochmsk & channel,
									  type,
									  bw,
									  actualbw);
		if (res)
			return res;
	}

	return 0;
}

int lms7octo_set_gain(struct xtrx_fe_obj* obj,
					  unsigned channel,
					  unsigned gain_type,
					  double gain,
					  double *actualgain)
{
	struct xtrx_lms7octo *dev = (struct xtrx_lms7octo *)obj;
	int res;
	unsigned ochmsk = 0;

	if (gain_type == XTRX_RX_PGA_GAIN) {
		if (channel & XTRX_CH_A) {
			dev->gain[0] = gain;
		}
		if (channel & XTRX_CH_B) {
			dev->gain[1] = gain;
		}

		uint8_t bbg;
		if (channel & XTRX_CH_A) {
			trf_gain_to(gain, actualgain, &bbg);
			dev->trf37_bb_gain[0] = bbg;
		}
		if (channel & XTRX_CH_B) {
			trf_gain_to(gain, actualgain, &bbg);
			dev->trf37_bb_gain[1] = bbg;
		}

		ochmsk = get_octo_chans(dev);
	}

	if (ochmsk & channel) {
		res = lms7octo_update_trf(dev, channel);
		if (res)
			return res;
	}
	if (~ochmsk & channel) {
		// Gain settings will override ADC path, filter out not our channels
		res = lms7nfe_set_gain((struct xtrx_fe_obj*)dev->lms,
							   ~ochmsk & channel,
							   gain_type,
							   gain,
							   actualgain);
		if (res)
			return res;
	}
	return 0;
}

int lms7octo_fe_set_refclock(struct xtrx_fe_obj* obj,
                             double refclock)
{
	struct xtrx_lms7octo *dev = (struct xtrx_lms7octo *)obj;
	return lms7nfe_fe_set_refclock((struct xtrx_fe_obj*)dev->lms,
	                               refclock);
}

#define ABSF(x) (((x) < 0) ? -(x) : (x))

int lms7octo_fe_set_freq(struct xtrx_fe_obj* obj,
						 unsigned channel,
						 unsigned type,
						 double freq,
						 double *actualfreq)
{
	struct xtrx_lms7octo *dev = (struct xtrx_lms7octo *)obj;
	int res;
	unsigned ochmsk = 0;

	if (type == XTRX_TUNE_RX_FDD || type == XTRX_TUNE_TX_AND_RX_TDD) {
		ochmsk = get_octo_chans(dev);
	}

	XTRXLLS_LOG("OCTO", XTRXLL_INFO_LMS, "LO CH:%d:%d  %02x -> %.3f\n",
				channel, ochmsk, dev->mdev->flags[0], freq / 1e6);

	if (ochmsk & channel) {
		if (!((dev->mdev->flags[0] & LO_SET) && (ABSF(dev->mdev->lo - freq) < 1))) {
			dev->mdev->lo = freq;
			dev->mdev->flags[0] |= LO_SET;

			//XTRXLLS_LOG("OCTO", XTRXLL_INFO, "Update LO to %.3f Mhz\n", freq / 1.0e6);
			res = lms7octo_lo_tune(dev);
			if (res)
				return res;
		}
		*actualfreq = freq;
	}
	if (~ochmsk & channel) {
		res = lms7nfe_fe_set_freq((struct xtrx_fe_obj*)dev->lms,
								  ~ochmsk & channel,
								  type,
								  freq,
								  actualfreq);
		if (res)
			return res;
	}

	if (type == XTRX_TUNE_RX_FDD || type == XTRX_TUNE_TX_AND_RX_TDD) {
		dev->lo = freq;
		dev->flags[0] |= LO_SET;
		if (type == XTRX_TUNE_TX_AND_RX_TDD) {
			dev->flags[0] |= LMS_TDD;
		} else {
			dev->flags[0] &= ~LMS_TDD;
		}
	}
	return 0;
}

static int is_rx_path(unsigned lna)
{
	switch (lna) {
	case XTRX_RX_L:
	case XTRX_RX_H:
	case XTRX_RX_W:
	case XTRX_RX_L_LB:
	case XTRX_RX_W_LB:
	case XTRX_RX_AUTO:
	case XTRX_RX_ADC_EXT:
		return true;
	}

	return false;
}

int lms7octo_fe_set_lna(struct xtrx_fe_obj* obj,
						unsigned channel,
						unsigned dir,
						unsigned lna)
{
	struct xtrx_lms7octo *dev = (struct xtrx_lms7octo *)obj;
	if (!is_rx_path(lna)) {
		return lms7nfe_fe_set_lna((struct xtrx_fe_obj*)dev->lms,
								  channel,
								  dir,
								  lna);
	}

	unsigned bocto_chans = get_octo_chans(dev);
	if (channel & XTRX_CH_A) {
		dev->rx_path[0] = lna;
	}
	if (channel & XTRX_CH_B) {
		dev->rx_path[1] = lna;
	}
	unsigned aocto_chans = get_octo_chans(dev);

	if (bocto_chans != aocto_chans) {
		return octo_switch_fe(dev, channel);
	}
	return 0;
}

int lms7octo_get_reg(struct xtrx_fe_obj* obj,
					 unsigned channel,
					 unsigned dir,
					 unsigned type,
					 uint64_t* outval)
{
	struct xtrx_lms7octo *dev = (struct xtrx_lms7octo *)obj;
	return lms7nfe_get_reg((struct xtrx_fe_obj*)dev->lms,
						   channel,
						   dir,
						   type,
						   outval);
}

int lms7octo_set_reg(struct xtrx_fe_obj* obj,
					 unsigned channel,
					 unsigned dir,
					 unsigned type,
					 uint64_t val)
{
	struct xtrx_lms7octo *dev = (struct xtrx_lms7octo *)obj;
	return lms7nfe_set_reg((struct xtrx_fe_obj*)dev->lms,
						   channel,
						   dir,
						   type,
						   val);
}

static const struct xtrx_fe_ops _lms7octo_ops = {
	lms7octo_dd_set_modes,
	lms7octo_dd_set_samplerate,

	lms7octo_bb_set_freq,
	lms7octo_bb_set_badwidth,
	lms7octo_set_gain,

	lms7octo_fe_set_refclock,
	lms7octo_fe_set_freq,
	lms7octo_fe_set_lna,
	lms7octo_set_gain,

	lms7octo_get_reg,
	lms7octo_set_reg,

	lms7octo_deinit,
};

void _lms7octo_init_base(struct xtrx_lms7octo *dev)
{
	dev->base.ops = &_lms7octo_ops;
}
