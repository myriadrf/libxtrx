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
#include "octo/xtrx_octo_api.h"

struct xtrx_lms7octocal
{
	struct xtrx_fe_obj base;

	struct xtrx_nfe_lms7 *lms;
	struct xtrxll_dev *master;
	struct xtrx_lms7octocal *mdev;

	double fcal;
	unsigned devno;
	bool active_cal_path;
};

enum {
	GPIO_DEF_FUNC =
		(0 << 26) | (1 << 24) |
		(0 << 22) | (0 << 20) | (0 << 18) | (0 << 16) |
		(0 << 14) | (1 << 12) | (1 << 10) | (1 << 8) |
		(0 << 6)  | (0 << 4)  | (0 << 2)  | (0 << 0),

	GPIO_EXSPI_FUNC =
		(1 << 20) | (0 << 18) | (1 << 16),
};

static void _lms7octocal_init_base(struct xtrx_lms7octocal *dev);

static int exspi_sync_cs(struct xtrxll_dev *dev, bool set)
{
	unsigned cmd = (set) ? (1 << XTRX_GPIO_EXT1) :
						   (1 << (XTRX_GPIO_EXT1 + XTRX_GPIOS_TOTAL));
	return xtrxll_set_param(dev, XTRXLL_PARAM_GPIO_CS, cmd);
}
static int exspi_mosi_cs(struct xtrxll_dev *dev, bool set)
{
	unsigned cmd = (set) ? (1 << XTRX_GPIO_EXT0) :
						   (1 << (XTRX_GPIO_EXT0 + XTRX_GPIOS_TOTAL));
	return xtrxll_set_param(dev, XTRXLL_PARAM_GPIO_CS, cmd);
}
static int exspi_sclk_cs(struct xtrxll_dev *dev, bool set)
{
	unsigned cmd = (set) ? (1 << XTRX_GPIO_EXT2) :
						   (1 << (XTRX_GPIO_EXT2 + XTRX_GPIOS_TOTAL));
	return xtrxll_set_param(dev, XTRXLL_PARAM_GPIO_CS, cmd);
}


static int exspi_mux_dir_out(struct xtrxll_dev *dev, bool out)
{
	// MUX is connected to SYNC
	return xtrxll_set_param(dev, XTRXLL_PARAM_GPIO_DIR,
							(out) ? (1 << XTRX_GPIO_EXT1) : 0);
}

static int exspi_mux_read(struct xtrxll_dev *dev, bool *outval)
{
	int in;
	int res = xtrxll_get_sensor(dev, XTRXLL_GPIO_IN, &in);
	if (res)
		return res;

	*outval = (in & (1 << XTRX_GPIO_EXT1)) ? true : false;
	return 0;
}

static int adf4355_spi(struct xtrxll_dev *dev, uint32_t out)
{
	int res;
	res = exspi_sync_cs(dev, false);
	if (res)
		return res;

	res = xtrxll_set_param(dev, XTRXLL_PARAM_EXT_SPI, out);
	if (res)
		return res;

	// TODO: wait for interrupt!
	usleep(5000);

	res = exspi_sync_cs(dev, true);
	if (res)
		return res;

	usleep(100);
	return 0;
}

static int soft_spi_dac(struct xtrxll_dev *dev, unsigned value)
{
	int res;
	res = xtrxll_set_param(dev,
						   XTRXLL_PARAM_GPIO_FUNC,
						   GPIO_DEF_FUNC);
	if (res)
		return res;

	res = xtrxll_set_param(dev, XTRXLL_PARAM_GPIO_DIR,
						(1 << XTRX_GPIO_EXT0) |
						(1 << XTRX_GPIO_EXT1) |
						(1 << XTRX_GPIO_EXT2));
	if (res)
		return res;

	usleep(1);

	res = exspi_sync_cs(dev, false);
	if (res)
		return res;
	res = exspi_mosi_cs(dev, false);
	if (res)
		return res;
	res = exspi_sclk_cs(dev, false);
	if (res)
		return res;

	int i,j;
	uint32_t regs[2] =
	{
		(0x40000 | 0x0101),
		(0x80000 | (0xffff & value)),
	};

	for (j = 0; j < 2; j++) {
		uint32_t reg = regs[j];

		for (i = 31; i >= 0; i--) {
			res = exspi_sclk_cs(dev, true);
			if (res)
				return res;
			res = exspi_mosi_cs(dev, reg & (1U << i));
			if (res)
				return res;

			if (i == 31) {
				res = exspi_sync_cs(dev, true);
				if (res)
					return res;
			}

			res = exspi_sclk_cs(dev, false);
			if (res)
				return res;
		}

		//exspi_miso_dclk(dev, true);
		res = exspi_sync_cs(dev, false);
		if (res)
			return res;

		for (i = 31; i >= 0; i--) {
			res = exspi_mosi_cs(dev, 0x0081200B & (1U << i));
			if (res)
				return res;
			res = exspi_sclk_cs(dev, true);
			if (res)
				return res;
			res = exspi_sclk_cs(dev, false);
			if (res)
				return res;
		}

		usleep(10);
	}

	return 0;
}



#define FREF  19200000

static int lms7octocal_check_adf(struct xtrxll_dev* lldev)
{
	int i;
	int res;
	bool miso_stp[2];
	res = xtrxll_set_param(lldev, XTRXLL_PARAM_GPIO_FUNC,
						   GPIO_DEF_FUNC | GPIO_EXSPI_FUNC);
	if (res)
		return res;

	for (i = 0; i < 2; i++) {
		res = exspi_mux_dir_out(lldev, true);
		if (res)
			return res;

		res = adf4355_muxout((spi_out_func_t)adf4355_spi, lldev,
							 (i == 0) ? MUXOUT_DGND : MUXOUT_DVDD);
		if (res)
			return res;

		res = exspi_mux_dir_out(lldev, false);
		if (res)
			return res;

		res = exspi_mux_read(lldev, &miso_stp[i]);
		if (res)
			return res;
	}

	XTRXLLS_LOG("OSYN", XTRXLL_ERROR, "MUX[0,1] -> %d,%d",
				miso_stp[0], miso_stp[1]);

	if (miso_stp[0] != false || miso_stp[1] != true)
		return -ENODEV;

	return 0;
}

static int check_adf4355_lock(struct xtrxll_dev *lldev, bool* locked)
{
	int res;

	// Wait for lock signal to stabilize, before that we can't drive
	// CS down
	usleep(1000000);

	res = exspi_mux_dir_out(lldev, false);
	if (res)
		return res;

	res = exspi_sync_cs(lldev, false);
	if (res)
		return res;

	res = exspi_mux_read(lldev, locked);
	if (res)
		return res;

	XTRXLLS_LOG("OSYN", *locked ? XTRXLL_INFO : XTRXLL_ERROR,
				"LO locked: %d", *locked);
	return 0;
}

int lms7octocal_init(struct xtrxll_dev* lldev,
					 unsigned flags,
					 const char* fename,
					 struct xtrx_fe_obj** obj)
{
	struct xtrx_lms7octocal *dev;
	int res;
	unsigned devno = GET_DEV_FROM_FLAGS(flags);
	bool locked;

	dev = (struct xtrx_lms7octocal*)malloc(sizeof(struct xtrx_lms7octocal));
	if (dev == NULL) {
		res = -errno;
		goto failed_mem;
	}
	memset(dev, 0, sizeof(struct xtrx_lms7octocal));
	if (*obj != NULL) {
		struct xtrx_lms7octocal* master = *(struct xtrx_lms7octocal**)obj;
		dev->mdev = master;
		dev->master = master->lms->lldev;
	}

	dev->devno = devno;
	dev->fcal = 0;

	res = lms7nfe_init(lldev, flags, fename, (struct xtrx_fe_obj**)&dev->lms);
	if (res)
		goto failed_lms7;


	if (dev->master == NULL) {
		dev->mdev = dev;
		dev->master = lldev;
		dev->active_cal_path = false;

		// Detect and check board
		res = lms7octocal_check_adf(lldev);
		if (res)
			return res;

		res = exspi_mux_dir_out(lldev, true);
		if (res)
			return res;

		usleep(100000);

		// Tune to 64.0Mhz as test cal frequency
		res = adf4355_tune((spi_out_func_t)adf4355_spi, lldev, 64e6, FREF,
						   ADF4355_EN_INIT | ADF4355_EN_A | ADF4355_EN_B);
		if (res)
			return res;

		res = check_adf4355_lock(lldev, &locked);
		if (res)
			return res;

		// Initialize & Drive DAC to mid scale
		res = exspi_mux_dir_out(lldev, true);
		if (res)
			return res;

		res = soft_spi_dac(lldev, 45000);
		if (res)
			return res;

		res = exspi_mux_dir_out(lldev, false);
		if (res)
			return res;
	}

	_lms7octocal_init_base(dev);
	*obj = (struct xtrx_fe_obj*)dev;
	return 0;

failed_lms7:
	free(dev);
failed_mem:
	return res;
}

static int lms7octocal_set_cal_path(struct xtrx_lms7octocal *dev, bool cal)
{
	int res;
	res = xtrxll_set_param(dev->master, XTRXLL_PARAM_GPIO_FUNC,
						   GPIO_DEF_FUNC | GPIO_EXSPI_FUNC);
	if (res)
		return res;

	res = exspi_mux_dir_out(dev->master, true);
	if (res)
		return res;

	res = adf4355_muxout((spi_out_func_t)adf4355_spi, dev->master,
						(cal) ? MUXOUT_DGND : MUXOUT_DVDD);
	if (res)
		return res;

	dev->active_cal_path = cal;
	XTRXLLS_LOG("OSYN", XTRXLL_WARNING, "Path set to %s\n",
				(cal) ? "calibration" : "lna");
	return 0;
}

static int lms7octocal_tune_fe(struct xtrx_lms7octocal *dev, double freq)
{
	bool locked;
	int res;
	if (dev->mdev->fcal == freq)
		return 0;

#if 0
	res = adf4355_muxout((spi_out_func_t)adf4355_spi, dev->master,
						 MUXOUT_DGND);
	if (res)
		return res;
#endif
	res = xtrxll_set_param(dev->master, XTRXLL_PARAM_GPIO_FUNC,
						   GPIO_DEF_FUNC | GPIO_EXSPI_FUNC);
	if (res)
		return res;

	res = exspi_mux_dir_out(dev->master, true);
	if (res)
		return res;

	res = adf4355_tune((spi_out_func_t)adf4355_spi, dev->master, freq, FREF,
					   ADF4355_EN_A | ADF4355_EN_B);
	if (res)
		return res;

	res = xtrxll_set_param(dev->master, XTRXLL_PARAM_EXT_SPI, 0x0081200B);
	if (res)
		return res;

	res = check_adf4355_lock(dev->master, &locked);
	if (res)
		return res;

	res = lms7octocal_set_cal_path(dev, dev->mdev->active_cal_path);
	if (res)
		return res;

	dev->mdev->fcal = freq;
	XTRXLLS_LOG("OSYN", XTRXLL_WARNING, "FE tuned %.3f Mhz\n", freq/1e6);
	return (locked) ? 0 : -EFAULT;
}

static int lms7octocal_update_dac(struct xtrx_lms7octocal *dev, unsigned dac_val)
{
	int res;
	res = exspi_mux_dir_out(dev->master, true);
	if (res)
		return res;

	res = soft_spi_dac(dev->master, dac_val);
	if (res)
		return res;

	res = exspi_mux_dir_out(dev->master, false);
	if (res)
		return res;

	XTRXLLS_LOG("OSYN", XTRXLL_WARNING, "DAC set to %d\n", dac_val);
	return 0;
}

int lms7octocal_dd_set_samplerate(struct xtrx_fe_obj* obj,
								  const struct xtrx_fe_samplerate* inrates,
								  struct xtrx_fe_samplerate* outrates)
{
	struct xtrx_lms7octocal *dev = (struct xtrx_lms7octocal *)obj;
	return lms7nfe_dd_set_samplerate((struct xtrx_fe_obj*)dev->lms,
									 inrates,
									 outrates);
}

int lms7octocal_dd_set_modes(struct xtrx_fe_obj* obj,
							 unsigned op,
							 const struct xtrx_dd_params *params)
{
	struct xtrx_lms7octocal *dev = (struct xtrx_lms7octocal *)obj;
	return lms7nfe_dd_set_modes((struct xtrx_fe_obj*)dev->lms,
								op, params);
}


int lms7octocal_bb_set_freq(struct xtrx_fe_obj* obj,
						 unsigned channel,
						 unsigned type,
						 double freq,
						 double* actualfreq)
{
	struct xtrx_lms7octocal *dev = (struct xtrx_lms7octocal *)obj;
	return lms7nfe_bb_set_freq((struct xtrx_fe_obj*)dev->lms,
							   channel,
							   type,
							   freq,
							   actualfreq);
}

int lms7octocal_bb_set_badwidth(struct xtrx_fe_obj* obj,
								unsigned channel,
								unsigned type,
								double bw,
								double* actualbw)
{
	struct xtrx_lms7octocal *dev = (struct xtrx_lms7octocal *)obj;
	return lms7nfe_bb_set_badwidth((struct xtrx_fe_obj*)dev->lms,
								   channel,
								   type,
								   bw,
								   actualbw);
}

int lms7octocal_set_gain(struct xtrx_fe_obj* obj,
						 unsigned channel,
						 unsigned gain_type,
						 double gain,
						 double *actualgain)
{
	struct xtrx_lms7octocal *dev = (struct xtrx_lms7octocal *)obj;
	return lms7nfe_set_gain((struct xtrx_fe_obj*)dev->lms,
							channel,
							gain_type,
							gain,
							actualgain);
}

int lms7octocal_fe_set_refclock(struct xtrx_fe_obj* obj,
                                double refclock)
{
	struct xtrx_lms7octocal *dev = (struct xtrx_lms7octocal *)obj;
	return lms7nfe_fe_set_refclock((struct xtrx_fe_obj*)dev->lms,
	                               refclock);
}

int lms7octocal_fe_set_freq(struct xtrx_fe_obj* obj,
							unsigned channel,
							unsigned type,
							double freq,
							double *actualfreq)
{
	struct xtrx_lms7octocal *dev = (struct xtrx_lms7octocal *)obj;
	if (type == XTRX_TUNE_EXT_FE) {
		if (actualfreq) {
			*actualfreq = freq;
		}
		return lms7octocal_tune_fe(dev, freq);
	}
	return lms7nfe_fe_set_freq((struct xtrx_fe_obj*)dev->lms,
							   channel,
							   type,
							   freq,
							   actualfreq);
}


int lms7octocal_fe_set_lna(struct xtrx_fe_obj* obj,
						   unsigned channel,
						   unsigned dir,
						   unsigned lna)
{
	struct xtrx_lms7octocal *dev = (struct xtrx_lms7octocal *)obj;
	return lms7nfe_fe_set_lna((struct xtrx_fe_obj*)dev->lms,
							  channel,
							  dir,
							  lna);
}

int lms7octocal_get_reg(struct xtrx_fe_obj* obj,
						unsigned channel,
						unsigned dir,
						unsigned type,
						uint64_t* outval)
{
	struct xtrx_lms7octocal *dev = (struct xtrx_lms7octocal *)obj;
	return lms7nfe_get_reg((struct xtrx_fe_obj*)dev->lms,
						   channel,
						   dir,
						   type,
						   outval);
}

int lms7octocal_set_reg(struct xtrx_fe_obj* obj,
						unsigned channel,
						unsigned dir,
						unsigned type,
						uint64_t val)
{
	struct xtrx_lms7octocal *dev = (struct xtrx_lms7octocal *)obj;
	switch (type) {
	case XTRX_OCTO_OSC_DAC:
		return lms7octocal_update_dac(dev, (unsigned)val);

	case XTRX_OCTO_CAL_PATH:
		if (dev->mdev->active_cal_path != ((bool)(val != 0))) {
			return lms7octocal_set_cal_path(dev, (bool)(val != 0));
		}
	}

	return lms7nfe_set_reg((struct xtrx_fe_obj*)dev->lms,
						   channel,
						   dir,
						   type,
						   val);
}

int lms7octocal_deinit(struct xtrx_fe_obj* obj)
{
	struct xtrx_lms7octocal *dev = (struct xtrx_lms7octocal *)obj;
	int res;

	res = lms7nfe_deinit((struct xtrx_fe_obj*)dev->lms);
	free(dev);

	return res;
}


static const struct xtrx_fe_ops _lms7octocal_ops = {
	lms7octocal_dd_set_modes,
	lms7octocal_dd_set_samplerate,

	lms7octocal_bb_set_freq,
	lms7octocal_bb_set_badwidth,
	lms7octocal_set_gain,

	lms7octocal_fe_set_refclock,
	lms7octocal_fe_set_freq,
	lms7octocal_fe_set_lna,
	lms7octocal_set_gain,

	lms7octocal_get_reg,
	lms7octocal_set_reg,

	lms7octocal_deinit,
};

void _lms7octocal_init_base(struct xtrx_lms7octocal *dev)
{
	dev->base.ops = &_lms7octocal_ops;
}
