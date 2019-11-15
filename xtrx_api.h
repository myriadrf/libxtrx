/*
 * Public xtrx API header file
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
#ifndef XTRX_API_H
#define XTRX_API_H

#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif


#define XTRX_API

/* High level API function */

typedef void (*xtrx_logfunc_t)(int sevirity,
							   const struct tm* stm,
							   int nsec,
							   const char sybsystem[4],
							   const char* function,
							   const char* file,
							   int line_no,
							   const char* fmt,
							   va_list list);
XTRX_API void xtrx_log_setfunc(xtrx_logfunc_t func);
XTRX_API void xtrx_log_setlevel(int sevirity, const char sybsystem[4]);


struct xtrx_dev;

enum xtrx_flags {
	XTRX_O_LOGLVL_MASK      = 0x000f,
	XTRX_O_LOGLVL_LMS7_OFF  = 4,
	XTRX_O_LOGLVL_LMS7_MASK = 0x00f0,
	XTRX_O_RESET            = 0x0100,
};

typedef struct gtime_data {
	uint32_t sec;
	uint32_t nsec;
} gtime_data_t;


/**
 * @brief master_ts
 */
typedef uint64_t master_ts;


/** Open XTRX device
 * @param device    Path to XTRX /dev entry
 * @param flags     Not implemented
 * @param[out] dev  XTRX device handle
 * @return 0 on success, errno on error
 */
XTRX_API int xtrx_open(const char* device, unsigned flags, struct xtrx_dev** dev);

enum {
    XTRX_OMI_DEBUGIF = 1,
    XTRX_OMI_FE_SET = 2,
};

typedef struct xtrx_open_multi_info {
    uint32_t flags;
    uint32_t flagsex;

    unsigned devcount;
    int loglevel;

    const char** devices;

    const char* frontend;
    void* reserved[32 - 1];
} xtrx_open_multi_info_t;

/** Open XTRX composed of multiply devices
 * @brief xtrx_open_multi
 * @param numdevs
 * @param devices
 * @param flags
 * @param dev
 * @return
 */
XTRX_API int xtrx_open_multi(const xtrx_open_multi_info_t* dinfo, struct xtrx_dev** dev);

/** Open XTRX device form semicolon separated device list
 * @param paramstring  Path to XTRX devices, semicolon separated followed by double semicolon and flags
 * @param[out] dev  XTRX device handle
 * @return number of devices on success, -errno on error
 *
 * String should not contain any whitespaces, all names should be in ASCII with
 * ending 0 character
 *
 * Examples:
 * NULL -- just first enumerated device and open with default parameters
 * "usb3380" -- Open usb3380 XTRX
 * ";;loglevel=7" -- Open first enumerated with specific arguments
 * "/dev/xtrx0;/dex/xtrx1;;fe=octoRFX6;loglevel=4"
 *
 * When @ref devices is NULL only first enumerated device is created.
 * Only 'loglevel' flag is parsed.
 */
XTRX_API int xtrx_open_string(const char* paramstring, struct xtrx_dev** dev);



/** Close XTRX device
 * @param dev       XTRX device handle
 */
XTRX_API void xtrx_close(struct xtrx_dev* dev);


typedef enum xtrx_clock_source {
	/// Use internal oscillator
	XTRX_CLKSRC_INT = 0,

	/// Use extrenal ref clock
	XTRX_CLKSRC_EXT = 1,

	/// Use extrenal ref clock and also sync all timed transactions to extrnal
	/// 1PPS source
	XTRX_CLKSRC_EXT_W1PPS_SYNC = 2,
} xtrx_clock_source_t;

XTRX_API int xtrx_set_ref_clk(struct xtrx_dev* dev, unsigned refclkhz, xtrx_clock_source_t clksrc);

enum {
	XTRX_DEVINFO_UNIQNAME_MAX = 64,
	XTRX_PROTO_MAX = 16,
	XTRX_SPEED_MAX = 16,
	XTRX_SERIAL_MAX = 32,
	XTRX_DEVID_MAX = 64,
};

typedef struct xtrx_device_info {
	char uniqname[XTRX_DEVINFO_UNIQNAME_MAX];
	char proto[XTRX_PROTO_MAX];
	char speed[XTRX_SPEED_MAX];
	char serial[XTRX_SERIAL_MAX];
	char devid[XTRX_DEVID_MAX];
} xtrx_device_info_t;

XTRX_API int xtrx_discovery(xtrx_device_info_t* devs, size_t maxbuf);


typedef enum xtrx_samplerate_flags {
	// Flags 1 throgh 8 are reserveed for debug
	XTRX_SAMPLERATE_DEBUG_NO_RX_SISO_LML = (1U << 0),
	XTRX_SAMPLERATE_DEBUG_SLOW_MCLK = (1U << 1),
	XTRX_SAMPLERATE_DEBUG_NO_RX_DECIM = (1U << 2),
	XTRX_SAMPLERATE_DEBUG_NO_TX_INTR = (1U << 3),

	XTRX_SAMPLERATE_FORCE_TX_INTR = (1U << 4),
	XTRX_SAMPLERATE_FORCE_RX_DECIM = (1U << 5),

	/* bits from 6 to 15 are reserved */

	XTRX_SAMPLERATE_DEBUG_NO_RX_FCLK_GEN = (1U << 16),
	XTRX_SAMPLERATE_DEBUG_NO_TX_SISO_LML = (1U << 17),
	XTRX_SAMPLERATE_DEBUG_NO_8MA_LML = (1U << 18),
	XTRX_SAMPLERATE_DEBUG_NO_VIO_SET = (1U << 19),

	/* bits from 20 to 29 are reserved */

	/** Update samplerate at runtime, timing can't be kept precise */
	XTRX_SAMPLERATE_FORCE_UPDATE = (1U << 30),
	XTRX_SAMPLERATE_AUTO_DECIM = (1U << 31),
} xtrx_samplerate_flags_t;

/** Set samplerate for the XTRX device
 *
 * This function configures CGEN block, determines best inerpolation/decimation and
 * configures LML inerface for the given clock configuration.
 *
 * @param dev       XTRX device handle
 * @param cgen_rate CGEN clock rate, 0 for autoselect
 * @param rxrate    RX sample rate after all decimation stages (as seen on the PCIe interaface), 0 to disable RX
 * @param txrate    TX sample rate before any interpolation (as seen on the PCIe interaface), 0 to disable TX
 * @param[out] actualcgen Actual CGEN clock
 * @param[out] actualrx   Actual RX clock
 * @param[out] actualtx   Actual TX clock
 * @return 0 on success, -EINVAL if combination of master/rx/tx clock can't be
 *         delivered or both rx and tx are zeros, -ERANGE this master clock
 *         is unavailable or failed to tune.
 */
XTRX_API int xtrx_set_samplerate(struct xtrx_dev* dev, double cgen_rate, double rxrate,
								 double txrate, unsigned flags, double* actualcgen, double* actualrx,
								 double* actualtx);


typedef enum xtrx_channel {
	XTRX_CH_A  = 1,
	XTRX_CH_B  = 2,
	XTRX_CH_AB = XTRX_CH_A | XTRX_CH_B,
	XTRX_CH_ALL = ~0U,
} xtrx_channel_t;


typedef enum xtrx_tune {
	XTRX_TUNE_RX_FDD,
	XTRX_TUNE_TX_FDD,
	XTRX_TUNE_TX_AND_RX_TDD,

	/** Tune baseband (DSP) frequency for RX */
	XTRX_TUNE_BB_RX,

	/** Tune baseband (DSP) frequency for TX */
	XTRX_TUNE_BB_TX,

	/** Extrenal FE frequency tune */
	XTRX_TUNE_EXT_FE,
} xtrx_tune_t;

XTRX_API int xtrx_tune(struct xtrx_dev* dev, xtrx_tune_t type, double freq, double *actualfreq);
XTRX_API int xtrx_tune_ex(struct xtrx_dev* dev, xtrx_tune_t type, xtrx_channel_t ch, double freq, double *actualfreq);


/**< Tune filter for specific bandwidth
 *
 * CGEN frequency must be set (see @ref xtrx_set_samplerate() call) before
 * tunning bandwidth. @ref xtrx_tune() should be called after bandwidth
 * callibration
 */
XTRX_API int xtrx_tune_tx_bandwidth(struct xtrx_dev* dev, xtrx_channel_t ch, double bw,
									double *actualbw);

XTRX_API int xtrx_tune_rx_bandwidth(struct xtrx_dev* dev, xtrx_channel_t ch, double bw,
									double *actualbw);


typedef enum xtrx_gain_type {
	XTRX_RX_LNA_GAIN,
	XTRX_RX_TIA_GAIN,
	XTRX_RX_PGA_GAIN,

	XTRX_TX_PAD_GAIN,

	XTRX_RX_LB_GAIN, /* loopback gain */
} xtrx_gain_type_t;

XTRX_API int xtrx_set_gain(struct xtrx_dev* dev, xtrx_channel_t ch, xtrx_gain_type_t gt,
						   double gain, double *actualgain);


typedef enum xtrx_antenna {
	XTRX_RX_L,
	XTRX_RX_H,
	XTRX_RX_W,

	XTRX_TX_H,
	XTRX_TX_W,

	XTRX_RX_L_LB, // loopback
	XTRX_RX_W_LB, // loopback

	XTRX_RX_AUTO,  // automatic selection
	XTRX_TX_AUTO,  // automatic selection

	XTRX_RX_ADC_EXT, // External ADC input
} xtrx_antenna_t;

XTRX_API int xtrx_set_antenna(struct xtrx_dev* dev, xtrx_antenna_t antenna);

XTRX_API int xtrx_set_antenna_ex(struct xtrx_dev* dev, xtrx_channel_t ch, xtrx_antenna_t antenna);

typedef enum xtrx_wire_format {
	XTRX_WF_8  = 1,
	XTRX_WF_12 = 2,
	XTRX_WF_16 = 3,
} xtrx_wire_format_t;

typedef enum xtrx_direction {
	XTRX_RX = 1,
	XTRX_TX = 2,
	XTRX_TRX = XTRX_RX | XTRX_TX
} xtrx_direction_t;

typedef enum xtrx_host_format {
	XTRX_IQ_FLOAT32 = 1,
	XTRX_IQ_INT16   = 2,
	XTRX_IQ_INT8    = 3,
} xtrx_host_format_t;

// Transmit functions
typedef enum xtrx_run_params_flags {
	XTRX_RUN_DIGLOOPBACK = 1,
	XTRX_RUN_RXLFSR      = 2,
	XTRX_RUN_GTIME       = 4,
} xtrx_run_params_flags_t;

typedef enum xtrx_run_sp_flags {
	XTRX_RSP_TEST_SIGNAL_A  = 2,
	XTRX_RSP_TEST_SIGNAL_B  = 4,
	XTRX_RSP_SWAP_AB        = 8,
	XTRX_RSP_SWAP_IQ        = 16,
	/** When set with XTRX_CH_AB stream only one channel, with
	 * ability to switch at runtine, otherwise ignore */
	XTRX_RSP_SISO_MODE      = 32,
	XTRX_RSP_SCALE          = 64,
	XTRX_RSP_NO_DC_CORR     = 128,
	XTRX_RSP_SWAP_IQB       = 256, /* swap IQ only in one channel B */

	XTRX_STREAMDSP_1        = 512,
	XTRX_STREAMDSP_2        = 1024,

	XTRX_RSP_SWAP_IQA       = 2048, /* swap IQ only in one channel A */
	XTRX_RSP_SISO_SWITCH    = 4096,
} xtrx_run_sp_flags_t;

typedef struct xtrx_run_stream_params {
	/** Encoding used in the transport (USB/PCIe/etc) */
	xtrx_wire_format_t wfmt;

	/** Host data encoding */
	xtrx_host_format_t hfmt;

	/** Affected channels */
	xtrx_channel_t     chs;

	/** Default packet size in samples (counted for the single channel) */
	uint32_t           paketsize;

	/** Flags of the stream, see xtrx_run_sp_flags_t*/
	uint32_t           flags;

	/** Optional scale value for XTRX_IQ_FLOAT32, it'll be [-scale, scale],
	 * by default it's [-1;1] */
	float              scale;

	/** Reserved for future extension to keep ABI structure the same size */
	uint32_t           reserved[12 - 6];
} xtrx_run_stream_params_t;


typedef struct xtrx_run_params {
	xtrx_direction_t         dir;
	unsigned                 nflags;

	xtrx_run_stream_params_t tx;
	xtrx_run_stream_params_t rx;

	master_ts                rx_stream_start;

	/** when set TX is constantly reapeating this buffer and doesn't expect
	 *  any xtrx_send_burst_sync() call
	 */
	void*                    tx_repeat_buf;

	gtime_data_t             gtime;

	/** Reserved for future extension to keep ABI structure the same size */
	uint32_t                 reserved[8];
} xtrx_run_params_t;


typedef enum xtrx_gtime_cmd {
    XTRX_GTIME_ENABLE_INT,      /**< Time is ignored, applied immediate */
    XTRX_GTIME_ENABLE_INT_WEXT, /**< Internal with ext generation */
    XTRX_GTIME_ENABLE_INT_WEXTE,
    XTRX_GTIME_ENABLE_EXT,
    XTRX_GTIME_DISABLE,
    XTRX_GTIME_GET_RESOLUTION,
    XTRX_GTIME_SET_GENSEC,
    XTRX_GTIME_GET_CUR,
    XTRX_GTIME_APPLY_CORRECTION,
    XTRX_GTIME_GET_GPSPPS_DELTA,
    XTRX_GTIME_SET_CURSEC,
    XTRX_GTIME_ENABLE_INT_WEXTENFW,
    XTRX_GTIME_ENABLE_EXTNFW,
    //XTRX_GTIME_ENABLE_AT_GPSPPS,
} xtrx_gtime_cmd_t;

XTRX_API int xtrx_gtime_op(struct xtrx_dev* dev, int devno,
                           xtrx_gtime_cmd_t cmd, gtime_data_t in,
                           gtime_data_t *out);

enum xtrx_gpios {
    XTRX_GPIO_ALL = -1,

    XTRX_GPIO1 = 0,
    XTRX_GPIO_PPS_I = XTRX_GPIO1,

    XTRX_GPIO2 = 1,
    XTRX_GPIO_PPS_O = XTRX_GPIO2,

    XTRX_GPIO3 = 2,
    XTRX_GPIO_TDD = XTRX_GPIO3,

    XTRX_GPIO4 = 3,

    XTRX_GPIO5 = 4,
    XTRX_GPIO_LED_WWAN = XTRX_GPIO5,

    XTRX_GPIO6 = 5,
    XTRX_GPIO_LED_WLAN = XTRX_GPIO6,

    XTRX_GPIO7 = 6,
    XTRX_GPIO_LED_WPAN = XTRX_GPIO7,

    XTRX_GPIO8 = 7,

    XTRX_GPIO9 = 8,
    XTRX_GPIO_EXT0 = XTRX_GPIO9,

    XTRX_GPIO10 = 9,
    XTRX_GPIO_EXT1 = XTRX_GPIO10,

    XTRX_GPIO11 = 10,
    XTRX_GPIO_EXT2 = XTRX_GPIO11,

    XTRX_GPIO12 = 11,
    XTRX_GPIO_EXT3 = XTRX_GPIO12,
    XTRX_GPIO_EPPS_O = XTRX_GPIO12,

    // Pseudo GPIOs
    XTRX_LED = 12,

    XTRX_SAFE = 13,

    XTRX_GPIOS_TOTAL = 14,
};

typedef enum xtrx_gpio_func {
    XTRX_GPIO_FUNC_IN,
    XTRX_GPIO_FUNC_OUT,

    // special function
    XTRX_GPIO_FUNC_PPS_O,
    XTRX_GPIO_FUNC_PPS_I,

    // gpio specific funcs
    XTRX_GPIO_FUNC_ALT0,
    XTRX_GPIO_FUNC_ALT1,
    XTRX_GPIO_FUNC_ALT2,
} xtrx_gpio_func_t;

XTRX_API int xtrx_gpio_configure(struct xtrx_dev* dev, int devno,
                                 int gpio_num, xtrx_gpio_func_t function);

XTRX_API int xtrx_gpio_out(struct xtrx_dev* dev, int devno, unsigned out);

XTRX_API int xtrx_gpio_clear_set(struct xtrx_dev* dev, int devno,
                                 unsigned clear_msk, unsigned set_msk);

XTRX_API int xtrx_gpio_in(struct xtrx_dev* dev, int devno, unsigned* in);

/**
 * @brief xtrx_run_params_init Initialize parameters with default values
 * @param params Parameters to initialize
 */
XTRX_API void xtrx_run_params_init(xtrx_run_params_t* params);

XTRX_API int xtrx_run_ex(struct xtrx_dev* dev, const xtrx_run_params_t* params);

XTRX_API int xtrx_stop(struct xtrx_dev* dev, xtrx_direction_t dir);


enum xtrx_send_ex_flags {
	XTRX_TX_DISCARDED_TO = 1,

	/** When set it's supposed that data are zeros and buffers can be NULL */
	XTRX_TX_SEND_ZEROS = 2,

	XTRX_TX_DONT_BUFFER = 4,

	XTRX_TX_TIMEOUT = 8,

	XTRX_TX_NO_DISCARD = 16,
};

typedef struct xtrx_send_ex_info {
	unsigned samples; /**< Number of sample in each user buffer */
	unsigned flags;
	master_ts ts;     /**< Timestamp of the first sample in the burst */

	const void* const* buffers;
	unsigned buffer_count;
	unsigned timeout;

	unsigned out_flags;
	unsigned out_samples; /**< Number of sample consumed in each user buffer */
	master_ts out_txlatets;
} xtrx_send_ex_info_t;

XTRX_API int xtrx_send_sync_ex(struct xtrx_dev* dev, xtrx_send_ex_info_t* info);


enum {
	MAX_RECV_BUFFERS = 2
};

enum xtrx_recv_ex_info_flags {
	/**< Do not recover in case of data overrun */
	RCVEX_STOP_ON_OVERRUN = 1,

	/**< Do not wait for buffer fullnes, get what is possible now */
	RCVEX_DONT_WAIT       = 2,

	/**< Do not fill skipped packet */
	RCVEX_DONT_INSER_ZEROS = 4,

	/**< Drop old packets when overflow occured */
	RCVEX_DROP_OLD_ON_OVERFLOW = 8,

	RCVEX_EXTRA_LOG = 16,

	RCVEX_TIMOUT = 32,

	RCVEX_REPORT_GTIME = 64,
};

enum xtrx_recv_ex_info_events {
	RCVEX_EVENT_OVERFLOW   = 1,
	RCVEX_EVENT_FILLED_ZERO = 2,
};

typedef struct xtrx_recv_ex_info {
	/* input number of samples to fill in each buffer */
	unsigned samples;

	unsigned buffer_count;
	void* const* buffers;

	unsigned flags;

	unsigned timeout;

	/* output: caught events */
	unsigned out_samples;   /** Number of filled samples in each buffer */
	unsigned out_events;
	master_ts out_first_sample;
	master_ts out_overrun_at;
	master_ts out_resumed_at;
} xtrx_recv_ex_info_t;

XTRX_API int xtrx_recv_sync_ex(struct xtrx_dev* dev, xtrx_recv_ex_info_t* info);

/* Low level calibration interface, statistics and hacking */
typedef enum xtrx_val {
	/* Underlying low-level object for internal testing */
	XTRX_UNDERLYING_LL = 0,

	/* RFIC specific calibration space */
	XTRX_RFIC_CORR_DC_EN = 0x1000,
	//XTRX_RFIC_CORR_DC_IQ,
	//XTRX_RFIC_CORR_GAIN_IQ,

	/* LMS7 specific values */
	XTRX_LMS7_XSP_SIGNAL = 0x1700,
	XTRX_LMS7_XSP_DC_IQ,
	XTRX_LMS7_RSSI,
	XTRX_LMS7_TEMP,
	XTRX_LMS7_DATA_RATE, /**< TSP/RSP data rate for DAC/ADC */
	XTRX_LMS7_PWR_MODE,
	XTRX_LMS7_VIO,

	/* Internal device, xtrx_direction_t should be set to 0 */
	XTRX_VCTCXO_DAC_VAL = 0x2000,
	XTRX_BOARD_TEMP,
	XTRX_IC_TEMP,
	XTRX_OSC_LATCH_1PPS,
	XTRX_WAIT_1PPS,
	XTRX_REF_REFCLK,
	XTRX_LML_PHY_PHASE,
	XTRX_LML_PHY_FBPHASE,
	XTRX_DSPFE_CMD,

	XTRX_TX_TIME,

	/* Performance counters */
	XTRX_PERF_SAMPLES = 0x3000,
	XTRX_PERF_UNOVFLOW,
	XTRX_PERF_LLFIFO,

	/* Direct access to RFIC regs, xtrx_direction_t ignored and chan
	 *  is used as index to RFIC onboard */
	XTRX_RFIC_REG_0 = 0x10000000,

	/* External front-end custom registers & control */
	XTRX_FE_CUSTOM_0 = 0x20000000,

	/* For internal use only */
	XTRX_DEBUG_0 = 0x30000000,
} xtrx_val_t;

XTRX_API int xtrx_val_set(struct xtrx_dev* dev, xtrx_direction_t dir,
						  xtrx_channel_t chan, xtrx_val_t type, uint64_t val);

XTRX_API int xtrx_val_get(struct xtrx_dev* dev, xtrx_direction_t dir,
						  xtrx_channel_t chan, xtrx_val_t type, uint64_t* val);


/* Misc & Debug functions */

#ifdef __cplusplus
}
#endif

#endif // XTRX_API_H
