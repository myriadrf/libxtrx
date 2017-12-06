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

#ifdef __cplusplus
extern "C" {
#endif


#define XTRX_API

/* High level API function */

typedef void (*xtrx_logfunc_t)(int sevirity, const char* message);
XTRX_API void xtrx_set_logfunction(xtrx_logfunc_t func);


struct xtrx_dev;

enum xtrx_flags {
	XTRX_O_LOGLVL_MASK      = 0x000f,
	XTRX_O_LOGLVL_LMS7_OFF  = 4,
	XTRX_O_LOGLVL_LMS7_MASK = 0x00f0,
	XTRX_O_RESET            = 0x0100,
};

/**
 * @brief master_ts
 */
typedef uint64_t master_ts;

typedef enum xtrx_open_flags {
	XTRX_CLAIM_DEV = 1, /**< claim exclusive access to the device (reqired for DMA access) */
	XTRX_CLAIM_UART_SIM = 2, /**< claim exclusive access to SIM card interface */
	XTRX_CLAIM_UART_GPS = 4, /**< claim exclusive access to GPS UART interface */
} xtrx_open_flags_t;

typedef struct xtrx_open_params {
	unsigned loglevel;
	xtrx_open_flags_t flags;
	const char* const* multidev;
} xtrx_open_params_t;

/** Open XTRX device
 * @param device    Path to XTRX /dev entry
 * @param flags     Not implemented
 * @param[out] dev  XTRX device handle
 * @return 0 on success, errno on error
 */
XTRX_API int xtrx_open(const char* device, unsigned flags, struct xtrx_dev** dev);

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

XTRX_API void xtrx_set_ref_clk(struct xtrx_dev* dev, unsigned refclkhz, xtrx_clock_source_t clksrc);

/** NOT IMPLEMENTED
 */
XTRX_API int xtrx_discovery(char* devices, size_t maxbuf);


typedef enum xtrx_samplerate_flags {
	// Flags 1 throgh 8 are reserveed for debug

	XTRX_SAMPLERATE_FORCE_TX_INTR = 16,
	XTRX_SAMPLERATE_FORCE_RX_DECIM = 32,
} xtrx_samplerate_flags_t;

/** Set samplerate for the XTRX device
 *
 * This function configures CGEN block, determines best inerpolation/decimation and
 * configures LML inerface for the given clock configuration.
 *
 * @param dev       XTRX device handle
 * @param cgen_rate CGEN clock rate, 0 for autoselect
 * @param rxrate    RX sample rate after all decimation stages (as seen on the PCIe interaface), 0 to disable RX
 * @param txrate    TX sample rate defore any interpolation (as seen on the PCIe interaface), 0 to disable TX
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

typedef enum xtrx_tune {
	XTRX_TUNE_RX_FDD,
	XTRX_TUNE_TX_FDD,
	XTRX_TUNE_TX_AND_RX_TDD,

	/** Tune baseband (DSP) frequency for RX */
	XTRX_TUNE_BB_RX,

	/** Tune baseband (DSP) frequency for TX */
	XTRX_TUNE_BB_TX,
} xtrx_tune_t;

XTRX_API int xtrx_tune(struct xtrx_dev* dev, xtrx_tune_t type, double freq, double *actualfreq);


typedef enum xtrx_channel {
	XTRX_CH_A  = 1,
	XTRX_CH_B  = 2,
	XTRX_CH_AB = XTRX_CH_A | XTRX_CH_B,
} xtrx_channel_t;

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

	XTRX_TX_L,
	XTRX_TX_W,

	XTRX_RX_L_LB, // loopback
	XTRX_RX_W_LB, // loopback
} xtrx_antenna_t;

XTRX_API int xtrx_set_antenna(struct xtrx_dev* dev, xtrx_antenna_t antenna);

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
} xtrx_run_sp_flags_t;

typedef struct xtrx_run_stream_params {
	/** Encoding used in the transport (USB/PCIe/etc) */
	xtrx_wire_format_t wfmt;

	/** Host data encoding */
	xtrx_host_format_t hfmt;

	/** Affected channels */
	xtrx_channel_t     chs;

	/** Default packet size in samples (counted for the single channel) */
	uint16_t           paketsize;

	/** Flags of the stream, see xtrx_run_sp_flags_t*/
	uint16_t           flags;

	/** Optional scale value for XTRX_IQ_FLOAT32, it'll be [-scale, scale],
	 * by default it's [-1;1] */
	float              scale;

	/** Reserved for future extension to keep ABI structure the same size */
	uint32_t           reserved[12 - 5];
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
} xtrx_run_params_t;

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
};

typedef struct xtrx_send_ex_info {
	unsigned samples; /**< Number of sample in each user buffer */
	unsigned flags;
	master_ts ts;     /**< Timestamp of the first sample in the burst */

	const void* const* buffers;
	unsigned buffer_count;

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
	/* RFIC specific calibration space */
	XTRX_RFIC_CORR_DC_EN = 0x1000,
	//XTRX_RFIC_CORR_DC_IQ,
	//XTRX_RFIC_CORR_GAIN_IQ,

	/* LMS7 specific values */
	XTRX_LMS7_XSP_SIGNAL = 0x1700,
	XTRX_LMS7_XSP_DC_IQ,
	XTRX_LMS7_RSSI,

	/* Internal device, xtrx_direction_t should be set to 0 */
	XTRX_VCTCXO_DAC_VAL = 0x2000,
	XTRX_BOARD_TEMP,
	XTRX_IC_TEMP,
	XTRX_OSC_LATCH_1PPS,

	/* Performance counters */
	XTRX_PERF_SAMPLES = 0x3000,
	XTRX_PERF_UNOVFLOW,

	/* Direct access to RFIC regs, xtrx_direction_t ignored and chan
	 *  is used as index to RFIC onboard */
	XTRX_RFIC_REG_0 = 0x10000000,
} xtrx_val_t;

XTRX_API int xtrx_val_set(struct xtrx_dev* dev, xtrx_direction_t dir,
						  xtrx_channel_t chan, xtrx_val_t type, uint64_t val);

XTRX_API int xtrx_val_get(struct xtrx_dev* dev, xtrx_direction_t dir,
						  xtrx_channel_t chan, xtrx_val_t type, uint64_t* val);


/* Misc & Debug functions */

/** Dump internal LMS7 state to file for debugging purposes
 * @param dev
 * @param path
 * @return
 */
XTRX_API int xtrx_debug_dump_lms(struct xtrx_dev* dev, const char* path);

#ifdef __cplusplus
}
#endif

#endif // XTRX_API_H
