#ifndef XTRX_FE_NLMS7_H
#define XTRX_FE_NLMS7_H

#include <stdint.h>
#include <stdbool.h>
#include <liblms7002m.h>
#include "xtrx_fe.h"

typedef struct xtrx_bparam
{
	bool set;
	unsigned value;
} xtrx_bparam_t;


struct xtrx_nfe_lms7
{
	struct xtrx_fe_obj base;

	struct xtrxll_dev* lldev;
	struct lms7_state lms_state;

	double cgen_clk;

	unsigned lmsnum;
	unsigned refclock;
	unsigned refclk_source;

	bool rx_no_siso_map;
	bool tx_no_siso_map;

	bool tx_run_a;
	bool tx_run_b;

	bool rx_run_a;
	bool rx_run_b;

	bool rx_port_1;

	uint8_t             rx_mmcm_div;
	uint8_t             tx_mmcm_div;
	uint8_t             rx_port_cfg;
	uint8_t             tx_port_cfg;

	bool                rx_lna_auto;
	bool                tx_lna_auto;

	unsigned rx_host_decim;
	unsigned tx_host_inter;

	unsigned            rxcgen_div;
	unsigned            txcgen_div;
	unsigned            rxtsp_div;     /* Div ratio at LML */
	unsigned            rxtsp_decim;   /* Decimation in TSP */
	unsigned            txtsp_div;
	unsigned            txtsp_interp;  /* Interpolation in TSP */

	unsigned            txant;
	unsigned            rxant;

	double              rx_lo;
	double              tx_lo;

	struct lml_map maprx;
	struct lml_map maptx;

	enum lml_mode lml_mode;
	unsigned lml_txdiv;
	unsigned lml_rxdiv;

	xtrx_bparam_t tx_bw[2];
	xtrx_bparam_t rx_bw[2];

	xtrx_bparam_t tx_dsp[2];
	xtrx_bparam_t rx_dsp[2];
};


int lms7nfe_init(struct xtrxll_dev* lldev,
				 unsigned flags,
				 const char* fename,
				 struct xtrx_fe_obj** obj);

int lms7nfe_dd_set_samplerate(struct xtrx_fe_obj* obj,
							 const struct xtrx_fe_samplerate* inrates,
							 struct xtrx_fe_samplerate* outrates);

int lms7nfe_dd_set_modes(struct xtrx_fe_obj* obj,
						unsigned op,
						const struct xtrx_dd_params *params);

int lms7nfe_bb_set_freq(struct xtrx_fe_obj* obj,
					   unsigned channel,
					   unsigned type,
					   double freq,
					   double* actualfreq);

int lms7nfe_bb_set_badwidth(struct xtrx_fe_obj* obj,
						   unsigned channel,
						   unsigned dir,
						   double bw,
						   double* actualbw);

int lms7nfe_set_gain(struct xtrx_fe_obj* obj,
					unsigned channel,
					unsigned gain_type,
					double gain,
					double *actualgain);

int lms7nfe_fe_set_freq(struct xtrx_fe_obj* obj,
					   unsigned channel,
					   unsigned type,
					   double freq,
					   double *actualfreq);

int lms7nfe_fe_set_lna(struct xtrx_fe_obj* obj,
					  unsigned channel,
					  unsigned dir,
					  unsigned lna);

int lms7nfe_get_reg(struct xtrx_fe_obj* obj,
					unsigned channel,
					unsigned dir,
					unsigned type,
					uint64_t* outval);

int lms7nfe_set_reg(struct xtrx_fe_obj* obj,
			   unsigned channel,
			   unsigned dir,
			   unsigned type,
			   uint64_t val);

int lms7nfe_deinit(struct xtrx_fe_obj* obj);


const struct lml_map lms7nfe_get_lml_portcfg(const struct xtrx_dd_chpar* par,
											 bool no_siso_map);
#endif
