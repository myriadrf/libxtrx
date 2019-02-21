/*
 * xtrx frontend proxy header file
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
#ifndef XTRX_FE_H
#define XTRX_FE_H

#include <xtrxll_api.h>
#include "xtrx_api.h"

// General abstraction layer of anlog to digital path and wise versa

struct xtrx_fe_ops;
struct xtrx_fe_obj
{
	const struct xtrx_fe_ops* ops;
};


struct xtrx_afe {
	// Desired rate on host side
	double rate;
	// Desired rate of AD/DA conversion
	double hwrate;

	// Extra host interpolation or decimation required
	unsigned host_di;
	unsigned refclk;
};

struct xtrx_fe_samplerate
{
	struct xtrx_afe adc;
	struct xtrx_afe dac;

	unsigned hwid;
	unsigned flags;
	unsigned refclk_source;
};

struct xtrx_dd_chpar {
	unsigned chs;
	unsigned flags;
};

struct xtrx_dd_params {
	unsigned dir;
	unsigned nflags;

	struct xtrx_dd_chpar rx;
	struct xtrx_dd_chpar tx;
};

enum xtrx_dd_ops {
	XTRX_FEDD_CONFIGURE,
	XTRX_FEDD_RESET,
};

// Tree like handles
struct xtrx_handle_info_t {
	unsigned handle_id;
	const char* cname;
	const char* const* aliases;
	unsigned flags;
	struct xtrx_handle_info_t* parent;
};


struct xtrx_fe_ops
{
	// DAC/ADC parts
	int (*dd_set_modes)(struct xtrx_fe_obj* obj,
						unsigned op,
						const struct xtrx_dd_params *params);

	int (*dd_set_samplerate)(struct xtrx_fe_obj* obj,
							 const struct xtrx_fe_samplerate* inrates,
							 struct xtrx_fe_samplerate* outrates);

	// BB part

	int (*bb_set_freq)(struct xtrx_fe_obj* obj,
					   unsigned channel,
					   unsigned dir,
					   double off_freq,
					   double* actualfreq);

	int (*bb_set_badwidth)(struct xtrx_fe_obj* obj,
						   unsigned channel,
						   unsigned dir,
						   double bw,
						   double* actualbw);

	int (*bb_set_gain)(struct xtrx_fe_obj* obj,
					   unsigned channel,
					   unsigned gain_type,
					   double gain,
					   double* actual);

	// RF part

	int (*fe_set_freq)(struct xtrx_fe_obj* obj,
					   unsigned channel,
					   unsigned dir,
					   double lo_freq,
					   double* actualfreq);

	int (*fe_set_lna)(struct xtrx_fe_obj* obj,
					   unsigned channel,
					   unsigned dir,
					   unsigned lna);

	int (*fe_set_gain)(struct xtrx_fe_obj* obj,
					   unsigned channel,
					   unsigned gain_type,
					   double gain,
					   double* actual);

	// TODO: add raw functions

	// TODO: add enumeration API

	// Ext API
	int (*get_reg)(struct xtrx_fe_obj* obj,
				   unsigned channel,
				   unsigned dir,
				   unsigned type,
				   uint64_t* outval);

	int (*set_reg)(struct xtrx_fe_obj* obj,
				   unsigned channel,
				   unsigned dir,
				   unsigned type,
				   uint64_t outval);

	int (*fe_deinit)(struct xtrx_fe_obj* obj);
};

#define GET_DEV_FROM_FLAGS(f)  ((f) & 0xff)
enum {
	XTRX_FE_MASTER           = 0x1000,
};

int xtrx_fe_init(struct xtrx_dev* dev,
				 struct xtrxll_dev* lldev,
				 unsigned flags,
				 const char* opts,
				 struct xtrx_fe_obj** obj);

#endif //XTRX_FE_H
