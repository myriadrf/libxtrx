/*
 * xtrx debug server header file
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
#ifndef XTRX_DEBUG_H
#define XTRX_DEBUG_H

#include <stdint.h>

struct xtrx_debug_ctx;
typedef struct xtrx_debug_ctx xtrx_debug_ctx_t;

enum dubug_cmd {
	DEBUG_RFIC_SPI_WR,
	DEBUG_RFIC_SPI_RD,
	DEBUG_BOARD_TEMP,
	DEBUG_GET_REFCLK,
	DEBUG_GET_ANT_RX,
	DEBUG_GET_ANT_TX,
	DEBUG_SET_ANT_RX,
	DEBUG_SET_ANT_TX,
	DEBUG_BOARD_DAC,
};

struct xtrx_debug_ops {
	int (*param_io)(void* obj, unsigned param, uint64_t val, uint64_t* oval);
};
typedef struct xtrx_debug_ops xtrx_debug_ops_t;

int xtrx_debug_init(const char *params,
					const xtrx_debug_ops_t* ops,
					void *obj,
					xtrx_debug_ctx_t** octx);
int xtrx_debug_free(xtrx_debug_ctx_t* ctx);

#endif
