/*
 * xtrx debug server source file
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
#include "xtrxll_port.h"
#include "xtrxll_log.h"
#include "xtrx_debug.h"

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <signal.h>
#ifndef WIN32
#include <sys/un.h>
#include <sys/socket.h>
#endif

#include <sys/types.h>
#include <sys/stat.h>

struct xtrx_debug_ctx
{
	void* obj;
	const xtrx_debug_ops_t *ops;
	pthread_t debug_thread;

	//FILE* fifo;

	int fd;
	int clifd;
};

int xtrx_debug_process_cmd(xtrx_debug_ctx_t* ctx, const char *cmd, unsigned len,
						   char* reply, unsigned rlen)
{
	uint64_t oval = 0;
	int res = -EINVAL;
	if (strncmp(cmd, "LMS", 3) == 0) {
		unsigned reg = 0;
		char chans = 0;
		int cnt = sscanf(cmd + 4, "%c,%x", &chans, &reg);
		if (cnt == 2) {
			uint64_t ch = (chans == 'A') ? 1 :
						  (chans == 'B') ? 2 :
						  (chans == 'C') ? 3 : 0;
			XTRXLL_LOG(XTRXLL_INFO, "xtrx_debug: LMS write to 0x%08x (%c => %d)\n",
					   reg, chans, (int)ch);
			uint64_t v = (ch << 32) | reg;
			res = ctx->ops->param_io(ctx->obj,
									 (reg & 0x80000000) ? DEBUG_RFIC_SPI_WR : DEBUG_RFIC_SPI_RD,
									 v,
									 &oval);
		} else {
			XTRXLL_LOG(XTRXLL_ERROR, "xtrx_debug: LMS failed to parse! %d\n", cnt);
		}
	} else if(strncmp(cmd, "TMP", 3) == 0) {
		res = ctx->ops->param_io(ctx->obj, DEBUG_BOARD_TEMP, 0, &oval);
	} else if(strncmp(cmd, "DAC", 3) == 0) {
		unsigned nval = 0;
		sscanf(cmd + 4, "%d", &nval);

		res = ctx->ops->param_io(ctx->obj, DEBUG_BOARD_DAC, nval, &oval);
	} else if(strncmp(cmd, "FREF", 4) == 0) {
		res = ctx->ops->param_io(ctx->obj, DEBUG_GET_REFCLK, 0, &oval);
	} else if(strncmp(cmd, "RDANT", 5) == 0) {
		uint64_t arx, atx;
		res = ctx->ops->param_io(ctx->obj, DEBUG_GET_ANT_RX, 0, &arx);
		if (res)
			goto fail;
		res = ctx->ops->param_io(ctx->obj, DEBUG_GET_ANT_TX, 0, &atx);
		if (res)
			goto fail;

		oval = (atx << 2) | arx;
	} else if(strncmp(cmd, "WRANT", 5) == 0) {
		int nant = 0;
		sscanf(cmd + 6, "%d", &nant);
		unsigned arx = nant & 3, atx = (nant >> 2) & 1;

		res = ctx->ops->param_io(ctx->obj, DEBUG_SET_ANT_RX, arx, &oval);
		if (res)
			goto fail;
		res = ctx->ops->param_io(ctx->obj, DEBUG_SET_ANT_TX, atx, &oval);
		if (res)
			goto fail;
	} else {
		XTRXLL_LOG(XTRXLL_ERROR, "xtrx_debug: unrecognised command! %c\n", cmd[0]);
	}

fail:
	if (res == 0) {
		return snprintf(reply, rlen, "OK,%016" PRId64 "\n", oval);
	} else {
		return snprintf(reply, rlen, "FAIL,%d\n", res);
	}
}

#ifndef WIN32
static void* _xtrx_thread(void* param)
{
	int ret;
	struct sockaddr_un name;
	xtrx_debug_ctx_t* ctx = (xtrx_debug_ctx_t*)param;
	XTRXLL_LOG(XTRXLL_INFO, "Starting XTRX debug thread\n");
	pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
	const char* fifoname = "xtrx_debug_pipe";
	unlink(fifoname);

	int sock = socket(AF_UNIX, SOCK_STREAM, 0);
	if (sock == -1) {
		XTRXLL_LOG(XTRXLL_INFO, "Unable to create socket: error %d\n", errno);
		return NULL;
	}

	memset(&name, 0, sizeof(struct sockaddr_un));
	name.sun_family = AF_UNIX;
	strncpy(name.sun_path, fifoname, sizeof(name.sun_path) - 1);

	ret = bind(sock, (const struct sockaddr *) &name,
			   sizeof(struct sockaddr_un));
	if (ret == -1) {
		XTRXLL_LOG(XTRXLL_INFO, "Unable to bind socket: error %d\n", errno);
		close(sock);
		return NULL;
	}

	ret = listen(sock, 20);
	if (ret == -1) {
		XTRXLL_LOG(XTRXLL_INFO, "Unable to tisten to socket: error %d\n", errno);
		close(sock);
		return NULL;
	}

	ctx->fd = sock;
	for (;;) {
		ctx->clifd = -1;
		int data_socket = accept(sock, NULL, NULL);
		ctx->clifd = data_socket;
		if (data_socket == -1) {
			XTRXLL_LOG(XTRXLL_INFO, "Unable to accept socket: error %d\n", errno);
			close(sock);
			return NULL;
		}
		XTRXLL_LOG(XTRXLL_INFO, "Connection established\n");

		unsigned p = 0;
		char buffer[4096];
		char reply[4096];
		int replen;

		for (;;) {
			ssize_t res = read(data_socket, p + buffer, sizeof(buffer) - p);
			if (res <= 0) {
				XTRXLL_LOG(XTRXLL_INFO, "Connection closed\n");
				goto connection_closed;
			}

			char* end = strchr(buffer, '\n');
			if (end == NULL) {
				p += res;
				if (p == sizeof(buffer)) {
					XTRXLL_LOG(XTRXLL_INFO, "Incorrect CMD!\n");
					close(data_socket);
					goto connection_closed;
				}
				continue;
			}

			pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
			replen = xtrx_debug_process_cmd(ctx, buffer, end - buffer,
								   reply, sizeof(reply));
			pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

			if (replen > 0) {
				write(data_socket, reply, replen);
			}

			ssize_t ech = end - (p + buffer);
			if (ech > res) {
				XTRXLL_LOG(XTRXLL_INFO, "Moving extra %d/%d bytes\n",
						   (int)ech, (int)res);

				memmove(buffer, end + 1, ech - res);
				p = ech - res;
			}
		}

connection_closed:;
	}
	return NULL;
}
#endif


int xtrx_debug_init(const char *params,
					const xtrx_debug_ops_t *ops,
					void *obj,
					xtrx_debug_ctx_t** octx)
{
#ifndef WIN32
	int res;
	const char* fifoname = "xtrx_debug_pipe";
	int fd = mkfifo(fifoname, 0666);
	if (fd < 0 && errno != EEXIST) {
		int err = -errno;
		XTRXLL_LOG(XTRXLL_ERROR, "xtrx_debug: Unable to create FIFO, error %d\n",
				   err);
		return err;
	}

	xtrx_debug_ctx_t* ctx = (xtrx_debug_ctx_t*)malloc(sizeof(xtrx_debug_ctx_t));
	if (!ctx)
		return -ENOMEM;

	ctx->obj = obj;
	ctx->ops = ops;

	res = pthread_create(&ctx->debug_thread, NULL, _xtrx_thread, ctx);
	if (res)
		goto failed_create_thread;

	*octx = ctx;
	return 0;

failed_create_thread:
	free(ctx);
	return res;
#else
	return 0;
#endif
}


int xtrx_debug_free(xtrx_debug_ctx_t* ctx)
{
#ifndef WIN32
	close(ctx->fd);

	pthread_cancel(ctx->debug_thread);
	pthread_join(ctx->debug_thread, NULL);

	if (ctx->clifd != -1)
		close(ctx->clifd);
	free(ctx);
#endif
	return 0;
}
