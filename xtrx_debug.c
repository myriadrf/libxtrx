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

enum cmdvtype {
	CMD_VT_NONE,
	CMD_VT_DEC,
};

int xtrx_debug_process_cmd(xtrx_debug_ctx_t* ctx, char *cmd, unsigned len,
						   char* reply, unsigned rlen)
{
	uint64_t oval = 0;
	uint64_t param = 0;
	unsigned device = 0;
	int res = -EINVAL;
	int j;
	char *pptr[4] = {NULL,};
	char *str1 = NULL, *saveptr1 = NULL, *token = NULL;
	enum dubug_cmd dcmd;
	enum cmdvtype vt = CMD_VT_NONE;

	XTRXLLS_LOG("DBGP", XTRXLL_DEBUG, "got cmd: %s\n", cmd);

	for (j = 0, str1 = cmd; j < 4; j++, str1 = NULL) {
		token = strtok_r(str1, ",", &saveptr1);
		if (token == NULL)
			break;

		pptr[j] = token;
	}

	if (strcmp(cmd, "LMS") == 0) {
		if (j < 3) {
			goto incorrect_format;
		}
		param = strtol(pptr[2], NULL, 16);
		dcmd = (param & 0x80000000) ? DEBUG_RFIC_SPI_WR : DEBUG_RFIC_SPI_RD;
	} else if (strcmp(cmd, "TMP") == 0) {
		dcmd = DEBUG_BOARD_TEMP;
	} else if(strcmp(cmd, "DAC") == 0) {
		dcmd = DEBUG_BOARD_DAC;
		vt = CMD_VT_DEC;
	} else if(strcmp(cmd, "FREF") == 0) {
		dcmd = DEBUG_GET_REFCLK;
	} else if(strcmp(cmd, "ANTRX") == 0) {
		dcmd = DEBUG_ANT_RX;
		vt = CMD_VT_DEC;
		param = UINT_MAX;
	} else if(strcmp(cmd, "ANTTX") == 0) {
		dcmd = DEBUG_ANT_TX;
		vt = CMD_VT_DEC;
		param = UINT_MAX;
	} else if (strcmp(cmd, "DEVS") == 0) {
		dcmd = DEBUG_GET_DEVICES;
	} else if (strcmp(cmd, "RXIQA") == 0) {
		dcmd = DEBUG_GET_RXIQ_ODD;
	} else if (strcmp(cmd, "RXIQB") == 0) {
		dcmd = DEBUG_GET_RXIQ_MISS;
	} else if (strcmp(cmd, "VIO") == 0) {
		dcmd = DEBUG_VIO;
		vt = CMD_VT_DEC;
	} else if (strcmp(cmd, "VGP") == 0) {
		dcmd = DEBUG_V33;
		vt = CMD_VT_DEC;
	} else if (strcmp(cmd, "FGP") == 0) {
		dcmd = DEBUG_FGP_CTRL;
		vt = CMD_VT_DEC;
		param = UINT_MAX;
	} else if (strcmp(cmd, "GETREG") == 0) {
		dcmd = DEBUG_XTRX_GET_REG;
		vt = CMD_VT_DEC;
		param = UINT_MAX;
	} else {
		XTRXLLS_LOG("DBGP", XTRXLL_ERROR, "unrecognized command! %s\n", cmd);
		goto incorrect_format;
	}

	if (j > 1) {
		device = strtol(pptr[1], NULL, 16);
	}

	if (vt != CMD_VT_NONE && j > 2) {
		param = strtoll(pptr[2], NULL, 10);
	}

	res = ctx->ops->param_io(ctx->obj, dcmd, device, param, &oval);

incorrect_format:
	if (res == 0) {
		return snprintf(reply, rlen, "OK,%016" PRIx64 "\n", oval);
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
	XTRXLLS_LOG("DBGP", XTRXLL_INFO, "Starting XTRX debug thread\n");
	pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
	const char* fifoname = "xtrx_debug_pipe";
	unlink(fifoname);

	int sock = socket(AF_UNIX, SOCK_STREAM, 0);
	if (sock == -1) {
		XTRXLLS_LOG("DBGP", XTRXLL_INFO, "Unable to create socket: error %d\n", errno);
		return NULL;
	}

	memset(&name, 0, sizeof(struct sockaddr_un));
	name.sun_family = AF_UNIX;
	strncpy(name.sun_path, fifoname, sizeof(name.sun_path) - 1);

	ret = bind(sock, (const struct sockaddr *) &name,
			   sizeof(struct sockaddr_un));
	if (ret == -1) {
		XTRXLLS_LOG("DBGP", XTRXLL_INFO, "Unable to bind socket: error %d\n", errno);
		close(sock);
		return NULL;
	}

	ret = listen(sock, 20);
	if (ret == -1) {
		XTRXLLS_LOG("DBGP", XTRXLL_INFO, "Unable to tisten to socket: error %d\n", errno);
		close(sock);
		return NULL;
	}

	ctx->fd = sock;
	for (;;) {
		ctx->clifd = -1;
		int data_socket = accept(sock, NULL, NULL);
		ctx->clifd = data_socket;
		if (data_socket == -1) {
			XTRXLLS_LOG("DBGP", XTRXLL_INFO, "Unable to accept socket: error %d\n", errno);
			close(sock);
			return NULL;
		}
		XTRXLLS_LOG("DBGP", XTRXLL_INFO, "Connection established\n");

		unsigned p = 0;
		char buffer[4096];
		char reply[4096];
		int replen;

		for (;;) {
			ssize_t res = read(data_socket, p + buffer, sizeof(buffer) - p);
			if (res <= 0) {
				XTRXLLS_LOG("DBGP", XTRXLL_INFO, "Connection closed\n");
				goto connection_closed;
			}

			char* end = strchr(buffer, '\n');
			if (end == NULL) {
				p += res;
				if (p == sizeof(buffer)) {
					XTRXLLS_LOG("DBGP", XTRXLL_INFO, "Incorrect CMD!\n");
					close(data_socket);
					goto connection_closed;
				}
				continue;
			}

			// Terminate string
			*end = 0;

			pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
			replen = xtrx_debug_process_cmd(ctx, buffer, end - buffer,
								   reply, sizeof(reply));
			pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

			if (replen > 0) {
				write(data_socket, reply, replen);
			}

			ssize_t ech = end - (p + buffer);
			if (ech > res) {
				XTRXLLS_LOG("DBGP", XTRXLL_INFO, "Moving extra %d/%d bytes\n",
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
		XTRXLLS_LOG("DBGP", XTRXLL_ERROR, "Unable to create FIFO file `%s`, error %d\n",
				   fifoname, err);
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
