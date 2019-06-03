/*
 * xtrx frontend proxy source file
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
#include "xtrx_fe.h"
#include <errno.h>
#include <string.h>

int lms7nfe_init(struct xtrxll_dev* lldev,
				 unsigned flags,
				 const char* fename,
				 struct xtrx_fe_obj** obj);

int lms7octo_init(struct xtrxll_dev* lldev,
				  unsigned flags,
				  const char* fename,
				  struct xtrx_fe_obj** obj);

int lms7octocal_init(struct xtrxll_dev* lldev,
					 unsigned flags,
					 const char* fename,
					 struct xtrx_fe_obj** obj);

int auto_init(struct xtrxll_dev* lldev,
			  unsigned flags,
			  const char* fename,
			  struct xtrx_fe_obj** obj)
{
#if 0
	int res = lms7octo_init(lldev, flags, fename, obj);
	if (res == 0 || (res && res != -ENODEV))
		return res;
#endif
	return lms7nfe_init(lldev, flags, fename, obj);
}

typedef int (*fe_function_t)(struct xtrxll_dev* lldev,
							 unsigned flags,
							 const char* fename,
							 struct xtrx_fe_obj** obj);

struct fe_dictionary {
	const char* fename;
	fe_function_t init;
};

int xtrx_fe_init(struct xtrx_dev* dev,
				 struct xtrxll_dev *lldev,
				 unsigned flags,
				 const char* fename,
				 struct xtrx_fe_obj** obj)
{
	const struct fe_dictionary fes[] = {
		{ "octoCAL", lms7octocal_init },
		{ "octoRFX6", lms7octo_init },
		{ "lms7", lms7nfe_init },
		{ "auto", auto_init }
	};

	if (fename == NULL)
		return auto_init(lldev, flags, fename, obj);

	for (unsigned i = 0; i < sizeof(fes) / sizeof(fes[0]); i++) {
		if (strncmp(fename, fes[i].fename, strlen(fes[i].fename)) == 0)
			return fes[i].init(lldev, flags, fename, obj);
	}

	// No frontend were found
	return -EINVAL;
}
