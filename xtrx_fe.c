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
#include "xtrxll_api.h"

#ifdef HAVE_LMS_NFE
int lms7nfe_init(struct xtrxll_dev* lldev,
				unsigned flags,
				struct xtrx_fe_obj** obj);
#else
int lms7fe_init(struct xtrxll_dev* lldev,
				unsigned flags,
				struct xtrx_fe_obj** obj);
#endif

int xtrx_fe_init(struct xtrxll_dev* lldev,
				 unsigned flags,
				 struct xtrx_fe_obj** obj)
{
#ifdef HAVE_LMS_NFE
	return lms7nfe_init(lldev, flags, obj);
#else
	return lms7fe_init(lldev, flags, obj);
#endif
}
