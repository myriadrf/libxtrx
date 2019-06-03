/*
 * xtrx high level demo file
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
#include "mainwindow.h"


class RxThread : public QThread
{
	Q_OBJECT
public:
	RxThread(MainWindow *parent = 0);
	~RxThread();

	static const int MAX_DEVS = 16;

	void run();

	unsigned fft_skip;
	unsigned fft_avg;
	bool soft_ampl_calc;
	bool calc_max;

signals:
	void newRxData(int);

protected:
	MainWindow* _wnd;
};
