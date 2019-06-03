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
#include "rxthread.h"
#include <QtCore>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "assert.h"

RxThread::RxThread(MainWindow *parent)
: QThread(parent)
, fft_skip(128)
, fft_avg(8)
, soft_ampl_calc(false)
, calc_max(false)
, _wnd(parent)
{
}

RxThread::~RxThread()
{
}




#pragma GCC optimize ("O3")
static unsigned reverse512(unsigned n) {
   return (((n >> 8) & 1u) << 0) |
		  (((n >> 7) & 1u) << 1) |
		  (((n >> 6) & 1u) << 2) |
		  (((n >> 5) & 1u) << 3) |
		  (((n >> 4) & 1u) << 4) |
		  (((n >> 3) & 1u) << 5) |
		  (((n >> 2) & 1u) << 6) |
		  (((n >> 1) & 1u) << 7) |
		  (((n >> 0) & 1u) << 8);
}

void RxThread::run()
{
	const unsigned max_samples = 8192;

	union {
		uint64_t tmp_buffer[max_samples * MAX_DEVS];
		uint16_t tmp_buffer2[max_samples * 4 * MAX_DEVS];
	};
	xtrx_recv_ex_info_t rx_nfo;
	void *buffers[MAX_DEVS];
	for (int i = 0; i < MAX_DEVS; ++i) {
		buffers[i] = &tmp_buffer[max_samples * i];
	}

	rx_nfo.samples = (soft_ampl_calc) ? 2*max_samples : 4*max_samples;
	rx_nfo.buffer_count = _wnd->devices;
	rx_nfo.buffers = buffers;
	rx_nfo.flags = RCVEX_DONT_INSER_ZEROS;
	rx_nfo.timeout = 1000;

	for (int i=0; i<512; ++i) {
		int aidx = reverse512(i);
		_wnd->x[aidx] = aidx;
	}
	for (int i=0; i<512; ++i) {
		assert(_wnd->x[i] == i);
	}

	unsigned n = 0;
	unsigned vd = 0;
	const unsigned MAX_AVG = (soft_ampl_calc) ? max_samples / 512 : 4*max_samples / 512;
	bool clear = true;

	for (unsigned k = 0; !isInterruptionRequested(); k++) {
		unsigned skip = *(volatile unsigned*)&fft_skip;
		unsigned avg = *(volatile unsigned*)&fft_avg;
		if (avg > MAX_AVG)
			avg = MAX_AVG;
		if (avg < 1)
			avg = 1;

		int res = xtrx_recv_sync_ex(_wnd->dev, &rx_nfo);
		if (res) {
			return;
		}

		for (int d = 0; d < _wnd->devices; ++d) {
			QVector<double>* py[] = {&_wnd->y1[d], &_wnd->y2[d], &_wnd->y3[d], &_wnd->y4[d]};
			QVector<double>* pz[] = {&_wnd->z1[d], &_wnd->z2[d], &_wnd->z3[d], &_wnd->z4[d]};
			QVector<double>& y = *(py[vd]);
			QVector<double>& z = *(pz[vd]);

		/*
		if (calc_max && clear) {
			z = *pz[vd];
			for (int i=0; i<512; ++i) {
				z[i] = -65536;
			}
			clear = false;
		}
		*/

		if ((skip > 1) && (k % skip != skip-1)) {
			/*
			if (calc_max && !soft_ampl_calc) {
				double mulc = 10.0 / 1024 / log2(10);

				for (n = 0; n < MAX_AVG; n++) {
					for (unsigned i=0; i<512; ++i) {
						uint16_t v = tmp_buffer2[512 * n + reverse512(i)];
						int q = ((int)v) - 65535;

						int j = (i + 256) % 512;
						double vv = mulc * ((double)q);
						z[j] = std::max(z[j], vv*avg);
					}
				}
			}
			*/
			continue;
		}

		for (int i=0; i<512; ++i) {
			y[i] = 0;
		}
		if (calc_max) {
			for (int i=0; i<512; ++i) {
				z[i] = -65536;
			}
		}

		if (soft_ampl_calc) {
			for (n = 0; n < avg; n++) {
				for (unsigned i=0; i<512; ++i) {
					uint64_t v = tmp_buffer[max_samples * d + 512 * n + reverse512(i)];

					uint16_t ai = (v >> 0)  & 0xffff;
					uint16_t bi = (v >> 16) & 0xffff;
					uint16_t aq = (v >> 32) & 0xffff;
					uint16_t bq = (v >> 48) & 0xffff;

					assert((bi >> 8) == (ai & 0xff));
					assert((bq >> 8) == (aq & 0xff));

					uint32_t comb_re = ((((uint32_t)ai) << 18) | (((uint32_t)bi) << 10));
					uint32_t comb_im = ((((uint32_t)aq) << 18) | (((uint32_t)bq) << 10));

					double  c_re = ((int32_t)(comb_re)) / 32768.0 / 65536.0;
					double  c_im = ((int32_t)(comb_im)) / 32768.0 / 65536.0;

					int j = (i + 256) % 512;
					double vv = 10*log10(c_re * c_re  +  c_im * c_im) / avg;

					y[j] += vv;
					if (calc_max) {
						z[j] = std::max(z[j], vv*avg);
					}
				}
			}
		} else {
			double mulc = 10.0 / 1024 / log2(10) / avg;

			for (n = 0; n < avg; n++) {
				for (unsigned i=0; i<512; ++i) {
					uint16_t v = tmp_buffer2[max_samples * 4 * d + 512 * n + reverse512(i)];
					int q = ((int)v) - 65535;

					int j = (i + 256) % 512;
					double vv = mulc * ((double)q);

					_wnd->wf_add_point(j, v);

					y[j] += vv;
					if (calc_max) {
						z[j] = std::max(z[j], vv*avg);
					}
				}
				_wnd->wf_feed_line();
			}
			if (calc_max) {
				double mulc = 10.0 / 1024 / log2(10);
				for (; n <MAX_AVG;n++) {
					for (unsigned i=0; i<512; ++i) {
						uint16_t v = tmp_buffer2[max_samples * 4 * d + 512 * n + reverse512(i)];
						int q = ((int)v) - 65535;

						int j = (i + 256) % 512;
						double vv = mulc * ((double)q);

						_wnd->wf_add_point(j, v);

						z[j] = std::max(z[j], vv);
					}
					_wnd->wf_feed_line();
				}

			}
		}
		}

		emit newRxData(vd);

		vd++;
		if (vd > 3)
			vd = 0;

		clear = true;
	}
}
