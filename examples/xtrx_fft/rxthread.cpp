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

#include <xtrxdsp_fft.h>
#include "assert.h"

RxThread::RxThread(MainWindow *parent)
: FFTThread(parent)
{
}

void RxThread::run()
{
	const unsigned FFTS_SIZE = 512;
	const unsigned MAX_FFTS = _wnd->max_ffts;

	const unsigned max_samples = FFTS_SIZE * MAX_FFTS;

	uint16_t fft_data_buffer[max_samples * MAX_DEVS];
	xtrx_recv_ex_info_t rx_nfo;

	void *buffers[MAX_DEVS];
	for (int i = 0; i < MAX_DEVS; ++i) {
		buffers[i] = &fft_data_buffer[max_samples * i];
	}

	rx_nfo.samples = 2 * max_samples;
	rx_nfo.buffer_count = _wnd->devices;
	rx_nfo.buffers = buffers;
	rx_nfo.flags = RCVEX_DONT_INSER_ZEROS;
	rx_nfo.timeout = 5000;

	for (unsigned i=0; i<FFTS_SIZE; ++i) {
		_wnd->x[i] = i;
	}

	unsigned vd = 0;
	unsigned bidx = MAX_FFTS;

	for (uint64_t k = 0; !isInterruptionRequested(); k++) {
		unsigned avg = *(volatile unsigned*)&fft_avg;

		if (avg < 1)
			avg = 1;

		double scale = 1. / avg;
		double ly[FFTS_SIZE];

		for (unsigned aidx = 0; aidx < avg; aidx++, bidx++) {
			if (bidx >= MAX_FFTS) {
				int res = xtrx_recv_sync_ex(_wnd->dev, &rx_nfo);
				if (res) {
					return;
				}
				bidx = 0;
			}

			for (int d = 0; d < _wnd->devices; ++d) {
				QVector<double>* py[] = {&_wnd->y1[d], &_wnd->y2[d], &_wnd->y3[d], &_wnd->y4[d]};
				QVector<double>& y = *(py[vd]);

				if (aidx == 0) {
					for (unsigned i = 0; i < FFTS_SIZE; ++i) {
						y[i] = 0;
					}
				}

				xtrxdsp_fft_realign_pwr_d(fft_data_buffer + max_samples * d + FFTS_SIZE * bidx,
										  FFTS_SIZE,
										  scale,
										  ly);

				for (unsigned i = 0; i < FFTS_SIZE; ++i) {
					y[i] += ly[i];
				}
			}
		}

		emit newRxData(vd);

		vd++;
		if (vd > 3)
			vd = 0;
	}

#if 0
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
	rx_nfo.timeout = 5000;

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
				for (unsigned i = 0; i < 512; i += 4) {
					uint16_t v[4];
					unsigned idx = max_samples * 4 * d + 512 * n + reverse(i >> 1);//reverse512(i);
					v[0] = tmp_buffer2[idx];
					v[1] = tmp_buffer2[idx + 256];
					v[2] = tmp_buffer2[idx + 128];
					v[3] = tmp_buffer2[idx + 384];

					int q[4];
					q[0] = ((int)v[0]) - 65535 + 8192;
					q[1] = ((int)v[1]) - 65535 + 8192;
					q[2] = ((int)v[2]) - 65535 + 8192;
					q[3] = ((int)v[3]) - 65535 + 8192;

					double vv[4];
					vv[0] = mulc * ((double)q[0]);
					vv[1] = mulc * ((double)q[1]);
					vv[2] = mulc * ((double)q[2]);
					vv[3] = mulc * ((double)q[3]);

					int j = i ^ 256;

					y[j+0] += vv[0];
					y[j+1] += vv[1];
					y[j+2] += vv[2];
					y[j+3] += vv[3];

					//_wnd->wf_add_point(j+0, v[0]);
					//_wnd->wf_add_point(j+1, v[1]);
					//_wnd->wf_add_point(j+2, v[2]);
					//_wnd->wf_add_point(j+3, v[3]);

					if (calc_max) {
						z[j+0] = std::max(z[j+0], vv[0]*avg);
						z[j+1] = std::max(z[j+1], vv[1]*avg);
						z[j+2] = std::max(z[j+2], vv[2]*avg);
						z[j+3] = std::max(z[j+3], vv[3]*avg);
					}
				}

				//_wnd->wf_feed_line();
			}
#if 0
			if (calc_max) {
				double mulc = 10.0 / 1024 / log2(10);
				for (; n <MAX_AVG;n++) {
					for (unsigned i=0; i<512; ++i) {
						uint16_t v = tmp_buffer2[max_samples * 4 * d + 512 * n + reverse512(i)];
						int q = ((int)v) - 65535 + 8192;

						int j = (i + 256) % 512;
						double vv = mulc * ((double)q);

						_wnd->wf_add_point(j, v);

						z[j] = std::max(z[j], vv);
					}
					_wnd->wf_feed_line();
				}
			}
#endif
		}
		}

		emit newRxData(vd);

		vd++;
		if (vd > 3)
			vd = 0;

		clear = true;
	}

#endif

}
