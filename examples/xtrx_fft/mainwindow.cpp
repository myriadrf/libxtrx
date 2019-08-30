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
#include "ui_mainwindow.h"

#include <QTimer>
#include <QDebug>
#include <unistd.h>

#include "rxthread.h"
#ifdef WITH_FFTW
#include "fftwthread.h"
#endif
#include <math.h>

FFTThread::FFTThread(MainWindow *parent)
	: QThread(parent)
	, fft_avg(8)
	, calc_max(false)
	, _wnd(parent)
{
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
	x(16384),
	cwnd(0),
	ui(new Ui::MainWindow)
{
    ui->setupUi(this);
	dev = NULL;

	QCustomPlot* customPlot = ui->widget;

	Qt::GlobalColor clrs[8] = {
		Qt::blue,	Qt::red,	Qt::green,	Qt::lightGray,
		Qt::cyan,	Qt::magenta,Qt::yellow,	Qt::darkGray,
	};

	Qt::GlobalColor mclrs[8] = {
		Qt::darkBlue,	Qt::darkRed,	Qt::darkGreen,	Qt::gray,
		Qt::darkCyan,	Qt::darkMagenta,Qt::darkYellow,	Qt::black,
	};

	// add two new graphs and set their look:
	for (int i = 0; i < MAX_DEVS; ++i) {
		customPlot->addGraph();
		customPlot->graph(i)->setPen(QPen(clrs[i])); // line color blue for first graph
	}

	for (int i = 0; i < MAX_DEVS; ++i) {
		customPlot->addGraph();
		customPlot->graph(MAX_DEVS + i)->setPen(QPen(mclrs[i])); // line color blue for first graph
	}

	connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
	connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));

	customPlot->xAxis2->setVisible(true);
	customPlot->xAxis2->setTickLabels(false);
	customPlot->yAxis2->setVisible(true);
	customPlot->yAxis2->setTickLabels(false);

	//customPlot->yAxis->setRange(0.00001, 1);
	customPlot->yAxis->setRange(-91, -19);
	customPlot->xAxis->setRange(0, 511);

	customPlot->setInteractions(QCP::iRangeZoom);
	//customPlot->yAxis->setScaleType(QCPAxis::stLogarithmic);
	//customPlot->yAxis2->setScaleType(QCPAxis::stLogarithmic);

	//rx_thread = new RxThread(this);
	//connect(rx_thread, SIGNAL(newRxData(int)), this, SLOT(redraw(int)));

#ifndef WITH_FFTW
	ui->softfft->setHidden(true);
	ui->fft_size->setHidden(true);
	ui->fft_wnd->setHidden(true);
#endif

	rx_thread = NULL;

	update_devs();

	xtrx_log_setlevel(3, "DEF");
}

void MainWindow::update_devs()
{
	ui->cbDev->clear();

	const unsigned MAX_DEVS = 32;
	xtrx_device_info_t devs[MAX_DEVS];
	int res = xtrx_discovery(devs, MAX_DEVS);
	if (res <= 0)
		return;

	for (int i = 0; i < res; i++) {
		ui->cbDev->addItem(QString::fromLatin1(devs[i].uniqname));
	}
}

void MainWindow::redraw(int idx)
{
	QCustomPlot* customPlot = ui->widget;

	if (idx > 3)
		idx = 3;
	else if (idx < 0)
		idx = 0;

	for (int i = 0; i < devices; i++) {
		QVector<double>* py[] = {&y1[i], &y2[i], &y3[i], &y4[i]};
		QVector<double>* pz[] = {&z1[i], &z2[i], &z3[i], &z4[i]};

		customPlot->graph(i)->setData(x, *py[idx]);

		if (draw_max) {
			customPlot->graph(MAX_DEVS + i)->setData(x, *pz[idx]);
		}
	}

#if defined(QCUSTOMPLOT_VERSION) && (QCUSTOMPLOT_VERSION >= 0x020000)
	customPlot->replot(QCustomPlot::rpImmediateRefresh);
#else
	customPlot->replot(QCustomPlot::rpImmediate);
#endif
}


void MainWindow::on_rescale_clicked()
{
	for (int i = 0; i < devices; i++) {
		ui->widget->graph(i)->rescaleAxes();
		if (draw_max) {
			ui->widget->graph(MAX_DEVS + i)->rescaleAxes(true);
		}
	}

	int ymax = 65535 + ui->widget->yAxis->range().upper * 1024 * log2(10) / 10;
	int ymin = 65535 + ui->widget->yAxis->range().lower * 1024 * log2(10) / 10;

	qDebug() << "MAX: " << ymax << " MIN:" << ymin;

	ui->waterfall->update_scale(ymin, ymax);
}

void MainWindow::on_lna_currentIndexChanged(int idx)
{
	xtrx_antenna_t a[] = { XTRX_RX_AUTO, XTRX_RX_L, XTRX_RX_W, XTRX_RX_H, XTRX_RX_ADC_EXT,
						 XTRX_RX_L_LB, XTRX_RX_W_LB, XTRX_RX_H_LB};
	xtrx_set_antenna(dev, a[idx]);
}

void MainWindow::on_txband_currentIndexChanged(int idx)
{
	xtrx_antenna_t a[] = { XTRX_TX_AUTO, XTRX_TX_H, XTRX_TX_W };
	xtrx_set_antenna(dev, a[idx]);
}

void MainWindow::update_wnds()
{
	for (int j = 0; j < MAX_WNDS; j++) {
		wnd[j].resize(fft_size);
	}

	// None
	// Hann
	// Hamming
	// Blackman
	// Nuttall

	float wc[5] = {0, 0, 0, 0, 0};

	for (int i = 0; i < fft_size; i++) {
		wnd[1][i] = (1 - cos(2 * M_PI * i / fft_size)) / 2;
		wnd[2][i] = 0.53836 - 0.46164 * cos(2 * M_PI * i / fft_size);
		wnd[3][i] = 0.42659 - 0.49656 * cos(2 * M_PI * i / fft_size) + 0.076849 * cos(4 * M_PI * i / fft_size);
		wnd[4][i] = 0.355768 - 0.487396 * cos(2 * M_PI * i / fft_size) + 0.144232 * cos(4 * M_PI * i / fft_size) - 0.012604 * cos(6 * M_PI * i / fft_size);

		wc[1] += wnd[1][i];
		wc[2] += wnd[2][i];
		wc[3] += wnd[3][i];
		wc[4] += wnd[4][i];
	}

	wnd_p_corr[0] = 0;
	for (int j = 1; j < 5; j++) {
		wnd_p_corr[j] = 10*logf(wc[j] / (fft_size));
		printf("WND[%d] = %3.f dB reduction\n", j, wnd_p_corr[j]);
	}
}

void MainWindow::on_btStartStop_clicked()
{
	int res;
	double samplerate = ui->smaplerate->value() * 1e6;
	double gain = ui->gain->value();
	QString devstr = ui->cbDev->currentText();

	const unsigned VIO = 3000;

	if (dev == NULL) {
		ui->widget->graph(0)->data()->clear();
		ui->widget->graph(1)->data()->clear();

		ui->statusbar->showMessage(QString("Samplerate %1 MSPS Gain: %2").arg(samplerate).arg(gain));
		//res = xtrx_open(devstr.toLatin1(), 4, &dev);
		res = xtrx_open_string(devstr.toLatin1(), &dev);
		if (res < 0)
			goto failed;

		bool soft_fft = ui->softfft->isChecked();
		devices = res;
		fft_size = 512;
		if (soft_fft) {
			switch (ui->fft_size->currentIndex()) {
			case 0: fft_size = 64; break;
			case 1: fft_size = 128; break;
			case 2: fft_size = 256; break;
			case 3: fft_size = 512; break;
			case 4: fft_size = 1024; break;
			case 5: fft_size = 2048; break;
			case 6: fft_size = 4096; break;
			case 7: fft_size = 8192; break;
			case 8: fft_size = 16384; break;
			}
		}


		x.resize(fft_size);
		for (int i = 0; i < devices; ++i) {
			y1[i].resize(fft_size);
			y2[i].resize(fft_size);
			y3[i].resize(fft_size);
			y4[i].resize(fft_size);

			z1[i].resize(fft_size);
			z2[i].resize(fft_size);
			z3[i].resize(fft_size);
			z4[i].resize(fft_size);
		}

		update_wnds();

		res = xtrx_val_set(dev, XTRX_TRX, XTRX_CH_ALL, XTRX_LMS7_VIO, VIO);
		if (res)
			goto failed_freq;

		//rx_thread->dev_cnt = res;
		res = xtrx_set_ref_clk(dev, 0, ui->cbe->isChecked() ? XTRX_CLKSRC_EXT : XTRX_CLKSRC_INT);
		if (res)
			goto failed_freq;

		res = xtrx_set_samplerate(dev, 0, samplerate, 0, 0, NULL, NULL, NULL);
		if (res)
			goto failed_freq;

		res = xtrx_val_set(dev, XTRX_TRX, XTRX_CH_ALL, XTRX_LMS7_VIO, VIO);
		if (res)
			goto failed_freq;


		delete rx_thread;
#ifdef WITH_FFTW
		if (soft_fft) {
			rx_thread = new FFTWThread(this);
		} else
#endif
		{
			rx_thread = new RxThread(this);
		}
		connect(rx_thread, SIGNAL(newRxData(int)), this, SLOT(redraw(int)));


		on_lna_currentIndexChanged(ui->lna->currentIndex());

		on_freq_valueChanged(ui->freq->value());
		on_bw_valueChanged(ui->bw->value());
		on_gain_valueChanged(ui->gain->value());

		on_fft_avg_valueChanged(ui->fft_avg->value());
		on_max_clicked();

		xtrx_run_params_t params;

		if (samplerate < 4e6) {
			max_ffts = 8;
		} else if (samplerate < 8e6) {
			max_ffts = 16;
		} else if (samplerate < 16e6) {
			max_ffts = 32;
		} else if (samplerate < 32e6) {
			max_ffts = 64;
		} else if (samplerate < 64e6) {
			max_ffts = 128;
		} else {
			max_ffts = 256;
		}

		if (soft_fft) {
			params.dir = XTRX_RX;
			params.rx_stream_start = 8192;
			params.nflags = 0;
			params.rx.chs = XTRX_CH_ALL;
			params.rx.flags = 0;
			params.rx.hfmt = XTRX_IQ_FLOAT32;
			params.rx.wfmt = XTRX_WF_16;
			params.rx.paketsize = fft_size*max_ffts;
			params.rx.flags = XTRX_RSP_NO_DC_CORR | XTRX_RSP_SISO_SWITCH |
					XTRX_RSP_SISO_MODE | (ui->cbb->isChecked() ? XTRX_RSP_SWAP_AB : 0);
		} else {
			params.dir = XTRX_RX;
			params.rx_stream_start = 32768;
			params.nflags = 0;
			params.rx.chs = XTRX_CH_ALL;
			params.rx.flags = 0;
			params.rx.hfmt = XTRX_IQ_INT8;
			params.rx.wfmt = XTRX_WF_8;
			params.rx.paketsize = fft_size*2*max_ffts;
			params.rx.flags = XTRX_RSP_NO_DC_CORR | XTRX_RSP_SISO_SWITCH |
					XTRX_RSP_SISO_MODE | XTRX_STREAMDSP_2 | (ui->cbb->isChecked() ? XTRX_RSP_SWAP_AB : 0);
		}
		res = xtrx_run_ex(dev, &params);
		if (res)
			goto failed_freq;

		ui->smaplerate->setEnabled(false);
		ui->softfft->setEnabled(false);
		ui->cbe->setEnabled(false);
		//ui->cbb->setEnabled(false);
		ui->btStartStop->setText(tr("Stop"));

		on_throttle_valueChanged(ui->throttle->value());

		rx_thread->start();

		ui->waterfall->start();
	} else {
		rx_thread->requestInterruption();
		if (!(rx_thread->wait(1000))) {
			rx_thread->terminate();
		}
		usleep(1000);
		xtrx_stop(dev, XTRX_TRX);
		xtrx_close(dev);
		dev = NULL;

		delete rx_thread;
		rx_thread = NULL;

		ui->smaplerate->setEnabled(true);
		ui->softfft->setEnabled(true);
		ui->cbe->setEnabled(true);
		//ui->cbb->setEnabled(true);
		ui->btStartStop->setText(tr("Start"));

		ui->waterfall->stop();
	}
	return;

failed_freq:
	xtrx_close(dev);
	dev = NULL;
failed:
	ui->statusbar->showMessage(QString("Failed to open %1 (%2) errno %3").arg(devstr).arg(strerror(-res)).arg(res));

	update_devs();
}

void MainWindow::on_max_clicked()
{
	draw_max = ui->max->isChecked();
	if (rx_thread) {
		rx_thread->calc_max = draw_max;
	}
}

void MainWindow::on_cbb_clicked()
{
	uint64_t val = ui->cbb->isChecked() ? 1 : 0;
	if (dev == NULL)
		return;

	xtrx_val_set(dev, XTRX_TRX, XTRX_CH_ALL,
				 (xtrx_val_t)(XTRX_FE_CUSTOM_0 + 2), val);
}

void MainWindow::on_cbAntD_clicked()
{
	uint64_t val = ui->cbAntD->isChecked() ? 1 : 0;
	if (dev == NULL)
		return;

	xtrx_val_set(dev, XTRX_TRX, XTRX_CH_ALL,
				 (xtrx_val_t)(XTRX_FE_CUSTOM_0 + 10), val);
}

void MainWindow::on_freq_valueChanged(double freq)
{
	if (dev == NULL)
		return;

	int res = xtrx_tune(dev, XTRX_TUNE_RX_FDD, freq * 1e6, NULL);
	if (res) {
		ui->statusbar->showMessage(QString("Failed to tune RX to %1 (%2) errno %3").arg(freq).arg(strerror(-res)).arg(res));
	}
}

void MainWindow::on_txFreq_valueChanged(double freq)
{
	if (dev == NULL)
		return;

	int res = xtrx_tune(dev, XTRX_TUNE_TX_FDD, freq * 1e6, NULL);
	if (res) {
		ui->statusbar->showMessage(QString("Failed to tune TX to %1 (%2) errno %3").arg(freq).arg(strerror(-res)).arg(res));
	}
}

void MainWindow::on_bw_valueChanged(double bw)
{
	if (dev == NULL)
		return;

	int res = xtrx_tune_rx_bandwidth(dev, XTRX_CH_ALL, bw * 1e6, NULL);
	if (res) {
		ui->statusbar->showMessage(QString("Failed to set RX bw to %1 (%2) errno %3").arg(bw).arg(strerror(-res)).arg(res));
	}
}

void MainWindow::on_txbw_valueChanged(double bw)
{
	if (dev == NULL)
		return;

	int res = xtrx_tune_tx_bandwidth(dev, XTRX_CH_ALL, bw * 1e6, NULL);
	if (res) {
		ui->statusbar->showMessage(QString("Failed to set TX bw to %1 (%2) errno %3").arg(bw).arg(strerror(-res)).arg(res));
	}
}

void MainWindow::on_gain_valueChanged(int gain)
{
	if (dev == NULL)
		return;

	int res = xtrx_set_gain(dev, XTRX_CH_ALL, XTRX_RX_LNA_GAIN, gain, 0);
	if (res) {
		ui->statusbar->showMessage(QString("Failed to set LNA gain to %1 (%2) errno %3").arg(gain).arg(strerror(-res)).arg(res));
	}
}

void MainWindow::on_bgain_valueChanged(int gain)
{
	if (dev == NULL)
		return;

	int res = xtrx_set_gain(dev, XTRX_CH_ALL, XTRX_RX_PGA_GAIN, gain-12.5, 0);
	if (res) {
		ui->statusbar->showMessage(QString("Failed to set PGA gain to %1 (%2) errno %3").arg(gain).arg(strerror(-res)).arg(res));
	}
}

void MainWindow::on_lb_attn_valueChanged(double gain)
{
	if (dev == NULL)
		return;

	int res = xtrx_set_gain(dev, XTRX_CH_ALL, XTRX_RX_LB_GAIN, -gain, 0);
	if (res) {
		ui->statusbar->showMessage(QString("Failed to set LB gain to %1 (%2) errno %3").arg(gain).arg(strerror(-res)).arg(res));
	}
}

void MainWindow::on_fft_avg_valueChanged(int avg)
{
	if (rx_thread) {
		rx_thread->fft_avg = avg;
	}
}

void MainWindow::on_throttle_valueChanged(int skip)
{
	if (dev == NULL)
		return;

	uint32_t reg = (1 << 27);
	if (skip > 1) {
		reg |= (1 << 16) | ((skip - 1) & 0xFF);
	}

	xtrx_val_set(dev, XTRX_TRX, XTRX_CH_ALL, XTRX_DSPFE_CMD, reg);
}

void MainWindow::on_cbcal_clicked()
{
	if (dev == NULL)
		return;

	int res = xtrx_octo_set_cal_path(dev, ui->cbcal->isChecked());
	if (res) {
		ui->statusbar->showMessage(QString("Failed switch CAL to %1 errno %2").arg(strerror(-res)).arg(res));
	}
}

void MainWindow::on_rf_lb_clicked()
{
	if (dev == NULL)
		return;

	xtrx_val_set(dev, XTRX_TRX, XTRX_CH_ALL,
				 (xtrx_val_t )(XTRX_FE_CUSTOM_0 + 11), ui->rf_lb->isChecked());


	xtrx_val_set(dev, XTRX_TX, XTRX_CH_ALL,
				 (xtrx_val_t )(XTRX_FE_CUSTOM_0 + 5),
				 (32768 << 16) | 32768);
}

void MainWindow::on_calFreq_valueChanged(double freq)
{
	if (dev == NULL)
		return;

	int res = xtrx_tune(dev, XTRX_TUNE_EXT_FE, freq * 1e6, NULL);
	if (res) {
		ui->statusbar->showMessage(QString("Failed to tune to %1 (%2) errno %3").arg(freq).arg(strerror(-res)).arg(res));
	}
}

void MainWindow::wf_add_point(int x, unsigned data)
{
	ui->waterfall->add_point(x, data);
}

void MainWindow::wf_feed_line()
{
	ui->waterfall->feed_line();
}

void MainWindow::on_fft_wnd_currentIndexChanged(int idx)
{
	cwnd = idx;
}

enum cal_dc_state {
	ST_NONE,
	ST_PROBE_FIRST,
	ST_PROBE_I_P,
	ST_PROBE_I_N,
	ST_PROBE_Q_P,
	ST_PROBE_Q_N,
};

struct cal_dc_state_data {
	int state, v_start;
	int best, best_loc;
	int dci_b, dcq_b;
	int dci, dcq;
	//int min_dci, min_dcq, max_dci, max_dcq;
};

int do_dc_cal(struct cal_dc_state_data* cd, int meas_dc)
{
	switch (cd->state) {
	case ST_NONE:
		cd->state = ST_PROBE_FIRST;
		cd->dci = cd->dcq = cd->v_start = 0;
		break;
	case ST_PROBE_FIRST:
		cd->state = ST_PROBE_I_P;
		cd->best_loc = cd->best = meas_dc;
		cd->dci_b = cd->dci;
		cd->dcq_b = cd->dcq;

		++cd->dci;
		break;
	case ST_PROBE_I_P:
		if (meas_dc < cd->best_loc) {
			cd->best_loc = meas_dc;
			cd->dci_b = cd->dci;

			++cd->dci;
		} else {
			cd->dci = cd->v_start - 1;
			cd->state = ST_PROBE_I_N;
		}
	case ST_PROBE_I_N:
		if (meas_dc < cd->best_loc) {
			cd->best_loc = meas_dc;
			cd->dci_b = cd->dci;

			--cd->dci;
		} else {
			cd->dci = cd->dci_b;
			cd->v_start = ++cd->dcq;
			cd->state = ST_PROBE_Q_P;
		}
	case ST_PROBE_Q_P:
		if (meas_dc < cd->best_loc) {
			cd->best_loc = meas_dc;
			cd->dcq_b = cd->dcq;

			++cd->dcq;
		} else {
			cd->dcq = cd->v_start - 1;
			cd->state = ST_PROBE_I_N;
		}
	case ST_PROBE_Q_N:
		if (meas_dc < cd->best_loc) {
			cd->best_loc = meas_dc;
			cd->dcq_b = cd->dcq;

			--cd->dcq;
		} else {
			cd->dcq = cd->dcq_b;
			if (cd->best_loc >= cd->best) {
				cd->state = ST_NONE;
				return 1;
			}

			cd->best = cd->best_loc;
			cd->v_start = ++cd->dci;
			cd->state = ST_PROBE_Q_P;
		}

	}

	return 0;
}


void MainWindow::on_cal_rxdc_clicked()
{
	int res;
/*
	for (int i = 0; i < 512; i++) {
		uint32_t reg = (1 << 28);
		reg |= (i << 16) | (1 << 15);
		xtrx_val_set(dev, XTRX_TRX, XTRX_CH_ALL, XTRX_DSPFE_CMD, reg);
	}
	*/

	uint32_t reg = (1 << 28) | (1 << 27);
	xtrx_val_set(dev, XTRX_TRX, XTRX_CH_ALL, XTRX_DSPFE_CMD, reg);
}


MainWindow::~MainWindow()
{
	if (rx_thread) {
		rx_thread->requestInterruption();
		if (!(rx_thread->wait(1000))) {
			rx_thread->terminate();
		}
	}
	if (dev) {
		xtrx_stop(dev, XTRX_TRX);
		xtrx_close(dev);
		dev = NULL;
	}
	//delete ui;

	delete rx_thread;
}


