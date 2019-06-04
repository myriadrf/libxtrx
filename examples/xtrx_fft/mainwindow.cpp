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

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
	x(512),
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
	customPlot->yAxis->setRange(-80, -19);
	customPlot->xAxis->setRange(0, 511);

	customPlot->setInteractions(QCP::iRangeZoom);
	//customPlot->yAxis->setScaleType(QCPAxis::stLogarithmic);
	//customPlot->yAxis2->setScaleType(QCPAxis::stLogarithmic);

	rx_thread = new RxThread(this);

	connect(rx_thread, SIGNAL(newRxData(int)), this, SLOT(redraw(int)));

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
	xtrx_antenna_t a[] = { XTRX_RX_AUTO, XTRX_RX_L, XTRX_RX_W, XTRX_RX_H, XTRX_RX_ADC_EXT };
	xtrx_set_antenna(dev, a[idx]);
}

void MainWindow::on_btStartStop_clicked()
{
	int res;
	double samplerate = ui->smaplerate->value() * 1e6;
	double gain = ui->gain->value();
	QString devstr = ui->cbDev->currentText();

	if (dev == NULL) {
		ui->widget->graph(0)->data()->clear();
		ui->widget->graph(1)->data()->clear();

		ui->statusbar->showMessage(QString("Samplerate %1 MSPS Gain: %2").arg(samplerate).arg(gain));
		//res = xtrx_open(devstr.toLatin1(), 4, &dev);
		res = xtrx_open_string(devstr.toLatin1(), &dev);
		if (res < 0)
			goto failed;
		devices = res;
		const int FFT_SIZE = 512;
		for (int i = 0; i < devices; ++i) {
			y1[i].resize(FFT_SIZE);
			y2[i].resize(FFT_SIZE);
			y3[i].resize(FFT_SIZE);
			y4[i].resize(FFT_SIZE);

			z1[i].resize(FFT_SIZE);
			z2[i].resize(FFT_SIZE);
			z3[i].resize(FFT_SIZE);
			z4[i].resize(FFT_SIZE);
		}

		//rx_thread->dev_cnt = res;
		res = xtrx_set_ref_clk(dev, 0, ui->cbe->isChecked() ? XTRX_CLKSRC_EXT : XTRX_CLKSRC_INT);
		if (res)
			goto failed_freq;

		res = xtrx_set_samplerate(dev, 0, samplerate, 0, 0, NULL, NULL, NULL);
		if (res)
			goto failed_freq;

		on_lna_currentIndexChanged(ui->lna->currentIndex());

		on_freq_valueChanged(ui->freq->value());
		on_bw_valueChanged(ui->bw->value());
		on_gain_valueChanged(ui->gain->value());

		on_fft_avg_valueChanged(ui->fft_avg->value());
		on_fft_skip_valueChanged(ui->fft_skip->value());
		on_max_clicked();

		rx_thread->soft_ampl_calc = ui->softlog->isChecked();
		xtrx_run_params_t params;

		if (rx_thread->soft_ampl_calc) {
			params.dir = XTRX_RX;
			params.rx_stream_start = 8192;
			params.nflags = 0;
			params.rx.chs = XTRX_CH_ALL;
			params.rx.flags = 0;
			params.rx.hfmt = XTRX_IQ_INT16;
			params.rx.wfmt = XTRX_WF_16;
			params.rx.paketsize = 8192;
			params.rx.flags = XTRX_STREAMDSP_1 | (ui->cbb->isChecked() ? XTRX_RSP_SWAP_AB : 0);
		} else {
			params.dir = XTRX_RX;
			params.rx_stream_start = 32768;
			params.nflags = 0;
			params.rx.chs = XTRX_CH_ALL;
			params.rx.flags = 0;
			params.rx.hfmt = XTRX_IQ_INT8;
			params.rx.wfmt = XTRX_WF_8;
			params.rx.paketsize = 0;
			params.rx.flags = XTRX_RSP_SISO_SWITCH | XTRX_RSP_SISO_MODE | XTRX_STREAMDSP_2 | (ui->cbb->isChecked() ? XTRX_RSP_SWAP_AB : 0);
		}
		res = xtrx_run_ex(dev, &params);
		if (res)
			goto failed_freq;

		ui->smaplerate->setEnabled(false);
		ui->softlog->setEnabled(false);
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


		ui->smaplerate->setEnabled(true);
		ui->softlog->setEnabled(true);
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
	rx_thread->calc_max = draw_max;
}

void MainWindow::on_cbb_clicked()
{
	uint64_t val = ui->cbb->isChecked() ? 1 : 0;
	if (dev == NULL)
		return;

	xtrx_val_set(dev, XTRX_TRX, XTRX_CH_ALL,
				 (xtrx_val_t)(XTRX_FE_CUSTOM_0 + 2), val);
}

void MainWindow::on_freq_valueChanged(double freq)
{
	if (dev == NULL)
		return;

	int res = xtrx_tune(dev, XTRX_TUNE_RX_FDD, freq * 1e6, NULL);
	if (res) {
		ui->statusbar->showMessage(QString("Failed to tune to %1 (%2) errno %3").arg(freq).arg(strerror(-res)).arg(res));
	}
}

void MainWindow::on_bw_valueChanged(double bw)
{
	if (dev == NULL)
		return;

	int res = xtrx_tune_rx_bandwidth(dev, XTRX_CH_ALL, bw * 1e6, NULL);
	if (res) {
		ui->statusbar->showMessage(QString("Failed to set bw to %1 (%2) errno %3").arg(bw).arg(strerror(-res)).arg(res));
	}
}

void MainWindow::on_gain_valueChanged(int gain)
{
	if (dev == NULL)
		return;

	int res = xtrx_set_gain(dev, XTRX_CH_ALL, XTRX_RX_LNA_GAIN, gain, 0);
	if (res) {
		ui->statusbar->showMessage(QString("Failed to set gain to %1 (%2) errno %3").arg(gain).arg(strerror(-res)).arg(res));
	}
}

void MainWindow::on_fft_avg_valueChanged(int avg)
{
	rx_thread->fft_avg = avg;
}

void MainWindow::on_fft_skip_valueChanged(int fft_skip)
{
	rx_thread->fft_skip = fft_skip;
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
}


