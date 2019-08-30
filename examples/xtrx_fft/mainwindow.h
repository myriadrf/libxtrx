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
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>
#include <xtrx_api.h>
#include "../../octo/xtrx_octo_api.h"
#include <vector>


class MainWindow;
namespace Ui {
class MainWindow;
}

class FFTThread : public QThread
{
	Q_OBJECT
public:
	FFTThread(MainWindow *parent = 0);
	virtual ~FFTThread() {}

	static const int MAX_DEVS = 16;

	unsigned fft_avg;
	bool calc_max;

signals:
	void newRxData(int);

protected:
	MainWindow* _wnd;
};



class MainWindow : public QMainWindow
{
    Q_OBJECT

	static const int MAX_DEVS = 8;
	static const int MAX_WNDS = 5;
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();


	QVector<float> wnd[MAX_WNDS];

	QVector<double> x;

	QVector<double> y1[MAX_DEVS];
	QVector<double> y2[MAX_DEVS];
	QVector<double> y3[MAX_DEVS];
	QVector<double> y4[MAX_DEVS];

	QVector<double> z1[MAX_DEVS];
	QVector<double> z2[MAX_DEVS];
	QVector<double> z3[MAX_DEVS];
	QVector<double> z4[MAX_DEVS];

	double dc_pwr[MAX_DEVS];
	float wnd_p_corr[MAX_WNDS];

	xtrx_dev* dev;

	unsigned cwnd;

	bool draw_max;

	void wf_add_point(int x, unsigned data);
	void wf_feed_line();

	void update_devs();
	void update_wnds();
public slots:
	void redraw(int);
	void on_btStartStop_clicked();
	void on_rescale_clicked();
	void on_max_clicked();

	void on_cbcal_clicked();
	void on_cbb_clicked();

	void on_cbAntD_clicked();

	void on_rf_lb_clicked();

	void on_cal_rxdc_clicked();

	void on_calFreq_valueChanged(double);

	void on_freq_valueChanged(double);
	void on_txFreq_valueChanged(double);

	void on_bw_valueChanged(double);
	void on_txbw_valueChanged(double);

	void on_lb_attn_valueChanged(double);

	void on_gain_valueChanged(int);
	void on_bgain_valueChanged(int);

	void on_fft_avg_valueChanged(int);

	void on_lna_currentIndexChanged(int);
	void on_txband_currentIndexChanged(int);
	void on_fft_wnd_currentIndexChanged(int);

	void on_throttle_valueChanged(int);

public:
//private:
    Ui::MainWindow *ui;
	FFTThread* rx_thread;
	int        devices;
	unsigned   max_ffts;
	int        fft_size;
};


#endif // MAINWINDOW_H
