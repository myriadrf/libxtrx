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
#include <QWidget>
#include <QTimer>

class Waterfall : public QWidget
{
	Q_OBJECT
public:
	Waterfall(QWidget* main);

	void feed_line();
	void add_point(int x, unsigned v);

	void update_scale(unsigned nymin, unsigned nymax);

	void start();
	void stop();

public slots:
	void updatePic();

protected:
	void paintEvent(QPaintEvent*);

	QRgb pixel_data[512*256];
	QRgb pixel2_data[512*256];

	QImage image;
	QImage imgIntense;

	QTimer timer;

	unsigned ymax;
	unsigned ymin;

	unsigned y;
};

