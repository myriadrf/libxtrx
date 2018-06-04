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
#include "waterfall.h"
#include <QImage>
#include <QTime>
#include <QPainter>
#include <QDebug>

Waterfall::Waterfall(QWidget* main)
	: QWidget(main)
	, image((uchar*)pixel_data, 512, 256, QImage::Format_RGB32)
	, imgIntense((uchar*)pixel2_data, 512, 256, QImage::Format_RGB32)
	, ymax(65535)
	, ymin(0)
{
	setBackgroundRole(QPalette::Base);
	setAutoFillBackground(true);

	timer.setInterval(25);
	connect(&timer, SIGNAL(timeout()), this, SLOT(updatePic()));

	y = 0;

	this->close();
}

void Waterfall::start()
{
	//timer.start();
}

void Waterfall::stop()
{
	//timer.stop();
}

void Waterfall::paintEvent(QPaintEvent*)
{
	QPainter painter(this);
	painter.drawImage(0, 0, image);

	//painter.drawImage(530, 0, imgIntense);
}

void Waterfall::updatePic()
{
	this->repaint();
}

#pragma GCC optimize ("O3")

void Waterfall::feed_line()
{
	memmove(pixel_data+512, pixel_data, 255*512*sizeof(QRgb));
/*
	for (unsigned i = 0; i < 256*512; i++) {
		pixel2_data[i] = (pixel2_data[i]*255) >> 8;
	}
	*/
}

void Waterfall::add_point(int x, unsigned v)
{
	int q = v - ymin;
	if (q < 0)
		q = 0;
	unsigned c = 255 * q / (ymax - ymin);
	if (c > 255)
		c = 255;

	pixel_data[x] = qRgb(c, c, 128);
/*
	pixel2_data[x + 512*(255-c)] += qRgb(0, 0, 4);
	if (pixel2_data[x + 512*(255-c)] > 255)
		pixel2_data[x + 512*(255-c)] |= 255;
	if (pixel2_data[x + 512*(255-c)] > 65535)
		pixel2_data[x + 512*(255-c)] |= 65535;
		*/
}


void Waterfall::update_scale(unsigned nymin, unsigned nymax)
{
	ymax = nymax;
	ymin = nymin;
}
