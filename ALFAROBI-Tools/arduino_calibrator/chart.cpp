/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the Qt Charts module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:GPL$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3 or (at your option) any later version
** approved by the KDE Free Qt Foundation. The licenses are as published by
** the Free Software Foundation and appearing in the file LICENSE.GPL3
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "chart.h"
#include <QtCharts/QAbstractAxis>
#include <QtCharts/QSplineSeries>
#include <QtCharts/QValueAxis>
#include <QtCore/QTime>
#include <QtCore/QDebug>

Chart::Chart(QGraphicsItem *parent, Qt::WindowFlags wFlags):
    QChart(QChart::ChartTypeCartesian, parent, wFlags),
    m_axis(new QValueAxis),
    m_step(0)
{
    qsrand((uint) QTime::currentTime().msec());

    m_roll = new QSplineSeries(this);
    m_pitch = new QSplineSeries(this);

    QPen green(Qt::green);
    green.setWidth(3);
    m_roll->setPen(green);
    m_roll->setName("Roll");

    QPen red(Qt::red);
    red.setWidth(3);
    m_pitch->setPen(red);
    m_pitch->setName("Pitch");

    addSeries(m_roll);
    addSeries(m_pitch);
    createDefaultAxes();
    setAxisX(m_axis, m_roll);
    setAxisX(m_axis, m_pitch);
    m_axis->setTickCount(5);
    axisX()->setRange(-10, 10);
    axisX()->hide();
    axisY()->setRange(-90, 90);
}

Chart::~Chart()
{

}

void Chart::updateRPY(double roll, double pitch, double yaw)
{
    qreal x = plotArea().width() / m_axis->tickCount();
    qreal y = (m_axis->max() - m_axis->min()) / m_axis->tickCount();
    m_x += y;
    if(roll < 180 )
        m_y_roll = roll;
    else
        m_y_roll = roll-360;
    if(pitch < 180)
        m_y_pitch = pitch;
    else
        m_y_pitch = pitch-360;
    m_roll->append(m_x, m_y_roll);
    m_pitch->append(m_x, m_y_pitch);
    scroll(x, 0);

}

void Chart::updateGyro(double roll, double pitch, double yaw)
{
    qreal x = plotArea().width() / m_axis->tickCount();
    qreal y = (m_axis->max() - m_axis->min()) / m_axis->tickCount();
    m_x += y;
    if(roll < 180 )
        m_y_roll = roll;
    else
        m_y_roll = roll-360;
    if(pitch < 180)
        m_y_pitch = pitch;
    else
        m_y_pitch = pitch-360;
    m_roll->append(m_x, m_y_roll);
    m_pitch->append(m_x, m_y_pitch);
    scroll(x, 0);

}
