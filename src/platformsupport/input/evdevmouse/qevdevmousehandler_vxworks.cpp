/****************************************************************************
**
** Copyright (C) 2014 Digia Plc and/or its subsidiary(-ies).
** Contact: http://www.qt-project.org/legal
**
** This file is part of the QtGui module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL21$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and Digia. For licensing terms and
** conditions see http://qt.digia.com/licensing. For further information
** use the contact form at http://qt.digia.com/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 or version 3 as published by the Free
** Software Foundation and appearing in the file LICENSE.LGPLv21 and
** LICENSE.LGPLv3 included in the packaging of this file. Please review the
** following information to ensure the GNU Lesser General Public License
** requirements will be met: https://www.gnu.org/licenses/lgpl.html and
** http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** In addition, as a special exception, Digia gives you certain additional
** rights. These rights are described in the Digia Qt LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "qevdevmousehandler_p.h"

#include <QSocketNotifier>
#include <QStringList>
#include <QPoint>
#include <QGuiApplication>
#include <QScreen>
#include <QLoggingCategory>
#include <qpa/qwindowsysteminterface.h>

#include <qplatformdefs.h>
#include <private/qcore_unix_p.h> // overrides QT_OPEN

#include <errno.h>

#include <evdevLib.h>

QT_BEGIN_NAMESPACE

Q_LOGGING_CATEGORY(qLcEvdevMouse, "qt.qpa.input")

QEvdevMouseHandler *QEvdevMouseHandler::create(const QString &device, const QString &specification)
{
    qCDebug(qLcEvdevMouse) << "create mouse handler for" << device << specification;

    bool compression = false;
    int jitterLimit = 0;
    bool abs = false;

    QStringList args = specification.split(QLatin1Char(':'));
    foreach (const QString &arg, args) {
        if (arg == QLatin1String("abs"))
            abs = true;
    }

    int fd;
    fd = qt_safe_open(device.toLocal8Bit().constData(), O_RDONLY | O_NONBLOCK, 0);
    if (fd >= 0) {
        return new QEvdevMouseHandler(device, fd, abs, compression, jitterLimit);
    } else {
        qWarning("Cannot open mouse input device '%s': %s", qPrintable(device), strerror(errno));
        return 0;
    }
}

QEvdevMouseHandler::QEvdevMouseHandler(const QString &device, int fd, bool abs, bool compression, int jitterLimit)
    : m_device(device), m_fd(fd), m_notify(0), m_x(0), m_y(0), m_prevx(0), m_prevy(0),
      m_abs(abs), m_compression(compression), m_buttons(0), m_prevInvalid(true)
{
    setObjectName(QLatin1String("Evdev Mouse Handler"));

    // socket notifier for events on the mouse device
    QSocketNotifier *notifier;
    notifier = new QSocketNotifier(m_fd, QSocketNotifier::Read, this);
    connect(notifier, SIGNAL(activated(int)), this, SLOT(readMouseData()));
}

QEvdevMouseHandler::~QEvdevMouseHandler()
{
    if (m_fd >= 0)
        qt_safe_close(m_fd);
}

void QEvdevMouseHandler::sendMouseEvent()
{
    int x;
    int y;

    x = m_x - m_prevx;
    y = m_y - m_prevy;

    if (m_prevInvalid) {
        x = y = 0;
        m_prevInvalid = false;
    }

    emit handleMouseEvent(x, y, m_abs, m_buttons);

    m_prevx = m_x;
    m_prevy = m_y;
}

void QEvdevMouseHandler::readMouseData()
{
    bool posChanged = false, btnChanged = false;
    Q_FOREVER {
        EV_DEV_EVENT ev;
        size_t n = read(m_fd, (char *)(&ev), sizeof(EV_DEV_EVENT));
        if (n < sizeof(EV_DEV_EVENT)) {
            sendMouseEvent();
            return;
        }
        switch (ev.type) {
        case EV_DEV_SYN:
            {
            break;
            }
        case EV_DEV_KEY: {
            Qt::MouseButton buttons = Qt::NoButton;
            switch (ev.code) {
            case EV_DEV_PTR_BTN_LEFT:
                buttons = Qt::LeftButton;
                break;
            case EV_DEV_PTR_BTN_RIGHT:
                buttons = Qt::MidButton;
                break;
            case EV_DEV_PTR_BTN_MIDDLE:
                buttons = Qt::RightButton;
                break;
            }
            if (ev.value)
                m_buttons |= buttons;
            else
                m_buttons &= ~buttons;
            btnChanged = true;
            break;
            }
        case EV_DEV_REL:
            switch (ev.code) {
            case EV_DEV_PTR_REL_X:
                m_x += ev.value;
                posChanged = true;
                break;
            case EV_DEV_PTR_REL_Y:
                m_y += ev.value;
                posChanged = true;
                break;
            }
            break;
        case EV_DEV_ABS:
            switch (ev.code) {
            case EV_DEV_PTR_ABS_X:
                m_x = ev.value;
                posChanged = true;
                break;
            case EV_DEV_PTR_ABS_Y:
                m_y = ev.value;
                posChanged = true;
                break;
            }
            break;
        }
    }

    if (btnChanged) {
        btnChanged = posChanged = false;
        sendMouseEvent();
    } else if (posChanged) {
        posChanged = false;
        sendMouseEvent();
    }
}

QT_END_NAMESPACE
