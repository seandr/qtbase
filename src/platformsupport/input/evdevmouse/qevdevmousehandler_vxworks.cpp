/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtGui module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:COMM$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see http://www.qt.io/terms-conditions. For further
** information use the contact form at http://www.qt.io/contact-us.
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
      m_abs(abs), m_compression(compression), m_buttons(0), m_button(Qt::NoButton), m_eventType(QEvent::None), m_prevInvalid(true)
{
    Q_UNUSED(jitterLimit)
    setObjectName(QLatin1String("Evdev Mouse Handler"));

    // socket notifier for events on the mouse device
    QSocketNotifier *notifier;
    notifier = new QSocketNotifier(m_fd, QSocketNotifier::Read, this);
    connect(notifier, &QSocketNotifier::activated,
            this, &QEvdevMouseHandler::readMouseData);
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

    emit handleMouseEvent(x, y, m_abs, m_buttons, m_button, m_eventType);

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
                buttons = Qt::RightButton;
                break;
            case EV_DEV_PTR_BTN_MIDDLE:
                buttons = Qt::MidButton;
                break;
            }
            if (ev.value)
                m_buttons |= buttons;
            else
                m_buttons &= ~buttons;
            m_button = buttons;
            m_eventType = ev.value != 0 ? QEvent::MouseButtonPress : QEvent::MouseButtonRelease;
            btnChanged = true;
            break;
            }
        case EV_DEV_REL:
            switch (ev.code) {
            case EV_DEV_PTR_REL_X:
                m_x += ev.value;
                m_eventType = QEvent::MouseMove;
                posChanged = true;
                break;
            case EV_DEV_PTR_REL_Y:
                m_y += ev.value;
                m_eventType = QEvent::MouseMove;
                posChanged = true;
                break;
            }
            break;
        case EV_DEV_ABS:
            switch (ev.code) {
            case EV_DEV_PTR_ABS_X:
                m_x = ev.value;
                m_eventType = QEvent::MouseMove;
                posChanged = true;
                break;
            case EV_DEV_PTR_ABS_Y:
                m_y = ev.value;
                m_eventType = QEvent::MouseMove;
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
