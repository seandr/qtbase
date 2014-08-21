/****************************************************************************
**
** Copyright (C) 2014 Digia Plc and/or its subsidiary(-ies).
** Contact: http://www.qt-project.org/legal
**
** This file is part of the plugins of the Qt Toolkit.
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

#ifndef QT_NO_VIRTUAL_PTR_VXWORKSFB

#include "qvxworksfbvirtualptr.h"

#include <QSocketNotifier>
#include <qpa/qwindowsysteminterface.h>
#include <private/qcore_unix_p.h> // overrides QT_OPEN

#include <errno.h>

#define MOUSE_BUTTON_1              0x01
#define MOUSE_BUTTON_2              0x02
#define MOUSE_BUTTON_3              0x04

typedef struct {
    int x;
    int y;
} POINT, VECTOR;

typedef struct {
    UINT16                  buttonState;    /* pointer button states */
    BOOL                    isAbsolute;     /* TRUE == absolute,
                                               FALSE == relative */
    union {
        POINT               absolute;       /* absolute coordinates */
        VECTOR              relative;       /* relative coordinates */
    } pos;
} RAW_PTR_DATA;

QVxWorksFbVirtualPtr::QVxWorksFbVirtualPtr(int dispWidth, int dispHeight)
{
    m_virtualfd = qt_safe_open("/virtualPtr/0", O_RDONLY, 0);

    if (m_virtualfd > 0) {
        m_virtualnotify = new QSocketNotifier(m_virtualfd, QSocketNotifier::Read, this);
        connect(m_virtualnotify, SIGNAL(activated(int)), this, SLOT(readVirtualMouseData()));
    }

    m_x = dispWidth / 2;
    m_y = dispHeight / 2;
    m_xend = dispWidth;
    m_yend = dispHeight;
}

QVxWorksFbVirtualPtr::~QVxWorksFbVirtualPtr()
{
    if (m_virtualfd > 0) {
        qt_safe_close(m_virtualfd);
        m_virtualfd = -1;
    }
}

void QVxWorksFbVirtualPtr::readVirtualMouseData()
{
    Q_FOREVER {
        RAW_PTR_DATA ev;
        size_t n = read(m_virtualfd, (char *)(&ev), sizeof(RAW_PTR_DATA));
        if (n < sizeof(RAW_PTR_DATA)) return;
        int buttons = 0;
        if (ev.buttonState & MOUSE_BUTTON_1)
            buttons |= Qt::LeftButton;
        if (ev.buttonState & MOUSE_BUTTON_2)
            buttons |= Qt::RightButton;
        if (ev.buttonState & MOUSE_BUTTON_3)
            buttons |= Qt::MidButton;
        m_buttons = buttons;
        if (ev.isAbsolute) {
            m_x = ev.pos.absolute.x;
            m_y = ev.pos.absolute.y;
        } else {
            m_x += ev.pos.relative.x;
            m_y += ev.pos.relative.y;
        }
        if (m_x < 0) m_x = 0;
        else if (m_x > m_xend) m_x = m_xend;
        if (m_y < 0) m_y = 0;
        else if (m_y > m_yend) m_y = m_yend;
        QPoint qtPos(m_x, m_y);
        QWindowSystemInterface::handleMouseEvent(0, qtPos, qtPos, Qt::MouseButtons(m_buttons));
    }
}

#endif // QT_NO_QPA_MOUSE_VXWORKS
