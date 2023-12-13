/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the plugins of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:COMM$
**
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** $QT_END_LICENSE$
**
**
**
**
**
**
**
**
**
**
**
**
**
**
**
**
**
**
**
****************************************************************************/

#ifndef QT_NO_VIRTUAL_KBD_VXWORKSFB

#include "qvxworksfbvirtualkbd.h"

#include <QSocketNotifier>
#include <qpa/qwindowsysteminterface.h>
#include <private/qcore_unix_p.h> // overrides QT_OPEN

#include <sioLib.h>

typedef struct {
    BOOL                    isKeyDown;      /* down transition */
    BOOL                    isKeyCode;      /* down transition */
    union {
        UINT32              scanCode;       /* key scancode */
        UINT32              keyCode;        /* mapped key value + modifiers */
    } value;
} RAW_KBD_DATA;

QVxWorksFbVirtualKbd::QVxWorksFbVirtualKbd()
{
    m_virtualfd = qt_safe_open("/virtualKbd/0", O_RDONLY, 0);

    if (m_virtualfd > 0) {
        m_virtualnotify = new QSocketNotifier(m_virtualfd, QSocketNotifier::Read, this);
        connect(m_virtualnotify, SIGNAL(activated(int)), this, SLOT(readVirtualKbdData()));
    }
}

QVxWorksFbVirtualKbd::~QVxWorksFbVirtualKbd()
{
    if (m_virtualfd > 0) {
        qt_safe_close(m_virtualfd);
        m_virtualfd = -1;
    }
}

bool QVxWorksFbVirtualKbd::filterInputEvent(quint16 &, qint32 &)
{
    return false;
}

typedef struct {
    int key;
    int keyShift;
    int unicode;
    int unicodeShift;
} KEYMAP;

void QVxWorksFbVirtualKbd::readVirtualKbdData()
{
    Q_FOREVER {
        RAW_KBD_DATA ev;
        size_t n = read(m_virtualfd, (char *)(&ev), sizeof(RAW_KBD_DATA));
        if (n < sizeof(RAW_KBD_DATA)) return;
    }
}

#endif // QT_NO_VIRTUAL_KBD_VXWORKSFB
