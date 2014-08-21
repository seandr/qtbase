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

#define MAX_KEYMAP 0x63+1

static KEYMAP keyMap[MAX_KEYMAP] = {
/* 0 */ { 0,          0,                                    0,          0 },
/* 1 */ { 0,          0,                                    0,          0 },
/* 2 */ { 0,          0,                                    0,          0 },
/* 3 */ { 0,          0,                                    0,          0 },
/* 4 */ { Qt::Key_A,  Qt::Key_A,                          'a',        'A' },
/* 5 */ { Qt::Key_B,  Qt::Key_B,                          'b',        'B' },
/* 6 */ { Qt::Key_C,  Qt::Key_C,                          'c',        'C' },
/* 7 */ { Qt::Key_D,  Qt::Key_D,                          'd',        'D' },
/* 8 */ { Qt::Key_E,  Qt::Key_E,                          'e',        'E' },
/* 9 */ { Qt::Key_F,  Qt::Key_F,                          'f',        'F' },
/* A */ { Qt::Key_G,  Qt::Key_G,                          'g',        'G' },
/* B */ { Qt::Key_H,  Qt::Key_H,                          'h',        'H' },
/* C */ { Qt::Key_I,  Qt::Key_I,                          'i',        'I' },
/* D */ { Qt::Key_J,  Qt::Key_J,                          'j',        'J' },
/* E */ { Qt::Key_K,  Qt::Key_K,                          'k',        'K' },
/* F */ { Qt::Key_L,  Qt::Key_L,                          'l',        'L' },
/*10 */ { Qt::Key_M,  Qt::Key_M,                          'm',        'M' },
/*11 */ { Qt::Key_N,  Qt::Key_N,                          'n',        'N' },
/*12 */ { Qt::Key_O,  Qt::Key_O,                          'o',        'O' },
/*13 */ { Qt::Key_P,  Qt::Key_P,                          'p',        'P' },
/*14 */ { Qt::Key_Q,  Qt::Key_Q,                          'q',        'Q' },
/*15 */ { Qt::Key_R,  Qt::Key_R,                          'r',        'R' },
/*16 */ { Qt::Key_S,  Qt::Key_S,                          's',        'S' },
/*17 */ { Qt::Key_T,  Qt::Key_T,                          't',        'T' },
/*18 */ { Qt::Key_U,  Qt::Key_U,                          'u',        'U' },
/*19 */ { Qt::Key_V,  Qt::Key_V,                          'v',        'V' },
/*1A */ { Qt::Key_W,  Qt::Key_W,                          'w',        'W' },
/*1B */ { Qt::Key_X,  Qt::Key_X,                          'x',        'X' },
/*1C */ { Qt::Key_Y,  Qt::Key_Y,                          'y',        'Y' },
/*1D */ { Qt::Key_Z,  Qt::Key_Z,                          'z',        'Z' },
/*1E */ { Qt::Key_1,  Qt::Key_Exclam,                     '1',        '!' },
/*1F */ { Qt::Key_2,  Qt::Key_At,                         '2',        '@' },
/*20 */ { Qt::Key_3,  Qt::Key_NumberSign,                 '3',        '#' },
/*21 */ { Qt::Key_4,  Qt::Key_Dollar,                     '4',        '$' },
/*22 */ { Qt::Key_5,  Qt::Key_Percent,                    '5',        '%' },
/*23 */ { Qt::Key_6,  Qt::Key_AsciiCircum,                '6',        '^' },
/*24 */ { Qt::Key_7,            Qt::Key_Ampersand,        '7',        '&' },
/*25 */ { Qt::Key_8,            Qt::Key_Asterisk,         '8',        '*' },
/*26 */ { Qt::Key_9,            Qt::Key_ParenLeft,        '9',        '(' },
/*27 */ { Qt::Key_0,            Qt::Key_ParenRight,       '0',        ')' },
/*28 */ { Qt::Key_Return,       0,                     0x000D,          0 },
/*29 */ { Qt::Key_Escape,       0,                     0x001B,          0 },
/*2A */ { Qt::Key_Backspace,    0,                     0x0008,          0 },
/*2B */ { Qt::Key_Tab,          0,                     0x0009,          0 },
/*2C */ { Qt::Key_Space,        0,                     0x0020,          0 },
/*2D */ { Qt::Key_Minus,        Qt::Key_Underscore,       '-',        '_' },
/*2E */ { Qt::Key_Equal,        Qt::Key_Plus,             '=',        '+' },
/*2F */ { Qt::Key_BracketLeft,  Qt::Key_BraceLeft,        '[',        '{' },
/*30 */ { Qt::Key_BracketRight, Qt::Key_BraceRight,       ']',        '}' },
/*31 */ { Qt::Key_Backslash,    Qt::Key_Bar,             '\\',        '|' },
/*32 */ { 0,                    0,                          0,          0 },
/*33 */ { Qt::Key_Semicolon,    Qt::Key_Colon,            ';',        ':' },
/*34 */ { Qt::Key_Apostrophe,   Qt::Key_QuoteDbl,        '\'',       '\"' },
/*35 */ { Qt::Key_QuoteLeft,    Qt::Key_AsciiTilde,       '`',        '~' },
/*36 */ { Qt::Key_Comma,        Qt::Key_Less,             ',',        '<' },
/*37 */ { Qt::Key_Period,       Qt::Key_Greater,          '.',        '>' },
/*38 */ { Qt::Key_Slash,        Qt::Key_Question,         '/',        '?' },
/*39 */ { Qt::Key_CapsLock,     0,                          0,          0 },
/*3A */ { Qt::Key_F1,           0,                          0,          0 },
/*3B */ { Qt::Key_F2,           0,                          0,          0 },
/*3C */ { Qt::Key_F3,           0,                          0,          0 },
/*3D */ { Qt::Key_F4,           0,                          0,          0 },
/*3E */ { Qt::Key_F5,           0,                          0,          0 },
/*3F */ { Qt::Key_F6,           0,                          0,          0 },
/*40 */ { Qt::Key_F7,           0,                          0,          0 },
/*41 */ { Qt::Key_F8,           0,                          0,          0 },
/*42 */ { Qt::Key_F9,           0,                          0,          0 },
/*43 */ { Qt::Key_F10,          0,                          0,          0 },
/*44 */ { Qt::Key_F11,          0,                          0,          0 },
/*45 */ { Qt::Key_F12,          0,                          0,          0 },
/*46 */ { Qt::Key_Print,        0,                          0,          0 },
/*47 */ { Qt::Key_ScrollLock,   0,                          0,          0 },
/*48 */ { Qt::Key_Pause,        0,                          0,          0 },
/*49 */ { Qt::Key_Insert,       0,                          0,          0 },
/*4A */ { Qt::Key_Home,         0,                          0,          0 },
/*4B */ { Qt::Key_PageUp,       0,                          0,          0 },
/*4C */ { Qt::Key_Delete,       0,                     0x007F,          0 },
/*4D */ { Qt::Key_End,          0,                          0,          0 },
/*4E */ { Qt::Key_PageDown,     0,                          0,          0 },
/*4F */ { Qt::Key_Right,        0,                          0,          0 },
/*50 */ { Qt::Key_Left,         0,                          0,          0 },
/*51 */ { Qt::Key_Down,         0,                          0,          0 },
/*52 */ { Qt::Key_Up,           0,                          0,          0 },
/*53 */ { Qt::Key_NumLock,      0,                          0,          0 },
/*54 */ { Qt::Key_Slash,        0,                        '/',          0 },
/*55 */ { Qt::Key_Asterisk,     0,                        '*',          0 },
/*56 */ { Qt::Key_Minus,        0,                        '-',          0 },
/*57 */ { Qt::Key_Plus,         0,                        '+',          0 },
/*58 */ { Qt::Key_Return,       0,                     0x000D,          0 },
/*59 */ { Qt::Key_End,          0,                          0,          0 },
/*5A */ { Qt::Key_Down,         0,                          0,          0 },
/*5B */ { Qt::Key_PageDown,     0,                          0,          0 },
/*5C */ { Qt::Key_Left,         0,                          0,          0 },
/*5D */ { Qt::Key_5,            0,                        '5',          0 },
/*5E */ { Qt::Key_Right,        0,                          0,          0 },
/*5F */ { Qt::Key_Home,         0,                          0,          0 },
/*60 */ { Qt::Key_Up,           0,                          0,          0 },
/*61 */ { Qt::Key_PageUp,       0,                          0,          0 },
/*62 */ { Qt::Key_Insert,       0,                          0,          0 },
/*63 */ { Qt::Key_Delete,       0,                     0x007F,          0 },
};

void QVxWorksFbVirtualKbd::readVirtualKbdData()
{
    Q_FOREVER {
        RAW_KBD_DATA ev;
        size_t n = read(m_virtualfd, (char *)(&ev), sizeof(RAW_KBD_DATA));
        if (n < sizeof(RAW_KBD_DATA)) return;
        //processKeyEvent(ev.value.keyCode, ev.value.keyCode, Qt::NoModifier, ev.isKeyDown, false);
    }
}

#endif // QT_NO_VIRTUAL_KBD_VXWORKSFB
