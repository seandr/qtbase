/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the plugins of the Qt Toolkit.
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

#ifndef QVXWORKSFB_VIRTUALPTR_H
#define QVXWORKSFB_VIRTUALPTR_H

#ifndef QT_NO_VIRTUAL_PTR_VXWORKSFB
#include <QObject>

QT_BEGIN_NAMESPACE

class QSocketNotifier;
class QVxWorksFbVirtualPtr : public QObject
{
    Q_OBJECT
public:
    QVxWorksFbVirtualPtr(int dispWidth, int dispHeight);
    ~QVxWorksFbVirtualPtr();

private slots:
    void readVirtualMouseData();

private:
    QSocketNotifier *m_virtualnotify;
    int              m_virtualfd;
    int              m_buttons;
    int              m_x, m_y;
    int              m_xend, m_yend;
    bool             m_buttonPressed;
};
QT_END_NAMESPACE

#endif

#endif // QVXWORKSFB_VIRTUALPTR_H
