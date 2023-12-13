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

#ifndef QVXWORKSFB_VIRTUALKBD_H
#define QVXWORKSFB_VIRTUALKBD_H

#ifndef QT_NO_VIRTUAL_KBD_VXWORKSFB
#include <QObject>

QT_BEGIN_NAMESPACE

class QSocketNotifier;
class QVxWorksFbVirtualKbd : public QObject
{
    Q_OBJECT
public:
    QVxWorksFbVirtualKbd();
    ~QVxWorksFbVirtualKbd();

    virtual bool filterInputEvent(quint16 &input_code, qint32 &input_value);

private slots:
    void readVirtualKbdData();

private:
    QSocketNotifier *m_virtualnotify;
    int              m_virtualfd;
};
QT_END_NAMESPACE

#endif

#endif // QVXWORKSFB_VIRTUALKBD_H
