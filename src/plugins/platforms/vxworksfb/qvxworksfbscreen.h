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

#ifndef QVXWORKSFBSCREEN_H
#define QVXWORKSFBSCREEN_H

#include <QtPlatformSupport/private/qfbscreen_p.h>

QT_BEGIN_NAMESPACE

class QVxWorksFbScreen : public QFbScreen
{
    Q_OBJECT
public:
    QVxWorksFbScreen(const QStringList &args);
    ~QVxWorksFbScreen();

    bool initialize();
    int width();
    int height();

public slots:
    QRegion doRedraw();

private:
    QStringList mArgs;
    int mFbFd;
    int mBytesPerLine;

    QSize mDispsize;

    QImage *mFbScreenImage;
    uchar *data;

    QPainter *mCompositePainter;
};

QT_END_NAMESPACE

#endif // QVXWORKSFBSCREEN_H
