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

#include "qvxworksfbintegration_fbdev.h"
#include "qvxworksfbscreen.h"
#include "qvxworksfbvirtualptr.h"
#include "qvxworksfbvirtualkbd.h"

#include <private/qcore_unix_p.h> // overrides QT_OPEN
#include <QSocketNotifier>

#include <QtPlatformSupport/private/qgenericunixfontdatabase_p.h>
#include <QtPlatformSupport/private/qgenericunixeventdispatcher_p.h>
#include <QtPlatformSupport/private/qfbbackingstore_p.h>
#include <QtPlatformSupport/private/qfbwindow_p.h>
#include <QtPlatformSupport/private/qfbcursor_p.h>

#include <QtGui/private/qguiapplication_p.h>
#include <QtGui/private/qpixmap_raster_p.h>

#include <QtGui/QPainter>

QT_BEGIN_NAMESPACE

class QVxWorksFbIntegrationPrivate
{
public:
    QVxWorksFbIntegrationPrivate(const QStringList &paramList);
    ~QVxWorksFbIntegrationPrivate();

    QPlatformFontDatabase *mFontDb;
    QVxWorksFbScreen *mPrimaryScreen;
    QList<QPlatformScreen *> mScreens;
    int fbfd;

#ifndef QT_NO_VIRTUAL_PTR_VXWORKSFB
    QVxWorksFbVirtualPtr *virtualPtr;
#endif
#ifndef QT_NO_VIRTUAL_KBD_VXWORKSFB
    QVxWorksFbVirtualKbd *virtualKbd;
#endif
};

QVxWorksFbIntegrationPrivate::QVxWorksFbIntegrationPrivate(const QStringList &paramList)
    : mFontDb(new QGenericUnixFontDatabase()),
    fbfd(-1)
{
    Q_UNUSED(paramList)
}

QVxWorksFbIntegrationPrivate::~QVxWorksFbIntegrationPrivate()
{
#ifndef QT_NO_VIRTUAL_PTR_VXWORKSFB
    delete virtualPtr;
#endif
#ifndef QT_NO_VIRTUAL_KBD_VXWORKSFB
    delete virtualKbd;
#endif
    delete mPrimaryScreen;
}

QVxWorksFbIntegration::QVxWorksFbIntegration(const QStringList &paramList)
    : d_ptr(new QVxWorksFbIntegrationPrivate(paramList))
{
    d_ptr->mPrimaryScreen = new QVxWorksFbScreen(paramList);
}

QVxWorksFbIntegration::~QVxWorksFbIntegration()
{
    close(d_ptr->fbfd);
    delete d_ptr;
}

void QVxWorksFbIntegration::initialize()
{
    if (d_ptr->mPrimaryScreen->initialize())
        screenAdded(d_ptr->mPrimaryScreen);
    else
        qWarning("vxworksfb: Failed to initialize screen");

#ifndef QT_NO_VIRTUAL_PTR_VXWORKSFB
    d_ptr->virtualPtr = new QVxWorksFbVirtualPtr(d_ptr->mPrimaryScreen->width(), d_ptr->mPrimaryScreen->height());
#endif
#ifndef QT_NO_VIRTUAL_KBD_VXWORKSFB
    d_ptr->virtualKbd = new QVxWorksFbVirtualKbd();
#endif
}

bool QVxWorksFbIntegration::hasCapability(QPlatformIntegration::Capability cap) const
{
    switch (cap) {
        case ThreadedPixmaps: return true;
        default:
            return QPlatformIntegration::hasCapability(cap);
    }
}

QPlatformPixmap *QVxWorksFbIntegration::createPlatformPixmap(QPlatformPixmap::PixelType type) const
{
    return new QRasterPlatformPixmap(type);
}

QList<QPlatformScreen *> QVxWorksFbIntegration::screens() const
{
    return d_ptr->mScreens;
}

QPlatformWindow *QVxWorksFbIntegration::createPlatformWindow(QWindow *window) const
{
    QFbWindow *w = new QFbWindow(window);
    d_ptr->mPrimaryScreen->addWindow(w);
    return w;
}

QPlatformBackingStore *QVxWorksFbIntegration::createPlatformBackingStore(QWindow *window) const
{
    return new QFbBackingStore(window);
}

QAbstractEventDispatcher *QVxWorksFbIntegration::createEventDispatcher() const
{
    return createUnixEventDispatcher();
}

QPlatformFontDatabase *QVxWorksFbIntegration::fontDatabase() const
{
    return d_ptr->mFontDb;
}
QT_END_NAMESPACE
