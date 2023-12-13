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

#include "qvxworksfbintegration_fbdev.h"
#include "qvxworksfbscreen.h"
#include "qvxworksfbvirtualptr.h"
#include "qvxworksfbvirtualkbd.h"

#include <private/qcore_unix_p.h> // overrides QT_OPEN
#include <QSocketNotifier>

#include <QtFontDatabaseSupport/private/qgenericunixfontdatabase_p.h>
#include <QtEventDispatcherSupport/private/qgenericunixeventdispatcher_p.h>
#include <QtFbSupport/private/qfbbackingstore_p.h>
#include <QtFbSupport/private/qfbwindow_p.h>
#include <QtFbSupport/private/qfbcursor_p.h>

#include <QtGui/private/qguiapplication_p.h>
#include <QtGui/private/qpixmap_raster_p.h>

#include <QtGui/QPainter>

#include <qpa/qplatforminputcontextfactory_p.h>

#if QT_CONFIG(evdev)
#include <QtInputSupport/private/qevdevmousemanager_p.h>
#include <QtInputSupport/private/qevdevkeyboardmanager_p.h>
#include <QtInputSupport/private/qevdevtouchmanager_p.h>
#endif

QT_BEGIN_NAMESPACE

class QVxWorksFbIntegrationPrivate
{
public:
    QVxWorksFbIntegrationPrivate(const QStringList &paramList);
    ~QVxWorksFbIntegrationPrivate();

    QPlatformFontDatabase *mFontDb;
    QVxWorksFbScreen *mPrimaryScreen;
    QList<QPlatformScreen *> mScreens;
    QPlatformInputContext *m_inputContext;

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
        QWindowSystemInterface::handleScreenAdded(d_ptr->mPrimaryScreen);
    else
        qWarning("vxworksfb: Failed to initialize screen");

#if QT_CONFIG(evdev)
    d_ptr->m_inputContext = QPlatformInputContextFactory::create();
    if (!qEnvironmentVariableIntValue("QT_QPA_FB_DISABLE_INPUT"))
        createInputHandlers();
#endif
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

QPlatformInputContext *QVxWorksFbIntegration::inputContext() const
{
    return d_ptr->m_inputContext;
}

void QVxWorksFbIntegration::createInputHandlers()
{
#if QT_CONFIG(evdev)
    new QEvdevKeyboardManager(QLatin1String("EvdevKeyboard"), QString(), this);
    new QEvdevMouseManager(QLatin1String("EvdevMouse"), QString(), this);
    new QEvdevTouchManager(QLatin1String("EvdevTouch"), QString() /* spec */, this);
#endif
}
QT_END_NAMESPACE
