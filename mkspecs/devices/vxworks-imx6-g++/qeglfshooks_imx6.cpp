/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the qmake spec of the Qt Toolkit.
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

#include "qeglfshooks.h"
#include <private/qmath_p.h>
#include <EGL/eglvivante.h>

#include <QDebug>

QT_BEGIN_NAMESPACE

class QEglFSIVxImx6Hooks : public QEglFSHooks
{
public:
    QEglFSIVxImx6Hooks();
    virtual void platformInit();
    virtual EGLNativeDisplayType platformDisplay() const;
    virtual QSizeF physicalScreenSize() const;
    virtual QSize screenSize() const;
    virtual int screenDepth() const;
    virtual EGLNativeWindowType createNativeWindow(QPlatformWindow *window, const QSize &size, const QSurfaceFormat &format);
    virtual void destroyNativeWindow(EGLNativeWindowType window);

private:
    QSize mScreenSize;
    EGLNativeDisplayType mNativeDisplay;
};

QEglFSIVxImx6Hooks::QEglFSIVxImx6Hooks()
{
    /*bool multiBufferNotEnabledYet = qEnvironmentVariableIsEmpty("FB_MULTI_BUFFER");
    bool multiBuffer = qEnvironmentVariableIsEmpty("QT_EGLFS_VXWORKS_NO_FB_MULTI_BUFFER");
    if (multiBufferNotEnabledYet && multiBuffer) {
        qWarning() << "QEglFSIVxImx6Hooks will set environment variable FB_MULTI_BUFFER=2 to enable double buffering and vsync.\n"
                   << "If this is not desired, you can override this via: export QT_EGLFS_VXWORKS_NO_FB_MULTI_BUFFER=1";
        qputenv("FB_MULTI_BUFFER", "2");
    }*/
}

void QEglFSIVxImx6Hooks::platformInit()
{
    QEGLDeviceIntegration::platformInit();

    int width, height;
    mNativeDisplay = (EGLNativeDisplayType) fbGetDisplayByIndex(framebufferIndex());
    fbGetDisplayGeometry(mNativeDisplay, &width, &height);
    mScreenSize.setHeight(height);
    mScreenSize.setWidth(width);
}

EGLNativeDisplayType QEglFSIVxImx6Hooks::platformDisplay() const
{
    return mNativeDisplay;
}

QSizeF QEglFSIVxImx6Hooks::physicalScreenSize() const
{
    static QSizeF size;

    if (size.isEmpty()) {
        // Note: in millimeters
        int width = qgetenv("QT_QPA_EGLFS_PHYSICAL_WIDTH").toInt();
        int height = qgetenv("QT_QPA_EGLFS_PHYSICAL_HEIGHT").toInt();

        if (width && height) {
            // no need to read fbdev
            size.setWidth(width);
            size.setHeight(height);
            return size;
        }

        QSize screenResolution = screenSize();
        int w = screenResolution.width();
        int h = screenResolution.height();

        const int defaultPhysicalDpi = 100;
        size.setWidth(w <= 0 ? screenResolution.width() * Q_MM_PER_INCH / defaultPhysicalDpi : qreal(w));
        size.setHeight(h <= 0 ? screenResolution.height() * Q_MM_PER_INCH / defaultPhysicalDpi : qreal(h));

        if (w <= 0 || h <= 0) {
            qWarning("EGLFS: Unable to query physical screen size, defaulting to %d dpi.\n"
                     "EGLFS: To override, set QT_QPA_EGLFS_PHYSICAL_WIDTH "
                     "and QT_QPA_EGLFS_PHYSICAL_HEIGHT (in millimeters).",
                     defaultPhysicalDpi);
        }

        // override fbdev from environment var setting
        if (width)
            size.setWidth(width);
        if (height)
            size.setWidth(height);
    }
    return size;
}

QSize QEglFSIVxImx6Hooks::screenSize() const
{
    return mScreenSize;
}

int QEglFSIVxImx6Hooks::screenDepth() const
{
    return 32;
}

EGLNativeWindowType QEglFSIVxImx6Hooks::createNativeWindow(QPlatformWindow *platformWindow,const QSize &size, const QSurfaceFormat &format)
{
    Q_UNUSED(platformWindow);
    Q_UNUSED(format);

    EGLNativeWindowType eglWindow = fbCreateWindow(mNativeDisplay, 0, 0, size.width(), size.height());
    return eglWindow;
}

void QEglFSIVxImx6Hooks::destroyNativeWindow(EGLNativeWindowType window)
{
    fbDestroyWindow(window);
}

QEglFSIVxImx6Hooks eglFSVxImx6Hooks;
QEglFSHooks *platformHooks = &eglFSVxImx6Hooks;

QT_END_NAMESPACE
