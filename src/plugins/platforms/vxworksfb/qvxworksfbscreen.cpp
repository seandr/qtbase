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

#include "qvxworksfbscreen.h"

#include <QtFontDatabaseSupport/private/qgenericunixfontdatabase_p.h>
#include <QtEventDispatcherSupport/private/qgenericunixeventdispatcher_p.h>
#include <QtFbSupport/private/qfbbackingstore_p.h>
#include <QtFbSupport/private/qfbwindow_p.h>
#include <QtFbSupport/private/qfbcursor_p.h>

#include <QtGui/private/qguiapplication_p.h>
#include <QtGui/private/qpixmap_raster_p.h>

#include <QtGui/QPainter>

#include <ioLib.h>
#include <fbdev.h>

QT_BEGIN_NAMESPACE

static int determineDepth(const FB_IOCTL_ARG &fbinfo)
{
    int depth = fbinfo.getFbInfo.bpp;
    if (depth== 24) {
        depth = fbinfo.getFbInfo.pixelFormat.redBits +
                fbinfo.getFbInfo.pixelFormat.greenBits +
                fbinfo.getFbInfo.pixelFormat.blueBits;
        if (depth <= 0)
            depth = 24; // reset if color component lengths are not reported
    } else if (depth == 16) {
        depth = fbinfo.getFbInfo.pixelFormat.redBits +
                fbinfo.getFbInfo.pixelFormat.greenBits +
                fbinfo.getFbInfo.pixelFormat.blueBits;
        if (depth <= 0)
            depth = 16; // reset if color component lengths are not reported
    }
    return depth;
}

static QRect determineGeometry(const FB_IOCTL_ARG &fbinfo, const QRect &userGeometry)
{
    //int xoff = fbinfo.xoffset;
    //int yoff = fbinfo.yoffset;
    int w, h;
    if (userGeometry.isValid()) {
        w = userGeometry.width();
        h = userGeometry.height();
        if ((uint)w > fbinfo.getVideoModes.pVideoModes->xres)
            w = fbinfo.getVideoModes.pVideoModes->xres;
        if ((uint)h > fbinfo.getVideoModes.pVideoModes->yres)
            h = fbinfo.getVideoModes.pVideoModes->yres;

        /*int xxoff = userGeometry.x(), yyoff = userGeometry.y();
        if (xxoff != 0 || yyoff != 0) {
            if (xxoff < 0 || xxoff + w > (int)(fbinfo.getVideoModes.pVideoModes->xres))
                xxoff = fbinfo.getVideoModes.pVideoModes->xres - w;
            if (yyoff < 0 || yyoff + h > (int)(fbinfo.getVideoModes.pVideoModes->yres))
                yyoff = fbinfo.getVideoModes.pVideoModes->yres - h;
            xoff += xxoff;
            yoff += yyoff;
        } else {
            xoff += (fbinfo.getVideoModes.pVideoModes->xres - w)/2;
            yoff += (binfo.getVideoModes.pVideoModes->yres - h)/2;
        }*/
    } else {
        w = fbinfo.getVideoModes.pVideoModes->xres;
        h = fbinfo.getVideoModes.pVideoModes->yres;
    }

    if (w == 0 || h == 0) {
        qWarning("Unable to find screen geometry, using 320x240");
        w = 320;
        h = 240;
    }

    return QRect(0, 0, w, h);
}

static QSizeF determinePhysicalSize(const FB_IOCTL_ARG &fbinfo, const QSize &mmSize, const QSize &res)
{
    Q_UNUSED(fbinfo)
    int mmWidth = mmSize.width(), mmHeight = mmSize.height();

    if (mmWidth <= 0 && mmHeight <= 0) {
        const int dpi = 72;
        mmWidth = qRound(res.width() * 25.4 / dpi);
        mmHeight = qRound(res.height() * 25.4 / dpi);
    } else if (mmWidth > 0 && mmHeight <= 0) {
        mmHeight = res.height() * mmWidth/res.width();
    } else if (mmHeight > 0 && mmWidth <= 0) {
        mmWidth = res.width() * mmHeight/res.height();
    }

    return QSize(mmWidth, mmHeight);
}

QVxWorksFbScreen::QVxWorksFbScreen(const QStringList &args)
    : mArgs(args), mFbFd(-1), mCompositePainter(0)
{
}

QVxWorksFbScreen::~QVxWorksFbScreen()
{
    if (mFbFd != -1) {
        close(mFbFd);
    }

    delete mCompositePainter;
}

bool QVxWorksFbScreen::initialize()
{
    QRegExp mmSizeRx(QLatin1String("mmsize=(\\d+)x(\\d+)"));
    QRegExp sizeRx(QLatin1String("size=(\\d+)x(\\d+)"));
    QRegExp offsetRx(QLatin1String("offset=(\\d+)x(\\d+)"));

    QSize userMmSize;
    QRect userGeometry;

    int userMmIdx = mArgs.indexOf(mmSizeRx);
    if (userMmIdx >= 0) {
        mmSizeRx.exactMatch(mArgs.at(userMmIdx));
        userMmSize = QSize(mmSizeRx.cap(1).toInt(), mmSizeRx.cap(2).toInt());
    }

    int userGeometryIdx = mArgs.indexOf(sizeRx);
    if (userGeometryIdx >= 0) {
        sizeRx.exactMatch(mArgs.at(userMmIdx));
        userGeometry.setSize(QSize(sizeRx.cap(1).toInt(), sizeRx.cap(2).toInt()));
    }

    int userOffsetIdx = mArgs.indexOf(offsetRx);
    if (userOffsetIdx >= 0) {
        offsetRx.exactMatch(mArgs.at(userMmIdx));
        userGeometry.setTopLeft(QPoint(offsetRx.cap(1).toInt(), sizeRx.cap(2).toInt()));
    }

    unsigned int i;
    char deviceName[FB_MAX_STR_CHARS];

    for (i = 0; i < FB_MAX_DEVICE; i++) {
        snprintf(deviceName, FB_MAX_STR_CHARS, "%s%d", FB_DEVICE_PREFIX, i);
        if ((mFbFd = QT_OPEN(deviceName, O_RDWR, 0666)) != -1)
            break;
    }

    if (i >= FB_MAX_DEVICE) {
        fprintf(stdout, "ERROR: Open errno:%08x\n", errno);
        return false;
    }

    FB_IOCTL_ARG arg;
    if (ioctl(mFbFd, FB_IOCTL_GET_FB_INFO, &arg) == -1) {
        fprintf(stdout, "ERROR: FB_IOCTL_GET_FB_INFO\n");
        return false;
    }

    FB_CONFIG config[FB_MAX_CONFIGS];
    arg.getConfigs.pConfigs = config;
    if (ioctl(mFbFd, FB_IOCTL_GET_CONFIGS, &arg.getConfigs) == -1) {
        fprintf(stdout, "ERROR: FB_IOCTL_GET_CONFIGS\n");
        return false;
    }

    FB_VIDEO_MODE fbModes[FB_MAX_VIDEO_MODES];
    bzero ((char *) &(fbModes[0]), sizeof(FB_VIDEO_MODE)*FB_MAX_VIDEO_MODES);
    arg.getVideoModes.pVideoModes = &(fbModes[0]);
    if (ioctl (mFbFd, FB_IOCTL_GET_VIDEO_MODES, &arg) == -1) {
        fprintf(stdout, "ERROR: FB_IOCTL_GET_VIDEO_MODE\n");
        return false;
    }

#if 0
    qDebug() << "bits per pixel: " << arg.getFbInfo.bpp;
    qDebug() << "video xres: " << arg.getVideoModes.pVideoModes->xres;
    qDebug() << "video yres: " << arg.getVideoModes.pVideoModes->yres;
    qDebug() << "videoName: " << arg.getVideoModes.pVideoModes->name;
#endif

    data = (unsigned char *)arg.getFbInfo.pFb;
    bzero((char *)data, arg.getFbInfo.width * (arg.getFbInfo.height) * (arg.getFbInfo.bpp)/8);

    QRect geometry = determineGeometry(arg, userGeometry);
    mGeometry = QRect(QPoint(0, 0), geometry.size());

    mBytesPerLine = arg.getFbInfo.stride;
    mDepth = determineDepth(arg);
    mFormat = QImage::Format_ARGB32_Premultiplied;

    mDispsize.setHeight(arg.getFbInfo.height);
    mDispsize.setWidth(arg.getFbInfo.width);

    QFbScreen::initializeCompositor();

    mPhysicalSize = determinePhysicalSize(arg, userMmSize, mGeometry.size());
    mFbScreenImage = new QImage(data, mGeometry.width(), mGeometry.height(),
                              mBytesPerLine, mFormat);

#ifndef QT_NO_CURSOR
    mCursor = new QFbCursor(this);
#endif
    return true;
}

QRegion QVxWorksFbScreen::doRedraw()
{
    QRegion touched = QFbScreen::doRedraw();

    if (touched.isEmpty())
        return touched;
    if (!mCompositePainter) {
        mCompositePainter = new QPainter(mFbScreenImage);
    }

    QVector<QRect> rects = touched.rects();
    for (int i = 0; i < rects.size(); i++) {
        mCompositePainter->drawImage(rects[i], mScreenImage, rects[i]);
    }
    return touched;
}


int QVxWorksFbScreen::width()
{
    return mDispsize.width();
}

int QVxWorksFbScreen::height()
{
    return mDispsize.height();
}

QT_END_NAMESPACE
