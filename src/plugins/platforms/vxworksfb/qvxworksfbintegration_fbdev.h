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

#ifndef QGRAPHICSSYSTEM_VXWORKSFB_H
#define QGRAPHICSSYSTEM_VXWORKSFB_H

#include <qpa/qplatformintegration.h>
#include <qpa/qplatformnativeinterface.h>

QT_BEGIN_NAMESPACE

class QVxWorksFbIntegrationPrivate;
class QVxWorksFbIntegration : public QPlatformIntegration, public QPlatformNativeInterface
{
public:
    QVxWorksFbIntegration(const QStringList &paramList);
    ~QVxWorksFbIntegration();

    void initialize() Q_DECL_OVERRIDE;
    bool hasCapability(QPlatformIntegration::Capability cap)  const Q_DECL_OVERRIDE;

    QPlatformPixmap *createPlatformPixmap(QPlatformPixmap::PixelType type)  const Q_DECL_OVERRIDE;
    QPlatformWindow *createPlatformWindow(QWindow *window)  const Q_DECL_OVERRIDE;
    QPlatformBackingStore *createPlatformBackingStore(QWindow *window)  const Q_DECL_OVERRIDE;
    QAbstractEventDispatcher *createEventDispatcher()  const Q_DECL_OVERRIDE;
    QPlatformFontDatabase *fontDatabase()  const Q_DECL_OVERRIDE;
    QPlatformInputContext *inputContext() const Q_DECL_OVERRIDE;

    QList<QPlatformScreen *> screens() const;

private:
    void createInputHandlers();

    QVxWorksFbIntegrationPrivate *d_ptr;
};

QT_END_NAMESPACE

#endif

