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

#include <qpa/qplatformintegrationplugin.h>
#include "qvxworksfbintegration_fbdev.h"

QT_BEGIN_NAMESPACE

class QVxWorksFbIntegrationPlugin : public QPlatformIntegrationPlugin
    {
    Q_OBJECT
    Q_PLUGIN_METADATA(IID QPlatformIntegrationFactoryInterface_iid FILE "vxworksfb.json")
public:
    QPlatformIntegration *create(const QString&, const QStringList&);
    };

QPlatformIntegration* QVxWorksFbIntegrationPlugin::create(const QString& system, const QStringList& paramList)
    {
    if (!system.compare(QLatin1String("vxworksfb"), Qt::CaseInsensitive))
        return new QVxWorksFbIntegration(paramList);

    return 0;
    }

QT_END_NAMESPACE

#include "main.moc"
