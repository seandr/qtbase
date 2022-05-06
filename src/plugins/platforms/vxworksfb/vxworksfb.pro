TARGET = qvxworksfb

QT += \
    core-private gui-private \
    eventdispatcher_support-private fb_support-private fontdatabase_support-private

qtHaveModule(input_support-private): \
    QT += input_support-private

HEADERS += \
    qvxworksfbintegration_fbdev.h \
    qvxworksfbscreen.h \
    qvxworksfbvirtualptr.h \
    qvxworksfbvirtualkbd.h

SOURCES += \
    main.cpp \
    qvxworksfbintegration_fbdev.cpp \
    qvxworksfbscreen.cpp \
    qvxworksfbvirtualptr.cpp \
    qvxworksfbvirtualkbd.cpp

CONFIG += qpa/genericunixfontdatabase
OTHER_FILES += vxworksfb.json

PLUGIN_TYPE = platforms
PLUGIN_CLASS_NAME = QVxWorksFbIntegrationPlugin
!equals(TARGET, $$QT_DEFAULT_QPA_PLUGIN): PLUGIN_EXTENDS = -
load(qt_plugin)
