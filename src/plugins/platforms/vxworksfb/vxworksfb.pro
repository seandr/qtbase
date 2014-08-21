TARGET = qvxworksfb

PLUGIN_TYPE = platforms
PLUGIN_CLASS_NAME = QVxWorksFbIntegrationPlugin
!equals(TARGET, $$QT_DEFAULT_QPA_PLUGIN): PLUGIN_EXTENDS = -
load(qt_plugin)

QT += core-private gui-private platformsupport-private

HEADERS +=\
    $$PWD/qvxworksfbintegration_fbdev.h \
    $$PWD/qvxworksfbscreen.h \
    $$PWD/qvxworksfbvirtualptr.h \
    $$PWD/qvxworksfbvirtualkbd.h

SOURCES +=\
    $$PWD/main.cpp \
    $$PWD/qvxworksfbintegration_fbdev.cpp \
    $$PWD/qvxworksfbscreen.cpp \
    $$PWD/qvxworksfbvirtualptr.cpp \
    $$PWD/qvxworksfbvirtualkbd.cpp

CONFIG += qpa/genericunixfontdatabase
OTHER_FILES += vxworksfb.json
