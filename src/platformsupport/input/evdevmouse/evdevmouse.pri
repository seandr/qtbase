HEADERS += \
    $$PWD/qevdevmousehandler_p.h \
    $$PWD/qevdevmousemanager_p.h

SOURCES += \
    $$PWD/qevdevmousemanager.cpp

qtConfig(libudev): \
    QMAKE_USE_PRIVATE += libudev

vxworks {
    SOURCES += \
        $$PWD/qevdevmousehandler_vxworks.cpp \
} else {
    SOURCES += \
        $$PWD/qevdevmousehandler.cpp \
}
