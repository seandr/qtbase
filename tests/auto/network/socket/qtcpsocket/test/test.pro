CONFIG += testcase

QT = core-private network-private testlib
SOURCES += ../tst_qtcpsocket.cpp

win32: QMAKE_USE += ws2_32
TARGET = tst_qtcpsocket

win32 {
  CONFIG(debug, debug|release) {
    DESTDIR = ../debug
} else {
    DESTDIR = ../release
  }
} else {
    DESTDIR = ../
}

CONFIG += unsupported/testserver
QT_TEST_SERVER_LIST = danted squid apache2 ftp-proxy vsftpd iptables cyrus
