CONFIG += testcase
TARGET = tst_networkselftest

SOURCES += tst_networkselftest.cpp
QT = core core-private network testlib

QT_TEST_SERVER_LIST = danted squid apache2 ftp-proxy vsftpd iptables cyrus
include($$dirname(_QMAKE_CONF_)/tests/auto/testserver.pri)
