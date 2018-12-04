CONFIG += testcase
TARGET = tst_qftp
SOURCES  += tst_qftp.cpp

requires(qtConfig(private_tests))
QT = core network network-private testlib

QT_TEST_SERVER_LIST = vsftpd ftp-proxy squid danted
include($$dirname(_QMAKE_CONF_)/tests/auto/testserver.pri)
