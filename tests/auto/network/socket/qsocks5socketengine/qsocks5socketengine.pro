CONFIG += testcase
TARGET = tst_qsocks5socketengine
SOURCES  += tst_qsocks5socketengine.cpp


include(../platformsocketengine/platformsocketengine.pri)


MOC_DIR=tmp

QT = core-private network-private testlib

requires(qtConfig(private_tests))

QT_TEST_SERVER_LIST = danted apache2 cyrus
include($$dirname(_QMAKE_CONF_)/tests/auto/testserver.pri)
