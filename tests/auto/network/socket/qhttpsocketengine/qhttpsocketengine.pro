CONFIG += testcase
TARGET = tst_qhttpsocketengine
SOURCES  += tst_qhttpsocketengine.cpp


include(../platformsocketengine/platformsocketengine.pri)

MOC_DIR=tmp

requires(qtConfig(private_tests))
QT = core-private network-private testlib

CONFIG += unsupported/testserver
QT_TEST_SERVER_LIST = squid danted cyrus apache2

vxworks: LIBS += $$QMAKE_LIBS_NETWORK
