CONFIG += testcase
TARGET = tst_platformsocketengine
SOURCES  += tst_platformsocketengine.cpp

include(../platformsocketengine/platformsocketengine.pri)

requires(qtConfig(private_tests))

MOC_DIR=tmp

QT = core-private network-private testlib

QT_TEST_SERVER_LIST = cyrus
include($$dirname(_QMAKE_CONF_)/tests/auto/testserver.pri)
