CONFIG += testcase
SOURCES  += ../tst_qtcpserver.cpp

win32:LIBS += -lws2_32

TARGET = ../tst_qtcpserver

win32 {
  CONFIG(debug, debug|release) {
    TARGET = ../../debug/tst_qtcpserver
} else {
    TARGET = ../../release/tst_qtcpserver
  }
}

QT = core network testlib

MOC_DIR=tmp

QT_TEST_SERVER_LIST = danted cyrus squid ftp-proxy
include($$dirname(_QMAKE_CONF_)/tests/auto/testserver.pri)
