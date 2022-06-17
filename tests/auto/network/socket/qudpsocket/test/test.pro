CONFIG += testcase
testcase.timeout = 800 # this test is slow
SOURCES  += ../tst_qudpsocket.cpp
INCLUDEPATH += ../../../../../shared/
QT = core network testlib

MOC_DIR=tmp

win32:debug_and_release {
  CONFIG(debug, debug|release) {
    DESTDIR = ../debug
} else {
    DESTDIR = ../release
  }
} else {
    DESTDIR = ../
}

TARGET = tst_qudpsocket

CONFIG += unsupported/testserver
QT_TEST_SERVER_LIST = danted echo
