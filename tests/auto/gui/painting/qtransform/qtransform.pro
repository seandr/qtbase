CONFIG += testcase
TARGET = tst_qtransform
SOURCES  += tst_qtransform.cpp
QT += testlib

unix:!darwin:!haiku:!integrity:!vxworks: LIBS += -lm
