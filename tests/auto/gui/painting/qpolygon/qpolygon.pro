CONFIG += testcase
TARGET = tst_qpolygon
QT += testlib
SOURCES  += tst_qpolygon.cpp

unix:!darwin:!haiku:!integrity:!vxworks: LIBS += -lm


