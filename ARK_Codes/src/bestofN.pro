#-------------------------------------------------
#
# Project created by QtCreator 2017-02-08T11:35:48
#
#-------------------------------------------------

QT       -= gui

QT += widgets

TARGET = bestofNExp
TEMPLATE = lib

DEFINES += bestofNEXP_LIBRARY

SOURCES += \
    kilobot.cpp \
    bestofNEnv.cpp \
    bestofNExp.cpp

HEADERS +=\
    kilobot.h \
    kilobotexperiment.h \
    kilobotenvironment.h \
    bestofNExp.h \
    bestofNEnv.h \
    global.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

INCLUDEPATH += /usr/local/include/
LIBS += -L/usr/local/lib \
        -lopencv_core
