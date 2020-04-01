#-------------------------------------------------
#
# Project created by QtCreator 2019-07-11T15:17:48
#
#-------------------------------------------------

QT       += core gui gamepad

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport serialport

TARGET = PermanentMagnetSystemApplication
TEMPLATE = app

CONFIG += c++11

SOURCES += daq.cpp\
        main.cpp \
        mainwindow.cpp \
        gamepadmonitor.cpp \
        qcustomplot.cpp

HEADERS += daq.h\
        mainwindow.h \
        gamepadmonitor.h \
        qcustomplot.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
INCLUDEPATH += "C:/Program Files (x86)/National Instruments/NI-DAQ/DAQmx ANSI C Dev/include"
LIBS += "C:/Program Files (x86)/National Instruments/NI-DAQ/DAQmx ANSI C Dev/lib/msvc/NIDAQmx.lib"
LIBS += -lXinput9_1_0
