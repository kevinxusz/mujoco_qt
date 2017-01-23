QT += core
QT -= gui

CONFIG +=c++11
#QMAKE_CXXFLAGS += -std=c++11
#QMAKE_CXXFLAGS += -Wall -Wextra -pedantic

TARGET = daq
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp
#SOURCES += simulate.cpp
#SOURCES += basic.cpp

HEADERS += \
    mujoco.h \
    mjdata.h \
    mjmodel.h \
    mjrender.h \
    mjvisualize.h \
    glfw3.h \
    mjxmacro.h


LIBS += -L$$PWD/../../mjpro140/bin/ -lglew -lglewegl -lglewosmesa -lmujoco140 -lmujoco140nogl /home/student/mjpro140/bin/libglfw.so.3
INCLUDEPATH += $$PWD/../../mjpro140/bin
DEPENDPATH += $$PWD/../../mjpro140/bin


LIBS += -lGL -lGLEW


