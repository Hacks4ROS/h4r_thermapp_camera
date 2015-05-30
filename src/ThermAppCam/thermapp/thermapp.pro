#-------------------------------------------------
#
# Project created by QtCreator 2015-01-03T09:13:54
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = thermapp
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    thermapp.c

HEADERS  += mainwindow.h \
    thermapp.h

FORMS    += mainwindow.ui


LIBS += -lrt -lpthread -lusb-1.0 -lm
