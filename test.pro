#-------------------------------------------------
#
# Project created by QtCreator 2019-01-30T15:59:49
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = calibration_tool
TEMPLATE = app

CONFIG += static

QMAKE_CXXFLAGS += -std=c++14

LIBS += -lboost_system \
        -lboost_filesystem \
        -lboost_thread \
        -lpcl_visualization \
        -lpcl_common \
        -lpcl_io \
        -lpcl_kdtree \
        -lpcl_search \
        -lvtkGUISupportQt-7.1 \
        -lvtkCommonCore-7.1 \
        -lvtkRenderingCore-7.1 \
        -lvtkCommonDataModel-7.1 \
        -lvtkCommonMath-7.1


INCLUDEPATH += ./include \
               ./lidar_driver/include \
               /usr/local/include/pcl-1.8 \
               /usr/include/eigen3 \
               /usr/local/include/vtk-7.1

SOURCES += src/main.cpp \
    src/main_window.cpp \
    src/add_lidar_dialog.cpp \
    lidar_driver/src/driver.cpp \
    lidar_driver/src/input.cpp \
    lidar_driver/src/rawdata.cpp \
    lidar_driver/src/lidar.cpp

HEADERS += include/main_window.h \
    include/add_lidar_dialog.h \
    lidar_driver/include/driver.h \
    lidar_driver/include/input.h \
    lidar_driver/include/rawdata.h \
    lidar_driver/include/lidar.h

FORMS += ui/main_window.ui \
         ui/add_lidar_dialog.ui
