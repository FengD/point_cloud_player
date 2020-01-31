#-------------------------------------------------
#
# Project created by QtCreator 2019-01-30T15:59:49
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = point_cloud_player
TEMPLATE = app

CONFIG += static

QMAKE_CXXFLAGS += -std=c++14

LIBS += \
  -lboost_system \
  -lboost_filesystem \
  -lboost_thread \
  -lpcl_visualization \
  -lpcl_common \
  -lpcl_io \
  -lpcl_kdtree \
  -lpcl_search \
  -lpcap \
  -lyaml-cpp \
  -lvtkGUISupportQt-7.1 \
  -lvtkCommonCore-7.1 \
  -lvtkRenderingCore-7.1 \
  -lvtkCommonDataModel-7.1 \
  -lvtkCommonMath-7.1


INCLUDEPATH += \
  include \
  include/component \
  lidar_driver/include \
  /usr/local/include/pcl-1.8 \
  /usr/include/eigen3 \
  /usr/local/include/vtk-7.1

SOURCES += \
  src/component/cloud_visualization.cpp \
  src/main.cpp \
  src/point_cloud_player_window.cpp \
  src/add_dialog.cpp \
  lidar_driver/src/driver.cpp \
  lidar_driver/src/input.cpp \
  lidar_driver/src/raw_data.cpp \
  lidar_driver/src/lidars_action.cpp \
  lidar_driver/src/lidars_creator_actioner.cpp \
  lidar_driver/src/raw_data_factory.cpp

HEADERS += \
  include/component/cloud_visualization.h \
  include/point_cloud_player_window.h \
  include/add_dialog.h \
  lidar_driver/include/driver.h \
  lidar_driver/include/input.h \
  lidar_driver/include/raw_data.h \
  lidar_driver/include/lidars_action.h \
  lidar_driver/include/lidars_creator_actioner.h \
  lidar_driver/include/raw_data_factory.h

FORMS += \
  ui/point_cloud_player_window.ui \
  ui/add_dialog.ui

RESOURCES += \
    test.qrc
