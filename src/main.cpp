// Copyright (C) 2020 Feng DING, Hirain
// License: Modified BSD Software License Agreement

#include <QApplication>
#include "point_cloud_player_window.h"

int main(int argc, char *argv[]) {
  QApplication application(argc, argv);
  application.addLibraryPath("../plugins");
  PointCloudPlayer mainWindow;
  mainWindow.show();
  return application.exec();
}
