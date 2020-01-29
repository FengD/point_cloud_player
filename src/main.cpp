#include "point_cloud_player_window.h"
#include <QApplication>

int main(int argc, char *argv[]) {
  QApplication application(argc, argv);
  application.addLibraryPath("./plugins");
  PointCloudPlayer mainWindow;
  mainWindow.show();
  return application.exec();
}
