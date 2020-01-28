#include "main_window.h"
#include <QApplication>

int main(int argc, char *argv[]) {
  QApplication application(argc, argv);
  MainWindow mainWindow;
  mainWindow.show();
  return application.exec();
}
