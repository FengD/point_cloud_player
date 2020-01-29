#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QDialog>
#include <QAction>
#include <QMenu>
#include <QMenuBar>
#include <QStatusBar>
#include <QSlider>
#include <QLCDNumber>
#include <QLabel>
#include <QComboBox>
#include <map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include "driver.h"
#include "lidar.h"
#include "QVTKWidget.h"
#include "cloud_visualization.h"
#include "add_dialog.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui {
class PointCloudPlayer;
}

class PointCloudPlayer : public QMainWindow {
  Q_OBJECT

public:
  explicit PointCloudPlayer(QWidget *parent = 0);
  ~PointCloudPlayer();

private:
  Ui::PointCloudPlayer *ui;
  QVTKWidget *cloudViewer;
  QMenuBar *menuBar;
  QMenu *menuFirst;
  QAction *actionAdd;
  QSlider *playSlider;
  QLCDNumber *frameNb;
  QPushButton *saveBtn, *playBtn, *pauseBtn, *nextBtn, *previousBtn, *repeatBtn;
  QDialog *addDialog;
  QComboBox *playSpeedComboBox;
  CloudVisualization *viewer;
  std::map<int, PointCloudT>  clouds;
  int currentCloudIndex;
  int cloudsSize;

  void init();
  void createPlaySpeedComboBox();
  void updateDisplay(const int &index);
  void addLidarAction(const CLidarConfig &config);
  void buttonsEnabled(const bool &isEnable);

private slots:
  void trigerMenu(QAction* act);
  void playSliderValueChanged(const int &value);

  void currentIndexChanged(const QString &text);

  void saveButtonPressed();
  void playButtonPressed();
  void pauseButtonPressed();
  void nextButtonPressed();
  void previousButtonPressed();
  void repeatButtonPressed();
};

#endif // MAINWINDOW_H
