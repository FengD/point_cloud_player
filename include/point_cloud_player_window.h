// Copyright (C) 2020 Feng DING, Hirain
// License: Modified BSD Software License Agreement

#ifndef POINT_CLOUD_PLAYER_WINDOW_H
#define POINT_CLOUD_PLAYER_WINDOW_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <QMainWindow>
#include <map>
#include <pcl/visualization/cloud_viewer.h>
#include "add_dialog.h"

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui {
class PointCloudPlayer;
}

namespace itd_lidar {
// class CloudVisualization;
namespace lidar_driver {
class Driver;
}
}

class PointCloudPlayer : public QMainWindow {
  Q_OBJECT

 public:
  explicit PointCloudPlayer(QWidget *parent = 0);
  ~PointCloudPlayer();

 private:
  Ui::PointCloudPlayer *ui_;
  int current_cloud_index_;
  int clouds_size_;
  int load_cloud_index_;
  float play_speed_;
  pcl::visualization::CloudViewer *viewer_;
  itd_lidar::lidar_driver::Driver *lidar_;
  std::map<int, PointCloudT>  clouds_;

  void InitParam();
  void InitSlotConnect();
  void CreatePlaySpeedComboBox();
  void UpdateDisplay(const int &index);
  void AddLidarAction(const CLidarConfig &config);
  void ButtonsEnabled(const bool &isEnable);
  void UpdatePushButtons(const bool &isEnable);
  void LidarCallback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud,
                     int timestamp);

  private slots:
  void trigerMenu(QAction *act);
  void playSliderMoved(const int &value);
  void currentIndexChanged(const QString &text);
  void saveButtonPressed();
  void playButtonPressed();
  void pauseButtonPressed();
  void nextButtonPressed();
  void previousButtonPressed();
  void repeatButtonPressed();
};

#endif  // POINT_CLOUD_PLAYER_WINDOW_H
