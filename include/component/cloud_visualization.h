// Copyright (C) 2020 Feng DING, Hirain
// License: Modified BSD Software License Agreement

#ifndef CLOUD_VISUALIZATION_H
#define CLOUD_VISUALIZATION_H

#include <pcl/visualization/pcl_visualizer.h>

namespace itd_lidar {
class CloudVisualization : public pcl::visualization::PCLVisualizer {
 public:
  CloudVisualization();
  CloudVisualization(const int &width, const int &height);
  virtual ~CloudVisualization();
};
}  // namespace itd_lidar

#endif  // CLOUD_VISUALIZATION_H
