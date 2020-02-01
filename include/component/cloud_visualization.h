// Copyright (C) 2020 Feng DING, Hirain
// License: Modified BSD Software License Agreement

#include <pcl/visualization/pcl_visualizer.h>

namespace itd_lidar {
class CloudVisualization : public pcl::visualization::PCLVisualizer {
 public:
  CloudVisualization();
  CloudVisualization(const int &width, const int &height);
  virtual ~CloudVisualization();
};
}  // namespace itd_lidar
