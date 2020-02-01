// Copyright (C) 2020 Feng DING, Hirain
// License: Modified BSD Software License Agreement

#include "cloud_visualization.h"

namespace itd_lidar {

CloudVisualization::CloudVisualization()
 : pcl::visualization::PCLVisualizer("view") {
  this->addCoordinateSystem(1.0);
}

CloudVisualization::CloudVisualization(const int &width, const int &height)
  : pcl::visualization::PCLVisualizer("view") {
  new (this) CloudVisualization();
  this->setSize(width, height);
}

CloudVisualization::~CloudVisualization() {}

}  // namespace itd_lidar
