/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2019 Feng DING, Hirain
 *
 *  License: Modified BSD Software License Agreement
 *
 */

#include <sys/wait.h>
#include <stdlib.h>
#include "lidar_driver/lidar.h"

Lidar::Lidar(std::string deviceIp, int dataPort, std::string lidarModel, int mode, std::string correctionFilePath, boost::function<void(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, double timestamp)> lidarCallback) {
  // init lidar driver
	lidarDriver = new lidar_driver::Driver(deviceIp, dataPort, lidarModel, mode, correctionFilePath, lidarCallback);
}

Lidar::~Lidar() {
	stop();
}

void Lidar::start() {
	stop();
	lidarDriver->start();
}

void Lidar::stop() {
	// stop lidar sdk
	lidarDriver->stop();
}
