/* mode: C++ -*-
*
*  Copyright (C) 2019 Feng DING, Hirain
*
*  License: Modified BSD Software License Agreement
*
*/

#include "lidars_creator_actioner.h"
#include "lidars_action.h"
#include "driver.h"

namespace itd_lidar {

namespace lidar_driver {

LidarsCreatorActioner::LidarsCreatorActioner() {
  lidarsNum_ = 0;
  lidarsAction_ = new LidarsAction();
}

LidarsCreatorActioner::~LidarsCreatorActioner() {
  delete lidarsAction_;
}

void LidarsCreatorActioner::LoadConfig(YAML::Node config) {
  lidarsNum_ = config["num"].as<int>();
  YAML::Node lidarParamList = config["lidarList"];
  int sizeLidarParamList = lidarParamList.size();
  if (lidarsNum_ != sizeLidarParamList) {
    printf("warning: the input num and the size of the lidar array is not equal.\n");
    return;
  }

  for (int i = 0; i < lidarsNum_; i++) {
    LidarConfig config;
    YAML::Node lidarParam = lidarParamList[i];
    config.ip =  lidarParam["srcIP"].as<std::string>();
    config.groupIp =  lidarParam["groupIP"].as<std::string>();
    config.port =  lidarParamList[i]["port"].as<int>();
    config.model =  lidarParam["model"].as<std::string>();
    config.mode =  lidarParam["mode"].as<int>();
    config.direction =  lidarParam["direction"].as<int>();
    config.version =  lidarParam["version"].as<int>();
    config.cutAngle =  lidarParam["cutAngle"].as<float>();
    YAML::Node correctionFileList = lidarParam["correctionFileList"];
    int sizeCorrectionfileList = correctionFileList.size();
    for (int j = 0; j < sizeCorrectionfileList; j++) {
      std::string correctionFile = correctionFileList[j].as<std::string>();
      config.correctionFileList.push_back(correctionFile);
    }
    lidarConfigs_.push_back(config);
  }
}

void LidarsCreatorActioner::SetLidarCallBackByIndex(int index, boost::function<void(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, int timestamp)> lidar_callback) {
  if (index >= 0 && index < lidarsNum_) {
    LidarConfig config = lidarConfigs_[index];
    Driver driver(config.ip, config.groupIp, config.port,
                  config.model, config.mode, config.direction,
                  config.version, config.correctionFileList,
                  config.cutAngle, lidar_callback);
    lidarsAction_->PushLidar(driver);
  } else {
    printf("The index %d does not exist.\n", index);
  }
}

void LidarsCreatorActioner::Start() {
  lidarsAction_->Start();
}

void LidarsCreatorActioner::Stop() {
  lidarsAction_->Stop();
}

}  // namespace lidar_driver
}  // namespace itd_lidar
