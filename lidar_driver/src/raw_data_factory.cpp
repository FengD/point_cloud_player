/**
  * Copyright (C) 2020 Hirain Technologies
  * License: Modified BSD Software License Agreement
  * Author: Feng DING
  * Description: This file used to define the udp input
  */
#include "raw_data_factory.h"
#include "raw_data.h"

namespace itd_lidar {

namespace lidar_driver {

RawDataFactory::RawDataFactory() {}
RawDataFactory::~RawDataFactory() {}

void RawDataFactory::setLidarModel(std::string lidar_model) {
  lidar_model_ = lidar_model;
}

std::string RawDataFactory::getLidarModel() {
  return lidar_model_;
}

RawData* RawDataFactory::createRawData() {
  if (lidar_model_ == "VLP16") {
    return new RawDataVlp16();
  } else if (lidar_model_ == "P40P") {
    return new RawDataP40P();
  } else if (lidar_model_ == "RSL32") {
    return new RawDataRSL32();
  } else if (lidar_model_ == "InnovizPro") {
    return new InnovizPro();
  } else if (lidar_model_ == "VLP32MR") {
    return new RawDataVlp32MR();
  }
  return NULL;
}

}  // namespace lidar_driver
}  // namespace itd_lidar
