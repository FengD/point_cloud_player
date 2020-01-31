/**
  * Copyright (C) 2020 Hirain Technologies
  * License: Modified BSD Software License Agreement
  * Author: Feng DING
  * Description: This file used to define the udp input
  */
#ifndef _RAW_DATA_FACTORY_H_
#define _RAW_DATA_FACTORY_H_

#include <string>

namespace itd_lidar {
namespace lidar_driver {

class RawData;

class RawDataFactory {
 private:
  std::string lidar_model_;
 public:
  RawDataFactory();
  ~RawDataFactory();
  void setLidarModel(std::string lidar_model);
  std::string getLidarModel();
  RawData* createRawData();
};

}  // namespace lidar_driver
}  // namespace itd_lidar

#endif  // _RAW_DATA_FACTORY_H_
