/* mode: C++ -*-
*
*  Copyright (C) 2019 Feng DING, Hirain
*
*  License: Modified BSD Software License Agreement
*
*/

#ifndef _LIDARS_CREATOR_ACTIONER_H_
#define _LIDARS_CREATOR_ACTIONER_H_

#include <vector>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

namespace itd_lidar {

namespace lidar_driver {

class LidarsAction;

class LidarsCreatorActioner {
 public:
  typedef struct {
    std::string ip;
    std::string groupIp;
    int port;
    std::string model;
    int mode;
    float cutAngle;
    int direction;
    int version;
    std::vector<std::string> correctionFileList;
  } LidarConfig;
  LidarsCreatorActioner();
  ~LidarsCreatorActioner();
  void LoadConfig(YAML::Node config);
  void SetLidarCallBackByIndex(int index, boost::function<void(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, int timestamp)> lidar_callback);
  void Start();
  void Stop();
 private:
  std::vector<LidarConfig> lidarConfigs_;
  LidarsAction *lidarsAction_;
  int lidarsNum_;
};

}  // namespace lidar_driver
}  // namespace itd_lidar

#endif  // _LIDARS_CREATOR_ACTIONER_H_
