/* mode: C++ -*-
*
*  Copyright (C) 2019 Feng DING, Hirain
*
*  License: Modified BSD Software License Agreement
*
*/

#ifndef _LIDARS_ACTION_H_
#define _LIDARS_ACTION_H_

#include <vector>
#include "driver.h"

namespace itd_lidar {

namespace lidar_driver {

class LidarsAction {
 public:
  LidarsAction();
  ~LidarsAction();
  void Start();
  void Stop();
  void PushLidar(Driver lidar);

 private:
  std::vector<Driver> lidars_;
};

}  // namespace lidar_driver
}  // namespace itd_lidar

#endif  // _LIDARS_ACTION_H_
