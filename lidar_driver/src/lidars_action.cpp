/* mode: C++ -*-
*
*  Copyright (C) 2019 Feng DING, Hirain
*
*  License: Modified BSD Software License Agreement
*
*/

#include "lidars_action.h"

namespace itd_lidar {

namespace lidar_driver {

LidarsAction::LidarsAction() {}
LidarsAction::~LidarsAction() {}
void LidarsAction::Start() {
  for (Driver &l : lidars_) {
    l.Start();
  }
}

void LidarsAction::Stop() {
  for (Driver &l : lidars_) {
    l.Stop();
  }
}

void LidarsAction::PushLidar(Driver lidar) {
  lidars_.push_back(lidar);
}

}  // namespace lidar_driver
}  // namespace itd_lidar
