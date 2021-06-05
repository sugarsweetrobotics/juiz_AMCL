#pragma once

#include "amcl_wrapper.h"

struct AMCL {
  std::shared_ptr<pf_t> pf;
  std::shared_ptr<map_t> map;
  std::shared_ptr<amcl::AMCLLaser> laser;
  std::shared_ptr<amcl::AMCLOdom> odom;
  juiz::TimedPose3D oldPose;
  juiz::TimedPose3D estimatedPose;
  laser_config laserConfig;
};

