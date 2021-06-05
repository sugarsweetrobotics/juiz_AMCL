
#include <juiz/juiz.h>
#include <juiz/container.h>

#include "AMCL.h"

using namespace juiz;

extern "C" {
  JUIZ_OPERATION  void* AMCL_update();
}

pf_vector_t convert(const juiz::Pose3D& pose) {
  pf_vector_t d;
  d.v[0] = pose.position.x;
  d.v[1] = -pose.position.y;
  auto ryp = juiz::QuaternionToEulerXYZ(pose.orientation);
  d.v[2] = -ryp.z;
  return d;
}

juiz::Pose3D convert(const double& x, const double& y, const double& th) {
  juiz::Pose3D p;
  p.position.x = x;
  p.position.y = -y;
  p.position.z = 0;
  p.orientation = juiz::EulerXYZToQuaternion({0, 0, -th});
  return p;
}


inline double angular_diff(const double x, const double y) {
  double d = x - y;
  while(d > M_PI) d-=2*M_PI;
  while(d < -M_PI) d+=2*M_PI;
  return d;
}

pf_vector_t diff(const juiz::Pose3D& x0, const juiz::Pose3D& x1) {
  pf_vector_t delta = pf_vector_zero();
  delta.v[0] = x0.position.x - x1.position.x;
  delta.v[1] = -(x0.position.y - x1.position.y);
  auto ryp0 = juiz::QuaternionToEulerXYZ(x0.orientation);
  auto ryp1 = juiz::QuaternionToEulerXYZ(x1.orientation);  
  delta.v[2] = -angular_diff(ryp0.z, ryp1.z);
  return delta;
}




JUIZ_OPERATION  void* AMCL_update() {
  return containerOperationFactory<AMCL>
    (
     {
       {"typeName", "update"},
       {"defaultArg", {
	   {"arg01", 0}
	 }}
     },
     [](auto& container, auto arg)
     {
       logger::trace("AMCL_setMap({}) called", arg);
       auto pose = toTimedPose3D(arg);
       amcl::AMCLOdomData odomData;
       odomData.pose = convert(pose.pose);
       odomData.delta = diff(pose.pose, container.oldPose.pose);
       container.odom->UpdateAction(container.pf.get(), (amcl::AMCLSensorData*)&odomData);
       
       container.oldPose = pose;
       auto est = convert(container.estimatedPose.pose);
       double x = est.v[0] + odomData.delta.v[0];
       double y = est.v[1] + odomData.delta.v[1];
       double th = est.v[2] + odomData.delta.v[2];
       container.estimatedPose.tm = pose.tm;
       container.estimatedPose.pose = convert(x, y, th);
       return toValue(container.estimatedPose);
     });
}

