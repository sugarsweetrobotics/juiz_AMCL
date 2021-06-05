
#include <juiz/juiz.h>
#include <juiz/container.h>

#include "AMCL.h"


//#if WIN32
#ifdef min

#define MIN(x, y) ((x > y) ? (y) : (x))
#define MAX(x, y) ((x > y) ? (x) : (y))

#else

#define MIN(x, y) std::min((x), (y))
#define MAX(x, y) std::max((x), (y))

#endif // min
//#endif // WIN32

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


amcl::AMCLLaserData* toLaserData(amcl::AMCLLaser* laser, const laser_config& config, const Value& v, const juiz::Pose3D& offset) {
  amcl::AMCLLaserData* pldata = new amcl::AMCLLaserData();
  amcl::AMCLLaserData& ldata = *pldata;
  ldata.sensor = laser;
  
  pf_vector_t laser_pose;
  laser_pose.v[0] = offset.position.x;
  laser_pose.v[1] = offset.position.y;
  laser_pose.v[2] = 0; // Angular Offset is not allowed currently
  laser->SetLaserPose(laser_pose);

  auto& ranges = v["ranges"].listValue();
  ldata.range_count = ranges.size();
  
  if(config.max_range_ > 0.0)
    ldata.range_max = MIN(Value::doubleValue(v["maxRange"]), config.max_range_);
  else
    ldata.range_max = Value::doubleValue(v["maxRange"]);
  const double range_min = MAX(Value::doubleValue(v["minRange"]), config.min_range_);
  const double angle_min = Value::doubleValue(v["minAngle"]);
  const double angle_max = Value::doubleValue(v["maxAngle"]);
  const double angle_increment = Value::doubleValue(v["resAngle"]);
  /*
  std::cout << "LaserData:" << std::endl;
  std::cout << "  range_max      : " << ldata.range_max << std::endl;
  std::cout << "  range_min      : " << range_min << std::endl;  
  std::cout << "  angle_min      : " << angle_min << std::endl;
  std::cout << "  angle_increment: " << angle_increment << std::endl;
  std::cout << "  range_count    : " << ldata.range_count << std::endl;
  */
  ldata.ranges = new double[ldata.range_count][2];

  bool inv_rotate = false;
  long range_count = 0;
  for(int i = 0;i < ldata.range_count;i++) {
    // amcl doesn't (yet) have a concept of min range.  So we'll map short
    // readings to max range.
    double r = ranges[i].doubleValue();
    if(r <= range_min) {
      /* if (inv_rotate) {
	 ldata.ranges[ldata.range_count - i - 1][0] = ldata.range_max;
	 } else {
	 ldata.ranges[i][0] = ldata.range_max;
	 } */
    } else {
      if (inv_rotate) {
        ldata.ranges[ldata.range_count - range_count - 1][0] = r;
      } else {
	ldata.ranges[range_count][0] = r;
      }
      // Compute bearing
      ldata.ranges[range_count][1] = -(angle_min + (i * angle_increment));
      range_count++;
    }
  }
  ldata.range_count = range_count;
  return pldata;
}



JUIZ_OPERATION  void* AMCL_resample() {
  return containerOperationFactory<AMCL>
    (
     {
       {"typeName", "resample"},
       {"defaultArg", {
	   {"scan", { /// Spec for URG04LXUG01
	       {"ranges", Value::list()},
	       {"maxRange", 5.0},
	       {"maxAngle", 270.0/2/180*M_PI},
	       {"minAngle", -270.0/2/180*M_PI},
	       {"resAngle", 270.0/180*M_PI/684}	       	       
	     }},
	   {"offset", {
	       {"position", {
		   {"x", 0.},
		   {"y", 0.},
		   {"z", 0.}
		 }},
	       {"orientation", {
		   {"x", 0.},
		   {"y", 0.},
		   {"z", 0.},
		   {"w", 1.0}		   
		 }}
	     }}
	 }}
     },
     [](auto& container, auto arg)
     {
       logger::trace("AMCL_setMap({}) called", arg);
       auto offset = toPose3D(arg["offset"]);
       auto laserData = toLaserData(container.laser.get(), container.laserConfig, arg, offset);
       container.laser->UpdateSensor(container.pf.get(), (amcl::AMCLSensorData*)laserData);
       pf_update_resample(container.pf.get());
       return arg;
     });
}

