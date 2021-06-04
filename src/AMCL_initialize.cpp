
#include <juiz/juiz.h>
#include <juiz/container.h>

#include "AMCL.h"
#include "amcl_wrapper.h"

using namespace juiz;

extern "C" {
  JUIZ_OPERATION  void* AMCL_initialize();
}


odom_config argToOdomConfig(const Value& v) {
  odom_config c;
  c.model_type_str_ = "diff";
  c.alpha1_ = 0.2;
  c.alpha2_ = 0.2;
  c.alpha3_ = 0.2;
  c.alpha4_ = 0.2;
  c.alpha5_ = 0.2;
  return c;
}

pf_config argToPFConfig(const Value& v) {
  pf_config c;
  auto vc = v["config"];
  c.min_particles_ = Value::intValue(vc["min_particles"], 500);
  c.max_particles_ = 5000;
  c.alpha_slow_ = 0.001;
  c.alpha_fast_ = 0.1;
  //pf_init_model_fn_t init_model_;
  c.selective_resampling_ = true;
  c.pf_err_ = 0.01;
  c.pf_z_ = 0.99;
  c.init_pose_[0] = 0.0;
  c.init_pose_[1] = 0.0;
  c.init_pose_[2] = 0.0;  
  c.init_cov_[0] = 0.25;
  c.init_cov_[1] = 0.25;
  c.init_cov_[2] = 0.68535;  
  c.d_thresh_ = 0.2;
  c.a_thresh_ = 0.5326;
  return c;
}

likelihood_field_model_config argToLikelihoodFieldConfig(const Value& v) {
  likelihood_field_model_config c;
  c.do_beamskip_ = true;
  c.beam_skip_distance_ = 0.5;
  c.beam_skip_threshold_ = 0.3;
  c.beam_skip_error_threshold_ = 0.9;
  c.max_dist_ = 4.0;
  return c;
}

laser_config argToLaserConfig(const Value& v) {
  laser_config c;
  c.max_beams_ = 30;
  //c.model_type_ = likelihood;
  c.model_type_str_ = "likelihood_field";
  c.z_hit_ = 0.95;
  c.z_short_ = 0.4;
  c.z_max_ = 0.1;
  c.z_rand_ = 0.1;
  c.sigma_hit_ = 0.4;
  c.lambda_short_ = 0.2;  
  //beam_model_config beam_model_;
  c.likelihood_model_ = argToLikelihoodFieldConfig(v);
  c.max_range_ = 5.0;
  c.min_range_ = 0.3;
  return c;
}
  

JUIZ_OPERATION  void* AMCL_initialize() {
  return containerOperationFactory<AMCL>
    (
     {
       {"typeName", "initialize"},
       {"defaultArg", {
	   {"arg01", 0}
	 }}
     },
     [](auto& container, auto arg)
     {
       logger::trace("AMCL_initialize({}) called", arg);
       // マップがセットされていればパーティクルフィルタを初期化
       auto pfconfig = argToPFConfig(arg);
       if (!(container.pf = std::shared_ptr<pf_t>(initPF(pfconfig, container.map.get())))) {
	 return Value::error(logger::error("AMCL_initialize failed. initPF failed."));
       }

       // レーザーの初期化
       auto laserconfig = argToLaserConfig(arg);
       if (!(container.laser = std::shared_ptr<amcl::AMCLLaser>(initLaser(laserconfig, container.map.get())))) {
	 return Value::error(logger::error("AMCL_initialize failed. initLaser failed."));
       }

       auto odomconfig = argToOdomConfig(arg);
       if (!(container.odom = std::shared_ptr<amcl::AMCLOdom>(initOdom(odomconfig)))) {
	 return Value::error(logger::error("AMCL_initialize failed. initOdom failed."));
       }
       
       return arg;
     });
}
