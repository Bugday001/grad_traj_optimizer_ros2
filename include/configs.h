#ifndef _CONFIGS_H
#define _CONFIGS_H

struct Configs
{
  int algorithm;
  double time_limit_1;
  double time_limit_2;
  double try_limit;
  double offset;
  double deltat;
  double bos;
  double vos;
  double aos;
  double gd_value;
  int gd_type;
  double retry_offset;
  
  double w_smooth;
  double w_collision;
  double w_collision_temp;
  double d0;
  double r;
  double alpha;

  double v0;
  double rv;
  double alphav;
  double a0;
  double ra;
  double alphaa;

  double sgm_time;
  double init_time;
  double mean_v;
};


#endif