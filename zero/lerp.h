#ifndef LERP_H
#define LERP_H

#include "math.h"

#define MAX_BUFFER_SIZE 100

struct buf_t {
  int buf_type[MAX_BUFFER_SIZE];
  double buf_ts[MAX_BUFFER_SIZE];
  vec3_t buf_data[MAX_BUFFER_SIZE];
  size_t size;
};

struct lerp_imu_t {
  buf_t buf;

  double gyr_ts[MAX_BUFFER_SIZE] = {0};
  vec3_t gyr_data[MAX_BUFFER_SIZE];
  double acc_ts[MAX_BUFFER_SIZE] = {0};
  vec3_t acc_data[MAX_BUFFER_SIZE];
};

void lerp_imu_init(lerp_imu_t *lerp) {
  /* lerp->buf; */

  /* lerp->gyr_ts; */
  /* lerp->gyr_data; */
  /* lerp->acc_ts; */
  /* lerp->acc_data; */
}

int lerp_imu_ready(const lerp_imu_t *lerp) {
  /* if (buf->buf_ts >= 3 && buf_type == 0) { */
  /*   return 1; */
  /* } */
  return 0;
}

void lerp_imu_add_accel(lerp_imu_t *lerp, vec3_t *accel) {
/*     buf_type_.push_back("A"); */
/*     buf_ts_.push_back(msg.header.stamp.toSec()); */
/*     buf_data_.emplace_back(msg.vector.x, msg.vector.y, msg.vector.z); */
}

void lerp_imu_add_gyro(lerp_imu_t *lerp, vec3_t *gyro) {
/*     if (buf_type_.size() && buf_type_.front() == "A") { */
/*       buf_type_.push_back("G"); */
/*       buf_ts_.push_back(msg.header.stamp.toSec()); */
/*       buf_data_.emplace_back(msg.vector.x, msg.vector.y, msg.vector.z); */
/*     } */
}

void lerp_imu_print(lerp_imu_t *lerp) {
  /* for (size_t i = 0; i < buf_ts_.size(); i++) { */
  /*   const double ts = buf_ts_.at(i); */
  /*   const int dtype = buf_type_.at(i); */
  /*   const vec3_t data = buf_data_.at(i); */
  /*   const double x = data(0); */
  /*   const double y = data(1); */
  /*   const double z = data(1); */
  /*   printf("[%.6f] - [%s] - (%.2f, %.2f, %.2f)\n", ts, dtype.c_str(), x, y, */
  /*           z); */
  /* } */
}

void lerp_imu_eval() {
  // Lerp data
  double t0 = 0;
  vec3_t d0;
  double t1 = 0;
  vec3_t d1;
  bool t0_set = false;

  /* std::deque<double> lerp_ts; */
  /* std::deque<vec3_t> lerp_data; */

  double ts = 0.0;
  int dtype = 0;
  vec3_t data;

  /* while (buf_ts_.size()) { */
  /*   // Timestamp */
  /*   ts = buf_ts_.front(); */
  /*   buf_ts_.pop_front(); */
  /*  */
  /*   // Datatype */
  /*   dtype = buf_type_.front(); */
  /*   buf_type_.pop_front(); */
  /*  */
  /*   // Data */
  /*   data = buf_data_.front(); */
  /*   buf_data_.pop_front(); */
  /*  */
  /*   // Lerp */
  /*   if (t0_set == false && dtype == "A") { */
  /*     t0 = ts; */
  /*     d0 = data; */
  /*     t0_set = true; */
  /*  */
  /*   } else if (t0_set && dtype == "A") { */
  /*     t1 = ts; */
  /*     d1 = data; */
  /*  */
  /*     while (lerp_ts.size()) { */
  /*       const double lts = lerp_ts.front(); */
  /*       const vec3_t ldata = lerp_data.front(); */
  /*       const double dt = t1 - t0; */
  /*       const double alpha = (lts - t0) / dt; */
  /*  */
  /*       lerped_accel_ts_.push_back(lts); */
  /*       lerped_accel_data_.push_back(lerp(d0, d1, alpha)); */
  /*  */
  /*       lerped_gyro_ts_.push_back(lts); */
  /*       lerped_gyro_data_.push_back(ldata); */
  /*  */
  /*       lerp_ts.pop_front(); */
  /*       lerp_data.pop_front(); */
  /*     } */
  /*  */
  /*     t0 = t1; */
  /*     d0 = d1; */
  /*  */
  /*   } else if (t0_set && ts >= t0 && dtype == "G") { */
  /*     lerp_ts.push_back(ts); */
  /*     lerp_data.push_back(data); */
  /*   } */
  /* } */

  /* buf_ts_.push_back(ts); */
  /* buf_type_.push_back(dtype); */
  /* buf_data_.push_back(data); */
}

#endif  // LERP_H
