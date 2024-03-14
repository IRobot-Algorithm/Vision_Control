/*
 * @Description:
 * @Version: 1.0
 * @Autor: Qylann
 * @Date: 2022-03-29 22:25:40
 * @LastEditors: Qylann
 * @LastEditTime: 2022-04-05 12:40:53
 */
#pragma once

#include "ahrs_interface.hpp"

namespace sensor_sdk {

namespace imu {

class MahonyAHRS : public AHRSInterface {
 public:
  MahonyAHRS(double kp, double ki, double sampling_frequency)
      : kp_(kp),
        ki_(ki),
        sampling_frequency_(sampling_frequency) {}
  void init() override;
  void calculate() override;

 private:
  uint sampling_frequency_ = 1000;
  double kp_ = 2.0 * 0.5;
  double ki_ = 2.0 * 0.0;
};

}  // namespace imu

}  // namespace sensor_sdk
