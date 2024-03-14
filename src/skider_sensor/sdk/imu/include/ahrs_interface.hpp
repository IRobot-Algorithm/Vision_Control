/*
 * @Description:
 * @Version: 1.0
 * @Autor: Qylann
 * @Date: 2022-03-29 22:25:11
 * @LastEditors: Qylann
 * @LastEditTime: 2022-04-05 13:17:52
 */
#pragma once

#include <iostream>

#include <Eigen/Eigen>

namespace sensor_sdk {

namespace imu {

typedef struct ImuRawSixAxis {
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float accl_x;
  float accl_y;
  float accl_z;
} ImuRawSixAxis;

class AHRSInterface {
 public:
  Eigen::Quaterniond Quat_;
  ImuRawSixAxis imu_raw_;

  AHRSInterface() {
    Quat_.w() = 1.0;
    Quat_.x() = 0.0;
    Quat_.y() = 0.0;
    Quat_.z() = 0.0;
  }

  virtual void init() = 0;
  void update(const ImuRawSixAxis& imu_raw) {
    memcpy(&imu_raw_, &imu_raw, sizeof(ImuRawSixAxis));
  }
  virtual void calculate() = 0;
};

}  // namespace imu

}  // namespace sensor_sdk
