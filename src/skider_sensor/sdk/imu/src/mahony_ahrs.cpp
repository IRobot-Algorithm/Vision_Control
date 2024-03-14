/*
 * @Description:
 * @Version: 1.0
 * @Autor: Qylann
 * @Date: 2022-03-30 14:11:36
 * @LastEditors: Qylann
 * @LastEditTime: 2022-04-05 12:41:48
 */
#include "mahony_ahrs.hpp"

namespace sensor_sdk {

namespace imu {

void MahonyAHRS::init() {
}

void MahonyAHRS::calculate() {
  static Eigen::Vector3d accl_integral_bias_(0, 0, 0);

  Eigen::Vector4d quat_(Quat_.w(), Quat_.x(), Quat_.y(), Quat_.z());
  Eigen::Vector3d gyro_(imu_raw_.gyro_x, imu_raw_.gyro_y + 0.075, imu_raw_.gyro_z);
  Eigen::Vector3d accl_(imu_raw_.accl_x, imu_raw_.accl_y, imu_raw_.accl_z);
  Eigen::Vector3d accl_hat_(
      quat_(1) * quat_(3) - quat_(0) * quat_(2),
      quat_(0) * quat_(1) + quat_(2) * quat_(3),
      quat_(0) * quat_(0) + quat_(3) * quat_(3) - 0.5);
  Eigen::Vector3d accl_bias_(0, 0, 0);

  if (!((accl_(0) == 0.0f) && (accl_(1) == 0.0f) && (accl_(2) == 0.0f))) {
    accl_ /= sqrt(accl_.transpose() * accl_);
    accl_bias_ = accl_.cross(accl_hat_);
  }

  accl_integral_bias_ += accl_bias_;

  gyro_ += kp_ * accl_bias_ + ki_ * accl_integral_bias_;

  // 四元数更新
  Eigen::Vector4d dalta_quat_(
      0.5 * (-gyro_(0) * quat_(1) - gyro_(1) * quat_(2) - gyro_(2) * quat_(3)),
      0.5 * (gyro_(0) * quat_(0) - gyro_(1) * quat_(3) + gyro_(2) * quat_(2)),
      0.5 * (gyro_(0) * quat_(3) + gyro_(1) * quat_(0) - gyro_(2) * quat_(1)),
      0.5 * (-gyro_(0) * quat_(2) + gyro_(1) * quat_(1) + gyro_(2) * quat_(0)));
  quat_ += (1.0 / sampling_frequency_) * dalta_quat_;

  // 四元数归一化
  Quat_.w() = quat_(0);
  Quat_.x() = quat_(1);
  Quat_.y() = quat_(2);
  Quat_.z() = quat_(3);
  Quat_.normalize();
}

}  // namespace imu

}  // namespace sensor_sdk
