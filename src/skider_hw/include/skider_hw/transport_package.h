/*
 * @Description:
 * @Version: 1.0
 * @Autor: Qylann
 * @Date: 2022-04-05 16:03:01
 * @LastEditors: Qylann
 * @LastEditTime: 2022-04-05 16:06:21
 */

#ifndef TRANSPORT_PACKAGE_H
#define TRANSPORT_PACKAGE_H

namespace transport_package {

typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef signed short int16_t;
typedef unsigned short uint16_t;
typedef signed int int32_t;
typedef unsigned int uint32_t;

#pragma pack(push, 1)

typedef struct
{
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t accl_x;
  int16_t accl_y;
  int16_t accl_z;
} IMUMessage_t;

typedef struct
{
  // Real Time
  uint16_t ammo0_current_heat;
  uint16_t ammo1_current_heat;
  float ammo0_last_speed;
  float ammo1_last_speed;

  // Static
  uint16_t ammo0_cooling;
  uint16_t ammo1_cooling;
  uint16_t ammo0_max_heat;
  uint16_t ammo1_max_heat;
  uint8_t ammo0_max_speed;
  uint8_t ammo1_max_speed;

  // Robot Information
  uint8_t self_inf;
  uint8_t power_supply;
  uint8_t blood_remain;
  uint8_t blood_max;

} RefereeMessage_t;

typedef struct GimbalHWReceivePackage {
  uint8_t HeaderFrame;
  IMUMessage_t imu;
  uint8_t sbus[18];
  uint8_t EndFrame;
} GimbalHWReceivePackage;

#pragma pack(pop)

}  // namespace transport_package

#endif
