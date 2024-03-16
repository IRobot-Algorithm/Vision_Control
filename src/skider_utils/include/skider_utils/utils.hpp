/***
 * @file  utils.hpp
 * @brief 一些常用的工具函数
 */

#pragma once

#ifndef SKIDER_UTILS_HPP
#define SKIDER_UTILS_HPP

#include <cmath>

// enum ChassisState{
//     ChassisWeak = 0,
//     ChassisJoy = 1,
// };

// enum GimbalState{
//     GimbalWeak = 0,
//     GimbalJoy = 1,
//     GimbalRotor = 2
// };

enum class RobotState {
  ChassisWeakGimbalWeak = 0,
  ChassisWeakGimbalJoy = 1,
  ChassisWeakGimbalJoyRotor = 2,
  ChassisJoyGimbalWeak = 3,
  ChassisJoyGimbalJoy = 4,
  ChassisJoyGimbalJoyRotor = 5,
  AutoaimGimbalWeak = 6,
  AutoaimGimbalAutoaim = 7,
  AutoaimGimbalAutoaimRotor = 8,
};

namespace utils {

/***
 * @brief  符号函数
 * @param  value       输入值
 * @return             输入正数返回1，负数返回-1，0返回0
 */
int sign(double value) {
  if (value > 0) {
    return 1;
  } else if (value < 0) {
    return -1;
  } else {
    return 0;
  }
}

/***
 * @param  value       输入值
 * @param  min_value   下限
 * @param  max_value   上限
 * @return             若输入值在范围内则返回原值，否则返回0
 */
double deadline(double value, double min_value, double max_value) {
  if (value < min_value || value > max_value) {
    return 0;
  } else {
    return value;
  }
}

/***
 * @brief  把输入值限制在一个范围内
 * @param  input       输入值
 * @param  min_value   下限
 * @param  max_value   上限
 * @return             若输入值在范围内则返回原值，否则返回最近的边界值
 */
double constrain(double input, double min_value, double max_value) {
  if (input < min_value) {
    return min_value;
  } else if (input > max_value) {
    return max_value;
  } else {
    return input;
  }
}

/***
 * @brief   把输入值限制在一个周期内
 * @param   input       输入值
 * @param   min_value   周期下限
 * @param   max_value   周期上限
 * @return              若输入值超出周期范围，则返回限制后的值，否则返回原值
 */
double loopConstrain(double input, double min_value, double max_value) {
  double cycle = max_value - min_value;
  if (cycle < 0) {
    return input;
  }

  if (input > max_value) {
    while (input > max_value) {
      input -= cycle;
    }
  } else if (input < min_value) {
    while (input < min_value) {
      input += cycle;
    }
  }
  return input;
}

/***
 * @brief   限制输入值的绝对值
 * @param   input       输入值
 * @param   max_value   限制的最大绝对值
 * @return              限制后的值
 */
double absConstrain(double input, double max_value) {
  if (input > max_value) {
    return max_value;
  } else if (input < -max_value) {
    return -max_value;
  } else {
    return input;
  }
}

/***
 * @brief 角度转弧度
 * @param deg       角度
 * @return          弧度
 */
double degToRad(double deg) {
  return deg * M_PI / 180;
}

/***
 * @brief      四元数转欧拉角
 * @param[in]  q         四元数
 * @param[out] euler     欧拉角
 */
void quatToEuler(const double q[4], double euler[3]) {
  euler[0] = atan2f(2 * (q[0] * q[1] + q[2] * q[3]),
                    1 - 2 * (q[1] * q[1] + q[2] * q[2]));
  euler[1] = asinf(2 * (q[0] * q[2] - q[3] * q[1]));
  euler[2] = atan2f(2 * (q[0] * q[3] + q[1] * q[2]),
                    1 - 2 * (q[2] * q[2] + q[3] * q[3]));
}

}  // namespace utils

#endif  // SKIDER_UTILS_HPP

/* EOF */
