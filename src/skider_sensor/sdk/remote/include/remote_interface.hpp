/*
 * @Description:
 * @Version: 1.0
 * @Autor: Qylann
 * @Date: 2022-03-29 20:58:52
 * @LastEditors: Qylann
 * @LastEditTime: 2022-04-05 13:15:30
 */
#pragma once

#include <memory.h>
#include <iostream>
#include <string>
#include <vector>

namespace sensor_sdk {

namespace remote {
class RemoteBase {
 public:
  virtual void update(unsigned char *buffer) = 0;
  virtual void process() = 0;

  uint axes_size;
  std::vector<float> axes;
  uint buttons_size;
  std::vector<bool> buttons;
};

}  // namespace remote

}  // namespace sensor_sdk
