/*
 * @Description:
 * @Version: 1.0
 * @Autor: Qylann
 * @Date: 2022-03-29 21:48:29
 * @LastEditors: Qylann
 * @LastEditTime: 2022-04-05 13:18:54
 */
#pragma once

#include "remote_interface.hpp"

namespace sensor_sdk {
namespace remote {

typedef struct SbusFrame {
  float channal_rx;
  float channal_ry;
  float channal_lx;
  float channal_ly;
  float channal_dial;
  float mouse_x;
  float mouse_y;
  float mouse_z;

  bool switch_rdown;
  bool switch_rmid;
  bool switch_rup;
  bool switch_ldown;
  bool switch_lmid;
  bool switch_lup;

  bool mouse_click_left;
  bool mouse_click_right;
  bool keypress_w;
  bool keypress_s;
  bool keypress_a;
  bool keypress_d;
  bool keypress_shift;
  bool keypress_ctrl;
  bool keypress_q;
  bool keypress_e;
  bool keypress_r;
  bool keypress_f;
  bool keypress_g;
  bool keypress_z;
  bool keypress_x;
  bool keypress_c;
  bool keypress_v;
  bool keypress_b;
} SbusFrame;

enum KEYPRESS_OFFSET {
  W = ((uint16_t)1 << 0),
  S = ((uint16_t)1 << 1),
  A = ((uint16_t)1 << 2),
  D = ((uint16_t)1 << 3),
  SHIFT = ((uint16_t)1 << 4),
  CTRL = ((uint16_t)1 << 5),
  Q = ((uint16_t)1 << 6),
  E = ((uint16_t)1 << 7),
  R = ((uint16_t)1 << 8),
  F = ((uint16_t)1 << 9),
  G = ((uint16_t)1 << 10),
  Z = ((uint16_t)1 << 11),
  X = ((uint16_t)1 << 12),
  C = ((uint16_t)1 << 13),
  V = ((uint16_t)1 << 14),
  B = ((uint16_t)1 << 15),
};

class SbusRemote : public RemoteBase {
 public:
  void update(unsigned char *buffer) override;
  void process() override;

 private:
  unsigned char sbus_buff_[18];
  SbusFrame sbus_frame_;
};

}  // namespace remote

}  // namespace sensor_sdk