#include "sbus_remote.hpp"

namespace sensor_sdk {

namespace remote {

void SbusRemote::update(unsigned char *buffer) {
  memcpy(sbus_buff_, buffer, 18);
}

void SbusRemote::process() {
  sbus_frame_.channal_rx = (((sbus_buff_[0] | (sbus_buff_[1] << 8)) & 0x07ff) - 1024) / 660.0;
  sbus_frame_.channal_ry = ((((sbus_buff_[1] >> 3) | (sbus_buff_[2] << 5)) & 0x07ff) - 1024) / 660.0;
  sbus_frame_.channal_lx = ((((sbus_buff_[2] >> 6) | (sbus_buff_[3] << 2) | (sbus_buff_[4] << 10)) & 0x07ff) - 1024) / 660.0;
  sbus_frame_.channal_ly = ((((sbus_buff_[4] >> 1) | (sbus_buff_[5] << 7)) & 0x07ff) - 1024) / 660.0;
  sbus_frame_.channal_dial = ((sbus_buff_[16] | (sbus_buff_[17] << 8)) - 1024) / 660.0;

  uint switch_right = ((sbus_buff_[5] >> 4) & 0x0003);
  switch (switch_right) {
    case 1:
      sbus_frame_.switch_rup = true;
      sbus_frame_.switch_rmid = false;
      sbus_frame_.switch_rdown = false;
      break;
    case 3:
      sbus_frame_.switch_rup = false;
      sbus_frame_.switch_rmid = true;
      sbus_frame_.switch_rdown = false;
      break;
    case 2:
      sbus_frame_.switch_rup = false;
      sbus_frame_.switch_rmid = false;
      sbus_frame_.switch_rdown = true;
      break;
    default:
      sbus_frame_.switch_rup = false;
      sbus_frame_.switch_rmid = false;
      sbus_frame_.switch_rdown = false;
      break;
  }
  uint switch_left = ((sbus_buff_[5] >> 4) & 0x000C) >> 2;
  switch (switch_left) {
    case 1:
      sbus_frame_.switch_lup = true;
      sbus_frame_.switch_lmid = false;
      sbus_frame_.switch_ldown = false;
      break;
    case 3:
      sbus_frame_.switch_lup = false;
      sbus_frame_.switch_lmid = true;
      sbus_frame_.switch_ldown = false;
      break;
    case 2:
      sbus_frame_.switch_lup = false;
      sbus_frame_.switch_lmid = false;
      sbus_frame_.switch_ldown = true;
      break;
    default:
      sbus_frame_.switch_lup = false;
      sbus_frame_.switch_lmid = false;
      sbus_frame_.switch_ldown = false;
      break;
  }
  sbus_frame_.mouse_x = (sbus_buff_[6] | (sbus_buff_[7] << 8)) / 32768.0;
  sbus_frame_.mouse_y = (sbus_buff_[8] | (sbus_buff_[9] << 8)) / 32768.0;
  sbus_frame_.mouse_z = (sbus_buff_[10] | (sbus_buff_[11] << 8)) / 32768.0;
  sbus_frame_.mouse_click_left = sbus_buff_[12];
  sbus_frame_.mouse_click_right = sbus_buff_[13];

  uint16_t keypress = sbus_buff_[14] | (sbus_buff_[15] << 8);
  sbus_frame_.keypress_w = keypress & KEYPRESS_OFFSET::W;
  sbus_frame_.keypress_s = keypress & KEYPRESS_OFFSET::S;
  sbus_frame_.keypress_a = keypress & KEYPRESS_OFFSET::A;
  sbus_frame_.keypress_d = keypress & KEYPRESS_OFFSET::D;
  sbus_frame_.keypress_shift = keypress & KEYPRESS_OFFSET::SHIFT;
  sbus_frame_.keypress_ctrl = keypress & KEYPRESS_OFFSET::CTRL;
  sbus_frame_.keypress_q = keypress & KEYPRESS_OFFSET::Q;
  sbus_frame_.keypress_e = keypress & KEYPRESS_OFFSET::E;
  sbus_frame_.keypress_r = keypress & KEYPRESS_OFFSET::R;
  sbus_frame_.keypress_f = keypress & KEYPRESS_OFFSET::F;
  sbus_frame_.keypress_g = keypress & KEYPRESS_OFFSET::G;
  sbus_frame_.keypress_z = keypress & KEYPRESS_OFFSET::Z;
  sbus_frame_.keypress_x = keypress & KEYPRESS_OFFSET::X;
  sbus_frame_.keypress_c = keypress & KEYPRESS_OFFSET::C;
  sbus_frame_.keypress_v = keypress & KEYPRESS_OFFSET::V;
  sbus_frame_.keypress_b = keypress & KEYPRESS_OFFSET::B;

  axes_size = 8;
  axes.clear();
  axes.push_back(sbus_frame_.channal_rx);
  axes.push_back(sbus_frame_.channal_ry);
  axes.push_back(sbus_frame_.channal_lx);
  axes.push_back(sbus_frame_.channal_ly);
  axes.push_back(sbus_frame_.channal_dial);
  axes.push_back(sbus_frame_.mouse_x);
  axes.push_back(sbus_frame_.mouse_y);
  axes.push_back(sbus_frame_.mouse_z);

  buttons_size = 24;
  buttons.clear();
  buttons.push_back(sbus_frame_.switch_rdown);
  buttons.push_back(sbus_frame_.switch_rmid);
  buttons.push_back(sbus_frame_.switch_rup);
  buttons.push_back(sbus_frame_.switch_ldown);
  buttons.push_back(sbus_frame_.switch_lmid);
  buttons.push_back(sbus_frame_.switch_lup);
  buttons.push_back(sbus_frame_.mouse_click_left);
  buttons.push_back(sbus_frame_.mouse_click_right);
  buttons.push_back(sbus_frame_.keypress_w);
  buttons.push_back(sbus_frame_.keypress_s);
  buttons.push_back(sbus_frame_.keypress_a);
  buttons.push_back(sbus_frame_.keypress_d);
  buttons.push_back(sbus_frame_.keypress_shift);
  buttons.push_back(sbus_frame_.keypress_ctrl);
  buttons.push_back(sbus_frame_.keypress_q);
  buttons.push_back(sbus_frame_.keypress_e);
  buttons.push_back(sbus_frame_.keypress_r);
  buttons.push_back(sbus_frame_.keypress_f);
  buttons.push_back(sbus_frame_.keypress_g);
  buttons.push_back(sbus_frame_.keypress_z);
  buttons.push_back(sbus_frame_.keypress_x);
  buttons.push_back(sbus_frame_.keypress_c);
  buttons.push_back(sbus_frame_.keypress_v);
  buttons.push_back(sbus_frame_.keypress_b);

  // memset(sbus_buff_, 0, 18);
}

}  // namespace remote

}  // namespace sensor_sdk
