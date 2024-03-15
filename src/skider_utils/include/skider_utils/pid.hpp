
/***
 * @file  pid.h
 * @brief PID控制器类
 */

#pragma once

#ifndef SKIDER_PID_HPP
#define SKIDER_PID_HPP

#include <memory>
#include <cstring>
#include <cfloat>

enum class PIDType {
  kPosition,
  kDelta,
};

/***
 * @brief PID控制器
 */
class PID {
 public:
  PID() = default;
  PID(double kp, double ki, double kd);
  PID(PIDType type, double kp, double ki, double kd, double max_out, double max_iout);
  PID(PIDType type, double kp, double ki, double kd, double max_out, double max_iout, double *external_diff_input);
  virtual double update(double set, double ref);
  void clear();
  void switchParameter(double kp, double ki, double kd, double max_out, double max_iout);
  [[nodiscard]] double value() const;

 protected:
  PIDType type_;

  double kp_;
  double ki_;
  double kd_;

  double max_out_{};
  double max_iout_{};

  double set_{};
  double feedback_{};

  double out_{};
  double p_out_{};
  double i_out_{};
  double d_out_{};
  double d_buf_[3]{};  // 0: latest, 1: last, 2: last last
  double error_[3]{};  // 0: latest, 1: last, 2: last last

  // 可以选择使用外部微分量输入
  // 比如在用姿态角数据进行控制时，可以选用陀螺仪数据作为微分量，而不是用PID算法计算出来的D值作为微分量
  double *external_diff_input_;
  bool use_external_diff_input_;
};  // class PID

/***
 * @brief 带过零处理的PID控制器，可以用于电机位置控制
 */
class RingPID : public PID {
 public:
  RingPID() = delete;
  RingPID(PIDType type, double kp, double ki, double kd, double max_out, double max_iout, double cycle);
  double update(double set, double ref) override;

 protected:
  void handleZeroCrossing();

  double cycle_;

};  // class RingPID

/***
 * @brief   限制输入值的绝对值
 * @tparam  T           输入值的类型
 * @param   input       输入值
 * @param   max_value   限制的最大绝对值
 * @return              限制后的值
 */
template <typename T>
static T absConstrain(T input, T max_value) {
  if (input > max_value) {
    return max_value;
  } else if (input < -max_value) {
    return -max_value;
  } else {
    return input;
  }
}

PID::PID(double kp, double ki, double kd)
    : type_(PIDType::kPosition),
      kp_(kp),
      ki_(ki),
      kd_(kd),
      max_out_(DBL_MAX),
      max_iout_(DBL_MAX),
      use_external_diff_input_(false) {}

PID::PID(PIDType type, double kp, double ki, double kd, double max_out, double max_iout)
    : type_(type),
      kp_(kp),
      ki_(ki),
      kd_(kd),
      max_out_(max_out),
      max_iout_(max_iout),
      use_external_diff_input_(false) {}

PID::PID(PIDType type, double kp, double ki, double kd, double max_out, double max_iout, double *external_diff_input)
    : type_(type),
      kp_(kp),
      ki_(ki),
      kd_(kd),
      max_out_(max_out),
      max_iout_(max_iout),
      external_diff_input_(external_diff_input),
      use_external_diff_input_(true) {}

/***
 * @brief   更新PID控制器的状态
 */
double PID::update(double set, double ref) {
  this->error_[2] = this->error_[1];
  this->error_[1] = this->error_[0];

  this->set_ = set;
  this->feedback_ = ref;
  this->error_[0] = set - ref;

  switch (this->type_) {
    case PIDType::kPosition:
      this->p_out_ = this->kp_ * this->error_[0];
      this->i_out_ += this->ki_ * this->error_[0];

      // update derivative term
      this->d_buf_[2] = this->d_buf_[1];
      this->d_buf_[1] = this->d_buf_[0];
      this->d_buf_[0] = use_external_diff_input_
                            ? *(this->external_diff_input_)
                            : (this->error_[0] - this->error_[1]);

      this->d_out_ = this->kd_ * this->d_buf_[0];
      this->i_out_ = absConstrain(this->i_out_, this->max_iout_);
      this->out_ = this->p_out_ + this->i_out_ + this->d_out_;
      this->out_ = absConstrain(this->out_, this->max_out_);
      break;

    case PIDType::kDelta:
      this->p_out_ = this->kp_ * (this->error_[0] - this->error_[1]);
      this->i_out_ = this->ki_ * this->error_[0];

      this->d_buf_[2] = this->d_buf_[1];
      this->d_buf_[1] = this->d_buf_[0];
      this->d_buf_[0] =
          use_external_diff_input_
              ? *(this->external_diff_input_)
              : (this->error_[0] - 2.0f * this->error_[1] + this->error_[2]);

      this->d_out_ = this->kd_ * this->d_buf_[0];
      this->out_ += this->p_out_ + this->i_out_ + this->d_out_;
      this->out_ = absConstrain(this->out_, this->max_out_);
      break;
  }

  return this->out_;
}

/***
 * @brief   清空PID控制器的状态
 */
void PID::clear() {
  this->set_ = 0;
  this->feedback_ = 0;
  this->out_ = 0;
  this->p_out_ = 0;
  this->i_out_ = 0;
  this->d_out_ = 0;
  memset(this->d_buf_, 0, sizeof(this->d_buf_));
  memset(this->error_, 0, sizeof(this->error_));
}

/***
 * @brief   改变PID控制器的参数
 */
void PID::switchParameter(double kp, double ki, double kd, double max_out, double max_iout) {
  this->kp_ = kp;
  this->ki_ = ki;
  this->kd_ = kd;
  this->max_out_ = max_out;
  this->max_iout_ = max_iout;
}

/***
 * @brief   获取PID控制器的输出
 */
double PID::value() const { return this->out_; }

RingPID::RingPID(PIDType type, double kp, double ki, double kd, double max_out, double max_iout, double cycle)
    : PID(type, kp, ki, kd, max_out, max_iout), cycle_(cycle) {}

double RingPID::update(double set, double ref) {
  this->error_[2] = this->error_[1];
  this->error_[1] = this->error_[0];

  this->set_ = set;
  this->feedback_ = ref;
  this->error_[0] = set - ref;

  this->handleZeroCrossing();  // 过零处理

  switch (this->type_) {
    case PIDType::kPosition:
      this->p_out_ = this->kp_ * this->error_[0];
      this->i_out_ += this->ki_ * this->error_[0];

      // update derivative term
      this->d_buf_[2] = this->d_buf_[1];
      this->d_buf_[1] = this->d_buf_[0];
      this->d_buf_[0] = use_external_diff_input_
                            ? *(this->external_diff_input_)
                            : (this->error_[0] - this->error_[1]);

      this->d_out_ = this->kd_ * this->d_buf_[0];
      this->i_out_ = absConstrain(this->i_out_, this->max_iout_);
      this->out_ = this->p_out_ + this->i_out_ + this->d_out_;
      this->out_ = absConstrain(this->out_, this->max_out_);
      break;

    case PIDType::kDelta:
      this->p_out_ = this->kp_ * (this->error_[0] - this->error_[1]);
      this->i_out_ = this->ki_ * this->error_[0];

      this->d_buf_[2] = this->d_buf_[1];
      this->d_buf_[1] = this->d_buf_[0];
      this->d_buf_[0] =
          use_external_diff_input_
              ? *(this->external_diff_input_)
              : (this->error_[0] - 2.0f * this->error_[1] + this->error_[2]);

      this->d_out_ = this->kd_ * this->d_buf_[0];
      this->out_ += this->p_out_ + this->i_out_ + this->d_out_;
      this->out_ = absConstrain(this->out_, this->max_out_);
      break;
  }

  return this->out_;
}

/***
 * @brief   过零处理
 * @brief   当误差跨越了周期的一半时，就进行过零处理
 * @brief   误差大于周期的一半时，减去一个周期
 * @brief   误差小于周期的一半时，加上一个周期
 */
void RingPID::handleZeroCrossing() {
  if (this->error_[0] > (this->cycle_ / 2)) {
    this->error_[0] -= this->cycle_;
  } else if (this->error_[0] < (-this->cycle_ / 2)) {
    this->error_[0] += this->cycle_;
  }
}

#endif  // SKIDER_PID_H

/* EOF */
