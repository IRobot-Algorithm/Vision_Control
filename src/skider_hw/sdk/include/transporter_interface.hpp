#pragma once

#include <memory>
#include <string>

namespace transporter_sdk {

class TransporterInterface {
 public:
  virtual bool open() = 0;
  virtual bool close() = 0;
  virtual int read(unsigned char* buffer, size_t len) = 0;
  virtual int write(unsigned char* buffer, size_t len) = 0;
};

}  // namespace transporter_sdk
