#pragma once

#include "VSRTL/core/vsrtl_component.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

class BoundsCheck : public Component {
public:
  BoundsCheck(std::string name, SimComponent *parent)
      : Component(name, parent) {

    // in_val.uValue() gives the unsigned value of in_val
    res_val << [=] { return in_val.uValue(); };

    overflow << [=] { return 0; };
  }

  /*
   * Any helper functions go here:
   */

  INPUTPORT(in_val, 32); // 32-bit input value

  OUTPUTPORT(res_val, 32); // 32-bit result value
  OUTPUTPORT(overflow, 1); // 1-bit overflow control signal
};

} // namespace core
} // namespace vsrtl
