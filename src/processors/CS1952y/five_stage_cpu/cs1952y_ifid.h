#pragma once

#include "VSRTL/core/vsrtl_component.h"
#include "VSRTL/core/vsrtl_register.h"

#include "../../RISC-V/riscv.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class CS1952yIFID : public Component {
public:
  CS1952yIFID(const std::string &name, SimComponent *parent)
      : Component(name, parent) {
    setDescription(
        "Instruction fetch/Instruction Decode stage separating register");
    CONNECT_REGISTERED_CLEN_INPUT(pc4, clear, enable);   // IF -> WB
    CONNECT_REGISTERED_CLEN_INPUT(pc, clear, enable);    // IF -> Ex
    CONNECT_REGISTERED_CLEN_INPUT(instr, clear, enable); // IF -> ID

    CONNECT_REGISTERED_CLEN_INPUT(valid, clear, enable);
  }

  // Data
  REGISTERED_CLEN_INPUT(pc, XLEN);
  REGISTERED_CLEN_INPUT(pc4, XLEN);
  REGISTERED_CLEN_INPUT(instr, XLEN);

  // Register bank controls
  INPUTPORT(enable, 1);
  INPUTPORT(clear, 1);

  // Valid signal. False when the register bank has been cleared. May be used by
  // UI to determine whether the NOP in the stage is a user-inserted nop or the
  // result of some pipeline action.
  REGISTERED_CLEN_INPUT(valid, 1);
};

} // namespace core
} // namespace vsrtl
