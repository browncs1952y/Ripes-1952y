#pragma once

#include "VSRTL/core/vsrtl_component.h"
#include "VSRTL/core/vsrtl_register.h"

#include "../../RISC-V/riscv.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class CS1952yMemMem : public Component {
public:
  CS1952yMemMem (const std::string &name, SimComponent *parent)
      : Component(name, parent) {
    setDescription("Execute/Memory stage separating register");
    CONNECT_REGISTERED_CLEN_INPUT(pc4, clear, enable); // IF -> WB
    CONNECT_REGISTERED_CLEN_INPUT(imm, clear, enable); // ID -> WB
    CONNECT_REGISTERED_CLEN_INPUT(rd_id, clear, enable); // ID -> WB
    CONNECT_REGISTERED_CLEN_INPUT(alu_res, clear, enable); // Ex -> WB
    
    CONNECT_REGISTERED_CLEN_INPUT(rd_sel, clear, enable); // ID -> WB
    CONNECT_REGISTERED_CLEN_INPUT(reg_wr, clear, enable); // ID -> WB
    
    CONNECT_REGISTERED_CLEN_INPUT(ecall, clear, enable);
    
    CONNECT_REGISTERED_CLEN_INPUT(valid, clear, enable);
  }

  // Data
  REGISTERED_CLEN_INPUT(pc4, XLEN);
  REGISTERED_CLEN_INPUT(imm, XLEN);
  REGISTERED_CLEN_INPUT(rd_id, 5);
  
  REGISTERED_CLEN_INPUT(alu_res, XLEN);
  
  REGISTERED_CLEN_INPUT(rd_sel, RdSel::width());
  REGISTERED_CLEN_INPUT(reg_wr, 1);
  
  REGISTERED_CLEN_INPUT(ecall, RVInstr::width());
  
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