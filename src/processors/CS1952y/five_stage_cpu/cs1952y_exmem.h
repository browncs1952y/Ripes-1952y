#pragma once

#include "VSRTL/core/vsrtl_component.h"
#include "VSRTL/core/vsrtl_register.h"

#include "../../RISC-V/riscv.h"

namespace vsrtl {
namespace core {
using namespace Ripes;
using common1sfinal::RdSel;

template <unsigned XLEN>
class CS1952yExMem : public Component {
public:
  CS1952yExMem(const std::string &name, SimComponent *parent)
      : Component(name, parent) {
    setDescription("Execute/Memory stage separating register");
    CONNECT_REGISTERED_CLEN_INPUT(pc4, clear, enable);   // IF -> WB
    CONNECT_REGISTERED_CLEN_INPUT(reg2, clear, enable);  // ID -> Mem
    CONNECT_REGISTERED_CLEN_INPUT(imm, clear, enable);   // ID -> WB
    CONNECT_REGISTERED_CLEN_INPUT(rd_id, clear, enable); // ID -> WB

    CONNECT_REGISTERED_CLEN_INPUT(alu_res, clear, enable);  // Ex -> Mem, WB
    CONNECT_REGISTERED_CLEN_INPUT(alu_zero, clear, enable); // Ex -> Mem
    CONNECT_REGISTERED_CLEN_INPUT(jb_pc, clear, enable);    // Ex -> Mem

    CONNECT_REGISTERED_CLEN_INPUT(mem_wr, clear, enable);   // ID -> Mem
    CONNECT_REGISTERED_CLEN_INPUT(mem_op, clear, enable);   // ID -> Mem
    CONNECT_REGISTERED_CLEN_INPUT(jump, clear, enable);     // ID -> Mem
    CONNECT_REGISTERED_CLEN_INPUT(branch, clear, enable);   // ID -> Mem
    CONNECT_REGISTERED_CLEN_INPUT(inv_zero, clear, enable); // ID -> Mem
    CONNECT_REGISTERED_CLEN_INPUT(rd_sel, clear, enable);   // ID -> WB
    CONNECT_REGISTERED_CLEN_INPUT(reg_wr, clear, enable);   // ID -> WB

    CONNECT_REGISTERED_CLEN_INPUT(ecall, clear, enable);

    CONNECT_REGISTERED_CLEN_INPUT(valid, clear, enable);
  }

  // Data
  REGISTERED_CLEN_INPUT(pc4, XLEN);
  REGISTERED_CLEN_INPUT(reg2, XLEN);
  REGISTERED_CLEN_INPUT(imm, XLEN);
  REGISTERED_CLEN_INPUT(rd_id, 5);

  REGISTERED_CLEN_INPUT(alu_res, XLEN);
  REGISTERED_CLEN_INPUT(alu_zero, 1);
  REGISTERED_CLEN_INPUT(jb_pc, XLEN);

  REGISTERED_CLEN_INPUT(mem_wr, 1);
  REGISTERED_CLEN_INPUT(mem_op, MemOp::width());
  REGISTERED_CLEN_INPUT(jump, 1);
  REGISTERED_CLEN_INPUT(branch, 1);
  REGISTERED_CLEN_INPUT(inv_zero, 1);
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
