#pragma once

#include "VSRTL/core/vsrtl_component.h"
#include "VSRTL/core/vsrtl_register.h"

#include "../../RISC-V/riscv.h"

namespace vsrtl {
namespace core {
using namespace Ripes;
using common1sfinal::ALU1Sel;
using common1sfinal::ALU2Sel;
using common1sfinal::RdSel;
using common1sfinal::PCAdd1;

template <unsigned XLEN>
class CS1952yIDEx : public Component {
public:
  CS1952yIDEx(const std::string &name, SimComponent *parent)
      : Component(name, parent) {
    setDescription("Instruction Decode/Execute stage separating register");
    CONNECT_REGISTERED_CLEN_INPUT(pc4, clear, enable); // IF -> WB
    CONNECT_REGISTERED_CLEN_INPUT(pc, clear, enable);  // IF -> Ex

    CONNECT_REGISTERED_CLEN_INPUT(reg1, clear, enable);  // ID -> Ex
    CONNECT_REGISTERED_CLEN_INPUT(reg2, clear, enable);  // ID -> Ex, Mem
    CONNECT_REGISTERED_CLEN_INPUT(imm, clear, enable);   // ID -> Ex, WB
    CONNECT_REGISTERED_CLEN_INPUT(rd_id, clear, enable); // ID -> WB

    CONNECT_REGISTERED_CLEN_INPUT(rs1_id, clear, enable); // for forwarding unit
    CONNECT_REGISTERED_CLEN_INPUT(rs2_id, clear, enable); // for forwarding unit

    CONNECT_REGISTERED_CLEN_INPUT(alu1_sel, clear, enable); // ID -> Ex
    CONNECT_REGISTERED_CLEN_INPUT(alu2_sel, clear, enable); // ID -> Ex
    CONNECT_REGISTERED_CLEN_INPUT(alu_op, clear, enable);   // ID -> Ex
    CONNECT_REGISTERED_CLEN_INPUT(pc_add1, clear, enable);  // ID -> Ex
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
  REGISTERED_CLEN_INPUT(pc, XLEN);
  REGISTERED_CLEN_INPUT(pc4, XLEN);
  REGISTERED_CLEN_INPUT(reg1, XLEN);
  REGISTERED_CLEN_INPUT(reg2, XLEN);
  REGISTERED_CLEN_INPUT(imm, XLEN);
  REGISTERED_CLEN_INPUT(rs1_id, 5);
  REGISTERED_CLEN_INPUT(rs2_id, 5);
  REGISTERED_CLEN_INPUT(rd_id, 5);
  REGISTERED_CLEN_INPUT(alu1_sel, ALU1Sel::width());
  REGISTERED_CLEN_INPUT(alu2_sel, ALU2Sel::width());
  REGISTERED_CLEN_INPUT(alu_op, ALUOp::width());
  REGISTERED_CLEN_INPUT(pc_add1, PCAdd1::width());
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
