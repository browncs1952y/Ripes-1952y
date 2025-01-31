#pragma once

#include "../../RISC-V/riscv.h"
#include "VSRTL/core/vsrtl_component.h"

namespace vsrtl {
namespace core {
using namespace Ripes;
using common1sfinal::ALU1Sel;
using common1sfinal::ALU2Sel;

template <unsigned XLEN>
class CS1952y6sHazard : public Component {
public:
  CS1952y6sHazard(std::string name, SimComponent *parent)
      : Component(name, parent) {
    ifid_clear << [=] { return 0; };
    idrr_clear << [=] { return 0; };
    rrex_clear << [=] { return 0; };
    exmem_clear << [=] { return 0; };

    if_enable << [=] { return 1; };
    idrr_enable << [=] { return 1; };
    rrex_enable << [=] { return 1; };
    exmem_enable << [=] { return 1; };
  }

  /* Use these inputs ONLY IF you decide hazard unit is in the ID stage */
  INPUTPORT(id_rs1_id, 5);
  INPUTPORT(id_rs2_id, 5);
  INPUTPORT_ENUM(id_rs1_sel, ALU1Sel);
  INPUTPORT_ENUM(id_rs2_sel, ALU2Sel);

  /* Use these inputs ONLY IF you decide hazard unit is in the RR stage */
  INPUTPORT(rr_rs1_id, 5);
  INPUTPORT(rr_rs2_id, 5);
  INPUTPORT_ENUM(rr_rs1_sel, ALU1Sel);
  INPUTPORT_ENUM(rr_rs2_sel, ALU2Sel);

  INPUTPORT(idrr_rd_id, 5);
  INPUTPORT(rrex_rd_id, 5);
  INPUTPORT(exmem_rd_id, 5);

  INPUTPORT_ENUM(idrr_mem_op, MemOp);
  INPUTPORT_ENUM(rrex_mem_op, MemOp);
  INPUTPORT_ENUM(exmem_mem_op, MemOp);

  INPUTPORT(jump_or_branch, 1);

  OUTPUTPORT(ifid_clear, 1);
  OUTPUTPORT(idrr_clear, 1);
  OUTPUTPORT(rrex_clear, 1);
  OUTPUTPORT(exmem_clear, 1);

  OUTPUTPORT(if_enable, 1);
  OUTPUTPORT(idrr_enable, 1);
  OUTPUTPORT(rrex_enable, 1);
  OUTPUTPORT(exmem_enable, 1);
};

} // namespace core
} // namespace vsrtl
