#pragma once

#include "../../RISC-V/riscv.h"
#include "VSRTL/core/vsrtl_component.h"

namespace vsrtl {
namespace core {
using namespace Ripes;
using common1sfinal::ALU1Sel;
using common1sfinal::ALU2Sel;

template <unsigned XLEN>
class CS1952y4sHazard : public Component {
public:
  CS1952y4sHazard(std::string name, SimComponent *parent)
      : Component(name, parent) {

    iex_clear << [=] { return 0; };
    exmem_clear << [=] { return 0; };
    memwb_clear << [=] { return 0; };

    pc_enable << [=] { return 1; };
    iex_enable << [=] { return 1; };
    exmem_enable << [=] { return 1; };
    memwb_enable << [=] { return 1; };
  }

  INPUTPORT(rs1_id, 5);
  INPUTPORT(rs2_id, 5);
  INPUTPORT_ENUM(alu1_sel, ALU1Sel);
  INPUTPORT_ENUM(alu2_sel, ALU2Sel);

  INPUTPORT(iex_rd_id, 5);
  INPUTPORT_ENUM(iex_mem_op, MemOp);
  INPUTPORT(exmem_rd_id, 5);
  INPUTPORT_ENUM(exmem_mem_op, MemOp);
  INPUTPORT(memwb_rd_id, 5);

  INPUTPORT(jump_or_branch, 1);

  OUTPUTPORT(iex_clear, 1);
  OUTPUTPORT(exmem_clear, 1);
  OUTPUTPORT(memwb_clear, 1);

  OUTPUTPORT(pc_enable, 1);
  OUTPUTPORT(iex_enable, 1);
  OUTPUTPORT(exmem_enable, 1);
  OUTPUTPORT(memwb_enable, 1);
};

} // namespace core
} // namespace vsrtl
