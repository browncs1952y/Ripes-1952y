#pragma once

#include "../../RISC-V/riscv.h"
#include "VSRTL/core/vsrtl_component.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class CS1952y6sHazard : public Component {
public:
  CS1952y6sHazard(std::string name, SimComponent *parent)
      : Component(name, parent) {

    ifid_clear << [=] { return 0; };
    idex_clear << [=] { return 0; };
    exm1_clear << [=] { return 0; };
    m1m2_clear << [=] { return 0; };
    m2wb_clear << [=] { return 0; };

    pc_enable << [=] { return 1; };
    ifid_enable << [=] { return 1; };
    idex_enable << [=] { return 1; };
    exm1_enable << [=] { return 1; };
    m1m2_enable << [=] { return 1; };
    m2wb_enable << [=] { return 1; };

  }

  INPUTPORT(rs1_id, 5);
  INPUTPORT(rs2_id, 5);
  INPUTPORT_ENUM(alu1_sel, ALU1Sel);
  INPUTPORT_ENUM(alu2_sel, ALU2Sel);

  INPUTPORT(idex_rd_id, 5);
  INPUTPORT_ENUM(idex_mem_op, MemOp);
  INPUTPORT(exm1_rd_id, 5);
  INPUTPORT_ENUM(exm1_mem_op, MemOp);
  INPUTPORT(m1m2_rd_id, 5);
  INPUTPORT(m2wb_rd_id, 5);

  INPUTPORT(jump_or_branch, 1);

  OUTPUTPORT(ifid_clear, 1);
  OUTPUTPORT(idex_clear, 1);
  OUTPUTPORT(exm1_clear, 1);
  OUTPUTPORT(m1m2_clear, 1);
  OUTPUTPORT(m2wb_clear, 1);

  OUTPUTPORT(pc_enable, 1);
  OUTPUTPORT(ifid_enable, 1);
  OUTPUTPORT(idex_enable, 1);
  OUTPUTPORT(exm1_enable, 1);
  OUTPUTPORT(m1m2_enable, 1);
  OUTPUTPORT(m2wb_enable, 1);
};

} // namespace core
} // namespace vsrtl
