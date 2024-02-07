#pragma once

#include "../../RISC-V/riscv.h"
#include "VSRTL/core/vsrtl_component.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class CS1952y5sHazard : public Component {
public:
  CS1952y5sHazard(std::string name, SimComponent *parent)
      : Component(name, parent) {

    idex_clear << [=] {
      return jump_or_branch.uValue() ||
             isLoadHazard(idex_mem_op.uValue(), idex_rd_id.uValue(),
                          rs1_id.uValue(), rs1_sel.uValue(), rs2_id.uValue(),
                          rs2_sel.uValue());
    };

    if_enable << [=] {
      return jump_or_branch.uValue() ||
             !isLoadHazard(idex_mem_op.uValue(), idex_rd_id.uValue(),
                           rs1_id.uValue(), rs1_sel.uValue(), rs2_id.uValue(),
                           rs2_sel.uValue());
    };

    ifid_clear << [=] { return jump_or_branch.uValue(); };
    exmem_clear << [=] { return jump_or_branch.uValue(); };
  }

  bool isLoadHazard(auto m_o, auto rd, auto rs1, auto rs1_s, auto rs2,
                    auto rs2_s) {
    return isLoad(m_o) && rd != 0 &&
           ((rs1_s == ALU1Sel::REG1 && rd == rs1) ||
            (rs2_s == ALU2Sel::REG2 && rd == rs2));
  }

  bool isLoad(auto m_o) {
    return m_o == MemOp::LB || m_o == MemOp::LH || m_o == MemOp::LW ||
           m_o == MemOp::LBU || m_o == MemOp::LHU;
  }

  INPUTPORT(rs1_id, 5);
  INPUTPORT(rs2_id, 5);
  INPUTPORT_ENUM(rs1_sel, ALU1Sel);
  INPUTPORT_ENUM(rs2_sel, ALU2Sel);

  INPUTPORT(idex_rd_id, 5);

  INPUTPORT_ENUM(idex_mem_op, MemOp);

  INPUTPORT(jump_or_branch, 1);

  OUTPUTPORT(idex_clear, 1);
  OUTPUTPORT(exmem_clear, 1);
  OUTPUTPORT(ifid_clear, 1);
  OUTPUTPORT(if_enable, 1);
};

} // namespace core
} // namespace vsrtl
