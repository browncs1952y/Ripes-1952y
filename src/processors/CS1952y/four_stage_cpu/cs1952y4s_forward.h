#pragma once

#include "../../RISC-V/riscv.h"
#include "VSRTL/core/vsrtl_component.h"
#include <iostream>

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class CS1952y4sForward : public Component {
public:
  CS1952y4sForward(std::string name, SimComponent *parent)
      : Component(name, parent) {
    fwd1_sel << [=] { return FwdSel::NoFwd; }; // fwd1_sel

    fwd2_sel << [=] { return FwdSel::NoFwd; }; // fwd2_sel
  }

  INPUTPORT(idex_rs1_id, 5);
  INPUTPORT(idex_rs2_id, 5);

  INPUTPORT(exmem_rd_id, 5);
  INPUTPORT(exmem_reg_wr, 1);
  INPUTPORT_ENUM(exmem_rd_sel, RdSel);

  INPUTPORT(memwb_rd_id, 5);
  INPUTPORT(memwb_reg_wr, 1);

  OUTPUTPORT_ENUM(fwd1_sel, FwdSel);
  OUTPUTPORT_ENUM(fwd2_sel, FwdSel);
};

} // namespace core
} // namespace vsrtl
