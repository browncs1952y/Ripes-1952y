#pragma once

#include "../../RISC-V/riscv.h"
#include "VSRTL/core/vsrtl_component.h"
#include <iostream>

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class CS1952y6sForward : public Component {
public:
  CS1952y6sForward(std::string name, SimComponent *parent)
      : Component(name, parent) {

    fwd1_sel << [=] { return Fwd6sSel::NoFwd; }; // fwd1_sel

    fwd2_sel << [=] { return Fwd6sSel::NoFwd; }; // fwd2_sel
  }

  INPUTPORT(idex_rs1_id, 5);
  INPUTPORT(idex_rs2_id, 5);

  INPUTPORT(exm1_rd_id, 5);
  INPUTPORT(exm1_reg_wr, 1);
  INPUTPORT_ENUM(exm1_rd_sel, RdSel);

  INPUTPORT(m1m2_rd_id, 5);
  INPUTPORT(m1m2_reg_wr, 1);
  INPUTPORT_ENUM(m1m2_rd_sel, RdSel);

  INPUTPORT(m2wb_rd_id, 5);
  INPUTPORT(m2wb_reg_wr, 1);

  OUTPUTPORT_ENUM(fwd1_sel, Fwd6sSel);
  OUTPUTPORT_ENUM(fwd2_sel, Fwd6sSel);
};

} // namespace core
} // namespace vsrtl
