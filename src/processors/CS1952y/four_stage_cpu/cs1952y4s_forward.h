#pragma once

#include "../../RISC-V/riscv.h"
#include "VSRTL/core/vsrtl_component.h"
#include "cs1952y4s_enums.h"
#include <iostream>

namespace vsrtl {
namespace core {
using namespace Ripes;
using common1sfinal::RdSel;

template <unsigned XLEN>
class CS1952y4sForward : public Component {
public:
  CS1952y4sForward(std::string name, SimComponent *parent)
      : Component(name, parent) {
    fwd1_sel << [=] { return Fwd4sSel::NoFwd; }; // fwd1_sel

    fwd2_sel << [=] { return Fwd4sSel::NoFwd; }; // fwd2_sel
  }

  INPUTPORT(idex_rs1_id, 5);
  INPUTPORT(idex_rs2_id, 5);

  INPUTPORT(exmem_rd_id, 5);
  INPUTPORT(exmem_reg_wr, 1);
  INPUTPORT_ENUM(exmem_rd_sel, RdSel);

  INPUTPORT(memwb_rd_id, 5);
  INPUTPORT(memwb_reg_wr, 1);

  OUTPUTPORT_ENUM(fwd1_sel, Fwd4sSel);
  OUTPUTPORT_ENUM(fwd2_sel, Fwd4sSel);
};

} // namespace core
} // namespace vsrtl
