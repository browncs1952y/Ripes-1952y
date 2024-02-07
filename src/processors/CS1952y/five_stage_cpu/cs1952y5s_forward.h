#pragma once

#include "../../RISC-V/riscv.h"
#include "VSRTL/core/vsrtl_component.h"
#include <iostream>

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class CS1952y5sForward : public Component {
public:
  CS1952y5sForward(std::string name, SimComponent *parent)
      : Component(name, parent) {

    fwd1_sel << [=] {
      if ((idex_rs1_id.uValue() == exmem_rd_id.uValue()) &&
          (exmem_reg_wr.uValue() == 1) && (exmem_rd_id.uValue() != 0)) {
        if (exmem_rd_sel.uValue() == RdSel::IMM) {
          return FwdSel::ExMemImm;
        } else {
          return FwdSel::ExMemAlu;
        }
      } else if ((idex_rs1_id.uValue() == memwb_rd_id.uValue()) &&
                 (memwb_reg_wr.uValue() == 1) && (memwb_rd_id.uValue() != 0)) {
        return FwdSel::WBMux;
      } else {
        return FwdSel::NoFwd;
      }
    }; // fwd1_sel

    fwd2_sel << [=] {
      if ((idex_rs2_id.uValue() == exmem_rd_id.uValue()) &&
          (exmem_reg_wr.uValue() == 1) && (exmem_rd_id.uValue() != 0)) {
        if (exmem_rd_sel.uValue() == RdSel::IMM) {
          return FwdSel::ExMemImm;
        } else {
          return FwdSel::ExMemAlu;
        }
      } else if ((idex_rs2_id.uValue() == memwb_rd_id.uValue()) &&
                 (memwb_reg_wr.uValue() == 1) && (memwb_rd_id.uValue() != 0)) {
        return FwdSel::WBMux;
      } else {
        return FwdSel::NoFwd;
      }
    }; // fwd2_sel
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
