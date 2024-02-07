#pragma once

#include "../../RISC-V/riscv.h"
#include "VSRTL/core/vsrtl_component.h"
#include <iostream>

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class CS1952y1sDecode : public Component {
public:
  CS1952y1sDecode(std::string name, SimComponent *parent)
      : Component(name, parent) {
    // Registers
    rs1 << [=] { return (instr.uValue() >> 15) & 0x001f; }; // bits 15 to 19
    rs2 << [=] { return (instr.uValue() >> 20) & 0x001f; }; // bits 20 to 24
    rd1 << [=] { return (instr.uValue() >> 7) & 0x001f; };  // bits 7 to 11

    // Immediates
    imm_I <<
        [=] { return instr.sValue() >> 20; }; // bits 20 to 31 (sign-extended)
    imm_Ishift <<
        [=] { return (instr.uValue() >> 20) & 0x001f; };  // bits 20 to 24
    imm_U << [=] { return instr.sValue() & 0xfffff000; }; // bits 12 to 31
    imm_S << [=] {
      return ((instr.sValue() & 0xfe000000) >> 20) |
             ((instr.uValue() >> 7) & 0x001f);
    }; // bits 7 to 11 and 25 to 31
    imm_J << [=] {
      return ((instr.sValue() >> 11) &
              0xfff00000) | // bit 31 (sign-extended to 12 bits)
             (instr.uValue() & 0x000ff000) |        // bits 12 to 19
             ((instr.uValue() & 0x00100000) >> 9) | // bit 20
             ((instr.uValue() & 0x7fe00000) >> 20); // bits 21 to 30
    };                                              // imm_J
    imm_B << [=] {
      return ((instr.sValue() >> 19) &
              0xfffff000) | // bit 31 (sign-extended to 20 bits)
             ((instr.uValue() & 0x00000080) << 4) |  // bit 7
             ((instr.uValue() & 0x7e000000) >> 20) | // bits 25 to 30
             ((instr.uValue() & 0x00000f00) >> 7);   // bits 8 to 11
    };

    // Opcode and funct fields
    opcode << [=] { return instr.uValue() & 0x007f; };         // bits 0 to 6
    funct3 << [=] { return (instr.uValue() >> 12) & 0x0007; }; // bits 12 to 14
    funct7 << [=] { return (instr.uValue() >> 25) & 0x007f; }; // bits 25 to 31

    ecall << [=] {
      if ((instr.uValue() & 0x007f) == 0b1110011) {
        return RVInstr::ECALL;
      } else {
        return RVInstr::NOP;
      }
    };
  }

  INPUTPORT(instr, 32);
  OUTPUTPORT(rs1, 5);
  OUTPUTPORT(rs2, 5);
  OUTPUTPORT(rd1, 5);
  OUTPUTPORT(imm_I, 32);
  OUTPUTPORT(imm_Ishift, 32);
  OUTPUTPORT(imm_U, 32);
  OUTPUTPORT(imm_S, 32);
  OUTPUTPORT(imm_J, 32);
  OUTPUTPORT(imm_B, 32);
  OUTPUTPORT(opcode, 7);
  OUTPUTPORT(funct3, 3);
  OUTPUTPORT(funct7, 7);
  OUTPUTPORT_ENUM(ecall, RVInstr);
};

} // namespace core
} // namespace vsrtl
