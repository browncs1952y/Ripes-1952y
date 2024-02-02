#pragma once

#include "../../RISC-V/riscv.h"
#include "VSRTL/core/vsrtl_component.h"
#include <iostream>

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class CS1952y1sTranslate : public Component {
public:

  CS1952y1sTranslate(std::string name, SimComponent *parent) : Component(name, parent) {
    // Registers
    rs1 << [=] { return (instr.uValue() >> 15) & 0x001f; }; // bits 15 to 19
    rd1 << [=] { return (instr.uValue() >> 7) & 0x001f; }; // bits 7 to 11
    
    // Immediate
    imm << [=] { return instr.sValue() >> 20; }; // bits 20 to 31 (sign-extended)
    
    // ALU control signal
    alu_ctrl << [=] {
        auto funct3 = (instr.uValue() >> 12) & 0x0007; // bits 12 to 14
        switch (funct3) {
            case 0b000: // ADDI
                return ALUOp::ADD;
            case 0b010: // SLTI
                return ALUOp::LT;
            case 0b011: // SLTIU
                return ALUOp::LTU;
            case 0b100: // XORI
                return ALUOp::XOR;
            case 0b110: // ORI
                return ALUOp::OR;
            case 0b111: // ANDI
                return ALUOp::AND;
            default:
                throw std::runtime_error("Invalid funct3 field");
            }
    };
    
    // detect exceptions
    ecall << [=] { if ((instr.uValue() & 0x007f) == 0b1110011) { return RVInstr::ECALL; } else { return RVInstr::NOP; } };
  }

  INPUTPORT(instr, 32);
  OUTPUTPORT(rs1, 5);
  OUTPUTPORT(rd1, 5);
  OUTPUTPORT(imm, 32);
  OUTPUTPORT_ENUM(alu_ctrl, ALUOp);

  OUTPUTPORT_ENUM(ecall, RVInstr);
};

} // namespace core
} // namespace vsrtl
