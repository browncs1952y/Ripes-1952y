#pragma once

#include "../../RISC-V/riscv.h"
#include "VSRTL/core/vsrtl_component.h"
#include <iostream>

#include "cs1952y1s_enums.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class CS1952y1sControl : public Component {
public:
  CS1952y1sControl(std::string name, SimComponent *parent)
      : Component(name, parent) {
    alu_ctrl << [=] {
      if (opcode.uValue() == 0b0010011) { // I-type
        switch (funct3.uValue()) {
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
      } else if (opcode.uValue() == 0b0110011) { // R-type
        switch (funct7.uValue()) {
        case 0b0000000:
          switch (funct3.uValue()) {
          case 0b000: // ADD
            return ALUOp::ADD;
          case 0b001: // SLL
            return ALUOp::SL;
          case 0b010: // SLT
            return ALUOp::LT;
          case 0b011: // SLTU
            return ALUOp::LTU;
          case 0b100: // XOR
            return ALUOp::XOR;
          case 0b101: // SRL
            return ALUOp::SRL;
          case 0b110: // OR
            return ALUOp::OR;
          case 0b111: // AND
            return ALUOp::AND;
          default:
            throw std::runtime_error("Unreachable code");
          }
        case 0b0100000:
          switch (funct3.uValue()) {
          case 0b000: // SUB
            return ALUOp::SUB;
          case 0b101: // SRA
            return ALUOp::SRA;
          default:
            throw std::runtime_error("Invalid funct3 field");
          }
        default:
          throw std::runtime_error("Invalid funct7 field");
        }
      } else {
        return ALUOp::NOP;
        // throw std::runtime_error("Opcode not recognized");
      }
    }; // alu_ctrl

    alu2_sel << [=] {
      switch (opcode.uValue()) {
      case 0b0010011: // I-type
        return ALU2Sel::IMM;
      case 0b0110011: // R-type
        return ALU2Sel::REG2;
      default:
        return ALU2Sel::IMM;
      }
    }; // alu_sel
  }

  INPUTPORT(opcode, 7);
  INPUTPORT(funct3, 3);
  INPUTPORT(funct7, 7);

  OUTPUTPORT_ENUM(alu_ctrl, ALUOp);
  OUTPUTPORT_ENUM(alu2_sel, ALU2Sel);
};

} // namespace core
} // namespace vsrtl
