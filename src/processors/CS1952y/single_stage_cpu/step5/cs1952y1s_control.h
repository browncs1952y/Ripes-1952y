#pragma once

#include "../../RISC-V/riscv.h"
#include "VSRTL/core/vsrtl_component.h"
#include <iostream>

#include "cs1952y1s_enums.h"

namespace vsrtl {
namespace core {
namespace cs1952y1snotes {
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
        case 0b001: // SLL
          return ALUOp::SL;
        case 0b010: // SLTI
          return ALUOp::LT;
        case 0b011: // SLTIU
          return ALUOp::LTU;
        case 0b100: // XORI
          return ALUOp::XOR;
        case 0b101: // SRLI and SRAI
          if (funct7.uValue() == 0) {
            return ALUOp::SRL;
          } else if (funct7.uValue() == 0b0100000) {
            return ALUOp::SRA;
          } else {
            throw std::runtime_error("Invalid upper 7 bits for SRLI/SRAI");
          }
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
      } else if (opcode.uValue() == 0b0000011) { // load
        return ALUOp::ADD;
      } else if (opcode.uValue() == 0b0100011) { // store
        return ALUOp::ADD;
      } else {
        return ALUOp::NOP;
        // throw std::runtime_error("Opcode not recognized");
      }
    }; // alu_ctrl

    alu2_sel << [=] {
      switch (opcode.uValue()) {
      case 0b0010011: // I-type
      case 0b0000011: // Load
      case 0b0100011: // Store
        return ALU2Sel::IMM;
      case 0b0110011: // R-type
        return ALU2Sel::REG2;
      default:
        return ALU2Sel::IMM;
      }
    }; // alu_sel

    imm_sel << [=] {
      switch (opcode.uValue()) {
      case 0b0010011: // Register-immediate
        if (funct3.uValue() == 0b001 || funct3.uValue() == 0b101) { // shift
          return ImmSel::Ishift;
        } else {
          return ImmSel::I;
        }
      case 0b0110111: // LUI
        return ImmSel::U;
      case 0b0100011: // Store
        return ImmSel::S;
      case 0b0000011: // Loads
      default:
        return ImmSel::I;
      }
    }; // imm_sel

    rd_sel << [=] {
      switch (opcode.uValue()) {
      case 0b0110111: // LUI
        return RdSel::IMM;
      case 0b0000011: // Loads
        return RdSel::MEM;
      default:
        return RdSel::ALU;
      }
    }; // rd_sel

    mem_op << [=] {
      if (opcode.uValue() == 0b0000011) { // Loads
        switch (funct3.uValue()) {
        case 0b000: // LB
          return MemOp::LB;
        case 0b001: // LH
          return MemOp::LH;
        case 0b010: // LW
          return MemOp::LW;
        case 0b100: // LBU
          return MemOp::LBU;
        case 0b101: // LHU
          return MemOp::LHU;
        default:
          throw std::runtime_error("Load width not recognized for 32-bit CPU");
        }
      } else if (opcode.uValue() == 0b0100011) { // Stores
        switch (funct3.uValue()) {
        case 0b000: // SB
          return MemOp::SB;
        case 0b001: // SH
          return MemOp::SH;
        case 0b010: // SW
          return MemOp::SW;
        default:
          throw std::runtime_error("Store width not recognized for 32-bit CPU");
        }
      } else {
        return MemOp::NOP;
      }
    }; // mem_op

    reg_w << [=] {
      switch (opcode.uValue()) {
      case 0b0010011: // register-immediate
      case 0b0110011: // register-register
      case 0b0110111: // LUI
      case 0b0000011: // load
        return 1;
      case 0b0100011: // store
      default:
        return 0;
      }
    }; // reg_w;

    mem_w << [=] { return opcode.uValue() == 0b0100011; }; // store
  }

  INPUTPORT(opcode, 7);
  INPUTPORT(funct3, 3);
  INPUTPORT(funct7, 7);

  OUTPUTPORT_ENUM(alu_ctrl, ALUOp);
  OUTPUTPORT_ENUM(alu2_sel, ALU2Sel);
  OUTPUTPORT_ENUM(imm_sel, ImmSel);
  OUTPUTPORT_ENUM(rd_sel, RdSel);

  OUTPUTPORT_ENUM(mem_op, MemOp);
  OUTPUTPORT(reg_w, 1);
  OUTPUTPORT(mem_w, 1);
};

} // namespace cs1952y1snotes
} // namespace core
} // namespace vsrtl
