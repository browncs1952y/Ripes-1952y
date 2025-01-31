#pragma once

#include "VSRTL/core/vsrtl_enum.h"

namespace vsrtl {
namespace core {
namespace cs1952y1snotes {
using namespace Ripes;
Enum(ALU2Sel, REG2, IMM);
Enum(ImmSel, I, Ishift, U, S, J);
Enum(RdSel, ALU, IMM, MEM, PC_L);
Enum(PCAdd1, PC, RS);
Enum(PCSel, INC = 0, J = 1);
} // namespace cs1952y1snotes
} // namespace core
} // namespace vsrtl
