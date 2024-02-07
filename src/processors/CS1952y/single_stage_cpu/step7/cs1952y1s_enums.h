#pragma once

#include "VSRTL/core/vsrtl_enum.h"

namespace vsrtl {
namespace core {
using namespace Ripes;
Enum(ALU1Sel, REG1, PC);
Enum(ALU2Sel, REG2, IMM);
Enum(ImmSel, I, Ishift, U, S, J);
Enum(RdSel, ALU, IMM, MEM, PC_L);
Enum(PCAdd1, PC, RS);
Enum(PCSel, INC = 0, J = 1);
} // namespace core
} // namespace vsrtl
