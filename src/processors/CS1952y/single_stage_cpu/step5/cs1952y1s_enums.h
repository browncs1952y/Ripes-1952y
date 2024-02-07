#pragma once

#include "VSRTL/core/vsrtl_enum.h"

namespace vsrtl {
namespace core {
using namespace Ripes;
Enum(ALU2Sel, REG2, IMM);
Enum(ImmSel, I, Ishift, U, S);
Enum(RdSel, ALU, IMM, MEM);
} // namespace core
} // namespace vsrtl
