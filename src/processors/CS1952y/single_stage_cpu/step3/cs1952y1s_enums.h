#pragma once

#include "VSRTL/core/vsrtl_enum.h"

namespace vsrtl {
namespace core {
namespace cs1952y1snotes {
using namespace Ripes;
Enum(ALU2Sel, REG2, IMM);
Enum(ImmSel, I, Ishift, U);
Enum(RdSel, ALU, IMM);
} // namespace cs1952y1snotes
} // namespace core
} // namespace vsrtl
