#pragma once

#include "../../RISC-V/riscv.h"
#include "VSRTL/core/vsrtl_component.h"
#include <iostream>

#include "../../RISC-V/rv_memory.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class CS1952y2CycleMem : public Component {
public:
    CS1952y2CycleMem(std::string name, SimComponent *parent) : Component(name, parent) {
    addr >> addr_reg->in;
    addr_reg->out >> data_mem->addr;
    
    data_in >> data_in_reg->in;
    data_in_reg->out >> data_mem->data_in;
    
    wr_en >> wr_en_reg->in;
    wr_en_reg->out >> data_mem->wr_en;
    
    op >> op_reg->in;
    op_reg->out >> data_mem->op;
    
    data_mem->data_out >> data_out;
    }
    
    SUBCOMPONENT(data_mem, TYPE(RVMemory<XLEN, XLEN>));
    
    SUBCOMPONENT(addr_reg, Register<XLEN>);
    SUBCOMPONENT(data_in_reg, Register<XLEN>);
    SUBCOMPONENT(wr_en_reg, Register<1>);
    SUBCOMPONENT(op_reg, Register<MemOp::width()>);
    
    INPUTPORT(addr, XLEN);
    INPUTPORT(data_in, XLEN);
    INPUTPORT(wr_en, 1);
    INPUTPORT_ENUM(op, MemOp);
    
    OUTPUTPORT(data_out, XLEN);
    };
}
}