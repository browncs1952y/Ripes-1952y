#pragma once

#include "VSRTL/core/vsrtl_adder.h"
#include "VSRTL/core/vsrtl_design.h"
#include "VSRTL/core/vsrtl_logicgate.h"
#include "VSRTL/core/vsrtl_multiplexer.h"

#include "../../ripesvsrtlprocessor.h"

#include "../../RISC-V/riscv.h"
#include "../../RISC-V/rv_ecallchecker.h"
#include "../../RISC-V/rv_memory.h"
#include "../../RISC-V/rv_registerfile.h"

#include "cs1952y_alu.h"
#include "cs1952y1s_control.h"
#include "cs1952y1s_decode.h"
#include "cs1952y1s_enums.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <typename XLEN_T>
class CS1952y1sCPU : public RipesVSRTLProcessor {
  static_assert(std::is_same<uint32_t, XLEN_T>::value,
                "Only supports 32-bit variant");
  static constexpr unsigned XLEN = sizeof(XLEN_T) * CHAR_BIT;

public:
  CS1952y1sCPU(const QStringList &extensions)
      : RipesVSRTLProcessor("CS1952y RISC-V processor (from scratch)") {
    m_enabledISA = std::make_shared<ISAInfo<XLenToRVISA<XLEN>()>>(extensions);    
    
    // ** ADVANCING THE PC **
    pc_sel->out >> pc_reg->in;
    pc_reg->out >> pc_inc->op1;
    4 >> pc_inc->op2;

    // Jumps and branches
    registers->r1_out >> pc_add1->get(PCAdd1::RS); // JALR
    pc_reg->out >> pc_add1->get(PCAdd1::PC); // JAL
    control->pc_add1 >> pc_add1->select;
    pc_add1->out >> pc_add->op1;
    imm_sel->out >> pc_add->op2;
    pc_inc->out >> pc_sel->get(PCSel::INC);
    pc_add->out >> pc_sel->get(PCSel::J);
    control->jump >> pc_sel->select;

    // ** Instruction memory **
    instr_mem->setMemory(m_memory);
    pc_reg->out >> instr_mem->addr;

    // ** Decode and Control **
    instr_mem->data_out >> decode->instr;
    decode->opcode >> control->opcode;
    decode->funct3 >> control->funct3;
    decode->funct7 >> control->funct7;

    // Immediates
    decode->imm_I >> imm_sel->get(ImmSel::I);
    decode->imm_Ishift >>  imm_sel->get(ImmSel::Ishift);
    decode->imm_U >> imm_sel->get(ImmSel::U);
    decode->imm_S >> imm_sel->get(ImmSel::S);
    decode->imm_J >> imm_sel->get(ImmSel::J);
    control->imm_sel >> imm_sel->select;

    // ** Registers **
    registers->setMemory(m_regMem);
    decode->rs1 >> registers->r1_addr;
    decode->rs2 >> registers->r2_addr;

    // Writeback
    decode->rd1 >> registers->wr_addr;
    control->reg_w >> registers->wr_en;
    alu->res >> rd_sel->get(RdSel::ALU);
    imm_sel->out >> rd_sel->get(RdSel::IMM);
    data_mem->data_out >> rd_sel->get(RdSel::MEM);
    pc_inc->out >> rd_sel->get(RdSel::PC_L);
    control->rd_sel >> rd_sel->select;
    rd_sel->out >> registers->data_in;
    
    // ** ALU **
    // Select op2
    registers->r2_out >> alu_op2_sel->get(ALU2Sel::REG2);
    imm_sel->out >> alu_op2_sel->get(ALU2Sel::IMM);
    control->alu2_sel >> alu_op2_sel->select;

    registers->r1_out >> alu->op1;
    alu_op2_sel->out >> alu->op2;
    control->alu_ctrl >> alu->ctrl;
    
    // Data memory
    data_mem->mem->setMemory(m_memory);
    alu->res >> data_mem->addr;
    registers->r2_out >> data_mem -> data_in;
    control->mem_w >> data_mem->wr_en;
    control->mem_op >> data_mem->op;
    
    decode->ecall >> ecallChecker->opcode;
    ecallChecker->setSyscallCallback(&trapHandler);
    0 >> ecallChecker->stallEcallHandling;
  }

  // PC
  SUBCOMPONENT(pc_reg, Register<XLEN>);
  SUBCOMPONENT(pc_inc, Adder<XLEN>);
  SUBCOMPONENT(pc_add, Adder<XLEN>);
  SUBCOMPONENT(pc_add1, TYPE(EnumMultiplexer<PCAdd1, XLEN>));
  SUBCOMPONENT(pc_sel, TYPE(EnumMultiplexer<PCSel, XLEN>));

  // Decode and Control
  SUBCOMPONENT(decode,  CS1952y1sDecode<XLEN>);
  SUBCOMPONENT(control, CS1952y1sControl<XLEN>);
  SUBCOMPONENT(imm_sel, TYPE(EnumMultiplexer<ImmSel, XLEN>));
  
  // Register File
  SUBCOMPONENT(registers, TYPE(RegisterFile<XLEN, false>));
  SUBCOMPONENT(rd_sel, TYPE(EnumMultiplexer<RdSel, XLEN>));

  // ALU
  SUBCOMPONENT(alu_op2_sel, TYPE(EnumMultiplexer<ALU2Sel, XLEN>));
  SUBCOMPONENT(alu, TYPE(CS1952yALU<XLEN>));
  
  // Address spaces
  ADDRESSSPACEMM(m_memory);
  ADDRESSSPACE(m_regMem);
  
  // Memories
  SUBCOMPONENT(instr_mem, TYPE(ROM<XLEN, c_RVInstrWidth>));
  SUBCOMPONENT(data_mem, TYPE(RVMemory<XLEN, XLEN>));
  
  SUBCOMPONENT(ecallChecker, EcallChecker);

  // Ripes interface compliance
  const ProcessorStructure &structure() const override { return m_structure; }
  unsigned int getPcForStage(StageIndex) const override {
    return pc_reg->out.uValue();
  }
  AInt nextFetchedAddress() const override { return pc_sel->out.uValue(); }
  QString stageName(StageIndex) const override { return "•"; }
  StageInfo stageInfo(StageIndex) const override {
    return StageInfo({pc_reg->out.uValue(),
                      isExecutableAddress(pc_reg->out.uValue()),
                      StageInfo::State::None});
  }
  void setProgramCounter(AInt address) override {
    pc_reg->forceValue(0, address);
    propagateDesign();
  }
  void setPCInitialValue(AInt address) override {
    pc_reg->setInitValue(address);
  }
  AddressSpaceMM &getMemory() override { return *m_memory; }
  VInt getRegister(const std::string_view &, unsigned i) const override {
    return registers->getRegister(i);
  }
  void finalize(FinalizeReason fr) override {
    if (fr == FinalizeReason::exitSyscall) {
      // Allow one additional clock cycle to clear the current instruction
      m_finishInNextCycle = true;
    }
  }
  bool finished() const override {
    return m_finished || !stageInfo({0, 0}).stage_valid;
  }
  const std::vector<StageIndex> breakpointTriggeringStages() const override {
    return {{0, 0}};
  }

  MemoryAccess dataMemAccess() const override {
    return memToAccessInfo(data_mem);
  }
  MemoryAccess instrMemAccess() const override {
    auto instrAccess = memToAccessInfo(instr_mem);
    instrAccess.type = MemoryAccess::Read;
    return instrAccess;
  }

  void setRegister(const std::string_view &, unsigned i, VInt v) override {
    setSynchronousValue(registers->_wr_mem, i, v);
  }

  void clockProcessor() override {
    // Single cycle processor; 1 instruction retired per cycle!
    m_instructionsRetired++;

    // m_finishInNextCycle may be set during Design::clock(). Store the value
    // before clocking the processor, and emit finished if this was the final
    // clock cycle.
    const bool finishInThisCycle = m_finishInNextCycle;
    Design::clock();
    if (finishInThisCycle) {
      m_finished = true;
    }
  }

  void reverse() override {
    m_instructionsRetired--;
    Design::reverse();
    // Ensure that reverses performed when we expected to finish in the
    // following cycle, clears this expectation.
    m_finishInNextCycle = false;
    m_finished = false;
  }

  void reset() override {
    Design::reset();
    m_finishInNextCycle = false;
    m_finished = false;
  }

  static ProcessorISAInfo supportsISA() { return RVISA::supportsISA<XLEN>(); }
  const ISAInfoBase *implementsISA() const override {
    return m_enabledISA.get();
  }
  std::shared_ptr<const ISAInfoBase> fullISA() const override {
    return RVISA::fullISA<XLEN>();
  }

  const std::set<std::string_view> registerFiles() const override {
    std::set<std::string_view> rfs;
    rfs.insert(RVISA::GPR);

    if (implementsISA()->extensionEnabled("F")) {
      rfs.insert(RVISA::FPR);
    }
    return rfs;
  }

private:
  bool m_finishInNextCycle = false;
  bool m_finished = false;
  std::shared_ptr<ISAInfoBase> m_enabledISA;
  ProcessorStructure m_structure = {{0, 1}};
};

} // namespace core
} // namespace vsrtl
