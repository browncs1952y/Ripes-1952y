#pragma once

#include "VSRTL/core/vsrtl_adder.h"
#include "VSRTL/core/vsrtl_collator.h"
#include "VSRTL/core/vsrtl_decollator.h"
#include "VSRTL/core/vsrtl_multiplexer.h"

#include "../../ripesvsrtlprocessor.h"

#include "../../RISC-V/riscv.h"
#include "../../RISC-V/rv_ecallchecker.h"
#include "../../RISC-V/rv_memory.h"
#include "../../RISC-V/rv_registerfile.h"

#include "boundscheck.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <typename XLEN_T>
class HW1cCircuit : public RipesVSRTLProcessor {
  static_assert(std::is_same<uint32_t, XLEN_T>::value,
                "Only supports 32-bit variant");
  static constexpr unsigned XLEN = sizeof(XLEN_T) * CHAR_BIT;

public:
  HW1cCircuit(const QStringList &extensions)
      : RipesVSRTLProcessor("HW1c circuit") {
    m_enabledISA = std::make_shared<ISAInfo<XLenToRVISA<XLEN>()>>(extensions);

    // Required memories
    registerFile->setMemory(m_regMem);
    instr_mem->setMemory(m_memory);
    data_mem->mem->setMemory(m_memory);

    // Instruction memory
    pc_inc->out >> instr_mem->addr;
    pc_inc->out >> pc_reg->in;
    pc_reg->out >> pc_inc->op1;
    4 >> pc_inc->op2;

    /*
     * Because of how VSTRL resolves components, we need to have a connected
     * graph with the register file in order for the register file to give valid
     * info. We hack this by connecting the pc_reg to the second register
     * address of the register file
     */
    pc_reg->out >> d->in;
    // Only get the first 5 bits of pc_reg
    for (int i = 0; i < 5; i++) {
      *d->out[i] >> *c->in[i];
    }
    c->out >> registerFile->r2_addr;

    // use x10 as the input to the bounds check
    0x0a >> registerFile->r1_addr;
    registerFile->r1_out >> bc->in_val;

    // Use the overflow flag to determine which register is written to
    bc->overflow >> of_mux->select;
    0xb >> *of_mux->ins[0];
    0xc >> *of_mux->ins[1];

    // Write bounds check output
    1 >> registerFile->wr_en;
    of_mux->out >> registerFile->wr_addr;
    bc->res_val >> registerFile->data_in;

    // Data memory (unused)
    0 >> data_mem->addr;
    0 >> data_mem->data_in;
    0 >> data_mem->wr_en;
    MemOp::NOP >> data_mem->op;

    // ecall checker (unused)
    RVInstr::NOP >> ecallChecker->opcode;
    ecallChecker->setSyscallCallback(&trapHandler);
    0 >> ecallChecker->stallEcallHandling;
  }

  SUBCOMPONENT(bc, BoundsCheck);
  SUBCOMPONENT(d, Decollator<XLEN>);
  SUBCOMPONENT(c, Collator<5>);
  SUBCOMPONENT(of_mux, TYPE(Multiplexer<2, 5>));

  // Memories
  ADDRESSSPACEMM(m_memory);
  ADDRESSSPACE(m_regMem);
  SUBCOMPONENT(registerFile, TYPE(RegisterFile<XLEN, false>));
  SUBCOMPONENT(instr_mem, TYPE(ROM<XLEN, c_RVInstrWidth>));
  SUBCOMPONENT(data_mem, TYPE(RVMemory<XLEN, XLEN>));
  SUBCOMPONENT(pc_reg, Register<XLEN>);
  SUBCOMPONENT(pc_inc, Adder<XLEN>);

  SUBCOMPONENT(ecallChecker, EcallChecker);

  // Ripes interface compliance
  const ProcessorStructure &structure() const override { return m_structure; }
  unsigned int getPcForStage(StageIndex) const override {
    return pc_reg->out.uValue();
  }
  AInt nextFetchedAddress() const override { return pc_inc->out.uValue(); }
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
    return registerFile->getRegister(i);
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
    setSynchronousValue(registerFile->_wr_mem, i, v);
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
