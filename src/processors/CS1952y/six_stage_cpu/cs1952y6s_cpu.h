#pragma once

#include "VSRTL/core/vsrtl_adder.h"
#include "VSRTL/core/vsrtl_design.h"
#include "VSRTL/core/vsrtl_logicgate.h"
#include "VSRTL/core/vsrtl_multiplexer.h"

#include "../../ripesvsrtlprocessor.h"

#include "../../RISC-V/riscv.h"
#include "../../RISC-V/rv_ecallchecker.h"
#include "../../RISC-V/rv_registerfile.h"
#include "cs1952y_2cycle_mem.h"

#include "../common/cs1952y1s_control.h"
#include "../common/cs1952y1s_decode.h"
#include "../common/cs1952y1s_enums.h"
#include "../single_stage_cpu/cs1952y_alu.h"

#include "../five_stage_cpu/cs1952y_exmem.h"
#include "cs1952y6s_idex.h"
#include "../five_stage_cpu/cs1952y_ifid.h"
#include "../five_stage_cpu/cs1952y_memwb.h"
#include "cs1952y_memmem.h"

#include "cs1952y6s_enums.h"
#include "cs1952y6s_forward.h"
#include "cs1952y6s_hazard.h"

namespace vsrtl {
namespace core {
using namespace Ripes;
using common1sfinal::CS1952y1sControl;
using common1sfinal::CS1952y1sDecode;
using common1sfinal::ALU1Sel;
using common1sfinal::ALU2Sel;
using common1sfinal::ImmSel;
using common1sfinal::RdSel;
using common1sfinal::PCAdd1;
using common1sfinal::PCSel;

template <typename XLEN_T>
class CS1952y6sCPU : public RipesVSRTLProcessor {
  static_assert(std::is_same<uint32_t, XLEN_T>::value,
                "Only supports 32-bit variant");
  static constexpr unsigned XLEN = sizeof(XLEN_T) * CHAR_BIT;

public:
  enum Stage { IF = 0, ID = 1, EX = 2, MEM1 = 3, MEM2 = 4, WB = 5, STAGECOUNT };
  CS1952y6sCPU(const QStringList &extensions)
      : RipesVSRTLProcessor("CS1952y 6s CPU") {
    m_enabledISA = std::make_shared<ISAInfo<XLenToRVISA<XLEN>()>>(extensions);

    // Set up memories
    instr_mem->setMemory(m_memory);
    registers->setMemory(m_regMem);
    data_mem->data_mem->mem->setMemory(m_memory);

    // ** INSTRUCTION FETCH **
    // Increment PC
    0 >> pc_reg->clear;
    pc_reg->out >> pc_inc->op1;
    4 >> pc_inc->op2;
    pc_inc->out >> pc_sel->get(PCSel::INC);
    pc_reg->out >> instr_mem->addr;

    // PC selection (J/B address and mux signal in mem stage)
    pc_sel->out >> pc_reg->in;

    // IF/ID
    pc_reg->out >> ifid_reg->pc_in;
    pc_inc->out >> ifid_reg->pc4_in;
    instr_mem->data_out >> ifid_reg->instr_in;

    1 >> ifid_reg->valid_in;

    // ** INSTRUCTION DECODE **
    ifid_reg->instr_out >> decode->instr;

    // Control
    decode->opcode >> control->opcode;
    decode->funct3 >> control->funct3;
    decode->funct7 >> control->funct7;

    // Hazards
    decode->rs1 >> hazard_unit->rs1_id;
    decode->rs2 >> hazard_unit->rs2_id;
    control->alu1_sel >> hazard_unit->alu1_sel;
    control->alu2_sel >> hazard_unit->alu2_sel;
    idex_reg->mem_op_out >> hazard_unit->idex_mem_op;
    idex_reg->rd_id_out >> hazard_unit->idex_rd_id;
    exmem_reg->mem_op_out >> hazard_unit->exm1_mem_op;
    exmem_reg->rd_id_out >> hazard_unit->exm1_rd_id;
    memmem_reg->rd_id_out >> hazard_unit->m1m2_rd_id;
    memwb_reg->rd_id_out >> hazard_unit->m2wb_rd_id;
    j_or_b->out >> hazard_unit->jump_or_branch;
    hazard_unit->pc_enable >> pc_reg->enable;
    hazard_unit->ifid_enable >> ifid_reg->enable;
    hazard_unit->idex_enable >> idex_reg->enable;
    hazard_unit->exm1_enable >> exmem_reg->enable;
    hazard_unit->m1m2_enable >> memmem_reg->enable;
    hazard_unit->m2wb_enable >> memwb_reg->enable;
    hazard_unit->ifid_clear >> ifid_reg->clear;
    hazard_unit->idex_clear >> idex_reg->clear;
    hazard_unit->exm1_clear >> exmem_reg->clear;
    hazard_unit->m1m2_clear >> memmem_reg->clear;
    hazard_unit->m2wb_clear >> memwb_reg->clear;

    // Read register file
    decode->rs1 >> registers->r1_addr;
    decode->rs2 >> registers->r2_addr;

    // Immediate
    decode->imm_I >> imm_sel->get(ImmSel::I);
    decode->imm_Ishift >> imm_sel->get(ImmSel::Ishift);
    decode->imm_S >> imm_sel->get(ImmSel::S);
    decode->imm_U >> imm_sel->get(ImmSel::U);
    decode->imm_B >> imm_sel->get(ImmSel::B);
    decode->imm_J >> imm_sel->get(ImmSel::J);
    control->imm_sel >> imm_sel->select;

    // ID/Ex
    ifid_reg->pc_out >> idex_reg->pc_in;
    ifid_reg->pc4_out >> idex_reg->pc4_in;
    registers->r1_out >> idex_reg->reg1_in;
    registers->r2_out >> idex_reg->reg2_in;
    imm_sel->out >> idex_reg->imm_in;
    decode->rd1 >> idex_reg->rd_id_in;

    decode->rs1 >> idex_reg->rs1_id_in;
    decode->rs2 >> idex_reg->rs2_id_in;

    control->alu1_sel >> idex_reg->alu1_sel_in;
    control->alu2_sel >> idex_reg->alu2_sel_in;
    control->alu_ctrl >> idex_reg->alu_op_in;
    control->pc_add1 >> idex_reg->pc_add1_in;
    control->mem_w >> idex_reg->mem_wr_in;
    control->mem_op >> idex_reg->mem_op_in;
    control->jump >> idex_reg->jump_in;
    control->branch >> idex_reg->branch_in;
    control->inv_zero >> idex_reg->inv_zero_in;
    control->rd_sel >> idex_reg->rd_sel_in;
    control->reg_w >> idex_reg->reg_wr_in;

    ifid_reg->valid_out >> idex_reg->valid_in;

    // ** EXECUTE **

    // ALU
    // Forwarding
    idex_reg->rs1_id_out >> forward->idex_rs1_id;
    idex_reg->rs2_id_out >> forward->idex_rs2_id;
    exmem_reg->rd_id_out >> forward->exm1_rd_id;
    exmem_reg->reg_wr_out >> forward->exm1_reg_wr;
    exmem_reg->rd_sel_out >> forward->exm1_rd_sel;
    memmem_reg->rd_id_out >> forward->m1m2_rd_id;
    memmem_reg->reg_wr_out >> forward->m1m2_reg_wr;
    memmem_reg->rd_sel_out >> forward->m1m2_rd_sel;
    memwb_reg->rd_id_out >> forward->m2wb_rd_id;
    memwb_reg->reg_wr_out >> forward->m2wb_reg_wr;

    exmem_reg->imm_out >> fwd1_sel->get(Fwd6sSel::ExM1Imm);
    exmem_reg->alu_res_out >> fwd1_sel->get(Fwd6sSel::ExM1Alu);
    memmem_reg->imm_out >> fwd1_sel->get(Fwd6sSel::M1M2Imm);
    memmem_reg->alu_res_out >> fwd1_sel->get(Fwd6sSel::M1M2Alu);
    rd_sel->out >> fwd1_sel->get(Fwd6sSel::WBMux);
    idex_reg->reg1_out >> fwd1_sel->get(Fwd6sSel::NoFwd);
    forward->fwd1_sel >> fwd1_sel->select;

    exmem_reg->imm_out >> fwd2_sel->get(Fwd6sSel::ExM1Imm);
    exmem_reg->alu_res_out >> fwd2_sel->get(Fwd6sSel::ExM1Alu);
    memmem_reg->imm_out >> fwd2_sel->get(Fwd6sSel::M1M2Imm);
    memmem_reg->alu_res_out >> fwd2_sel->get(Fwd6sSel::M1M2Alu);
    rd_sel->out >> fwd2_sel->get(Fwd6sSel::WBMux);
    idex_reg->reg2_out >> fwd2_sel->get(Fwd6sSel::NoFwd);
    forward->fwd2_sel >> fwd2_sel->select;

    fwd1_sel->out >> alu_op1_sel->get(ALU1Sel::REG1);
    idex_reg->pc_out >> alu_op1_sel->get(ALU1Sel::PC);
    idex_reg->alu1_sel_out >> alu_op1_sel->select;
    alu_op1_sel->out >> alu->op1;

    fwd2_sel->out >> alu_op2_sel->get(ALU2Sel::REG2);
    idex_reg->imm_out >> alu_op2_sel->get(ALU2Sel::IMM);
    idex_reg->alu2_sel_out >> alu_op2_sel->select;
    alu_op2_sel->out >> alu->op2;

    idex_reg->alu_op_out >> alu->ctrl;

    // J/B PC address
    idex_reg->pc_add1_out >> pc_add1->select;
    idex_reg->pc_out >> pc_add1->get(PCAdd1::PC);
    fwd1_sel->out >> pc_add1->get(PCAdd1::RS);
    pc_add1->out >> pc_add->op1;

    idex_reg->imm_out >> pc_add->op2;

    // Ex/Mem1
    idex_reg->pc4_out >> exmem_reg->pc4_in;
    fwd2_sel->out >> exmem_reg->reg2_in;
    idex_reg->imm_out >> exmem_reg->imm_in;
    idex_reg->rd_id_out >> exmem_reg->rd_id_in;

    alu->res >> exmem_reg->alu_res_in;
    alu->zero >> exmem_reg->alu_zero_in;
    pc_add->out >> exmem_reg->jb_pc_in;

    idex_reg->mem_wr_out >> exmem_reg->mem_wr_in;
    idex_reg->mem_op_out >> exmem_reg->mem_op_in;
    idex_reg->jump_out >> exmem_reg->jump_in;
    idex_reg->branch_out >> exmem_reg->branch_in;
    idex_reg->inv_zero_out >> exmem_reg->inv_zero_in;
    idex_reg->rd_sel_out >> exmem_reg->rd_sel_in;
    idex_reg->reg_wr_out >> exmem_reg->reg_wr_in;

    idex_reg->valid_out >> exmem_reg->valid_in;

    // ** MEMORY **

    // Data memory
    exmem_reg->alu_res_out >> data_mem->addr;
    exmem_reg->reg2_out >> data_mem->data_in;
    exmem_reg->mem_wr_out >> data_mem->wr_en;
    exmem_reg->mem_op_out >> data_mem->op;

    // XOR gate to invert ALU zero result
    exmem_reg->alu_zero_out >> *inv_zero->in[0];
    exmem_reg->inv_zero_out >> *inv_zero->in[1];

    // AND gate to calculate branch
    exmem_reg->branch_out >> *branch->in[0];
    inv_zero->out >> *branch->in[1];

    // OR gate to calculate jump or branch
    branch->out >> *j_or_b->in[1];
    exmem_reg->jump_out >> *j_or_b->in[0];

    // J/B address and mux signal
    exmem_reg->jb_pc_out >> pc_sel->get(PCSel::JB);
    j_or_b->out >> pc_sel->select;

    // Mem1/Mem2
    exmem_reg->alu_res_out >> memmem_reg->alu_res_in;
    exmem_reg->pc4_out >> memmem_reg->pc4_in;
    exmem_reg->imm_out >> memmem_reg->imm_in;
    exmem_reg->rd_id_out >> memmem_reg->rd_id_in;
    exmem_reg->rd_sel_out >> memmem_reg->rd_sel_in;
    exmem_reg->reg_wr_out >> memmem_reg->reg_wr_in;
    
    exmem_reg->valid_out >> memmem_reg->valid_in;

    // Mem2/WB
    memmem_reg->pc4_out >> memwb_reg->pc4_in;
    memmem_reg->imm_out >> memwb_reg->imm_in;
    memmem_reg->rd_id_out >> memwb_reg->rd_id_in;
    memmem_reg->alu_res_out >> memwb_reg->alu_res_in;
    
    data_mem->data_out >> memwb_reg->mem_res_in;
    
    memmem_reg->rd_sel_out >> memwb_reg->rd_sel_in;
    memmem_reg->reg_wr_out >> memwb_reg->reg_wr_in;
    
    memmem_reg->valid_out >> memwb_reg->valid_in;

    // ** WRITEBACK **

    // Select data
    memwb_reg->alu_res_out >> rd_sel->get(RdSel::ALU);
    memwb_reg->imm_out >> rd_sel->get(RdSel::IMM);
    memwb_reg->mem_res_out >> rd_sel->get(RdSel::MEM);
    memwb_reg->pc4_out >> rd_sel->get(RdSel::PC_L);
    memwb_reg->rd_sel_out >> rd_sel->select;
    rd_sel->out >> registers->data_in;

    // Writeback
    memwb_reg->rd_id_out >> registers->wr_addr;
    memwb_reg->reg_wr_out >> registers->wr_en;

    // Exceptions
    decode->ecall >> idex_reg->ecall_in;
    idex_reg->ecall_out >> exmem_reg->ecall_in;
    exmem_reg->ecall_out >> memmem_reg->ecall_in;
    memmem_reg->ecall_out >> memwb_reg->ecall_in;
    memwb_reg->ecall_out >> ecallChecker->opcode;
    ecallChecker->setSyscallCallback(&trapHandler);
    0 >> ecallChecker->stallEcallHandling;
  }

  // PC
  SUBCOMPONENT(pc_reg, RegisterClEn<XLEN>);
  SUBCOMPONENT(pc_inc, Adder<XLEN>); // ** ADVANCING THE PC **
  SUBCOMPONENT(pc_add, Adder<XLEN>);
  SUBCOMPONENT(pc_sel, TYPE(EnumMultiplexer<PCSel, XLEN>));
  SUBCOMPONENT(pc_add1, TYPE(EnumMultiplexer<PCAdd1, XLEN>));

  SUBCOMPONENT(j_or_b, TYPE(Or<1, 2>));
  SUBCOMPONENT(branch, TYPE(And<1, 2>));
  SUBCOMPONENT(inv_zero, TYPE(Xor<1, 2>));

  // Decoder
  SUBCOMPONENT(decode, CS1952y1sDecode<XLEN>);
  SUBCOMPONENT(control, CS1952y1sControl<XLEN>);
  SUBCOMPONENT(imm_sel, TYPE(EnumMultiplexer<ImmSel, XLEN>));

  // ALU
  SUBCOMPONENT(alu_op1_sel, TYPE(EnumMultiplexer<ALU1Sel, XLEN>));
  SUBCOMPONENT(alu_op2_sel, TYPE(EnumMultiplexer<ALU2Sel, XLEN>));
  SUBCOMPONENT(alu, TYPE(CS1952yALU<XLEN>));

  // Register File
  SUBCOMPONENT(rd_sel, TYPE(EnumMultiplexer<RdSel, XLEN>));
  SUBCOMPONENT(registers, TYPE(RegisterFile<XLEN, true>));

  // Address spaces
  ADDRESSSPACEMM(m_memory);
  ADDRESSSPACE(m_regMem);

  // Memories
  SUBCOMPONENT(instr_mem, TYPE(ROM<XLEN, c_RVInstrWidth>));
  SUBCOMPONENT(data_mem, TYPE(CS1952y2CycleMem<XLEN>));

  // Stage separating registers
  SUBCOMPONENT(ifid_reg, TYPE(CS1952yIFID<XLEN>));
  SUBCOMPONENT(idex_reg, TYPE(CS1952y6sIDEx<XLEN>));
  SUBCOMPONENT(exmem_reg, TYPE(CS1952yExMem<XLEN>));
  SUBCOMPONENT(memmem_reg, TYPE(CS1952yMemMem<XLEN>));
  SUBCOMPONENT(memwb_reg, TYPE(CS1952yMemWB<XLEN>));

  // Forwarding
  SUBCOMPONENT(forward, CS1952y6sForward<XLEN>);
  SUBCOMPONENT(fwd1_sel, TYPE(EnumMultiplexer<Fwd6sSel, XLEN>));
  SUBCOMPONENT(fwd2_sel, TYPE(EnumMultiplexer<Fwd6sSel, XLEN>));

  // Hazards
  SUBCOMPONENT(hazard_unit, CS1952y6sHazard<XLEN>);

  SUBCOMPONENT(ecallChecker, EcallChecker);

  // Ripes interface compliance
  const ProcessorStructure &structure() const override { return m_structure; }
  unsigned int getPcForStage(StageIndex idx) const override {
    // clang-format off
          switch (idx.index()) {
              case IF: return pc_reg->out.uValue();
              case ID: return ifid_reg->pc_out.uValue();
              case EX: return idex_reg->pc_out.uValue();
              case MEM1: 
                return exmem_reg->pc4_out.uValue() == 0 ?
                  0 : exmem_reg->pc4_out.uValue() - 4;
              case MEM2:
                return memmem_reg->pc4_out.uValue() == 0 ?
                  0 : memmem_reg->pc4_out.uValue() - 4;
              case WB:
                return memwb_reg->pc4_out.uValue() == 0 ?
                  0 : memwb_reg->pc4_out.uValue() - 4;
              default: assert(false && "Processor does not contain stage");
          }
          Q_UNREACHABLE();
    // clang-format on
  }
  AInt nextFetchedAddress() const override { return pc_sel->out.uValue(); }

  QString stageName(StageIndex idx) const override {
    // clang-format off
          switch (idx.index()) {
              case IF: return "IF";
              case ID: return "ID";
              case EX: return "EX";
              case MEM1: return "MEM1";
              case MEM2: return "MEM2";
              case WB: return "WB";
              default: assert(false && "Processor does not contain stage");
          }
          Q_UNREACHABLE();
    // clang-format on
  }

  StageInfo stageInfo(StageIndex stage) const override {
    bool stageValid = true;
    // Has the pipeline stage been filled?
    stageValid &= stage.index() <= m_cycleCount;

    // clang-format off
      // Has the stage been cleared?
      switch(stage.index()){
      case ID: stageValid &= ifid_reg->valid_out.uValue(); break;
      case EX: stageValid &= idex_reg->valid_out.uValue(); break;
      case MEM1: stageValid &= exmem_reg->valid_out.uValue(); break;
      case MEM2: stageValid &= memmem_reg->valid_out.uValue(); break;
      case WB: stageValid &= memwb_reg->valid_out.uValue(); break;
      default: case IF: break;
      }

      // Is the stage carrying a valid (executable) PC?
      switch(stage.index()){
      case ID: stageValid &= isExecutableAddress(ifid_reg->pc_out.uValue()); break;
      case EX: stageValid &= isExecutableAddress(idex_reg->pc_out.uValue()); break;
      case MEM1: stageValid &= isExecutableAddress(exmem_reg->pc4_out.uValue() == 0 ? 0 : exmem_reg->pc4_out.uValue() - 4); break;
      case MEM2: stageValid &= isExecutableAddress(memmem_reg->pc4_out.uValue() == 0 ? 0 : memmem_reg->pc4_out.uValue() - 4); break;
      case WB: stageValid &= isExecutableAddress(memwb_reg->pc4_out.uValue() == 0 ? 0 : memwb_reg->pc4_out.uValue() - 4); break;
      default: case IF: stageValid &= isExecutableAddress(pc_reg->out.uValue()); break;
      }

      // Are we currently clearing the pipeline due to a syscall exit? if such, all stages before the EX stage are invalid
      if(stage.index() < WB){
          stageValid &= !ecallChecker->isSysCallExiting();
      }
    // clang-format on

    // Gather stage state info
    StageInfo::State state = StageInfo ::State::None;
    switch (stage.index()) {
    case IF:
      break;
    case ID:
      if (m_cycleCount > ID && ifid_reg->valid_out.uValue() == 0) {
        state = StageInfo::State::Flushed;
      }
      break;
    case EX: {
      if (m_cycleCount > EX && idex_reg->valid_out.uValue() == 0) {
        state = StageInfo::State::Flushed;
      }
      break;
    }
    case MEM1: {
      if (m_cycleCount > MEM1 && exmem_reg->valid_out.uValue() == 0) {
        state = StageInfo::State::Flushed;
      }
      break;
    }
    case MEM2:
      if (m_cycleCount > MEM2 && memmem_reg->valid_out.uValue() == 0) {
        state = StageInfo::State::Flushed;
      }
      break;
    case WB: {
      if (m_cycleCount > WB && memwb_reg->valid_out.uValue() == 0) {
        state = StageInfo::State::Flushed;
      }
      break;
    }
    }

    return StageInfo({getPcForStage(stage), stageValid, state});
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
    if ((fr & FinalizeReason::exitSyscall) &&
        !ecallChecker->isSysCallExiting()) {
      // An exit system call was executed. Record the cycle of the execution,
      // and enable the ecallChecker's system call exiting signal.
      m_syscallExitCycle = m_cycleCount;
    }
    ecallChecker->setSysCallExiting(ecallChecker->isSysCallExiting() ||
                                    (fr & FinalizeReason::exitSyscall));
  }
  bool finished() const override {
    // The processor is finished when there are no more valid instructions in
    // the pipeline
    bool allStagesInvalid = true;
    for (int stage = IF; stage < STAGECOUNT; stage++) {
      allStagesInvalid &= !stageInfo({0, stage}).stage_valid;
      if (!allStagesInvalid)
        break;
    }
    return ecallChecker->isSysCallExiting() || allStagesInvalid;
  }
  const std::vector<StageIndex> breakpointTriggeringStages() const override {
    return {{0, IF}};
  }

  MemoryAccess dataMemAccess() const override {
    return memToAccessInfo(data_mem->data_mem);
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
    // An instruction has been retired if the instruction in the WB stage is
    // valid and the PC is within the executable range of the program
    if (memwb_reg->valid_out.uValue() != 0 &&
        isExecutableAddress(memwb_reg->pc4_out.uValue() == 0
                                ? 0
                                : memwb_reg->pc4_out.uValue() - 4)) {
      m_instructionsRetired++;
    }

    Design::clock();
  }

  void reverse() override {
    if (m_syscallExitCycle != -1 && m_cycleCount == m_syscallExitCycle) {
      // We are about to undo an exit syscall instruction. In this case, the
      // syscall exiting sequence should be terminate
      ecallChecker->setSysCallExiting(false);
      m_syscallExitCycle = -1;
    }
    Design::reverse();
    if (memwb_reg->valid_out.uValue() != 0 &&
        isExecutableAddress(memwb_reg->pc4_out.uValue() == 0
                                ? 0
                                : memwb_reg->pc4_out.uValue() - 4)) {
      m_instructionsRetired--;
    }
  }

  void reset() override {
    Design::reset();
    ecallChecker->setSysCallExiting(false);
    m_syscallExitCycle = -1;
  }

  static ProcessorISAInfo supportsISA() {
    return ProcessorISAInfo{
        std::make_shared<ISAInfo<XLenToRVISA<XLEN>()>>(QStringList()),
        {"M", "C"},
        {"M"}};
  }
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
  long long m_syscallExitCycle = -1;
  std::shared_ptr<ISAInfoBase> m_enabledISA;
  ProcessorStructure m_structure = {{0, 6}};
};

} // namespace core
} // namespace vsrtl
