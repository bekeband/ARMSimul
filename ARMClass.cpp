/* 
 * File:   ARMClass.cpp
 * Author: user
 * 
 * Created on 2016. február 20., 7:31
 */

#include <iostream>
#include <fstream>
#include "ARMClass.h"

string COND3[8] = {"eq", "ne", "hs", "lo", "ge", "lt", "mi", "pl"};
string COND3_HELP[8] = {"Z Equal", "¬Z Not equal", "¬C Unsigned Higher or same", "C Unsigned Lower", "N == V Signed Greater than or equal", 
  "N ⊕ V Signed Less than", "N Signed Minus / negative", "¬N Signed Plus / positive"};

string COND4[16] = {"eq", "ne", "hs", "lo", "ge", "lt", "mi", "pl", "ls", "gt", "le", "hi", "vs", "vc", "qs", "al"};
string COND4_HELP[16] = {"Z Equal", "¬Z Not equal", "¬C Unsigned Higher or same", "C Unsigned Lower", "N == V Signed Greater than or equal", 
  "N ⊕ V Signed Less than", "N Signed Minus / negative", "¬N Signed Plus / positive", "C ∨ Z Unsigned Lower or same", 
  "¬Z ∧ (N==V) Signed Greater than", "Z ∨ (N ⊕ V) Signed Less than or equal", "¬C ∧ ¬Z Unsigned Higher", "V Overflow", "¬V No overflow",
  "Q Fractional Saturation", "True Always"};

enum e_instr_type {two_reg, single_reg, return_and_test, K8_imm_single_reg};

ARMClass::ARMClass() {
  loaded_prg_size = 0;
  prg_memory_size = PRG_MEM_SIZE * 4;
  prg_memory = new uint8_t[PRG_MEM_SIZE * 4];
  regs.SP = ((PRG_MEM_SIZE - 1) * 4);
  regs.PC = 0x00000004;
  thumb_mode = false;
  spec_regs.PSR = 0x01000000;
  spec_regs.EPSR = 0x01000000;
  spec_regs.APSR.D32 = spec_regs.IPSR = 0;
  spec_regs.BASEPRI = spec_regs.CONTROL = spec_regs.FAULTMASK = spec_regs.PRIMASK = 0;
  SetDisasmAddr(0);
}

ARMClass::ARMClass(const ARMClass& orig) {
  prg_memory = orig.prg_memory;
}

int ARMClass::GetBinFile(std::string FileName)
{
  ifstream i(FileName.c_str(),ios::binary);
  if (i.is_open())
  {
    i.seekg(0, ios::end);
    loaded_prg_size = i.tellg();
    i.seekg(0, ios::beg);
    i.read((char*)prg_memory, loaded_prg_size);
    i.close();
    cout << "The " << FileName << " file is loaded in memory." << endl;
    return 0;
  }
  else
  {
    cout << "The" << FileName << " error" << endl;
    return -1;
  }
}

void ARMClass::PrintPrgMemory(int begin, int end)
{ P_ARM_PRG_STRUCT PPS;
  for (int i = begin; i < end; i += 4)
  {
//    pm = (char*)&prg_memory[i];
    PPS = (P_ARM_PRG_STRUCT)&prg_memory[i];
    printf("%2x(HH), %2x(HL), %2x(LH), %2x(LL)", PPS->S8HH, PPS->S8HL, PPS->S8LH, PPS->S8LL);
//    printf("%4x(HIGH), %4x(LOW), ", PPS->S16H, PPS->S16L);
//    printf("%8x,", PPS->S32);
  }
  printf("SP = %8x\n", regs.SP);
  cout << endl;
}

int ARMClass::PrintMnem()
{ P_ARM_PRG_STRUCT PPS; int retval = 2;
  PPS = (P_ARM_PRG_STRUCT)&prg_memory[DISASM_ADDR];
  
  printf("%04x\t:", DISASM_ADDR);
  sprintf(CODE_BUFFER, "%4x", PPS->S16L);
  
  switch (PPS->S16L & OP4BIT_MASK)
  {
    case BCOND_TEST: BranchCond(PPS->S16L); break; 
    default: 
    
    switch (PPS->S16L & OP8BIT_MASK)
    {
      case MOVRdRm_TEST: MovRdRm(PPS->S16L); break;
      
      default: 
        switch (PPS->S32 & BLTARGET_MASK)
        {
          case  BLTARGET_TEST: 
            BlTarget(PPS); retval = 4; break;
          case BLXTARGET_TEST:
            retval = 3;
          break;
          default: 
        switch (PPS->S16L & OP10BIT_MASK)
        {
          case   ANDRdRm_TEST:ANDRdRm(PPS->S16L);break;
          case   EORRdRm_TEST:EORRdRm(PPS->S16L);break;
          case   LSLRdRs_TEST:LSLRdRs(PPS->S16L);break;
          case   LSRRdRs_TEST:LSRRdRs(PPS->S16L);break;
          case   ASRRdRs_TEST:ASRRdRs(PPS->S16L);break;
          case   ADCRdRm_TEST:ADCRdRm(PPS->S16L);break;
          case   SBCRdRm_TEST:SBCRdRm(PPS->S16L);break;
          case   RORRdRs_TEST:RORRdRs(PPS->S16L);break;
          case   TSTRmRn_TEST:TSTRmRn(PPS->S16L);break;
          case   NEGRdRm_TEST:NEGRdRm(PPS->S16L);break;
          case   CMPRmRn_TEST:CMPRmRn(PPS->S16L);break;
          case   CMNRmRn_TEST:CMNRmRn(PPS->S16L);break;
          case   ORRRdRm_TEST:ORRRdRm(PPS->S16L);break;
          case   MULRdRm_TEST:MULRdRm(PPS->S16L);break;
          case   BICRmRd_TEST:BICRmRd(PPS->S16L);break;
          case   MVNRdRm_TEST:MVNRdRm(PPS->S16L);break;   
          default:
            switch (PPS->S16L & CB_MASK)
            {
              case  CBZ_TEST:   CBZ(PPS->S16L);break;
              case  CBNZ_TEST:  CBNZ(PPS->S16L);break;
              default:
                
                
                
                
                Undefinied(PPS->S16L); break;                
            }
            
        }
        }
      break;
    }
    break;
  }
  printf("%s\t %s\t;%s\n", CODE_BUFFER, MNEM_BUFFER, HELP_BUFFER);
//  printf("%04x\t: %4x %8s\t\t;%s\n", DISASM_ADDR, PPS->S16L, MNEM_BUFFER, HELP_BUFFER);
  return retval;
}

void ARMClass::WriteSimulateMode(bool new_mode)
{
  SM = new_mode;
}

bool ARMClass::TestOpCode(uint16_t TEST, uint16_t MASK, uint16_t OPCODE)
{
  return ((TEST & MASK) == OPCODE);
}

void ARMClass::Undefinied(uint16_t data)
{
  sprintf(HELP_BUFFER, "Undefinied instruction.");
  sprintf(MNEM_BUFFER, "UNDEF: %08x", data);
}

uint32_t ARMClass::SignExtent(uint32_t data, uint32_t MASK)
{
  if (data & MASK){data |= MASK;}
  return data;
}

uint32_t GetBlAddr(uint32_t base, uint32_t offset)
{
  return (base + ((offset * 2) + 4));
}

int ARMClass::GetCond(uint16_t data)
{ 
  return ((data >> 8) & 0x0F);
}

int ARMClass::GetReg(uint16_t data, int number)
{ uint16_t MASK = 0b0000000000000111;
  uint16_t result;
  MASK <<= (number * 3);
  result = data & MASK;
  result >>= (number * 3);
  return result;
}

void ARMClass::SetDisasmAddr(uint32_t new_addr)
{
  DISASM_ADDR = new_addr;
}

void ARMClass::SetPC(uint32_t new_PC)
{
  regs.PC = new_PC;
}
void ARMClass::AddDisasmAddr(uint32_t size)
{
  DISASM_ADDR += size;
}

void ARMClass::SetZFlag(uint32_t data)
{
  spec_regs.APSR.Z = (data == 0);
}

void ARMClass::SetCFlag(uint64_t data)
{
  spec_regs.APSR.C = (data & 0xFFFFFFFF00000000);
}

void ARMClass::SetNFlag(uint32_t data)
{
  spec_regs.APSR.N = (data && 0x80000000);
}

void ARMClass::SetVFlag(uint32_t data)
{
  
}

/* -------------------------------------------------------------------------------------    */

void ARMClass::MovRdRm(uint16_t data)
{
  int RD = GetReg(data, 0);
  int RM = GetReg(data, 1);
  regs.REGS[RD] = regs.REGS[RM];
  sprintf(HELP_BUFFER, "MOV Rd, Rm Rd = Rm Rd or Rm must be a *high register*");
  sprintf(MNEM_BUFFER, "MOV R%i, R%i", RD, RM);
}

void ARMClass::BranchCond(uint16_t data)
{ int condcode; uint32_t target_addr;
  target_addr = regs.PC + (int8_t)(((data & 0x0F) << 1));
  condcode = GetCond(data);
  sprintf(HELP_BUFFER, "B{<cond>} <Target Addr> PC = PC + (#OFF << 1) If %s is true", COND4_HELP[condcode].c_str());
  sprintf(MNEM_BUFFER, "B %s\t %08x", COND4[condcode].c_str(), target_addr);
}

void  ARMClass::ANDRdRm(uint16_t data)
{
  int RD = GetReg(data, 0);
  int RM = GetReg(data, 1);
  regs.REGS[RD] &= regs.REGS[RM];
  SetZFlag(regs.REGS[RD]);
  SetNFlag(regs.REGS[RD]);
  sprintf(HELP_BUFFER, "Rd:= Rd AND Rs");
  sprintf(MNEM_BUFFER, "AND R%i, R%i", RD, RM);
}
void  ARMClass::EORRdRm(uint16_t data)
{
  int RD = GetReg(data, 0);
  int RM = GetReg(data, 1);
  regs.REGS[RD] ^= regs.REGS[RM];
  SetZFlag(regs.REGS[RD]);
  SetNFlag(regs.REGS[RD]);
  sprintf(HELP_BUFFER, "Rd:= Rd EOR Rs");
  sprintf(MNEM_BUFFER, "EOR R%i, R%i", RD, RM);  
}
void  ARMClass::LSLRdRs(uint16_t data)
{ uint64_t TR;
  int RD = GetReg(data, 0);
  int RS = GetReg(data, 1);
  TR = regs.REGS[RD];
  TR <<= regs.REGS[RS]; 
  SetCFlag(TR);
  regs.REGS[RD] = TR & 0xFFFFFFFF;
  SetZFlag(regs.REGS[RD]);
  SetNFlag(regs.REGS[RD]);
  sprintf(HELP_BUFFER, "Rd:= Rd AND Rs");
  sprintf(MNEM_BUFFER, "LSL R%i, R%i", RD, RS);  
}

uint8_t ARMClass::CalcShift(uint16_t data)
{ uint8_t retval;
  retval = (data & 0b0000001000000000) >> 3;
  retval |= (((data & 0xFF) >> 2) & 0xFE);
  return retval;
}

void  ARMClass::CBZ(uint16_t data)
{ uint8_t shift; uint32_t target_addr;
  int RN = GetReg(data, 0); 
  shift = CalcShift(data);
  target_addr = regs.PC + shift + 4;
  SetZFlag(regs.REGS[RN]);
  if (spec_regs.APSR.Z) regs.PC == target_addr;
  sprintf(HELP_BUFFER, "CBZ <Rn>,<%08x>", target_addr);
  sprintf(MNEM_BUFFER, "CBZ R%i %08x", RN, target_addr); //COND4[condcode].c_str(), target_addr);
}
void  ARMClass::CBNZ(uint16_t data)
{ uint8_t shift; uint32_t target_addr;
  int RN = GetReg(data, 0); 
  shift = CalcShift(data);
  target_addr = regs.PC + shift + 4;
  SetZFlag(regs.REGS[RN]);
  if (!spec_regs.APSR.Z) regs.PC == target_addr;  
  sprintf(HELP_BUFFER, "CBNZ <Rn>,<%08x>", target_addr);
  sprintf(MNEM_BUFFER, "CBNZ R%i %08x", RN, target_addr); //COND4[condcode].c_str(), target_addr);
}

void  ARMClass::LSRRdRs(uint16_t data)
{
  
}
void  ARMClass::ASRRdRs(uint16_t data)
{
  
}
void  ARMClass::ADCRdRm(uint16_t data)
{
  
}
void  ARMClass::SBCRdRm(uint16_t data)
{
  
}
void  ARMClass::RORRdRs(uint16_t data)
{
  
}
void  ARMClass::TSTRmRn(uint16_t data)
{
  
}
void  ARMClass::NEGRdRm(uint16_t data)
{
  
}
void  ARMClass::CMPRmRn(uint16_t data)
{
  
}
void  ARMClass::CMNRmRn(uint16_t data)
{
  
}
void  ARMClass::ORRRdRm(uint16_t data)
{
  
}
void  ARMClass::MULRdRm(uint16_t data)
{
  
}
void  ARMClass::BICRmRd(uint16_t data)
{
  
}
void  ARMClass::MVNRdRm(uint16_t data)
{
  
}

/*---------------------------- 32 bits -------------------------------------------*/

void ARMClass::BlTarget(P_ARM_PRG_STRUCT pdata)
{ int32_t offset_h, offset_l; int32_t offset; char b[40];

if ((pdata->S16H & OP5BIT_MASK) == BLTARGET_TEST_L)
{ 
  offset_h = (pdata->S16L & 0b0000011111111111);
  offset_l = (pdata->S16H & 0b0000011111111111);
  offset_h <<= 11;
  offset = offset_h | offset_l;
  offset = SignExtent(offset, 0xFFE00000);
  sprintf(b, " %4x", pdata->S16H);
  strcat(CODE_BUFFER, b);
  sprintf(HELP_BUFFER, "BL <%08x>", GetBlAddr(DISASM_ADDR, offset));
  sprintf(MNEM_BUFFER, "BL %08x", GetBlAddr(DISASM_ADDR, offset));    
} else
{
  Undefinied(pdata->S16L);
}

}




ARMClass::~ARMClass() {
  delete prg_memory;
}

