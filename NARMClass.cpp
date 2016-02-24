/* 
 * File:   NARMClass.cpp
 * Author: user
 * 
 * Created on 2016. február 24., 9:26
 */

#include "NARMClass.h"

NARMClass::NARMClass() {
  
  /*if ConditionPassed() then
EncodingSpecificOperations();
next_instr_addr = PC;
LR = next_instr_addr<31:1> : '1';
if toARM then
SelectInstrSet(InstrSet_ARM);
BranchWritePC(Align(PC,4) + imm32);
else
SelectInstrSet(InstrSet_Thumb);
BranchWritePC(PC + imm32);*/
  
}

NARMClass::NARMClass(const NARMClass& orig) {
}

NARMClass::~NARMClass() {
}

void NARMClass::LSRSRdRmImm5(uint16_t data)
{
/* if ConditionPassed() then
EncodingSpecificOperations();
(result, carry) = Shift_C(R[m], SRType_LSR, shift_n, APSR.C);
R[d] = result;
if setflags then
APSR.N = result<31>;
APSR.Z = IsZeroBit(result);
APSR.C = carry;
// APSR.V unchanged*/
}

void NARMClass::BXRm(uint16_t data)
{
/* if ConditionPassed() then
EncodingSpecificOperations();
BXWritePC(R[m]);*/  
}

/*void NARMClass::ADD_WRdRnC(P_ARM_PRG_STRUCT pdata)
{
 if ConditionPassed() then
EncodingSpecificOperations();
(result, carry, overflow) = AddWithCarry(R[n], imm32, '0');
R[d] = result;
if setflags then
APSR.N = result<31>;
APSR.Z = IsZeroBit(result);
APSR.C = carry;
APSR.V = overflow;*/