/* 
 * File:   ARMClass.h
 * Author: user
 *
 * Created on 2016. február 20., 7:31
 */

#ifndef ARMCLASS_H
#define	ARMCLASS_H

#include <string>

using namespace std;

#define   PRG_MEM_SIZE      20000
#define   DATA_MEM_SIZE     0xFFFF
#define   STATIC_DATA_SIZE  0x4000
#define   PRG_ORIGIN        0x0000000000000000

#define   OP4BIT_MASK       0b1111000000000000
#define   OP5BIT_MASK       0b1111100000000000
#define   OP6BIT_MASK       0b1111110000000000
#define   OP7BIT_MASK       0b1111111000000000
#define   OP8BIT_MASK       0b1111111100000000
#define   OP9BIT_MASK       0b1111111110000000
#define   OP10BIT_MASK      0b1111111111000000
#define   OP13BIT_MASK      0b1111000000000111
#define   ALL_MASK          0xFFFF
#define   CB_MASK           0b1111110100000000

#define   BLTARGET_MASK     0xD000F800
#define   MOVWRdimm16_MASK  0x8000FBF0
#define   ADD_WRdRnC_MASK   0x8000FBE0
#define   MOV_WRdC_MASK     0x8000FBEF
#define   LDMIA_WRnRegs_MASK  0x2000FFD2    
#define   B_WLAB_MASK       0xD000F800

#define   BLTARGET_TEST     0xD000F000
#define   BLXTARGET_TEST    0xC000F000

#define   BCOND_TEST        0b1101000000000000
#define   SWI_TEST          0b1101000000000000
#define   MOVRdRm_TEST      0b0100011000000000
#define   BLTARGET_TEST_L   0b1111100000000000
#define   BLTARGET_TEST_U   0b1111000000000000
#define   CBZ_TEST          0b1011000100000000
#define   CBNZ_TEST         0b1011100100000000

#define   MOVWRdimm16_TEST  0x0000F240
#define   MOVTRdimm16_TEST  0x0000F2C0
#define   ADD_WRdRnC_TEST   0x0000F100
#define   LDMIA_WRnRegs_TEST  0x0000E890
#define   MOV_WRdC_TEST     0x0000F04F
#define   B_WLAB_TEST_01    0x9000F000
#define   B_WLAB_TEST_02    0x8000F000

#define   NOP_TEST          0b1011111100000000

/* ------------------------ 5 bites halfword ----------------------------*/

#define   MOVSRDIMM_TEST    0b0010000000000000
#define   STRHRtRnImm_TEST  0b1000000000000000
#define   ASRSRdRmImm_TEST  0b0001000000000000
#define   LDRBRtRnImm_TEST  0b0111100000000000
#define   LDRHRtRnImm_TEST  0b1000100000000000
#define   STRBRtRnImm_TEST  0b0111000000000000
#define   STRHRtRnImm_TEST  0b1000000000000000
#define   LSRSRdRmImm5_TEST 0b0000100000000000

/* ------------------------ 5 bites halfword -----------------------------*/
/* ------------------------- 9 bites halfword ----------------------------*/
#define   BXRm_TEST         0b0100011100000000
/* ------------------------- 9 bites halfword ----------------------------*/
#define   MOVSRdRm_TEST     0x0000
#define   ANDRdRm_TEST      0b0100000000000000
#define   EORRdRm_TEST      0b0100000001000000
#define   LSLRdRs_TEST      0b0100000010000000
#define   LSRRdRs_TEST      0b0100000011000000
#define   ASRRdRs_TEST      0b0100000100000000
#define   ADCRdRm_TEST      0b0100000101000000
#define   SBCRdRm_TEST      0b0100000110000000
#define   RORRdRs_TEST      0b0100000111000000
#define   TSTRmRn_TEST      0b0100001000000000
#define   NEGRdRm_TEST      0b0100001001000000
#define   CMPRmRn_TEST      0b0100001010000000
#define   CMNRmRn_TEST      0b0100001011000000
#define   ORRRdRm_TEST      0b0100001100000000
#define   MULRdRm_TEST      0b0100001101000000
#define   BICRmRd_TEST      0b0100001110000000
#define   MVNRdRm_TEST      0b0100001111000000


#define   MOV_REG_IMM_MASK  0b1111000000000000
#define   MOV_REG_IMM_TEST  0b0011000000000000

#define   SINGLE_REG_MASK   0b0101110000000000
#define   RET_AND_TEST_MASK 0b0101111000000000

struct  ARM_PRG_STRUCT{
  union {
    struct {
    uint32_t  S32;
    };
    struct {
    uint16_t  S16L;
    uint16_t  S16H;
    };
    struct {
    uint8_t   S8HL;
    uint8_t   S8HH;
    uint8_t   S8LL;
    uint8_t   S8LH;
    };
  };
} ;

typedef ARM_PRG_STRUCT* P_ARM_PRG_STRUCT;

union ARM_REG_STRUCT{
struct {
  uint32_t  R0;
  uint32_t  R1;
  uint32_t  R2;
  uint32_t  R3;
  uint32_t  R4;
  uint32_t  R5;
  uint32_t  R6;
  uint32_t  R7;
  uint32_t  R8;
  uint32_t  R9;
  uint32_t  R10;
  union {
    struct {
    uint32_t  R11;
    uint32_t  R12;
    uint32_t  R13;
    uint32_t  R14; 
    uint32_t  R15;
    };
    struct {
    uint32_t  R11K;
    uint32_t  R12K;
    uint32_t  SP;
    uint32_t  LR; 
    uint32_t  PC;
    };
  };
};
struct {
  uint32_t  REGS[16];
};
struct {
  ARM_PRG_STRUCT REGST[16];
};
};

union APSR_STRUCT {
struct  {
  unsigned N:1;
  unsigned Z:1;
  unsigned C:1;
  unsigned V:1;
  unsigned Q:1;
  unsigned RES:27;
  };
struct {
  uint32_t D32;
};
};

struct ARM_SPEC_REG_STRUCT
{
  uint32_t  PSR; /*Provide ALU fl ags (zero fl ag, carry fl ag), execution status, and current executing interrupt number*/
  APSR_STRUCT  APSR; 
  uint32_t  IPSR;
  uint32_t  EPSR;
  uint32_t  PRIMASK;  /* Disable all interrupts except the nonmaskable interrupt (NMI) and HardFault */
  uint32_t  FAULTMASK;  /* Disable all interrupts except the NMI */
  uint32_t  BASEPRI;    /*Disable all interrupts of specifi c priority level or lower priority level */
  uint32_t  CONTROL;    /* Defi ne privileged status and stack pointer selection */
};

class ARMClass {
  
public:
  ARMClass();
  ARMClass(const ARMClass& orig);
  virtual ~ARMClass();
  int GetBinFile(std::string FileName);
  int PrintMnem();
  int Simulate();  
  void PrintPrgMemory(int, int);
  void SetPC(uint32_t new_PC);
  void SetDisasmAddr(uint32_t new_addr);
  void AddDisasmAddr(uint32_t size);
  void WriteSimulateMode(bool new_mode);
  
private:
  
  char MNEM_BUFFER[64];
  char HELP_BUFFER[256];
  char CODE_BUFFER[40];
  
  string dstring;
  
  uint32_t prg_memory_size;
  uint32_t loaded_prg_size;
  //uint32_t  data_memory_size;
  
  ARM_PRG_STRUCT STATIC_DATA[STATIC_DATA_SIZE];
  
  uint8_t*  prg_memory;
  //uint8_t*  data_memory; 
  //uint16_t  data_memory_offset;  
          
  ARM_REG_STRUCT regs;
  ARM_SPEC_REG_STRUCT spec_regs;
  uint32_t DISASM_ADDR;
  
  bool SM;
  bool thumb_mode;
  
  bool TestOpCode(uint16_t TEST, uint16_t MASK, uint16_t OPCODE);
  int GetReg(uint16_t data, int number);
  int GetCond(uint16_t data);
  void BranchCond(uint16_t data);
  void Undefinied(uint16_t data);
  void MovRdRm(uint16_t data);
  void BlTarget(P_ARM_PRG_STRUCT pdata);
  uint32_t SignExtent(uint32_t data, uint32_t MASK);
  void  ANDRdRm(uint16_t data);
  void  EORRdRm(uint16_t data);
  void  LSLRdRs(uint16_t data);    
  void  LSRRdRs(uint16_t data);     
  void  ASRRdRs(uint16_t data);      
  void  ADCRdRm(uint16_t data);     
  void  SBCRdRm(uint16_t data);    
  void  RORRdRs(uint16_t data);   
  void  TSTRmRn(uint16_t data);    
  void  NEGRdRm(uint16_t data);     
  void  CMPRmRn(uint16_t data);    
  void  CMNRmRn(uint16_t data);    
  void  ORRRdRm(uint16_t data);    
  void  MULRdRm(uint16_t data);     
  void  BICRmRd(uint16_t data);    
  void  MVNRdRm(uint16_t data);  
  
  void  MOVSRdRm(uint16_t data);
  void  MOVRsImm(uint16_t data);
  void  ASRSRdRmImm(uint16_t data);
  void  STRHRtRnImm(uint16_t data);
  void  STRBRtRnImm(uint16_t data);
  void  LDRHRtRnImm(uint16_t data);
  void  LDRBRtRnImm(uint16_t data);
  void  NOP();
  
  void  CBZ(uint16_t data);
  void  CBNZ(uint16_t data);
  
  
  void SetZFlag(uint32_t data);
  void SetCFlag(uint64_t data);
  void SetNFlag(uint32_t data);
  void SetVFlag(uint32_t data);
  
  void  MOVWRdimm16(P_ARM_PRG_STRUCT pdata);
  void  MOVTRdimm16(P_ARM_PRG_STRUCT pdata);
  
  uint8_t CalcShift(uint16_t data);
  
  bool IsDataBound(uint32_t address);
  ARM_PRG_STRUCT Calcdimm16(ARM_PRG_STRUCT pdata);
  int Get5bitOffset(uint16_t data);
  int Get5bitShift(uint16_t data);
  
  void LSRSRdRmImm5(uint16_t data);
  
  void BXRm(uint16_t);
  
  int Test32BitsCommands(P_ARM_PRG_STRUCT PPS);
  int Test4bitsMaskCommans(uint16_t data);
  int Test5bitsMaskCommans(uint16_t data);
  int Test8bitsMaskCommans(uint16_t data);
  int Test9bitsMaskCommans(uint16_t data);
 
  void ADD_WRdRnC(P_ARM_PRG_STRUCT pdata);
  void LDMIA_WRnRegs(P_ARM_PRG_STRUCT PPS);
  void MOV_WRdC(P_ARM_PRG_STRUCT PPS);
  
  void B_WLab_I(P_ARM_PRG_STRUCT PPS);
  void B_WLab_T(P_ARM_PRG_STRUCT PPS);
  
  int Test10bitMask(uint16_t data);
  int Test9bitMask(uint16_t data);
  int Test8bitMask(uint16_t data);
  int Test5bitMask(uint16_t data);
  int Test4bitMask(uint16_t data);
  
};

#endif	/* ARMCLASS_H */

