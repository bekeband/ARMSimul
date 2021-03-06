/* 
 * File:   main.cpp
 * Author: user
 *
 * Created on 2016. február 20., 6:49
 */

#include <cstdlib>
#include <iostream>

#include "ARMClass.h"

using namespace std;

#define BEGIN_ADDR 0x0136

uint32_t PC_ADDR = BEGIN_ADDR;

/*
 * 
 */
int main(int argc, char** argv) {
  int codesize;
  while (argc--)
  {
    
  }
  ARMClass ARM = ARMClass();
  cout << "Hello World!" << endl;
  ARM.GetBinFile("firmware.bin");
//  ARM.PrintPrgMemory(0x0F4, 0x100);
  ARM.SetDisasmAddr(BEGIN_ADDR);
  ARM.SetPC(PC_ADDR);
  ARM.WriteSimulateMode(true);
  for (int i = 0; i < 30; i++)
  {
    codesize = ARM.Simulate();
    ARM.AddDisasmAddr(codesize);
    PC_ADDR += codesize;
    ARM.SetPC(PC_ADDR);
//    ARM.SetDisasmAddr(0x1D94 + i);
  };
  return 0;
}

