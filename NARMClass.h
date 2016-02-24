/* 
 * File:   NARMClass.h
 * Author: user
 *
 * Created on 2016. február 24., 9:26
 */

#ifndef NARMCLASS_H
#define	NARMCLASS_H

#include <stdint.h>

class NARMClass {
public:
  NARMClass();
  NARMClass(const NARMClass& orig);
  virtual ~NARMClass();
private:

  void LSRSRdRmImm5(uint16_t data);
  void BXRm(uint16_t data);
  
//  void ADD_WRdRnC(P_ARM_PRG_STRUCT pdata);
};

#endif	/* NARMCLASS_H */

