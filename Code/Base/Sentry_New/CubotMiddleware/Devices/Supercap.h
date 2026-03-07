#ifndef SUPERCAP_H
#define SUPERCAP_H
#include "driver_can.h"
typedef struct
{
	  struct
  {
	  float    Voltage;
	  float    Current;
	  uint8_t  Supercap_Mode;
		uint8_t  Supercap_Flag;
		uint8_t  Supercap_Charge;
		uint8_t  Supercap_Charge_mode;
		uint8_t  voltage_flag;
  }cap_state;
}Supercap;
extern Supercap super_cap;

void SupercapControl(CAN_Object can,Supercap* Cap);

uint8_t Supercap_rxCallBack(CAN_Object canx, CAN_RxBuffer rxBuffer , Supercap* Cap); 
#endif
