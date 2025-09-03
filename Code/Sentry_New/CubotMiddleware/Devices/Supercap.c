#include "dr16.h"
#include "Supercap.h"
#include "fdcan.h"
#include "driver_can.h"
#include "motor.h"
#include "referee.h"
#include "hardware_config.h"
CAN_TxBuffer txBuffer0x6FFforCAN1={
	.Identifier = 0x6ff
};
Supercap super_cap;

uint8_t Supercap_rxCallBack(CAN_Object canx, CAN_RxBuffer rxBuffer ,Supercap* Cap)
{
	if(rxBuffer.Header.Identifier == 0x601)
	{
	Cap->cap_state.Voltage = ((rxBuffer.Data[0]<<8)+rxBuffer.Data[1])*0.01f;
	Cap->cap_state.Current = ((rxBuffer.Data[2]<<8)+rxBuffer.Data[3]);
	}
	return 0;
}

void SupercapControl(CAN_Object can, Supercap* Cap)
{
	Cap->cap_state.voltage_flag = 1;
	
	if(Cap->cap_state.Voltage<5.0f)
	{
		Cap->cap_state.voltage_flag=0;
	}
	if(Cap->cap_state.Supercap_Mode == 1 && Cap->cap_state.voltage_flag==1)
	{
		txBuffer0x6FFforCAN1.Data[0] = 1;
		Cap->cap_state.Supercap_Flag = 1;
	}
	else
	{
		txBuffer0x6FFforCAN1.Data[0] = 0;
		Cap->cap_state.Supercap_Flag = 0;
	}
	if(Cap->cap_state.Supercap_Charge_mode==0)
		Cap->cap_state.Supercap_Charge=0;
	else if (Cap->cap_state.Supercap_Charge_mode==1)
		Cap->cap_state.Supercap_Charge=1;


	txBuffer0x6FFforCAN1.Data[1] = referee2022.game_robot_status.chassis_power_limit;
	txBuffer0x6FFforCAN1.Data[2] = 80;
	txBuffer0x6FFforCAN1.Data[3] = referee2022.power_heat_data.chassis_power;
	txBuffer0x6FFforCAN1.Data[4] = referee2022.power_heat_data.chassis_current;
	txBuffer0x6FFforCAN1.Data[5] = referee2022.power_heat_data.chassis_volt;
	txBuffer0x6FFforCAN1.Data[6] = 0;
	txBuffer0x6FFforCAN1.Data[7] = 0;
	
	CAN_Send(&can, &txBuffer0x6FFforCAN1);   
}
