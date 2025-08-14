#include "stm32h7xx_hal.h"
#include "usart.h"

#ifndef ET08_H
#define ET08_H

#include "driver_usart.h"



#define ET08_rxBufferLengh 25	 //< et08���ջ��������ݳ���


/**
  * @brief  ���ջ�������������, ����rcң�������ݡ�mouse������ݺ� keyflag ��������
  */
typedef struct { 
 struct {       
	 uint16_t ch0;       
	 uint16_t ch1;       
	 uint16_t ch2;      
	 uint16_t ch3;  
     uint8_t  sA; 	 
	 uint8_t  s1; 
	 uint8_t  s2;
     uint8_t  sD;	 
	 uint8_t  s1_last; 
	 uint8_t  s2_last; 
	}rc; 
 	uint8_t  isUnpackaging;  	 //< ����״̬��־λ����������в���ȡ����
	uint8_t  isOnline;
}RC_Ctrl_ET; 
 

/**
  * @brief  dr16���ݲ�ֽ��㺯��
	* @param[in] rc_ctrl  	Ϊ�ýṹ�帳ֵ
	* @param[in] recBuffer  ���ڽ��ջ�����
	* @param[in] len        �������������ݳ��ȣ�δʹ�ã�������ƥ�亯������
  */


/**
  * @brief  dr16���ݲ�ֽ��㺯��
	* @param[in] recBuffer  ���ڽ��ջ�����
	* @param[in] len        �������������ݳ��ȣ�δʹ�ã�������ƥ�亯������
  * @retval    RC_Ctl     ���ջ��������ͣ���������Ͱ�����Ϣ
  */
uint8_t ET08_callback(uint8_t * recBuffer, uint16_t len);

/**
  * @brief  ��ʼ�����ջ��������͵����ݣ��������Ͱ�����Ϣ����
	* @param[in] RC_Ctl  ���ջ����������׵�ַ����������Ͱ�����Ϣ
  * @retval    RC_Ctl  ���ջ����������׵�ַ����������Ͱ�����Ϣ
  */
void ET08Init(RC_Ctrl_ET* RC_Ctl);

void ET08_online_protection();



void ET08_DataUnpack(RC_Ctrl_ET* rc_ctrl, uint8_t * recBuffer, uint16_t len );


extern RC_Ctrl_ET rc_Ctrl_et;


#endif
