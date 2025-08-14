	/**@file  dr16.c
	* @brief    设备层
	* @details  主要包括构建串口管理器，提供串口初始化和用户回调重定义
	* @author      RyanJiao  any question please send mail to 1095981200@qq.com
	* @date        2021-10-9
	* @version     V1.0
	* @copyright    Copyright (c) 2021-2121  中国矿业大学CUBOT战队
	**********************************************************************************
	* @attention
	* 硬件平台: STM32H750VBT \n
	* SDK版本：-++++
	* @par 修改日志:
	* <table>
	* <tr><th>Date        <th>Version  <th>Author    <th>Description
	* <tr><td>2021-8-12  <td>1.0      <td>RyanJiao  <td>创建初始版本
	* </table>
	*
	**********************************************************************************
	 ==============================================================================
														How to use this driver  
	 ==============================================================================
	 

		********************************************************************************
		* @attention
		* 硬件平台: STM32H750VBT \n
		* SDK版本：-++++
		* if you had modified this file, please make sure your code does not have many 
		* bugs, update the version NO., write dowm your name and the date, the most
		* important is make sure the users will have clear and definite understanding 
		* through your new brief.
		********************************************************************************
	*/

#include "dr16.h"
#include "hardware_config.h"
#include "control_logic.h"

RC_Ctrl rc_Ctrl = {.isUnpackaging = 0,
                   .is_online      = 0};

/**
 * @brief  创建dr16的接收机缓存数组
 */
uint8_t DR16_recData[DR16_rxBufferLengh] __attribute__((at(0x24006010)));
uint8_t DR16_RxBuffer0[36];
uint8_t DR16_RxBuffer1[36];

/**
 * @brief  创建dr16串口缓存区数据结构
 */
UART_RxBuffer uart1_buffer = {
    .Data = DR16_recData,
    .DataSize = DR16_rxBufferLengh};

/**
 * @brief 遥控器串口中断回调函数
 *
 * @param recBuffer
 * @param len
 * @return uint8_t
 */
uint8_t DR16_Callback(uint8_t *recBuffer, uint16_t len)
{
    
    DR16_DataUnpack(&rc_Ctrl, recBuffer);
	if(rc_Ctrl.FPS > 30)
	rc_Ctrl.online_cnt = 0;
	
    return 0;
}

/**
 * @brief  初始化接收机数据类型的数据，将杆量和按键信息归零
 */
void DR16Init(RC_Ctrl *rc_ctrl)
{
    rc_ctrl->rc.ch0          = 1024;
    rc_ctrl->rc.ch1          = 1024;
    rc_ctrl->rc.ch2          = 1024;
    rc_ctrl->rc.ch3          = 1024;
    rc_ctrl->rc.s1           = 3;
    rc_ctrl->rc.s2           = 3;
    rc_ctrl->rc.sw           = 1024;
	rc_ctrl->online_cnt      = 50;
}

/**
 * @brief  创建dr16的接收机缓存数组, 并对全局变量rc_Ctrl赋值，以供其他函数调用
 */
void DR16_DataUnpack(RC_Ctrl *rc_ctrl, uint8_t *recBuffer)
{
	rc_ctrl->FPS++;
	if (rc_ctrl->FPS>1000)   rc_ctrl->FPS=1000;
    // 解算期间不允许读取数据
    rc_ctrl->isUnpackaging = 1;
    uint8_t correct_num    = 0;
    correct_num            = 0;
    if (((recBuffer[0] | (recBuffer[1] << 8)) & 0x07ff) <= 1684 && ((recBuffer[0] | (recBuffer[1] << 8)) & 0x07ff) >= 364)
        correct_num++;
    if ((((recBuffer[1] >> 3) | (recBuffer[2] << 5)) & 0x07ff) <= 1684 && (((recBuffer[1] >> 3) | (recBuffer[2] << 5)) & 0x07ff) >= 364)
        correct_num++;
    if ((((recBuffer[2] >> 6) | (recBuffer[3] << 2) | (recBuffer[4] << 10)) & 0x07ff) <= 1684 && (((recBuffer[2] >> 6) | (recBuffer[3] << 2) | (recBuffer[4] << 10)) & 0x07ff) >= 364)
        correct_num++;
    if ((((recBuffer[4] >> 1) | (recBuffer[5] << 7)) & 0x07ff) <= 1684 && (((recBuffer[4] >> 1) | (recBuffer[5] << 7)) & 0x07ff) >= 364)
        correct_num++;
    if ((((recBuffer[5] >> 4) & 0x000C) >> 2) == 1 || (((recBuffer[5] >> 4) & 0x000C) >> 2) == 2 || (((recBuffer[5] >> 4) & 0x000C) >> 2) == 3)
        correct_num++;
    if (((recBuffer[5] >> 4) & 0x0003) == 1 || ((recBuffer[5] >> 4) & 0x0003) == 2 || ((recBuffer[5] >> 4) & 0x0003) == 3)
        correct_num++;
    if (correct_num == 6) {
        rc_ctrl->rc.ch0 = (recBuffer[0] | (recBuffer[1] << 8)) & 0x07ff;
        rc_ctrl->rc.ch1 = ((recBuffer[1] >> 3) | (recBuffer[2] << 5)) & 0x07ff;
        rc_ctrl->rc.ch2 = ((recBuffer[2] >> 6) | (recBuffer[3] << 2) | (recBuffer[4] << 10)) & 0x07ff;
        rc_ctrl->rc.ch3 = ((recBuffer[4] >> 1) | (recBuffer[5] << 7)) & 0x07ff;
        rc_ctrl->rc.s1  = ((recBuffer[5] >> 4) & 0x000C) >> 2;
        rc_ctrl->rc.s2  = ((recBuffer[5] >> 4) & 0x0003);
        rc_ctrl->rc.sw  = (uint16_t)(recBuffer[16] | (recBuffer[17] << 8)) & 0x7ff;

        if ((rc_ctrl->rc.ch0 > 1020) && (rc_ctrl->rc.ch0 < 1028)) // 遥控器零飘
            rc_ctrl->rc.ch0 = 1024;
        if ((rc_ctrl->rc.ch1 > 1020) && (rc_ctrl->rc.ch1 < 1028))
            rc_ctrl->rc.ch1 = 1024;
        if ((rc_ctrl->rc.ch2 > 1020) && (rc_ctrl->rc.ch2 < 1028))
            rc_ctrl->rc.ch2 = 1024;
        if ((rc_ctrl->rc.ch3 > 1020) && (rc_ctrl->rc.ch3 < 1028))
            rc_ctrl->rc.ch3 = 1024;
  
    }

    rc_ctrl->isUnpackaging = 0; // 解算完成标志位，允许读取

}
