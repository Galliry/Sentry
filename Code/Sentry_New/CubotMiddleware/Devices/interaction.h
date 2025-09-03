#ifndef INTERACTION_H__
#define INTERACTION_H__
#include "stm32h7xx.h"


#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


#define SCREEN_WIDTH 1080
#define SCREEN_LENGTH 1920


//< cmd_id
#define ROBOTS_INTERACTION_ID 0x0301  //< ???????

//< data_cmd_id
#define DRAW_CHAR_ID 0x0110					  //< ???????????
#define DELETE_GRAPHIC 0x0100         //< ??????????
#define DRAW_SINGLE_GRAPHIC 0x0101    //< ???????????
#define DRAW_DOUBLE_GRAPHIC 0x0102    //< ???????????
#define DRAW_FIVE_GRAPHIC 0x0103      //< ???????????
#define DRAW_SEVEN_GRAPHIC 0x0104      //< ???????????


//< robots' & clients' ID
#define RobotRedHero 1
#define RobotRedEngineer 2
#define RobotRedInfantryNO3 3
#define RobotRedInfantryNO4 4
#define RobotRedInfantryNO5 5
#define RobotRedAerial 6
#define RobotRedSentry 7
#define RobotRedRadar 9

#define ClientRedHero 0x0101
#define ClientRedEngineer 0x0102
#define ClientRedInfantryNO3 0x0103
#define ClientRedInfantryNO4 0x0104
#define ClientRedInfantryNO5 0x0105
#define ClientRedAerial 0x0106

#define RobotBlueHero 101
#define RobotBlueEngineer 102
#define RobotBlueInfantryNO3 103
#define RobotBlueInfantryNO4 104
#define RobotBlueInfantryNO5 105
#define RobotBlueAerial 106
#define RobotBlueSentry 107
#define RobotBlueRadar 109

#define ClientBlueHero 0x0165
#define ClientBlueEngineer 0x0166
#define ClientBlueInfantryNO3 0x0167
#define ClientBlueInfantryNO4 0x0168
#define ClientBlueInfantryNO5 0x0169
#define ClientBlueAerial 0x016A


#define HighVolt 27
#define LowVolt 16



typedef enum
{
	NONE_delete    = 0,
	GRAPHIC_delete = 1,
	ALL_delete     = 2
}delete_Graphic_Operate;  //ext_client_custom_graphic_delete_t:uint8_t operate_type
/*??????*/


//bit 0-2
typedef enum
{
	NONE   = 0,/*???*/
	ADD    = 1,/*????*/
	MODIFY = 2,/*????*/
	DELETE = 3,/*????*/
}Graphic_Operate;

typedef enum
{
	LINE      = 0,//??
	RECTANGLE = 1,//??
	CIRCLE    = 2,//??
	OVAL      = 3,//??
	ARC       = 4,//??
	FLOAT     = 5,//???
	INT       = 6,//???
	CHAR      = 7 //??
}Graphic_Type;

typedef enum
{
	RED_BLUE  = 0,//????	
	YELLOW    = 1,
	GREEN     = 2,
	ORANGE    = 3,
	FUCHSIA   = 4,	/*???*/
	PINK      = 5,
	CYAN_BLUE = 6,	/*??*/
	BLACK     = 7,
	WHITE     = 8
}Graphic_Color;


/*
 * ??????
 */
typedef __packed struct
{
	uint8_t SOF;			    //0xA5
	uint16_t data_length; //data ???
	uint8_t seq;				  //???
	uint8_t CRC8;
}frame_header_t;


/*
 * ??????,????????????id????id
 */
typedef __packed struct //?????
{
 uint16_t data_cmd_id;
 uint16_t sender_ID;
 uint16_t receiver_ID;
}ext_student_interactive_header_data_t;


typedef __packed struct //0x100 ????
{
	uint8_t operate_tpye; 
	uint8_t layer; 
}ext_client_custom_graphic_delete_t;


/*
 * ????????????
 */
typedef __packed struct 
{ 
uint8_t graphic_name[3]; //????
uint32_t operate_tpye:3; //????				0 ?				1 ??				2	??				3	??
uint32_t graphic_tpye:3; //????				0 ??			1 ??				2 ??				4	??				5 ???				6 ???				7 ??
uint32_t layer:4; //??
uint32_t color:4; //??
uint32_t start_angle:9;//????
uint32_t end_angle:9;//????
uint32_t width:10; //??
uint32_t start_x:11; //??X??
uint32_t start_y:11;//??Y??
uint32_t radius:10; //???????
uint32_t end_x:11; //??X??
uint32_t end_y:11; //??Y??
}graphic_data_struct_t;


typedef struct
{
	//< frame_header(5-byte) 
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) 
	uint16_t cmd_id;
	
	//< data(51-byte)
	ext_student_interactive_header_data_t ext_student_interactive_header_data; 
	graphic_data_struct_t grapic_data_struct;																		
	uint8_t data[30];																				
	
	//< frame_tail(2-byte,CRC16)
	uint8_t CRC16[2];
}graphic_string_t;        


typedef struct {
	
	graphic_string_t graphic_string_brain;//对应字符图形的结构体
	
/*
 * UI????????????,?30???
 */
 __packed struct
{ 
	//< frame_header(5-byte) ????
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ???
	uint16_t cmd_id;
	
	//< data(21-byte)
	ext_student_interactive_header_data_t ext_student_interactive_header_data;  //< ?6???
	ext_client_custom_graphic_delete_t ext_client_custom_graphic_delete;																		//< ?15???

	//< frame_tail(2-byte,CRC16,????)
	uint8_t CRC16[2];
}ext_client_graphic_delete;  


__packed struct
{ 
	//< frame_header(5-byte) ????
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ???
	uint16_t cmd_id;
	
	//< data(81-byte)
	ext_student_interactive_header_data_t ext_student_interactive_header_data;  	//< ?6???
	graphic_data_struct_t grapic_data_struct[7];																	//< ?30???

	//< frame_tail(2-byte,CRC16,????)
	uint8_t CRC16[2];
}referee_draw_basic_1_struct;  

__packed struct
{ 
	//< frame_header(5-byte) ????
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ???
	uint16_t cmd_id;
	
	//< data(81-byte)
	ext_student_interactive_header_data_t ext_student_interactive_header_data;  	//< ?6???
	graphic_data_struct_t grapic_data_struct[7];																	//< ?30???

	//< frame_tail(2-byte,CRC16,????)
	uint8_t CRC16[2];
}referee_draw_basic_2_struct;  

__packed struct
{ 
	//< frame_header(5-byte) ????
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ???
	uint16_t cmd_id;
	
	//< data(81-byte)
	ext_student_interactive_header_data_t ext_student_interactive_header_data;  	//< ?6???
	graphic_data_struct_t grapic_data_struct[7];																	//< ?30???

	//< frame_tail(2-byte,CRC16,????)
	uint8_t CRC16[2];
}referee_draw_patterning_1_struct;  



__packed struct
{ 
	//< frame_header(5-byte) ????
	frame_header_t frame_header;   
	
	//< cmd_id(2-byte) ???
	uint16_t cmd_id;
	
	//< data(81-byte)
	ext_student_interactive_header_data_t ext_student_interactive_header_data;  	//< ?6???
	graphic_data_struct_t grapic_data_struct[7];																	//< ?30???

	//< frame_tail(2-byte,CRC16,????)
	uint8_t CRC16[2];
}referee_draw_patterning_2_struct; 

}UI_t;


typedef struct {
	uint8_t control_line_cap;
	float VOLT;
	uint8_t control_hatch;
	uint8_t control_aiming;
	uint8_t control_roll;
	float pitch_data;
	float state_angle;
	uint8_t brain_state;
	uint8_t dafu_state;
}control_UI_ALL;


extern UI_t UI;
extern control_UI_ALL control_UI_all;


void referee_draw_string(uint8_t robot_id,char *string,graphic_string_t *patterning,uint8_t string_dex,uint8_t on_layer,uint8_t color,uint16_t size,uint16_t width, uint16_t x,uint16_t y);
void referee_draw_string_modify(uint8_t robot_id,char *string,graphic_string_t *patterning);

/*?????????*/
void draw_graph(graphic_data_struct_t *patterning, uint16_t index, uint8_t control_way, uint8_t graph, uint8_t layer, uint8_t color, uint32_t Sa, uint32_t Ea, uint8_t With,  uint16_t x,uint16_t y, uint8_t R, uint16_t Ex, uint16_t Ey);
void draw_graph_modify(graphic_data_struct_t *patterning);

/*??????*/
void referee_graphic_delete(uint8_t del_layer, uint8_t operation, uint8_t robot_id);

/*????,???,???,????*/
void update(uint32_t cnt, uint8_t robot_id, uint8_t change);


void UI_show(void);

void referee_draw_basic_1(uint8_t robot_id);
void referee_draw_basic_1_modify(uint8_t robot_id);

void referee_draw_basic_2(uint8_t robot_id);
void referee_draw_basic_2_modify(uint8_t robot_id);

void referee_draw_basic(uint8_t robot_id,uint32_t cnt);

void referee_draw_patterning_1(uint8_t robot_id);
void referee_draw_patterning_1_basic(uint8_t robot_id);
void referee_draw_patterning_1_basic_modify(uint8_t robot_id);


void referee_draw_patterning_2(uint8_t robot_id);
void referee_draw_patterning_2_basic(uint8_t robot_id);
void referee_draw_patterning_2_basic_modify(uint8_t robot_id);

void referee_draw_patterning_all_1(uint8_t robot_id,uint32_t cnt);

void referee_draw_modify_basic(uint8_t robot_id,uint32_t cnt);
void referee_draw_modify(uint8_t robot_id,uint32_t cnt);

/**
  * @brief  ?????????ID??????????ID
  * @param[in]  robot_id  	  ?????ID
  * @param[out] client_id  	  ??????ID
	*/
uint16_t referee_get_receiver_ID(uint32_t sender_id);




//extern uint8_t Data_Pack[19];
extern uint8_t UI_state;

unsigned char Get_CRC8_Check(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
uint16_t Get_CRC16_Check(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);

void Interact_Pack_Shoot(void);
void Client_Display(void);

#endif
