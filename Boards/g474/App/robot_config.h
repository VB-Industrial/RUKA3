/*
 * ruka_joints.h
 *
 *  Created on: Jan 31, 2024
 *      Author: VR
 */

#ifndef INC_RUKA_JOINTS_H_
#define INC_RUKA_JOINTS_H_

#include "../../../App/robot_config.h"

#ifndef JOINT_N
#error "JOINT_N must be provided by the RUKA3_JOINT_INDEX CMake setting"
#endif

#define AGENT_JS_SUB_PORT 1001

#define JOINT_N_1 101
#define JOINT_N_2 102
#define JOINT_N_3 103
#define JOINT_N_4 104
#define JOINT_N_5 105
#define JOINT_N_6 106

#define JS_PUB_1_PORT_ID 1111
#define JS_SUB_1_PORT_ID 1121


#define JS_PUB_2_PORT_ID 1112
#define JS_SUB_2_PORT_ID 1122


#define JS_PUB_3_PORT_ID 1113
#define JS_SUB_3_PORT_ID 1123


#define JS_PUB_4_PORT_ID 1114
#define JS_SUB_4_PORT_ID 1124

 
#define JS_PUB_5_PORT_ID 1115
#define JS_SUB_5_PORT_ID 1125


#define JS_PUB_6_PORT_ID 1116
#define JS_SUB_6_PORT_ID 1126


#define AGENT_IMU_PORT 1337

//#define JS_PUB_PORT_ID
//#define JS_SUB_PORT_ID

#if JOINT_N == 1
#define JS_PUB_PORT_ID JS_PUB_1_PORT_ID
#define JS_SUB_PORT_ID JS_SUB_1_PORT_ID
constexpr bool js_control_enabled = true;
constexpr int8_t start_angle_according_to_DH = 0;
constexpr int8_t DH_rotation = -1; 
constexpr uint8_t gear_ratio = 72;
constexpr encoder_data electric_offset = 1205; 
constexpr encoder_data gear_encoder_zero = 1206; 
constexpr int8_t motor_encoder_sign = -1;
constexpr int8_t motor_command_sign = -1;
constexpr int8_t gear_encoder_sign = -1;
constexpr float upper_limit = 0;
constexpr float lower_limit = 0;
#endif

#if JOINT_N == 2
#define JS_PUB_PORT_ID JS_PUB_2_PORT_ID
#define JS_SUB_PORT_ID JS_SUB_2_PORT_ID
constexpr bool js_control_enabled = true;
constexpr int8_t start_angle_according_to_DH = 0;
constexpr int8_t DH_rotation = -1; 
constexpr uint8_t gear_ratio = 36;
constexpr encoder_data electric_offset = 555; 
constexpr encoder_data gear_encoder_zero = 11000; 
constexpr int8_t motor_encoder_sign = -1;
constexpr int8_t motor_command_sign = -1;
constexpr int8_t gear_encoder_sign = 1;
constexpr float upper_limit = 0;
constexpr float lower_limit = 0;
#endif

#if JOINT_N == 3
#define JS_PUB_PORT_ID JS_PUB_3_PORT_ID
#define JS_SUB_PORT_ID JS_SUB_3_PORT_ID
constexpr bool js_control_enabled = true;
constexpr int8_t start_angle_according_to_DH = 0;
constexpr int8_t DH_rotation = 1; 
constexpr uint8_t gear_ratio = 36;
constexpr encoder_data electric_offset = 1320; 
constexpr encoder_data gear_encoder_zero = 9956;
constexpr int8_t motor_encoder_sign = -1;
constexpr int8_t motor_command_sign = -1;
constexpr int8_t gear_encoder_sign = 1;
constexpr float upper_limit = 0;
constexpr float lower_limit = 0;
#endif

#if JOINT_N == 4
#define JS_PUB_PORT_ID JS_PUB_4_PORT_ID
#define JS_SUB_PORT_ID JS_SUB_4_PORT_ID
constexpr bool js_control_enabled = true;
constexpr int8_t start_angle_according_to_DH = 0;
constexpr int8_t DH_rotation = 1; 
constexpr uint8_t gear_ratio = 36;
constexpr encoder_data electric_offset = 16179; 
constexpr encoder_data gear_encoder_zero = 5000; 
constexpr int8_t motor_encoder_sign = -1;
constexpr int8_t motor_command_sign = 1;
constexpr int8_t gear_encoder_sign = -1;
constexpr float upper_limit = 0;
constexpr float lower_limit = 0;
#endif

#if JOINT_N == 5
#define JS_PUB_PORT_ID JS_PUB_5_PORT_ID
#define JS_SUB_PORT_ID JS_SUB_5_PORT_ID
constexpr bool js_control_enabled = true;
constexpr int8_t start_angle_according_to_DH = 0;
constexpr int8_t DH_rotation = -1; 
constexpr uint8_t gear_ratio = 36;
constexpr encoder_data electric_offset = 0; 
constexpr encoder_data gear_encoder_zero = 0; 
constexpr int8_t motor_encoder_sign = 1;
constexpr int8_t motor_command_sign = -1;
constexpr int8_t gear_encoder_sign = 1;
constexpr float upper_limit = 0;
constexpr float lower_limit = 0;
#endif


#if JOINT_N == 6
#define JS_PUB_PORT_ID JS_PUB_6_PORT_ID
#define JS_SUB_PORT_ID JS_SUB_6_PORT_ID
constexpr bool js_control_enabled = true;
constexpr int8_t start_angle_according_to_DH = 0;
constexpr int8_t DH_rotation = -1; 
constexpr uint8_t gear_ratio = 36;
constexpr encoder_data electric_offset = 0; 
constexpr encoder_data gear_encoder_zero = 0; 
constexpr int8_t motor_encoder_sign = 1;
constexpr int8_t motor_command_sign = -1;
constexpr int8_t gear_encoder_sign = 1;
constexpr float upper_limit = 0;
constexpr float lower_limit = 0;
#endif


#endif /* INC_RUKA_JOINTS_H_ */
