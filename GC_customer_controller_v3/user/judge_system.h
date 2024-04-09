#ifndef JUDGE_SYSTEM_H
#define JUDGE_SYSTEM_H

#include "CRC.h"
#include "main.h"

typedef __packed struct 
{
    float angle1;
    float angle2;
    float angle3;
    float angle4;
}customer_data_pack;


typedef __packed struct
{
	uint8_t  SOF;						
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
}FrameHeader;  

typedef __packed struct
{
 uint8_t game_type : 4;
 uint8_t game_progress : 4;
 uint16_t stage_remain_time;
 uint64_t SyncTimeStamp;
}game_status_t;

typedef __packed struct
{
 uint8_t winner;
}game_result_t;

typedef __packed struct
{
 uint16_t red_1_robot_HP;
 uint16_t red_2_robot_HP;
 uint16_t red_3_robot_HP;
 uint16_t red_4_robot_HP;
  uint16_t red_5_robot_HP;
 uint16_t red_7_robot_HP;
 uint16_t red_outpost_HP;
 uint16_t red_base_HP;
 uint16_t blue_1_robot_HP;
 uint16_t blue_2_robot_HP;
 uint16_t blue_3_robot_HP;
 uint16_t blue_4_robot_HP;
 uint16_t blue_5_robot_HP;
 uint16_t blue_7_robot_HP;
 uint16_t blue_outpost_HP;
 uint16_t blue_base_HP;
}game_robot_HP_t;

typedef __packed struct
{
 uint32_t event_data;
}event_data_t;

typedef __packed struct
{
 uint8_t reserved;
 uint8_t supply_robot_id;
 uint8_t supply_projectile_step;
 uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef __packed struct
{
 uint8_t level;
 uint8_t offending_robot_id;
 uint8_t count;
}referee_warning_t;

typedef __packed struct
{
 uint8_t dart_remaining_time;
 uint16_t dart_info;
}dart_info_t;

typedef __packed struct
{
 uint8_t robot_id;
uint8_t robot_level;
uint16_t current_HP; 
 uint16_t maximum_HP;
uint16_t shooter_barrel_cooling_value;
uint16_t shooter_barrel_heat_limit;
uint16_t chassis_power_limit;
 uint8_t power_management_gimbal_output : 1;
uint8_t power_management_chassis_output : 1; 
 uint8_t power_management_shooter_output : 1;
}robot_status_t;

typedef __packed struct
{
 uint16_t chassis_voltage;
 uint16_t chassis_current;
 float chassis_power;
 uint16_t buffer_energy;
 uint16_t shooter_17mm_1_barrel_heat;
 uint16_t shooter_17mm_2_barrel_heat;
 uint16_t shooter_42mm_barrel_heat;
}power_heat_data_t;

typedef __packed struct
{
 float x;
 float y;
 float angle;
}robot_pos_t;

typedef __packed struct
{
 uint8_t recovery_buff;
 uint8_t cooling_buff;
 uint8_t defence_buff;
 uint8_t vulnerability_buff;
 uint16_t attack_buff;
}buff_t;

typedef __packed struct
{
 uint8_t airforce_status;
 uint8_t time_remain;
}air_support_data_t;

typedef __packed struct
{
 uint8_t armor_id : 4;
 uint8_t HP_deduction_reason : 4;
}hurt_data_t;

typedef __packed struct
{
 uint8_t bullet_type;
 uint8_t shooter_number;
 uint8_t launching_frequency;
 float initial_speed;
}shoot_data_t;

typedef __packed struct
{
 uint16_t projectile_allowance_17mm;
 uint16_t projectile_allowance_42mm;
 uint16_t remaining_gold_coin;
}projectile_allowance_t;

typedef __packed struct
{
 uint32_t rfid_status;
}rfid_status_t;

typedef __packed struct
{
 uint8_t dart_launch_opening_status;
 uint8_t reserved;
 uint16_t target_change_time;
 uint16_t latest_launch_cmd_time;
}dart_client_cmd_t;

typedef __packed struct
{
 float hero_x;
 float hero_y;
 float engineer_x;
 float engineer_y;
 float standard_3_x;
 float standard_3_y;
 float standard_4_x;
 float standard_4_y;
 float standard_5_x;
 float standard_5_y;
}ground_robot_position_t;

typedef __packed struct
{
 uint8_t mark_hero_progress;
 uint8_t mark_engineer_progress;
 uint8_t mark_standard_3_progress;
 uint8_t mark_standard_4_progress;
 uint8_t mark_standard_5_progress;
 uint8_t mark_sentry_progress;
}radar_mark_data_t;

typedef __packed struct
{
 uint32_t sentry_info;
} sentry_info_t;

typedef __packed struct
{
 uint8_t radar_info;
} radar_info_t;

typedef __packed struct
{
 uint16_t data_cmd_id;
 uint16_t sender_id;
 uint16_t receiver_id;
 uint8_t user_data[113];
}robot_interaction_data_t;

typedef __packed struct
{
uint8_t delete_type;
uint8_t layer;
}interaction_layer_delete_t;

typedef __packed struct
{ 
uint8_t figure_name[3]; 
uint32_t operate_tpye:3; 
uint32_t figure_tpye:3; 
uint32_t layer:4; 
uint32_t color:4; 
uint32_t details_a:9;
uint32_t details_b:9;
uint32_t width:10; 
uint32_t start_x:11; 
uint32_t start_y:11; 
uint32_t details_c:10; 
uint32_t details_d:11; 
uint32_t details_e:11; 
}interaction_figure_t;

typedef __packed struct
{
 interaction_figure_t interaction_figure[2];
}interaction_figure_2_t;

typedef __packed struct
{
interaction_figure_t interaction_figure[5];
}interaction_figure_3_t;

typedef __packed struct
{
interaction_figure_t interaction_figure[7];
}interaction_figure_4_t;



typedef __packed struct 
{
    game_status_t game_status;
    game_result_t game_result;
    game_robot_HP_t game_robot_HP;
    event_data_t event_data;
    ext_supply_projectile_action_t ext_supply_projectile_action;
    referee_warning_t referee_warning;
    dart_info_t dart_info;
    robot_status_t robot_status;
    power_heat_data_t power_heat_data;
    robot_pos_t robot_pos;
    buff_t buff;
    air_support_data_t air_support_data;
    hurt_data_t hurt_data;
    shoot_data_t shoot_data;
    projectile_allowance_t projectile_allowance;
    rfid_status_t rfid_status;
    dart_client_cmd_t dart_client_cmd;
    ground_robot_position_t ground_robot_position;
    radar_mark_data_t radar_mark_data;
    sentry_info_t sentry_info_;
    radar_info_t radar_info_;
    robot_interaction_data_t robot_interaction_data;
    interaction_layer_delete_t interaction_layer_delete;
    interaction_figure_t interaction_figure;
    interaction_figure_2_t interaction_figure_2;
    interaction_figure_3_t interaction_figure_3;
    interaction_figure_4_t interaction_figure_4;
}JUDGE_SYSTEM;



#endif
