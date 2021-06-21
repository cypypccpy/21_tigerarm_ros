//
// Modified by Carcuis on 2021/4/20.
//

#ifndef INC_21_VISION_ENGINEER_DATASTRUCT_H
#define INC_21_VISION_ENGINEER_DATASTRUCT_H

/**
 * @brief 数据结构体 --发送
 */
#pragma pack(1)
struct DataStruct_Send
{
    uint8_t Flag;
    float x_up;
    float y_up;
    float z_up;
    float x_down;
    float y_down;
    float z_down; //yaw
    bool get_mineral_success_sign;
    uint8_t sign;
    uint8_t surface;
    uint8_t move_w = 0;
    uint8_t move_s = 0;
    uint8_t move_a = 0;
    uint8_t move_d = 0;
    uint8_t move_q = 0;
    uint8_t move_e = 0;
    uint8_t move_shift = 0;
    uint8_t End;
};
#pragma pack()

/**
 * @brief 数据结构体 --接收
 */
#pragma pack(1)
struct DataStruct_Get
{
    uint8_t switch_camera_sign;
    uint8_t current_status;
    uint8_t program_control;
};
#pragma pack()

#endif //INC_21_VISION_ENGINEER_DATASTRUCT_H
