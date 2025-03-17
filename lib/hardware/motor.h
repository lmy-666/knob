#ifndef MOTOR_H
#define MOTOR_H

#include <SimpleFOC.h>

struct XKnobConfig {
    // 档位总数
    int32_t num_positions;        
    // 当前挡位
    int32_t position;             
    // 单个档位宽度（弧度）
    float position_width_radians; 
    // 正常旋转时的制动强度
    float detent_strength_unit;  
    // 超出界限后的制动强度
    float endstop_strength_unit;  
    // 触发档位切换的阈值比例。档位宽度 × snap_point（例如 60° × 0.55 = 33°）。
    float snap_point; 
    // 描述符            
    char descriptor[50];          
};

void motor_init(void);


#endif 
