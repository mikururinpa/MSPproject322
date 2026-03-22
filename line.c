#include "line.h"
#include "Motor.h"
#include "Encoder.h"
#include "sensor.h"
#include "Delay.h"
#include "oled_hardware_i2c.h" 

int Velocity_PWM1, Velocity_PWM2;
int encoder_A, encoder_B;
int Turn_Lock_State = 0;

float Velcity_Kp1 = 0.0001f, Velcity_Ki1 = 0.0000001f, Velcity_Kd1 = 1.06f;
float Velcity_Kp2 = 0.0001f, Velcity_Ki2 = 0.0000001f, Velcity_Kd2 = 1.06f;

int PWM_BASIC = 2600;
int OFFSET_RIGHT = 180; 
int OFFSET_LEFT  = 0;   

pid right = {0, 0, 0};
pid left = {0, 0, 0};
char RorL = 'S'; 

int Vel_A_ControlVelocity = 0, Vel_A_Last_bias = 0;
int Vel_B_ControlVelocity = 0, Vel_B_Last_bias = 0;

void PID_Reset(void){
    left.now = 0; left.last = 0; left.sum = 0;
    right.now = 0; right.last = 0; right.sum = 0;
    Vel_A_ControlVelocity = 0; Vel_A_Last_bias = 0;
    Vel_B_ControlVelocity = 0; Vel_B_Last_bias = 0;
    RorL = 'S'; Turn_Lock_State = 0;
    Read_Encoder(4); Read_Encoder(2);
    motor(0, 0); Delay_ms(200);
}

int Velocity_A(int target, int encoder) {
    int Bias = target - encoder;
    Vel_A_ControlVelocity += Velcity_Kd1 * (Bias - Vel_A_Last_bias) + Velcity_Ki1 * Bias + Velcity_Kp1 * Bias;
    Vel_A_Last_bias = Bias;
    return Vel_A_ControlVelocity;
}

int Velocity_B(int target, int encoder) {
    int Bias = target - encoder;
    Vel_B_ControlVelocity += Velcity_Kd2 * (Bias - Vel_B_Last_bias) + Velcity_Ki2 * Bias + Velcity_Kp2 * Bias;
    Vel_B_Last_bias = Bias;
    return Vel_B_ControlVelocity;
}

int whiteall(void) {
    if (L3 + L2 + L1 + M0 + M1 + R1 + R2 + R3 == 0) return 1;
    else return 0;
}

void error_get(void) {
    int error1 = 40, error2 = 400, error3 = 1800; 
    
    // 回到内侧4个传感器时解除状态锁
    if (L1 == 1 || M0 == 1 || M1 == 1 || R1 == 1) { 
        Turn_Lock_State = 0; 
    }

    if (Turn_Lock_State != 2) { 
        if (L1 == 1) { left.now -= error1; right.now += error1; } 
        if (L2 == 1) { left.now -= error2; right.now += error2; } 
        if (L3 == 1) { left.now -= error3; right.now += error3; RorL = 'L'; Turn_Lock_State = 1; } 
    }

    if (Turn_Lock_State != 1) {
        if (R1 == 1) { left.now += error1; right.now -= error1; } 
        if (R2 == 1) { left.now += error2; right.now -= error2; } 
        if (R3 == 1) { left.now += error3; right.now -= error3; RorL = 'R'; Turn_Lock_State = 2; } 
    }
}

void go_along_last(char RorL, int direction) {
    int turn_speed = -2500;   
    int forward_speed = 3500; 
    
    if (RorL == 'L') {
        motor(forward_speed, turn_speed); 
    } else if (RorL == 'R') {
        motor(turn_speed, forward_speed); 
    } else {
        motor(PWM_BASIC + OFFSET_RIGHT, PWM_BASIC + OFFSET_LEFT);      
    }
}

void setspeed_pid(int direction) {
    int PWM_Right, PWM_Left; 
    
    left.now = 0; right.now = 0;
    error_get();
    
    int encoder_Right = Read_Encoder(4);  
    int encoder_Left  = Read_Encoder(2);  
    
    if (whiteall()) {
        go_along_last(RorL, direction);
        left.last = 0; right.last = 0;
        
        // 【修正力度变大问题】：在脱线转弯时清空PID积分项，防止转弯力度累加
        left.sum = 0; right.sum = 0; 
        Vel_A_ControlVelocity = 0; Vel_B_ControlVelocity = 0;
        return; 
    }
    
    left.sum += left.now; right.sum += right.now;
    int Integral_Limit = 2000; 
    if (left.sum > Integral_Limit) left.sum = Integral_Limit;
    if (left.sum < -Integral_Limit) left.sum = -Integral_Limit;
    if (right.sum > Integral_Limit) right.sum = Integral_Limit;
    if (right.sum < -Integral_Limit) right.sum = -Integral_Limit;
    
    Velocity_PWM1 = (PWM_BASIC + OFFSET_RIGHT) + Kp * right.now + Ki * right.sum + Kd * (right.now - right.last); 
    Velocity_PWM2 = (PWM_BASIC + OFFSET_LEFT)  + Kp * left.now  + Ki * left.sum  + Kd * (left.now - left.last);   
    
    PWM_Right = Velocity_A(Velocity_PWM1, encoder_Right); 
    PWM_Left  = Velocity_B(Velocity_PWM2, encoder_Left);  
    
    motor(PWM_Right, PWM_Left);
    
    left.last = left.now; right.last = right.now;
}

// ==========================================
// 三段式状态机跑圈：完全按照“白进黑 -> 锁定0.15s -> 黑进白 -> 计数”逻辑
// ==========================================
void Run_Square_Map(int target_loops) {
    int Total_Corners = target_loops * 4; 
    int Corner_Count = 0;
    int Current_Loop = 0;
    
    uint32_t T_Start_Run = time;      
    uint32_t T_Compensation = 0;      
    uint32_t Turn_Start_Time = 0; 
    int Is_First_Corner = 1; 
    
    // 状态机：
    // 0 = 直道等待 (白进黑)
    // 1 = 刚入弯，锁定 0.15 秒
    // 2 = 等待出弯 (黑进白)
    int Corner_State = 0; 
    
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t*)"Map: Square", 16); 
    OLED_ShowString(0, 2, (uint8_t*)"Loop: 0 / ", 16);   
    OLED_ShowNum(80, 2, target_loops, 1, 16);           
    OLED_ShowString(0, 4, (uint8_t*)"Corner: 0", 16);    
    
    // 起步延时，防抖
    while (time - T_Start_Run < 200) {
        setspeed_pid(1);
        Delay_ms(5);
    }

    while (1) {
        setspeed_pid(1);
        Delay_ms(5);
        
        // --- 状态 0：直道，等待 L3/R3 变黑 ---
        if (Corner_State == 0) {
            if (L3 == 1 || R3 == 1) { 
                Corner_State = 1;         // 进入转弯锁定状态
                Turn_Start_Time = time;   // 记录转弯开始时间
                
                if (Is_First_Corner) { 
                    T_Compensation = time - T_Start_Run; 
                    Is_First_Corner = 0; 
                }
            }
        }
        
        // --- 状态 1：转弯中，锁定 150ms ---
        else if (Corner_State == 1) {
            // 此阶段只转弯，不处理传感器信号
            if (time - Turn_Start_Time > 150) {
                Corner_State = 2;         // 0.15 秒结束后，准备检测出弯
            }
        }
        
        // --- 状态 2：检测出弯，等待 L3/R3 变白 ---
        else if (Corner_State == 2) {
            if (L3 == 0 && R3 == 0) {
                Corner_State = 0;         // 恢复为直道状态
                Corner_Count++;           // 【此时转弯彻底结束，计 1 次过弯】
                
                Current_Loop = Corner_Count / 4; 
                OLED_ShowNum(48, 2, Current_Loop, 1, 16); 
                OLED_ShowNum(64, 4, Corner_Count, 2, 16); 
            }
        }

        // 达到目标弯道数，停车补偿
        if (Corner_Count >= Total_Corners) {
            uint32_t T_End_Start = time;
            OLED_ShowString(0, 6, (uint8_t*)"Auto Homing...", 16); 
            
            uint32_t Final_Run_Time = (uint32_t)(T_Compensation * 0.80f);
            
            while (time - T_End_Start < Final_Run_Time) {
                setspeed_pid(1);
                Delay_ms(5);
            }
            
            motor(0, 0); 
            return; 
        }
    }
}

void Run_Normal_Map(int target_loops) {
    Run_Square_Map(target_loops);
}