#include "ti_msp_dl_config.h"
#include "Delay.h"
#include "Motor.h"
#include "PWM.h"
#include "sensor.h"
#include "Encoder.h"
#include "line.h"
#include "Key.h"             
#include "menu.h"            
#include "oled_hardware_i2c.h" 

volatile uint32_t time = 0; 
void SysTick_Handler(void) {
    time++;  
}

int main(void) {
    // 1. 初始化底层硬件
    SYSCFG_DL_init();
    NVIC_EnableIRQ(ENCODER_R_INT_IRQN);
    DL_SYSTICK_enable();

    // 2. 初始化核心外设
    Delay_Init();
    Motor_Init();
    key_init();    
    
    // 延时等待 OLED 稳定，防止雪花屏
    Delay_ms(500); 
    OLED_Init();   

    while(1) {
        // 3. 呼出菜单界面 (程序会停在这里，直到按下确认键发车)
        Menu_Init();
        Menu_Run(); 
        
        // 4. 发车倒计时 (坐标为页地址)
        OLED_Clear();
        Delay_ms(50); 
        OLED_ShowString(0, 2, (uint8_t*)"  Ready...  ", 16); 
        Delay_ms(1000); 
        OLED_Clear();
        Delay_ms(50);
        OLED_ShowString(0, 2, (uint8_t*)"    GO!     ", 16); 
        
        // 5. 重置 PID，准备发车
        PID_Reset();

        // 6. 执行跑图 (将菜单里的 Loop_Target 传进去！)
        if (Map_Selection == 1) {
            Run_Normal_Map(Loop_Target);
        } else {
            Run_Square_Map(Loop_Target);
        }
        
        // 7. 跑完目标圈数后：刹车 -> 提示完成 -> 等待2秒 -> 自动回菜单
        motor(0, 0);
        OLED_Clear();
        Delay_ms(50);
        OLED_ShowString(0, 2, (uint8_t*)"  Finished! ", 16);
        Delay_ms(2000); 
    }
}