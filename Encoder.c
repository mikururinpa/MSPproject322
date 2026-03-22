#include "Encoder.h"
#include "ti_msp_dl_config.h" 

volatile int16_t encoder_r_count = 0;

void Encoder_Init_Tim2(void) { }
void Encoder_Init_Tim4(void) { }

int Read_Encoder(uint8_t TIMX) {
    int Encoder_TIM;
    if (TIMX == 4) { 
        // 【治本修复 1】：右轮是 1倍频软件中断，必须乘 4 归一化！
        // 否则两边数据永远不对等！
        Encoder_TIM = encoder_r_count * 4;
        encoder_r_count = 0; 
    }
    else if (TIMX == 2) { 
        // 左轮是 4倍频硬件QEI，保持原样
        Encoder_TIM = (int16_t)DL_TimerG_getTimerCount(ENCODER_L_INST);
        DL_TimerG_setTimerCount(ENCODER_L_INST, 0); 
    }
    else { Encoder_TIM = 0; }
    return Encoder_TIM;
}

// 软件外部中断保持不变
void GROUP1_IRQHandler(void) {
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case ENCODER_R_INT_IIDX:
            switch (DL_GPIO_getPendingInterrupt(ENCODER_R_PORT)) {
                case ENCODER_R_PH_A_IIDX:
                    if (DL_GPIO_readPins(ENCODER_R_PORT, ENCODER_R_PH_B_PIN)) {
                        encoder_r_count++; 
                    } else {
                        encoder_r_count--; 
                    }
                    break;
                default: break;
            }
            break;
        default: break;
    }
}