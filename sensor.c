#include "sensor.h"
#include "ti_msp_dl_config.h"

void SENSOR_GPIO_Config(void) { 
    // 引脚初始化已在 SysConfig (ti_msp_dl_config.c) 中完成
}

// 保持和 STM32 一致：读到高电平给算法返回 1 (代表黑线)，低电平返回 0 (代表白纸)
unsigned char digtal(unsigned char channel) {
    switch(channel) {
        case 1: return (DL_GPIO_readPins(SENSOR_PORT_S1_PORT, SENSOR_PORT_S1_PIN) ? 1 : 0);
        case 2: return (DL_GPIO_readPins(SENSOR_PORT_S2_PORT, SENSOR_PORT_S2_PIN) ? 1 : 0);
        case 3: return (DL_GPIO_readPins(SENSOR_PORT_S3_PORT, SENSOR_PORT_S3_PIN) ? 1 : 0);
        case 4: return (DL_GPIO_readPins(SENSOR_PORT_S4_PORT, SENSOR_PORT_S4_PIN) ? 1 : 0);
        case 5: return (DL_GPIO_readPins(SENSOR_PORT_S5_PORT, SENSOR_PORT_S5_PIN) ? 1 : 0);
        case 6: return (DL_GPIO_readPins(SENSOR_PORT_S6_PORT, SENSOR_PORT_S6_PIN) ? 1 : 0);
        case 7: return (DL_GPIO_readPins(SENSOR_PORT_S7_PORT, SENSOR_PORT_S7_PIN) ? 1 : 0);
        case 8: return (DL_GPIO_readPins(SENSOR_PORT_S8_PORT, SENSOR_PORT_S8_PIN) ? 1 : 0);
        default: return 0;
    }
}