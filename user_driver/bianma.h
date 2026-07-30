#ifndef BIANMA_H
#define BIANMA_H

#include "ti_msp_dl_config.h"

#include <stdint.h>

/** 初始化旋转编码器状态；GPIO引脚由SysConfig统一初始化。 */
void Bianma_Init(void);

/** 返回1表示顺时针一步，-1表示逆时针一步，0表示没有转动。 */
int8_t Bianma_GetRotation(void);

/** 每1ms调用一次；检测到一次消抖后的按下事件时返回1。 */
uint8_t Bianma_Button_Pressed(void);

/** 直接读取按键物理电平：按下返回1，松开返回0，不做消抖。 */
uint8_t Bianma_Button_IsPressed(void);

/* 保留旧PID调参接口，兼容工程中已有调用。 */
uint8_t Bianma_GetParamIndex(void);
void Bianma_ProcessEncoder(int *p, int *i, int *d);

#endif /* BIANMA_H */
