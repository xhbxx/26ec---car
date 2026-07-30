#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 两个独立程序只能选择一个参与本次编译。 */
#define APP_MODE_LINE_TRACKING          (0U)
#define APP_MODE_MS6DSV_TEST            (1U)

/*
 * 修改这里选择入口：
 * APP_MODE_LINE_TRACKING：编译 main.c 的循迹主函数。
 * APP_MODE_MS6DSV_TEST：编译 test1.c 的传感器测试主函数。
 */
#define APP_RUN_MODE                    AAPP_MODE_LINE_TRACKING

#endif /* APP_CONFIG_H */
