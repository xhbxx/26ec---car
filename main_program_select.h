#ifndef MAIN_PROGRAM_SELECT_H
#define MAIN_PROGRAM_SELECT_H

/* 保留的原主程序：按下编码器后执行245 -> 130 -> 350。 */
#define MAIN_PROGRAM_CURRENT           (0U)
/* 新主程序：上电后先用旋钮选择模式1、2或3。 */
#define MAIN_PROGRAM_THREE_MODES       (1U)

/*
 * 在这里选择烧录后运行哪个主程序：
 * MAIN_PROGRAM_CURRENT     = 完全使用修改前的当前main逻辑；
 * MAIN_PROGRAM_THREE_MODES = 使用旋钮选择四种工作模式的新main逻辑（名称为历史兼容保留）。
 */
#define MAIN_PROGRAM_SELECT            MAIN_PROGRAM_THREE_MODES

int Current_Main(void);
int New_Main(void);

#endif /* MAIN_PROGRAM_SELECT_H */
