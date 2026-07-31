#include "main_program_select.h"

/**
 * 工程唯一的main入口。
 * 只修改main_program_select.h中的MAIN_PROGRAM_SELECT即可切换新旧程序，
 * 不需要删除文件，也不会同时编译出两个main函数。
 */
int main(void)
{
#if MAIN_PROGRAM_SELECT == MAIN_PROGRAM_CURRENT
    return Current_Main();
#else
    return New_Main();
#endif
}
