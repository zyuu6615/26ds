#ifndef __APP_H
#define __APP_H

#include "main.h"

/* 应用层入口：由 Core/Src/main.c 调用 */

void App_Init(void);   /* 外设初始化完成后调用一次 */
void App_Run(void);    /* 在主循环中反复调用 */

#endif /* __APP_H */
