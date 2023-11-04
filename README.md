# TJU_Powerline_Patrol
STM32F103C8T6 board code in "TJU Power Line Patrol" project

此工程采用STM32 CubeMX工具生成，使用FreeRTOS V2.0系统

主要实现功能：

从RS232接口读取数据(S.Bus各通道量)，转换为PELCOD-D协议，从RS485接口输出
