/// @file  motor_uart.h
/// @brief 电机串口配置文件
///
///        电机波特率配置为了230400
///        电机型号PD20-1-1210,串口通信
/// @author zhouhao <1198826224@qq.com>

#ifndef MOTOR_UART_H_
#define MOTOR_UART_H_

/// @brief 串口初始化
///
/// @param None
/// @return -1串口打开失败 否则为串口的句柄
int MotorUartInit(void);

#endif /* MOTOR_UART_H_ */
