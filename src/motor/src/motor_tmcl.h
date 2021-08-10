/// @file  motor_tmcl.h
/// @brief 电机驱动控制
///
///        测头的电机驱动控制,版本V1.00
///        电机型号PD20-1-1210,串口通信
/// @author zhouhao <1198826224@qq.com>

#ifndef MOTOR_TMCL_H_
#define MOTOR_TMCL_H_

/// @brief 创建电机进程
///
///        使用的父进程创建子进程的方法，勿要重复调用
/// @param None
/// @return -1：消息创建失败 -2:进程创建失败 >0:消息队列的id
int MotorCreateProcess(void);

/// @brief 关闭电机进程
///
///        关闭进程同时删除消息队列，勿频繁调用
/// @param None
/// @return None
void MotorDestroyProcess(void);

/// @brief 电机控制函数-扫描
///
///        内含消息队列发送，勿调用太频繁（>5ms）
/// @param speed 速度,51200=1r/s,512=0.01r/s依次类推
/// @return <0发送失败 正常时为发送消息的数量
int MotorCtlScan(int speed);

/// @brief 电机控制函数-回归原点
///
///        内含消息队列发送，勿调用太频繁（>5ms）
/// @return <0发送失败 正常时为发送消息的数量
int MotorCtlZero(void);

/// @brief 电机控制函数-中线
///
///        内含消息队列发送，勿调用太频繁（>5ms）
/// @return <0发送失败 正常时为发送消息的数量
int MotorCtlCenter(void);



#endif /* MOROT_TMCL_H_ */
