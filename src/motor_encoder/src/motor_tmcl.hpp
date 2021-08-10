/// @file  motor_tmcl.hpp
/// @brief 电机驱动控制
///
///        测头的电机驱动控制,版本V1.00
///        电机型号PD20-1-1210,串口通信，带编码器
/// @author zhouhao <1198826224@qq.com>

#ifndef MOTOR_CONTROL_SRC_MOTOR_TMCL_HPP_
#define MOTOR_CONTROL_SRC_MOTOR_TMCL_HPP_

#include<iostream>
#include <chrono>
#include <atomic>
#include  <thread>
#include <queue>

namespace motor_tmcl
{

#pragma pack(1)
typedef struct
{
    unsigned char address;
    unsigned char command;
    unsigned char type;
    unsigned char motor;
    int value;
    unsigned char sum_check;
}MotorUartTxFarmeType;
#pragma pack()

/// @brief 电机反馈回调函数类型
///
///        回调时表示相应动作完成，具体动作类型由参数确定
/// @param 1：归零动作到位 2：扫描动作到位 3：中心动作到位
/// @return none
typedef void (*MotorArriveFunType)(int t_param);

class MotorTmcl
{
public:
    
    explicit MotorTmcl(MotorArriveFunType function);
    ~MotorTmcl();
    
    /// @brief 电机控制函数-扫描
    ///
    ///        勿调用太频繁（>5ms）
    /// @param speed 速度,51200=1r/s,512=0.01r/s依次类推
    /// @return none    
    void MotorCtlScan(int speed);
    /// @brief 电机控制函数-回归原点
    ///
    ///        勿调用太频繁（>5ms）
    /// @return none
    void MotorCtlZero(void);
    /// @brief 电机控制函数-中线
    ///
    ///        勿调用太频繁（>5ms）
    /// @return none
    void MotorCtlCenter(void);

private:
    void _MotorTask(void);
    unsigned short _ChangeWordFormat(unsigned short value);
    int _ChangeDWordFormat(int value);
    unsigned char static _MotorCheckSumCalc(unsigned char *data,unsigned char len);
    void _MotorSendFrame(unsigned char cmd,
                         unsigned char cmd_type,
                         int value);
    void _MotorCtlJudgment(void);
    void _MotorUartTx(void);
    void _UsbRxDataClassify(void);
    void _MotorTmclPackage(unsigned char rx_data);
    void _MotorUartRx(unsigned char *rx_buf);
    void _MotorEnqueueTx(MotorUartTxFarmeType frame);
    int _MotorDequeueTx(MotorUartTxFarmeType *temp);
    void _MotorCoderQuery(void);//
    void _MotorCailbInit(void);
    void _MotorCfgInit(void);
    void _MotorInfoQuery(void);
    char _MotorTimeManage(void);
    void MotorCtl(char motor_mode,int speed);
    MotorArriveFunType _MotorArriveFun;

private:
    class _motorParamType;
    std::unique_ptr<_motorParamType> _motorParamPtr;

    std::atomic<bool> ctl_flag;
    std::atomic<char> ctl_mode;
    std::atomic<int>  ctl_value;
    std::queue<MotorUartTxFarmeType> _motorQueue;

    std::atomic<bool> _interrupt;

    std::thread _init;

};

}


#endif /* MOTOR_CONTROL_SRC_MOTOR_TMCL_HPP_ */
