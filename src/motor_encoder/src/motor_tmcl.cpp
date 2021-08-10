/// @file  motor_tmcl.cpp
/// @brief 电机驱动控制
///
///        测头的电机驱动控制,版本V1.00
///        电机型号PD20-1-1210,串口通信，带编码器
/// @author zhouhao <1198826224@qq.com>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include "motor_tmcl.hpp"

#include "motor_uart.hpp"



namespace motor_tmcl
{

#define MOTOR_MAX_BUFF 256
#define MOTOR_TX_FARMEBUFF 18
#define MOTOR_INIT_OFFSET 0
#define MOTOR_POS_ZERO   -(0 + MOTOR_INIT_OFFSET)
#define MOTOR_POS_ZERO2  -(3200 + MOTOR_INIT_OFFSET) 
#define MOTOR_POS_CENTER -(6400 + MOTOR_INIT_OFFSET)
#define MOTOR_POS_END    -(9600 + MOTOR_INIT_OFFSET)
#define MOTOR_POS_INIT   3610

//Opcodes of all TMCL commands that can be used in direct mode
#define TMCL_ROR 1
#define TMCL_ROL 2
#define TMCL_MST 3
#define TMCL_MVP 4
#define TMCL_SAP 5
#define TMCL_GAP 6
#define TMCL_STAP 7
#define TMCL_RSAP 8
#define TMCL_SGP 9
#define TMCL_GGP 10
#define TMCL_STGP 11
#define TMCL_RSGP 12
#define TMCL_RFS 13
#define TMCL_SIO 14
#define TMCL_GIO 15
#define TMCL_SCO 30
#define TMCL_GCO 31
#define TMCL_CCO 32

//Opcodes of TMCL control functions (to be used to run or abort a TMCL program in the module)
#define TMCL_APPL_STOP 128
#define TMCL_APPL_RUN 129
#define TMCL_APPL_RESET 131

//Options for MVP commandds
#define MVP_ABS 0
#define MVP_REL 1
#define MVP_COORD 2
//Options for GAP commandds
#define GAP_TAR_POS 0
#define GAP_ACT_POS  1
#define GAP_TAR_SPEED 2
#define GAP_ACT_SPEED 3
//Options for SAP commandds
#define SAP_TAR_POS 0
#define SAP_ACT_POS 1
#define SAP_MAX_SPEED 4
#define SAP_MAX_CUR 6


//时间管理标识
enum MotorTimeFlagNo
{
    M_TIMER_NULL_N = 0,
    M_TIMER_2MS_N,
    M_TIMER_5MS_N,
    M_TIMER_10MS_N,
    M_TIMER_50MS_N,
    M_TIMER_100MS_N,
};

enum MotorMsgCtlStateNo
{
    M_MSG_ZERO = 0,
    M_MSG_CENTER,
    M_MSG_SCAN1,
    M_MSG_SCAN2,
};

enum MotorUartProStateNo
{
    M_PRO_RX_WAIT1_N = 0,
    M_PRO_RX_WAIT2_N,
    M_PRO_RX_WAIT3_N,
    M_PRO_RX_ARRANGE_N,
    M_PRO_RX_CHECK_SUM_N,
    M_PRO_END_N,
};



class MotorTmcl:: _motorParamType
{
public:
    int uart_fd;
    int tar_pos;
    int act_pos;
    int tar_speed;
    int act_speed;
    char rx_pro_state;
    unsigned char now_len;
    unsigned int now_count;
    unsigned int last_count;
    int short timer_out;
    char control_state;
    char msg_ctl_state;
    int msg_flag;
    int msg_scan_speed;
    int coder_pos;
    unsigned char coder_count;
    unsigned char buf[20];
};

//class MotorTmcl
//{
//private:
//    std::queue<MotorUartTxFarmeType> _motorQueue;
//
//};

MotorTmcl::MotorTmcl(MotorArriveFunType function)
{
    _interrupt = false;
    _motorParamPtr = std::make_unique<_motorParamType>();
    _init = std::thread(&MotorTmcl::_MotorTask, this);//The first parameter inside the class needs to use this to give the address
    if(function != NULL)
    {
        _MotorArriveFun = function;
    }
}

MotorTmcl::~MotorTmcl()
{
    _interrupt = true;
    _init.join();
	_motorParamPtr.reset();
}

unsigned short MotorTmcl::_ChangeWordFormat(unsigned short value)
{
    return ((value & 0xFF00)>>8) + ((value & 0x00FF)<<8);
}

int MotorTmcl::_ChangeDWordFormat(int value)
{
    return (((unsigned int)value & 0xFF000000)>>24)\
            + (((unsigned int)value & 0x00FF0000)>>8) \
            + (((unsigned int)value & 0x0000FF00)<<8) \
            + (((unsigned int)value & 0x000000FF)<<24);
}

unsigned char MotorTmcl::_MotorCheckSumCalc(unsigned char *data,unsigned char len)
{
    unsigned short sum = 0;
    unsigned char res;

    for(unsigned char i=0;i<len;i++)
    {
        sum += data[i];
    }
    res = sum&0x00FF;

    return res;
}

/// @brief 主控来控制电机的接口
///
///        预留
/// @param motor_mode 电机的操作的模式
/// @param speed 速度,51200=1r/s,512=0.01r/s依次类推
/// @return <0发送失败 正常时为发送消息的数量
void MotorTmcl::MotorCtl(char motor_mode,int speed)
{

}

/// @brief 电机控制函数-扫描
///
///        勿调用太频繁（>5ms）
/// @param speed 速度,51200=1r/s,512=0.01r/s依次类推
/// @return none
void MotorTmcl::MotorCtlScan(int speed)
{
    ctl_mode = 1;
    ctl_flag = 1;
    ctl_value = speed;
}

/// @brief 电机控制函数-回归原点
///
///        勿调用太频繁（>5ms）
/// @return none
void MotorTmcl::MotorCtlZero(void)
{
	ctl_mode = 0;
	ctl_flag = 1;
}

/// @brief 电机控制函数-中线
///
///        勿调用太频繁（>5ms）
/// @return none
void MotorTmcl::MotorCtlCenter(void)
{
	ctl_mode = 2;
	ctl_flag = 1;
}

void MotorTmcl::_MotorSendFrame(unsigned char cmd,
                     unsigned char cmd_type,
                     int value)
{
    MotorUartTxFarmeType frame;

    frame.address = 1;
    frame.command = cmd;
    frame.type = cmd_type;
    frame.motor = 0;
    frame.value = _ChangeDWordFormat(value);
    frame.sum_check = _MotorCheckSumCalc((unsigned char*)&frame,sizeof(MotorUartTxFarmeType)-1);
    //Queue entry
    _MotorEnqueueTx(frame);
}


/// @brief queue入口
///
///        将数据写入缓存区
/// @param frame 发送的报文格式
/// @return none
void MotorTmcl::_MotorEnqueueTx(MotorUartTxFarmeType frame)
{
    if(_motorQueue.size() > 20)
    {
        return;
    }
    _motorQueue.push(frame);
}

/// @brief queue出口
///
///        读取要发送的缓存区数据
/// @return none
int MotorTmcl::_MotorDequeueTx(MotorUartTxFarmeType *temp)
{
    int res = 0;
    bool empty1;

    empty1 = _motorQueue.empty();
    if(empty1 == false)
    {
        *temp = _motorQueue.front();
        _motorQueue.pop();
        res = 1;
    }

    return res;
}

void MotorTmcl::_MotorCtlJudgment(void)
{

    if(_motorParamPtr->coder_count != 0xFF)
    {
        return;
    }

    if(_motorParamPtr->msg_ctl_state == M_MSG_SCAN1
    && _motorParamPtr->act_pos == MOTOR_POS_ZERO2)
    {
        //扫描时先让位置归0
        _MotorSendFrame(TMCL_SAP,SAP_MAX_SPEED,_motorParamPtr->msg_scan_speed);
        _MotorSendFrame(TMCL_MVP,MVP_ABS,MOTOR_POS_END);
        _motorParamPtr->tar_pos = MOTOR_POS_END;
        _motorParamPtr->msg_ctl_state = M_MSG_SCAN2;
        _motorParamPtr->msg_flag = 2;   //扫描
    }

    if(ctl_flag == 1)
    {
        ctl_flag = 0;
        switch(ctl_mode)
        {
            case 0:
                _MotorSendFrame(TMCL_SAP,SAP_MAX_SPEED,512000);
                _MotorSendFrame(TMCL_MVP,MVP_ABS,MOTOR_POS_ZERO);
                _motorParamPtr->tar_pos = MOTOR_POS_ZERO;
                _motorParamPtr->msg_ctl_state = M_MSG_ZERO;
                _motorParamPtr->msg_flag = 1;   //归零
                break;
            case 1:
                if(_motorParamPtr->act_pos == MOTOR_POS_ZERO2)
                {
                    _MotorSendFrame(TMCL_SAP,SAP_MAX_SPEED,ctl_value);
                    _MotorSendFrame(TMCL_MVP,MVP_ABS,MOTOR_POS_END);
                    _motorParamPtr->tar_pos = MOTOR_POS_END;
                    _motorParamPtr->msg_ctl_state = M_MSG_SCAN2;
                    _motorParamPtr->msg_flag = 2; //扫描
                }
                else
                {
                    _MotorSendFrame(TMCL_SAP,SAP_MAX_SPEED,512000);
                    _MotorSendFrame(TMCL_MVP,MVP_ABS,MOTOR_POS_ZERO2);
                    _motorParamPtr->msg_scan_speed = ctl_value;
                    _motorParamPtr->tar_pos = MOTOR_POS_ZERO2;
                    _motorParamPtr->msg_ctl_state = M_MSG_SCAN1;
                }
                break;
            case 2:
                _MotorSendFrame(TMCL_SAP,SAP_MAX_SPEED,512000);
                _MotorSendFrame(TMCL_MVP,MVP_ABS,MOTOR_POS_CENTER);
                _motorParamPtr->tar_pos = MOTOR_POS_CENTER;
                _motorParamPtr->msg_ctl_state = M_MSG_CENTER;
                _motorParamPtr->msg_flag = 3;  //中心
                break;
            default:
                break;
        }
    }
}

/// @brief 将缓存区数据发送至串口
///
///        数据的发送，注意调用时序和及时清理缓存
/// @param none
/// @return none
void MotorTmcl::_MotorUartTx(void)
{
    MotorUartTxFarmeType temp_f1;
    int res = 0;
    //Queue exit
    res = _MotorDequeueTx(&temp_f1);
    if(res == 1)
    {
        write(_motorParamPtr->uart_fd,&temp_f1,sizeof(MotorUartTxFarmeType));
    }
}

/// @brief 接收处理
///
///        数据的接收，暂时只收实际位置
/// @param none
/// @return none
void MotorTmcl::_UsbRxDataClassify(void)
{
    int value;

    value = (int)(((unsigned int)_motorParamPtr->buf[4] << 24) |
                 ((unsigned int)_motorParamPtr->buf[5] << 16) |
                 ((unsigned int)_motorParamPtr->buf[6] << 8) |
                 ((unsigned int)_motorParamPtr->buf[7]));

    switch(_motorParamPtr->buf[3])
    {
    case TMCL_GAP:
        if(_motorParamPtr->coder_count == 0xFF)
        {
            _motorParamPtr->act_pos = value;
        }
        else
        {
            _motorParamPtr->coder_pos = value;
            _motorParamPtr->coder_count ++;
        }
        break;
    case TMCL_MVP:
        break;
    default:
        break;
    }
}

/// @brief 对收到的数据进行组包
///
///        组包函数，按包头校验等判断包的正确性
/// @param rx_date 接收到的字节
/// @return none
void MotorTmcl::_MotorTmclPackage(unsigned char rx_data)
{
    unsigned char sum_check = 0;

    switch(_motorParamPtr->rx_pro_state)
    {
        case M_PRO_RX_WAIT1_N:
            _motorParamPtr->now_len = 0;
            if(rx_data == 2)
            {
                _motorParamPtr->buf[_motorParamPtr->now_len++] = rx_data;
                _motorParamPtr->rx_pro_state = M_PRO_RX_WAIT2_N;
            }
            break;
        case M_PRO_RX_WAIT2_N:

            if(rx_data == 1)
            {
                _motorParamPtr->buf[_motorParamPtr->now_len++] = rx_data;
                _motorParamPtr->rx_pro_state = M_PRO_RX_WAIT3_N;
            }else
            {
                _motorParamPtr->rx_pro_state = M_PRO_RX_WAIT1_N;
            }
            break;
        case M_PRO_RX_WAIT3_N:

            if(rx_data == 0x64)
            {
                _motorParamPtr->buf[_motorParamPtr->now_len++] = rx_data;
                _motorParamPtr->rx_pro_state = M_PRO_RX_ARRANGE_N;
            }
            else
            {
                _motorParamPtr->rx_pro_state = M_PRO_RX_WAIT1_N;
            }
            break;
        case M_PRO_RX_ARRANGE_N:

            _motorParamPtr->buf[_motorParamPtr->now_len++] = rx_data;
            if(_motorParamPtr->now_len >= 8)
            {
                _motorParamPtr->rx_pro_state = M_PRO_RX_CHECK_SUM_N;
            }
            break;
        case M_PRO_RX_CHECK_SUM_N:

            _motorParamPtr->buf[_motorParamPtr->now_len] = rx_data;
            sum_check = _MotorCheckSumCalc(&_motorParamPtr->buf[0],sizeof(MotorUartTxFarmeType)-1);
            if(_motorParamPtr->buf[_motorParamPtr->now_len] ==  sum_check)
            {
                //数据处理
                _UsbRxDataClassify();
                _motorParamPtr->timer_out = 0;
                //printf("CRC OK /r/n");
            }
            else
            {
                //printf("sumcheck error");
            }

            _motorParamPtr->now_len = 0;
            _motorParamPtr->rx_pro_state = M_PRO_RX_WAIT1_N;
            break;
        case M_PRO_END_N:
            break;
        default:
            break;

    }
}

void MotorTmcl::_MotorUartRx(unsigned char *rx_buf)
{
    int res = 0;

    res = read(_motorParamPtr->uart_fd,rx_buf,MOTOR_MAX_BUFF);
    if(res > 0)
    {
        for(int i=0;i<res;i++)
        {
            _MotorTmclPackage(rx_buf[i]);
        }
    }
}

void MotorTmcl::_MotorCailbInit(void)
{
    _MotorSendFrame(TMCL_SAP,SAP_MAX_CUR,32);
    _MotorSendFrame(TMCL_SAP,187,1);
    //_MotorSendFrame(TMCL_MVP,MVP_REL,-15360);
    _MotorSendFrame(TMCL_SAP,SAP_MAX_SPEED,512000);
    _MotorSendFrame(TMCL_SAP,5,1011990);
    _MotorSendFrame(TMCL_SAP,17,1011990);
    _MotorSendFrame(TMCL_SAP,15,555630);
    _MotorSendFrame(TMCL_SAP,18,555630);
    _MotorSendFrame(TMCL_SAP,16,25600);
    _motorParamPtr->coder_count = 0;
}

void MotorTmcl::_MotorCfgInit(void)
{
    _motorParamPtr->tar_pos = 0;
    _motorParamPtr->act_pos = 0;
    _MotorSendFrame(TMCL_SAP,SAP_MAX_CUR,60);//运作电流
    _MotorSendFrame(TMCL_SAP,SAP_TAR_POS,0);
    _MotorSendFrame(TMCL_SAP,SAP_ACT_POS,0);
    _MotorSendFrame(TMCL_SAP,SAP_MAX_SPEED,512000);
    _MotorSendFrame(TMCL_SAP,5,1011990);
    _MotorSendFrame(TMCL_SAP,17,1011990);
    _MotorSendFrame(TMCL_SAP,15,555630);
    _MotorSendFrame(TMCL_SAP,18,555630);
    _MotorSendFrame(TMCL_SAP,16,25600);
    _MotorSendFrame(TMCL_MVP,MVP_ABS,0);
    _motorParamPtr->tar_pos = 0;
}

//定期查询电机的位置
void MotorTmcl::_MotorInfoQuery(void)
{
    if(_motorParamPtr->coder_count != 0xFF)
    {
        return;
    }
    _MotorSendFrame(TMCL_GAP,GAP_ACT_POS,2);
    if(_motorParamPtr->msg_flag != 0
    && _motorParamPtr->tar_pos == _motorParamPtr->act_pos)
    {
        if(_MotorArriveFun != NULL)
        {
            _MotorArriveFun(_motorParamPtr->msg_flag);
        }
        _motorParamPtr->msg_flag = 0;
    }
}

//时间控制
char MotorTmcl::_MotorTimeManage(void)
{
    char res = M_TIMER_NULL_N;

    _motorParamPtr->now_count++;
    if(_motorParamPtr->now_count >= 1000)
    {
        _motorParamPtr->now_count= 0;
        _motorParamPtr->timer_out++;
        if(_motorParamPtr->timer_out > 3)
        {
            _motorParamPtr->timer_out = 3;
            //可加入超时反馈
            std::cout << "MOTOR TIME OUT" << std::endl;
        }
    }

    if((_motorParamPtr->now_count+1)%5 == 0)
    {
        res = M_TIMER_5MS_N;
    }

    if(((_motorParamPtr->now_count+2)%20) == 0)
    {
        res = M_TIMER_10MS_N;
    }

    if(((_motorParamPtr->now_count+3)%50) == 0)
    {
        res = M_TIMER_50MS_N;
    }

    if(((_motorParamPtr->now_count+4)%500) == 0)
    {
        res = M_TIMER_100MS_N;
    }

    return res;
}

void MotorTmcl::_MotorCoderQuery(void)
{
    int temp_data = 0;

    if(_motorParamPtr->coder_count == 0xFF)
    {
        return;
    }

    //这里定时200ms调用
    if(_motorParamPtr->coder_count == 0)
    {
        _MotorSendFrame(TMCL_GAP,215,0);
    }
    else if(_motorParamPtr->coder_count > 0)
    {
        if(_motorParamPtr->coder_pos <= (MOTOR_POS_INIT+1) && _motorParamPtr->coder_pos >= (MOTOR_POS_INIT-1))
        {
            _motorParamPtr->coder_count = 0xFF;
            _MotorCfgInit(); //初始位置成功后配置
        }
        else
        {
            //计算移动量移动
            temp_data = ((int)MOTOR_POS_INIT - _motorParamPtr->coder_pos) * 25 / 2;
            _MotorSendFrame(TMCL_MVP,MVP_REL,temp_data);
            _motorParamPtr->coder_count = 0;            
        }
    }
}

void MotorTmcl::_MotorTask(void)
{
    unsigned char *rx_buff = new unsigned char[MOTOR_MAX_BUFF];


    _motorParamPtr->uart_fd = MotorUartInit();
    if(_motorParamPtr->uart_fd < 0)
    {
        delete rx_buff;
        return;
    }

    _MotorCailbInit();
    while(_interrupt == false)
    {

        _MotorUartRx(rx_buff);
        _MotorCtlJudgment(); //接收外部控制

        switch(_MotorTimeManage())
        {
            case M_TIMER_5MS_N:
                break;
            case M_TIMER_10MS_N:
                _MotorUartTx();
                break;
            case M_TIMER_50MS_N:
                _MotorInfoQuery();
                break;
            case M_TIMER_100MS_N:
                _MotorCoderQuery();  //查询编码器位置并移动过去
                break;
            default:
                break;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
    delete rx_buff;         // 释放内存
}



}
