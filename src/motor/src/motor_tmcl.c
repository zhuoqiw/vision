/// @file  motor_tmcl.c
/// @brief 电机驱动控制
///
///        测头的电机驱动控制,版本V1.00
///        电机型号PD20-1-1210,串口通信
/// @author zhouhao <1198826224@qq.com>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <wait.h>
#include <fcntl.h>

#include "motor_uart.h"

#define PRINTF_INFO(name,fmt,arg...)           printf("["name"-INFO.L%d]"fmt"\n", __LINE__, ##arg)
#define PRINTF_ERROR(name,fmt,arg...)          printf("["name"-ERROR.L%d]\033[30m\033[31m"fmt"\033[0m\n",__LINE__, ##arg)

#define MOTOR_TMCL_INFO(fmt,arg...)            PRINTF_INFO("MOTOR_TMCL",fmt,##arg)
#define MOTOR_TMCL_ERROR(fmt,arg...)            PRINTF_ERROR("MOTOR_ERROR",fmt,##arg)

#define MOTOR_MAX_BUFF 256
#define MOTOR_TX_FARMEBUFF 18
#define MOTOR_INIT_OFFSET 2048
#define MOTOR_POS_ZERO   (0 + MOTOR_INIT_OFFSET)
#define MOTOR_POS_CENTER (6400 + MOTOR_INIT_OFFSET)
#define MOTOR_POS_END    (12800 + MOTOR_INIT_OFFSET)

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

//消息归属类型
enum MsgDateTypeNo
{
    MSG_MASTE_N = 1,
    MSG_MOOTR_N,
};

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

enum MotorUartProStateNo
{
    M_PRO_RX_WAIT1_N = 0,
    M_PRO_RX_WAIT2_N,
    M_PRO_RX_WAIT3_N,
    M_PRO_RX_ARRANGE_N,
    M_PRO_RX_CHECK_SUM_N,
    M_PRO_END_N,
};

enum MotorMsgCtlStateNo
{
    M_MSG_ZERO = 0,
    M_MSG_CENTER,
    M_MSG_SCAN1,
    M_MSG_SCAN2,
};


#pragma pack(1)
typedef struct
{
    unsigned char motor_mode;
    int value;
}MotorMsgDateType;

typedef struct
{
    long msg_type;
    MotorMsgDateType msg_data;
}MotorMsgType;


typedef struct
{
    unsigned char address;
    unsigned char command;
    unsigned char type;
    unsigned char motor;
    int value;
    unsigned char sum_check;
}MotorUartTxFarmeType;

typedef struct
{
    unsigned char tx_head_no;
    unsigned char tx_tail_no;
    MotorUartTxFarmeType frame[MOTOR_TX_FARMEBUFF];
    unsigned buff_size;
}MotorUartTxBuffType;

typedef struct
{
    unsigned short now_count;
    unsigned short last_count;
    int tar_pos;
    int act_pos;
    int act_pos_txt;
    FILE *fd_txt;
    int tar_speed;
    int act_speed;
    char rx_pro_state;
    unsigned char now_len;
    int timer_out;
    char control_state;
    char msg_ctl_state;
    int msg_out_timer;
    int msg_scan_speed;
    unsigned char buf[20];
}MotorParamType;
#pragma pack()


pid_t motorPid = 0;
int motorQid = -3;

unsigned short ChangeWordFormat(unsigned short value)
{
    return ((value & 0xFF00)>>8) + ((value & 0x00FF)<<8);
}

int ChangeDWordFormat(int value)
{
    return (((unsigned int)value & 0xFF000000)>>24)\
            + (((unsigned int)value & 0x00FF0000)>>8) \
            + (((unsigned int)value & 0x0000FF00)<<8) \
            + (((unsigned int)value & 0x000000FF)<<24);
}

//累加和校验
unsigned char static _MotorCheckSumCalc(unsigned char *data,unsigned char len)
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
///        根据控制模式的发相应的消息给电机进程
/// @param qid 消息队列的ID
/// @param motor_mode 电机的操作的模式
/// @param speed 速度,51200=1r/s,512=0.01r/s依次类推
/// @return <0发送失败 正常时为发送消息的数量
int MotorCtl(int qid,char motor_mode,int speed)
{
    MotorMsgType msg_maste_tx;
    int res;

    if(qid < 0)
    {
        return -2;
    }

    msg_maste_tx.msg_data.motor_mode = motor_mode;
    msg_maste_tx.msg_data.value = speed;
    msg_maste_tx.msg_type = MSG_MASTE_N;

    res = msgsnd(qid,&msg_maste_tx,sizeof(MotorMsgDateType),IPC_NOWAIT);
    if(res < 0)
    {
        MOTOR_TMCL_ERROR("msg send error\n");
    }

    return res;
}

/// @brief 电机控制函数-扫描
///
///        内含消息队列发送，勿调用太频繁（>5ms）
/// @param qid 消息队列的ID
/// @param speed 速度,51200=1r/s,512=0.01r/s依次类推
/// @return <0发送失败 正常时为发送消息的数量
int MotorCtlScan(int speed)
{
    int res;
    if(motorQid < 0)
    {
       return -1;
    }
    res =  MotorCtl(motorQid,1,speed);
    return res;
}

/// @brief 电机控制函数-回归原点
///
///        内含消息队列发送，勿调用太频繁（>5ms）
/// @param qid 消息队列的ID
/// @return <0发送失败 正常时为发送消息的数量
int MotorCtlZero(void)
{
    int res;
    if(motorQid < 0)
    {
       return -1;
    }
    res =  MotorCtl(motorQid,0,0);
    return res;
}

/// @brief 电机控制函数-中线
///
///        内含消息队列发送，勿调用太频繁（>5ms）
/// @param qid 消息队列的ID
/// @return <0发送失败 正常时为发送消息的数量
int MotorCtlCenter(void)
{
    int res;
    if(motorQid < 0)
   {
      return -1;
   }
    res =  MotorCtl(motorQid,2,0);
    return res;
}

/// @brief 环形缓存区初始化
///
///        建立环形结构初始化,以实现发送时序
/// @param *ring 发送缓存区的结构指针
/// @return none
static void _MotorInitTxQueue(MotorUartTxBuffType *ring)
{
    ring->tx_head_no = 0;
    ring->tx_tail_no = 0;
    ring->buff_size = MOTOR_TX_FARMEBUFF;
}

/// @brief 环形缓存区入口
///
///        将数据写入缓存区
/// @param frame 发送的报文格式
/// @param *ring 发送缓存区的结构指针
/// @return none
static void _MotorEnqueueTx(MotorUartTxFarmeType frame, MotorUartTxBuffType *ring)
{
    if(((ring->tx_tail_no+1) == ring->tx_head_no)
     ||((ring->tx_tail_no == ring->buff_size)
     &&(ring->tx_head_no == 0)))
    {
        MOTOR_TMCL_ERROR("queue buff is full");
    }
    else
    {
        ring->frame[ring->tx_tail_no] = frame;
        ring->tx_tail_no++;

        if(ring->tx_tail_no >= ring->buff_size)
        {
            ring->tx_tail_no = 0;
        }
    }
}

/// @brief 环形缓存区出口
///
///        读取要发送的缓存区数据
/// @param *ring 发送缓存区的结构指针
/// @return none
static MotorUartTxFarmeType *_MotorDequeueTx(MotorUartTxBuffType *ring)
{
    MotorUartTxFarmeType *temp;

    if(ring->tx_head_no == ring->tx_tail_no)
    {
        temp = 0;
    }
    else
    {
        temp = &ring->frame[ring->tx_head_no];
        ring->tx_head_no++;

        if(ring->tx_head_no >= ring->buff_size)
        {
            ring->tx_head_no = 0;
        }
    }

    return temp;
}

/// @brief 报文存入缓存区
///
///        应用部分和环形缓存区的中间层
/// @param cmd 电机控制的命令
/// @param cmd_type 电机控制的命令具体类型
/// @param valude 电机控制的命令值
/// @param *ring 发送缓存区的结构指针
/// @return none
static void _MotorSendFrame(unsigned char cmd,
                            unsigned char cmd_type,
                            int value,
                            MotorUartTxBuffType *ring)
{
    MotorUartTxFarmeType frame;

    frame.address = 1;
    frame.command = cmd;
    frame.type = cmd_type;
    frame.motor = 0;
    frame.value = ChangeDWordFormat(value);
    frame.sum_check = _MotorCheckSumCalc((unsigned char*)&frame,sizeof(MotorUartTxFarmeType)-1);
    _MotorEnqueueTx(frame,ring);
}

/// @brief 消息接收
///
///        电机进程对主进程的消息接收处理
/// @param qid 消息的id
/// @param *ring 发送缓存区的结构指针
/// @param 电机的相关参数
/// @return none
static void _MotorMsgRx(int qid,
                        MotorUartTxBuffType *ring,
                        MotorParamType *param)
{
    ssize_t msg_res;
    MotorMsgType msg;


    if(param->msg_ctl_state == M_MSG_SCAN1
    && param->act_pos == MOTOR_POS_ZERO)
    {   //扫描时先让位置归0
        _MotorSendFrame(TMCL_SAP,SAP_MAX_SPEED,param->msg_scan_speed,ring);
        _MotorSendFrame(TMCL_MVP,MVP_ABS,MOTOR_POS_END,ring);
        param->tar_pos = MOTOR_POS_END;
        param->msg_ctl_state = M_MSG_SCAN2;
    }

    msg_res = msgrcv(qid,&msg,sizeof(MotorMsgDateType),MSG_MASTE_N,IPC_NOWAIT);
    if(msg_res > 0)
    {
        param->msg_out_timer = 0;
        switch(msg.msg_data.motor_mode)
        {
        case 0:
            _MotorSendFrame(TMCL_SAP,SAP_MAX_SPEED,512000,ring);
            _MotorSendFrame(TMCL_MVP,MVP_ABS,MOTOR_POS_ZERO,ring);
            param->tar_pos = MOTOR_POS_ZERO;
            param->msg_ctl_state = M_MSG_ZERO;
            break;
        case 1:
            if(param->act_pos == 0)
            {
                _MotorSendFrame(TMCL_SAP,SAP_MAX_SPEED,msg.msg_data.value,ring);
                _MotorSendFrame(TMCL_MVP,MVP_ABS,MOTOR_POS_END,ring);
                param->tar_pos = MOTOR_POS_END;
                param->msg_ctl_state = M_MSG_SCAN2;
            }
            else
            {
                _MotorSendFrame(TMCL_SAP,SAP_MAX_SPEED,512000,ring);
                _MotorSendFrame(TMCL_MVP,MVP_ABS,MOTOR_POS_ZERO,ring);
                param->msg_scan_speed = msg.msg_data.value;
                param->tar_pos = MOTOR_POS_ZERO;
                param->msg_ctl_state = M_MSG_SCAN1;
            }
            break;
        case 2:
            _MotorSendFrame(TMCL_SAP,SAP_MAX_SPEED,512000,ring);
            _MotorSendFrame(TMCL_MVP,MVP_ABS,MOTOR_POS_CENTER,ring);
            param->tar_pos = MOTOR_POS_CENTER;
            param->msg_ctl_state = M_MSG_CENTER;
            break;
        default:
            break;
        }
    }
    else
    {
        //not msg
    }
}

/// @brief 将缓存区数据发送至串口
///
///        数据的发送，注意调用时序和及时清理缓存
/// @param fd 文件的id
/// @param *ring 发送缓存区的结构指针
/// @return none
static void _MotorUartTx(int fd, MotorUartTxBuffType *ring)
{
    MotorUartTxFarmeType *temp_f1 = NULL;

    temp_f1 = _MotorDequeueTx(ring);
    if(temp_f1 != 0)
    {
        write(fd,temp_f1,sizeof(MotorUartTxFarmeType));
    }
}

/// @brief 接收处理
///
///        数据的接收，暂时只收实际位置
/// @param 电机的相关参数
/// @return none
static void UsbRxDataClassify(MotorParamType *param)
{
    int value;

    value = (int)(((unsigned int)param->buf[4] << 24) |
                 ((unsigned int)param->buf[5] << 16) |
                 ((unsigned int)param->buf[6] << 8) |
                 ((unsigned int)param->buf[7]));

    switch(param->buf[3])
    {
    case TMCL_GAP:
        param->act_pos = value;
        break;
    case TMCL_MVP:
        break;
    default:
        break;

    }
}

//对收到的数据进行组包
/// @brief 对收到的数据进行组包
///
///        组包函数，按包头校验等判断包的正确性
/// @param 电机的相关参数
/// @return none
static void _MotorTmclPackage(unsigned char rx_data,MotorParamType *param)
{
    unsigned char sum_check = 0;

    switch(param->rx_pro_state)
    {
        case M_PRO_RX_WAIT1_N:
            param->now_len = 0;
            if(rx_data == 2)
            {
                param->buf[param->now_len++] = rx_data;
                param->rx_pro_state = M_PRO_RX_WAIT2_N;
            }
            break;
        case M_PRO_RX_WAIT2_N:

            if(rx_data == 1)
            {
                param->buf[param->now_len++] = rx_data;
                param->rx_pro_state = M_PRO_RX_WAIT3_N;
            }else
            {
                param->rx_pro_state = M_PRO_RX_WAIT1_N;
            }
            break;
        case M_PRO_RX_WAIT3_N:

            if(rx_data == 0x64)
            {
                param->buf[param->now_len++] = rx_data;
                param->rx_pro_state = M_PRO_RX_ARRANGE_N;
            }
            else
            {
                param->rx_pro_state = M_PRO_RX_WAIT1_N;
            }
            break;
        case M_PRO_RX_ARRANGE_N:

            param->buf[param->now_len++] = rx_data;
            if(param->now_len >= 8)
            {
                param->rx_pro_state = M_PRO_RX_CHECK_SUM_N;
            }
            break;
        case M_PRO_RX_CHECK_SUM_N:

            param->buf[param->now_len] = rx_data;
            sum_check = _MotorCheckSumCalc(&param->buf[0],sizeof(MotorUartTxFarmeType)-1);
            if(param->buf[param->now_len] ==  sum_check)
            {
                //数据处理
                UsbRxDataClassify(param);
                param->timer_out = 0;
            }
            else
            {
                MOTOR_TMCL_ERROR("sumcheck error \r\n");
            }

            param->now_len = 0;
            param->rx_pro_state = M_PRO_RX_WAIT1_N;
            break;
        case M_PRO_END_N:
            break;
        default:
            break;
    }
}

//接收串口数据
static void _MotorUartRx(int fd, unsigned char *rx_buf, MotorParamType *param)
{
    int res = 0;

    res = read(fd, rx_buf, MOTOR_MAX_BUFF);
    if(res > 0)
    {
        for(int i=0;i<res;i++)
        {
            _MotorTmclPackage(rx_buf[i],param);
        }
    }
}

static void _MotorCalibInit(MotorParamType *param, MotorUartTxBuffType *ring)
{

    _MotorSendFrame(TMCL_SAP,SAP_MAX_CUR,32,ring);
    _MotorSendFrame(TMCL_SAP,187,1,ring);
    _MotorSendFrame(TMCL_MVP,MVP_REL,-15360,ring);
    _MotorSendFrame(TMCL_SAP,SAP_MAX_SPEED,12800,ring);
    _MotorSendFrame(TMCL_SAP,5,1011990,ring);
    _MotorSendFrame(TMCL_SAP,17,1011990,ring);
    _MotorSendFrame(TMCL_SAP,15,555630,ring);
    _MotorSendFrame(TMCL_SAP,18,555630,ring);
    _MotorSendFrame(TMCL_SAP,16,25600,ring);
}

static void _MotorCfgInit(MotorParamType *param, MotorUartTxBuffType *ring)
{
    param->tar_pos = 0;
    param->act_pos = 0;
    _MotorSendFrame(TMCL_SAP,SAP_MAX_CUR,120,ring);//
    _MotorSendFrame(TMCL_SAP,SAP_TAR_POS,0,ring);//param->tar_pos
    _MotorSendFrame(TMCL_SAP,SAP_ACT_POS,0,ring);//param->act_pos
    _MotorSendFrame(TMCL_SAP,SAP_MAX_SPEED,512000,ring);
    _MotorSendFrame(TMCL_SAP,5,1011990,ring);
    _MotorSendFrame(TMCL_SAP,17,1011990,ring);
    _MotorSendFrame(TMCL_SAP,15,555630,ring);
    _MotorSendFrame(TMCL_SAP,18,555630,ring);
    _MotorSendFrame(TMCL_SAP,16,25600,ring);


    _MotorSendFrame(TMCL_MVP,MVP_ABS,0,ring);
    param->tar_pos = 0;

}

//定期查询电机的位置
static void _MotorInfoQuery(MotorUartTxBuffType *ring)
{
    _MotorSendFrame(TMCL_GAP,GAP_ACT_POS,2,ring);
}

//时间控制
static char _MotorTimeManage(MotorParamType *param)
{
    char res = M_TIMER_NULL_N;

    param->now_count++;
    if(param->now_count >= 1000)
    {
        param->now_count = 0;
        param->timer_out++;
        if(param->timer_out > 3)
        {
            MOTOR_TMCL_ERROR("MOTOR TIME OUT");
            param->timer_out = 3;

        }
    }

    if((param->now_count+1)%5 == 0)
    {
        res = M_TIMER_5MS_N;
    }

    if(((param->now_count+2)%10) == 0)
    {
        res = M_TIMER_10MS_N;
    }

    if(((param->now_count+3)%50) == 0)
    {
        res = M_TIMER_50MS_N;
    }

    if(((param->now_count+4)%100) == 0)
    {
        res = M_TIMER_100MS_N;
    }


    return res;
}

void _MotorTxtInit(MotorParamType *param)
{
    char temp_str[20];
    char *ptr;

    param->fd_txt = fopen("./motor_cfg.txt","rt+");
    if(param->fd_txt  == NULL)
    {
        param->fd_txt = fopen("./motor_cfg.txt","w");
        fclose(param->fd_txt);
        param->fd_txt  = NULL;
        printf("not txt \n");
        return;
    }

    fseek(param->fd_txt,0,SEEK_SET);
    ptr = fgets(temp_str,20,param->fd_txt);
    if(ptr != NULL)
    {
        ptr = index(temp_str,'=');
        param->act_pos = atoi(&ptr[1]);
        param->tar_pos = param->act_pos;
        param->act_pos_txt = param->act_pos;
    }
}

void MotorTxtWrite(MotorParamType *param)
{
    if(param->fd_txt == NULL)
    {
        printf("txt err");
        return;
    }

    if(param->act_pos_txt != param->act_pos)
    {
        param->act_pos_txt = param->act_pos;
        fseek(param->fd_txt,0,SEEK_SET);
        fprintf(param->fd_txt,"INIT_POS=%d\n",param->act_pos);
        fflush(param->fd_txt);
    }
}
//电机进程
static void _MotorProcess(int qid)
{
	unsigned char *rx_buff;
    MotorUartTxBuffType *tx_buff;
    MotorParamType  *param;
    int fd = 0;
    int flag_timer = 0;

    if(((rx_buff = malloc(MOTOR_MAX_BUFF*sizeof(unsigned char))) == NULL)
     ||((tx_buff = malloc(sizeof(MotorUartTxBuffType))) == NULL)
     ||((param = malloc(sizeof(MotorParamType))) == NULL))
    {
        MOTOR_TMCL_ERROR("malloc buff failed!\n");
        exit(-1);
    }
    memset(param,0,sizeof(MotorParamType));

    fd = MotorUartInit();
    if(fd < 0)
    {
        exit(-2);
        return;
    }
    _MotorInitTxQueue(tx_buff);

    while(1)
    {
        if(flag_timer == 0)
        {
            _MotorCalibInit(param,tx_buff);
        }
        else if(flag_timer == 3000)
        {
            _MotorCfgInit(param,tx_buff);
        }
        else if(flag_timer > 4000)
        {
            _MotorMsgRx(qid,tx_buff,param);
        }

        if(flag_timer < 5000)
        {
            flag_timer++;
        }

        _MotorUartRx(fd,rx_buff,param);

        switch(_MotorTimeManage(param))
        {
            case M_TIMER_5MS_N:
                break;
            case M_TIMER_10MS_N:
                _MotorUartTx(fd,tx_buff);
                break;
            case M_TIMER_50MS_N:
                _MotorInfoQuery(tx_buff);
                break;
            case M_TIMER_100MS_N:
                break;
            default:
                break;
        }
        usleep(1000);
    }
    exit(-1);
}

/// @brief 关闭电机进程
///
///        关闭进程同时删除消息队列，勿频繁调用
/// @param None
/// @return None
void MotorDestroyProcess(void)
{
    if(motorPid > 0)
    {
        kill(motorPid,SIGKILL);
        waitpid(motorPid,NULL,0);
        msgctl(motorQid, IPC_RMID, NULL);//删除消息队列
        motorQid = -3;
        motorPid = 0;
    }
}

/// @brief 创建电机进程
///
///        使用的父进程创建子进程的方法，勿要重复调用
/// @param None
/// @return -1：消息创建失败 -2:进程创建失败 0:成功
int MotorCreateProcess(void)
{
    pid_t pid;
    int qid;

    if(motorPid != 0)
    {
        return -2;
    }

    if(motorQid < 0)
    {
        qid = msgget(IPC_PRIVATE , IPC_CREAT | 0666);
        if(qid == -1)
        {
            MOTOR_TMCL_ERROR("msg error\n");
            return -1;
        }
        motorQid = qid;
    }

    pid = fork();
    if(pid == -1)
    {
        MOTOR_TMCL_ERROR("Fork error\n");
        return -2;
    }
    else if(pid == 0)//子进程
    {
        //执行代码
        _MotorProcess(qid);
        exit(-1);
    }
    else//父进程
    {
        //获取子进程的id
        motorPid = pid;
    }

    return 0;
}
