/// @file  attitude_726.c
/// @brief 瑞芬倾角仪通讯代码
///
///        测头的倾角仪数据获取,版本V1.00
///        倾角仪型号HCA726-TTL
///        波特率9600bps,查询式获取数据
/// @author zhouhao <1198826224@qq.com>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <string.h>
#include <sys/ioctl.h>

#define ATT_PORT_PATH "/dev/InclinometerPort"
#define ATT_RX_MAX_BUFF 256


enum
{
    Att726RxWait1_N = 0,
    Att726RxWait2_N,
    Att726RxWait3_N,
    Att726RxArrange_N,
    Att726RxCheckSum_N,
    Att726End_N,
};

#pragma pack(1)
typedef struct
{
    unsigned char state;
    unsigned char buff[16];
    float ang_x;
    float ang_y;
    unsigned char flag;
    unsigned char now_len;
}AttPackParamType;
#pragma pack()

static int attUartFd = -1;


//累加和校验
static unsigned char _AttCheckSumCalc(unsigned char *data,unsigned char len)
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

static int _AttUartOpen(int fd, const char *ptrPath)
{
    //printf("try to open %s ...\n", ptrPath);
    fd = open(ptrPath, O_RDWR | O_NOCTTY | O_NONBLOCK);//非阻塞式读写
    if (-1 == fd){
        perror("Can't Open Serial Port");
        return(-1);
    }
    //else
        //printf("open %s .....\n", ptrPath);

    //printf("fd-open=%d\n",fd);
    return fd;
}

/* 五个参量 fd打开文件 speed设置波特率 bit数据位设置   nevent奇偶校验位 stop停止位 */
static int _AttUartSet(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;
    int baud;

    if ( tcgetattr( fd,&oldtio) != 0) {
        perror("SetupSerial 1");
        return -1;
    }

    bzero( &newtio, sizeof( newtio ) );
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch( nBits )
    {
        case 7: newtio.c_cflag |= CS7;
        break;
        case 8: newtio.c_cflag |= CS8;
        break;
    }

    switch( nEvent )
    {
        case 'O':
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'N':
            newtio.c_cflag &= ~PARENB;
            break;
    }

    switch( nSpeed )
    {
        case 1200:   baud = B1200;   break;
        case 2400:   baud = B2400;   break;
        case 4800:   baud = B4800;   break;
        case 9600:   baud = B9600;   break;
        case 19200:  baud = B19200;  break;
        case 38400:  baud = B38400;  break;
        case 57600:  baud = B57600;  break;
        case 115200: baud = B115200; break;
        case 230400: baud = B230400; break;
        case 460800: baud = B460800; break;
        default:     baud = B9600;   break;
    }
    cfsetispeed(&newtio, baud);
    cfsetospeed(&newtio, baud);

    if( nStop == 1 )       newtio.c_cflag &= ~CSTOPB;
    else if ( nStop == 2 ) newtio.c_cflag |= CSTOPB;

    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
//串口只是传输数据，不需要串口来处理，则使用原始模式(Raw Mode)
    newtio.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
    newtio.c_oflag  &= ~OPOST;   /*Output*/

    tcflush(fd, TCIFLUSH);
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
        perror("com set error");
        return -1;
    }
    //printf("set done!\n");

    return 0;
}

/// @brief 转换函数
///
///        按通讯协议，收到的数据需要转换才能变可用的BCD码
/// @param *ptrParam 相关组包参数
/// @return 转换后的可用数据
float _AttBcdChange(unsigned char *ptrData)
{
    int tempVaule = 0;
    float res = 0;

    tempVaule = ptrData[0]&0x0F;
    for(int i=0; i<3;i++)
    {
        tempVaule = (tempVaule*10)+((ptrData[i+1]&0xF0)>>4);
        tempVaule = (tempVaule*10)+(ptrData[i+1]&0x0F);
    }

    if((ptrData[0]&0x10))
    {
        tempVaule = -tempVaule;
    }

    res = (float)tempVaule/10000;

    return res;
}

/// @brief 组包函数
///
///        对收到的数据进行组包校验
/// @param tData 当前收到的字节值
/// @param *ptrParam 相关组包参数
/// @return none
void _Att726ParsingPackage(unsigned char tData,
                          AttPackParamType *ptrParam)
{
    int sum_check = 0;

    switch(ptrParam->state)
    {
    case Att726RxWait1_N:
        ptrParam->now_len = 0;
        if(tData == 0x68)
        {
            ptrParam->buff[ptrParam->now_len++] = tData;
            ptrParam->state = Att726RxWait2_N;
        }
        break;
    case Att726RxWait2_N:

        if(tData == 0x0F)
        {
            ptrParam->buff[ptrParam->now_len++] = tData;
            ptrParam->state = Att726RxWait3_N;
        }
        else
        {
            ptrParam->state = Att726RxWait1_N;
        }
        break;
    case Att726RxWait3_N:
        if(tData == 0x00)
        {
            ptrParam->buff[ptrParam->now_len++] = tData;
            ptrParam->state = Att726RxArrange_N;
        }
        else
        {
            ptrParam->state = Att726RxWait1_N;
        }
        break;
    case Att726RxArrange_N:

        ptrParam->buff[ptrParam->now_len++] = tData;
        if(ptrParam->now_len >= 15)
        {
            ptrParam->state = Att726RxCheckSum_N;
        }
        break;
    case Att726RxCheckSum_N:

        ptrParam->buff[ptrParam->now_len] = tData;
        sum_check = _AttCheckSumCalc(&ptrParam->buff[1],14);

        if(sum_check == tData)
        {
            ptrParam->ang_x = _AttBcdChange(&ptrParam->buff[4]);
            ptrParam->ang_y = _AttBcdChange(&ptrParam->buff[8]);
            ptrParam->flag = 1;
        }
        else
        {
            printf("sumcheck err ! \n");
        }
        ptrParam->state = Att726RxWait1_N;
        break;
    default:
        break;
    }
}

/// @brief 发送角度查询命令
///
/// @param fd 所打开串口文件的id
/// @return 成功发送的数据长度
int _AttQueryCmdTx(int fd)
{
    unsigned char txData[6];
    int res;

    txData[0] = 0x68;
    txData[1] = 0x04;
    txData[2] = 0x00;
    txData[3] = 0x04;
    txData[4] = 0x08;

    res = write(fd,txData,5);
    return res;
}

/// @brief 获取角度
///
///        获取倾角仪采集的x轴角度和y轴角度
/// @param *ptrX 所采集到的x轴数据将会存入此数据
/// @param *ptrY 所采集到的y轴数据将会存入此数据
/// @return 0:关闭成功 -1:未收到数据 -2:未成功组包
int AttGetParam(float *ptrX, float *ptrY)
{
    fd_set readfd;
    int res = 0, ret = 0;
    struct timeval timeout = {0, 100000};
    unsigned char buf[ATT_RX_MAX_BUFF] = {0};
    AttPackParamType attParam;

    res = _AttQueryCmdTx(attUartFd);
    if(res < 0)
    {
        return -3;
    }

    FD_ZERO(&readfd);
    FD_SET(attUartFd, &readfd);
    ret = select(attUartFd + 1, &readfd, NULL, NULL, &timeout); //文件打开方式使用非阻塞式时，需要使用select函数

    if (ret < 0)
    {
        return -1;
    }

    if (FD_ISSET(attUartFd, &readfd))
    {
        usleep(2000);
        res = read(attUartFd, buf, ATT_RX_MAX_BUFF);

        if (res == 0)
        {
            printf("Gyroscope module link error!\r\n");
            return -1;
        }
        else if (res == -1)
        {
            printf("Gyroscope read uart data fail!\r\n");
            return -1;
        }
    }
    else
    {
        printf("Gyroscope module can't read the uart file!\r\n");
        return -1;
    }

    if(res > 0)
    {
        memset(&attParam,0,sizeof(AttPackParamType));
        for(int j=0;j<res;j++)
        {
            _Att726ParsingPackage(buf[j],&attParam);
        }

        if(attParam.flag == 1)
        {
            *ptrX = attParam.ang_x;
            *ptrY = attParam.ang_y;
            return 0;
        }
        else
        {
            return -2;
        }
    }
    else
    {
        return -1;
    }
}


/// @brief 打开串口
///
///        打开与倾角仪通讯的串口
/// @param None
/// @return -1:打开串口失败  >0:所打开串口文件的id
int AttUartOpen(void)
{

    if((attUartFd = _AttUartOpen(attUartFd, ATT_PORT_PATH)) < 0)
    {
        return -1;
    }

    if((_AttUartSet(attUartFd,115200,8,'N',1)) < 0)
    {
        return -1;
    }

    return attUartFd;
}

/// @brief 关闭串口
///
///        关闭与倾角仪通讯的串口
/// @param None
/// @return 0:关闭成功
int AttUartClose(void)
{
    close(attUartFd);

    return 0;
}



