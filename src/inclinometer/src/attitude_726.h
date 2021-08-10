/// @file  attitude_726.h
/// @brief 瑞芬倾角仪通讯代码
///
///        测头的倾角仪数据获取,版本V1.00
///        倾角仪型号HCA726-TTL
/// @author zhouhao <1198826224@qq.com>

#ifndef ATTITUDE_726_H_
#define ATTITUDE_726_H_

/// @brief 打开串口
///
///        打开与倾角仪通讯的串口
/// @param None
/// @return -1:打开串口失败  >0:所打开串口文件的id
int AttUartOpen(void);

/// @brief 关闭串口
///
///        关闭与倾角仪通讯的串口
/// @param None
/// @return 0:关闭成功
int AttUartClose(void);

/// @brief 获取角度
///
///        获取倾角仪采集的x轴角度和y轴角度
/// @param *ptrX 所采集到的x轴数据将会存入此数据
/// @param *ptrY 所采集到的y轴数据将会存入此数据
/// @return 0:关闭成功 -1:未收到数据 -2:未成功组包
int AttGetParam(float *ptrX, float *ptrY);


#endif /* ATTITUDE_726_H_ */
