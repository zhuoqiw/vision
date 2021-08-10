
### 使用示例
```c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "motor_tmcl.h"

//使用示例(详细使用方法见头文件接口备注)
int main(int argc,char *argv[])
{
    int qid;//消息id
    qid = MotorCreateProcess();//创建进程
    sleep(10);

    while(1)
    {
        int res;
        res = MotorCtlScan(3200);//进行扫描，设置速度
        sleep(10);
        res = MotorCtlZero();//回归零点
        sleep(10);
        res = MotorCtlCenter();//移至中心
        sleep(10);
        res = MotorCtlScan(32000);//进行扫描设置速度
        sleep(10);
    }
}
```






