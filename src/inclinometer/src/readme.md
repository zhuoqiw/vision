
### 使用示例
```c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "attitude_726.h"


//使用示例，详细使用方法见头文件注释
int main(int argc,char *argv[])
{
    float x,y;

    while(1)
    {
        //首先打开串口
        AttUartOpen();
		//获取x轴和y轴的数据
        AttGetParam(&x,&y);
        printf("angx: %f angy: %f \r\n",x,y);
		//获取到后关闭串口(也可打开一次后不再关闭)
        AttUartClose();
        sleep(5);
    }
}
```






