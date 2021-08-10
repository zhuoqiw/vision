
### 使用示例
```c
#include <iostream>
#include <utility>
#include <thread>
#include <chrono>
#include <functional>
#include <atomic>
#include <unistd.h>
#include "motor_tmcl.hpp"


std::unique_ptr<motor_tmcl::MotorTmcl> _t1;

//到位反馈的回调
void ReachCallback(int type)
{
    std::cout << "Successful arrival" << type <<std::endl;
}

int main()
{

	_t1 = std::make_unique<motor_tmcl::MotorTmcl>(ReachCallback);
	while(1)
	{
        //移动至初始点
        _t1->MotorCtlZero();
	    sleep(10);
        //移动至中心
	    _t1->MotorCtlCenter();
	    sleep(10);
        //扫描流程
        _t1->MotorCtlScan(3600);
        sleep(10);         
	}

}
```






