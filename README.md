# ROS2阿克曼开发平台开源方案介绍

## 0、前言

### 0.1、项目背景：

为了满足更多的同学可以**快速**、**低成本**、**高兼容性**的搭建自己的阿克曼ROS2开发平台，所以出现本开源项目。

> 温馨提示：本次开源的硬件和软件是完全兼容差速和阿克曼方案的，也就是说你如果要做差速，更改机械部分之后，只需要改极少的配置文件就可以做到兼容。

开源项目发布方：**公众号：小白学移动机器人**

开源项目的内容：机械模型、底层驱动板PCB、底层驱动程序、ROS2部分功能包（启动功能包，描述功能包）等等

开源项目github地址获取：在公众号小白学移动机器人，发送：**ROS2阿克曼**，即可获取。

项目文件结构如下：

![](https://zhaokx-notes.oss-cn-shenzhen.aliyuncs.com/image/image-20221023110108099.png)



### 0.2、开源项目的总体方案：

![](https://zhaokx-notes.oss-cn-shenzhen.aliyuncs.com/image/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202022-10-22%20192528.png)

### 0.3、开源项目的总体架构：

![](https://zhaokx-notes.oss-cn-shenzhen.aliyuncs.com/image/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202022-09-12%20220959.png)

###0.4、开源项目的底层控制方案：

![](https://zhaokx-notes.oss-cn-shenzhen.aliyuncs.com/image/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202022-10-22%20192348.png)

## 1、机械模型呈现

**详见：1.阿克曼小车机械模型文件**

这里机械板材的设计做了高兼容性：

> 1. 核心板安装兼容：树莓派、香橙派3LTS、jetson nano
> 2. 雷达兼容：RPLIDAR A1M8、乐动LD14、乐动LD19

左视图如下：

![](https://zhaokx-notes.oss-cn-shenzhen.aliyuncs.com/image/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202022-10-22%20131738.png)

俯视图如下：

![](https://zhaokx-notes.oss-cn-shenzhen.aliyuncs.com/image/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202022-10-22%20131813.png)

## 2、硬件方案介绍

详见：2.阿克曼小车硬件方案介绍

### 2.1、底层驱动板

**功能模块设计如下：**

![](https://zhaokx-notes.oss-cn-shenzhen.aliyuncs.com/image/1.%E8%AE%BE%E8%AE%A1%E5%8A%9F%E8%83%BD%E6%A8%A1%E5%9D%97%E5%9B%BE.png)

**主控电路板PCB如下：**

![](https://zhaokx-notes.oss-cn-shenzhen.aliyuncs.com/image/2.%E5%9F%BA%E4%BA%8ESTM32%E5%8D%95%E7%89%87%E6%9C%BA%E5%BA%95%E5%B1%82PCB%E5%9B%BE.png)

**12V-5V 5A 电源板如下：**

![](https://zhaokx-notes.oss-cn-shenzhen.aliyuncs.com/image/3.%E7%94%B5%E6%BA%90%E6%9D%BF%E5%9B%BE%EF%BC%88%E8%B4%AD%E4%B9%B0mini%20560%2012V-5V%205A%EF%BC%89.png)

### 2.2、核心板（香橙派3LTS）

开发板功能如下图：详见：官网：http://www.orangepi.cn/

![](https://zhaokx-notes.oss-cn-shenzhen.aliyuncs.com/image/%E9%A6%99%E6%A9%99%E6%B4%BE3LTS-%E6%AD%A3%E9%9D%A2.png)

## 3、底层驱动程序工程文件

详见：4.底层驱动程序工程文件

> 程序很简单，需要的同学可以自己看一下。
>
> 如果仍存在问题，可以参考我的系列博客文章解答你的问题。
>
> 博客链接：https://blog.csdn.net/zhao_ke_xue/article/details/108138981

博客详细的讲解的每一个控制模块的内容，如下所示：

![](https://zhaokx-notes.oss-cn-shenzhen.aliyuncs.com/image/image-20221022234607205.png)

**部分主流程代码如下：**

```c
#include "sys.h"

/*===================================================================
程序功能：ROS小车底层代码（全部） 差速底盘和阿克曼底盘通用
程序编写：公众号：小白学移动机器人
其他    ：如果对代码有任何疑问，可以私信小编，一定会回复的。
=====================================================================
------------------关注公众号，获得更多有趣的分享---------------------
===================================================================*/
int main(void)
{ 
	//发送计数
	char sendCount=0;
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//禁用JTAG 启用 SWD
	
	MY_NVIC_PriorityGroupConfig(2);	//=====设置中断分组
	
	delay_init();	    	        //=====延时函数初始化
	LED_Init();                     //=====LED初始化    程序灯	
	BEEP_Init();                    //=====蜂鸣器初始化，异常报警
	
	usart1_init(115200);	        //=====串口1初始化  树莓派通信
	usart3_init(115200);            //=====串口3初始化  蓝牙，控制和调试

	IIC_Init();                     //=====IIC初始化    读取MPU6050数据
	MPU6050_initialize();           //=====MPU6050初始化	
	DMP_Init();                     //=====初始化DMP 

	Encoder_Init_TIM3();            //=====初始化右轮编码器接口
	Encoder_Init_TIM4();            //=====初始化左轮编码器接口
	
	MY_ADC_Init();                  //=====adc初始化    电池电量检测
	
	Motor_PWM_Init(7200-1, 1-1);    //=====初始化PWM 10KHZ(AT8870)，用于驱动电机 
	Steer_PWM_Init(60000-1, 24-1);  //=====初始化PWM 50HZ，用于驱动舵机

	PID_Init();                     //=====PID初始化
	
	MBOT_EXTI_Init();               //=====MPU6050 5ms定时中断初始化

	while(1)
	{
		//给树莓派发送速度，角度,这里速度已经乘以1000,角度(0.0-359.0)乘以50
		if(sendCount==0)//发送  14.4ms  发送一次数据 70Hz 左右
		{
			//发送需要一定的延时
			usartSendData(USART1,(short)leftSpeedNow,(short)rightSpeedNow,(short)(yaw*50),sendCtrlFlag);  //1ms
			//蓝牙调试时用，不调试注释
			// pcShow();                                                           //2.2ms单个float数据
			sendCount++;
		}
		else
		{
			sendCount++;
			if(sendCount==25)
				sendCount=0;
		}
		//获取角度		
		getAngle(&yaw,&yaw_acc_error);                                                     
	} 
}

//中断服务函数
void USART1_IRQHandler()
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
 	 {
		 USART_ClearITPendingBit(USART1,USART_IT_RXNE);//首先清除中断标志位
		 usartReceiveOneData(USART1,&leftSpeedSet,&rightSpeedSet,&frontAngleSet,&receCtrlFlag);
	 }
}
```

**中断主流程代码如下：**

```c
/**************************************************************************
函数功能：所有的控制代码都在这里面
          5ms定时中断由MPU6050的INT引脚触发		
**************************************************************************/
void EXTI9_5_IRQHandler(void) 
{                                                         
	EXTI_ClearITPendingBit(EXTI_Line8);                             //===清除LINE8线路挂起位		
	
	Led_Flash(200);                                                 //===LED闪烁，证明程序正常运行	
	
	yaw_acc_error += FIVE_MS_ERROR;								    //===yaw漂移误差累加
	
	Get_battery_volt_average(&Voltage,100);		                    //===电池电压测量，单位mv，100次取一次平均
	
	Get_Motor_Speed(&leftSpeedNow,&rightSpeedNow);                  //===获取左右轮子真实速度
	
	pid_Task_Letf.speedSet  = leftSpeedSet;	                        //===给速度设定值和实时值赋值
	pid_Task_Right.speedSet = rightSpeedSet;
	pid_Task_Letf.speedNow  = leftSpeedNow;
	pid_Task_Right.speedNow = rightSpeedNow;
	
	Pid_Ctrl(&motorLeft,&motorRight);                               //===执行PID控制函数
	Steer_Ctrl(frontAngleSet,&motorFrontSteer);                     //===执行舵机，差速小车无需修改完全兼容

	if(Turn_Off(Voltage) == 0)                                      //===如果电量异常低
	{	
		BEEP_Disable();                                             //===正常情况，蜂鸣器失能 
		Set_Pwm(motorLeft,motorRight,motorFrontSteer);              //===赋值给PWM寄存器 		
	} 	  
	else
	{
		BEEP_Flash(20);                                            	//===电量报警
	}
} 
```

## 4、机械模型转URDF模型文件

详见：5.机械模型转URDF模型文件介绍

**rviz显示如下图：**

![](https://zhaokx-notes.oss-cn-shenzhen.aliyuncs.com/image/image-20221022230442665.png)

## 5、阿克曼结构运动模型详解

详见：6.阿克曼结构运动模型详解

![](https://zhaokx-notes.oss-cn-shenzhen.aliyuncs.com/image/image-20221022230733457.png)

![](https://zhaokx-notes.oss-cn-shenzhen.aliyuncs.com/image/image-20221022230825100.png)

## 6、阿克曼ROS2相关功能包

详见：7.ROS2相应功能包

```bash
.
└── src
    ├── mbot_bringup        # 机器人启动功能包 和 底层单片机通信、发布里程计、tf、订阅控制话题
    ├── mbot_description    # 机器人描述功能包，机械模型转的urdf
    └── sllidar_ros2-humble # RPLIDAR A1M8 的ROS2驱动
```

程序很简单，需要的同学可以自己看一下。

## 7、系统配置以及功能包启动说明

详见：8.系统配置以及功能包启动

![](https://zhaokx-notes.oss-cn-shenzhen.aliyuncs.com/image/image-20221022232141811.png)

##  8、总结

如果你觉得本开源项目还不错，可以积极的分享到你的同学、朋友。

**如果你感觉，我的文章比较适合你，关注我，点个赞，给你不一样的惊喜。**

![](https://zhaokx-notes.oss-cn-shenzhen.aliyuncs.com/image/end.gif)

![](https://zhaokx-notes.oss-cn-shenzhen.aliyuncs.com/image/end2.gif)