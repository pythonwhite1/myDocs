# STM32学习笔记

## 一、新建工程

 1、打开Keil5软件，点击新建工程

![image-20230824143151359](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230824143151359.png)

  2、在弹出的窗口中创建保存项目的文件夹（STM32_2），在文件夹中创建三个新文件夹Library、Start、User，输入新建工程名（Project），点击保存。

![image-20230824143547986](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230824143547986.png)

 3、在弹出的窗口中选择对应的单片机型号

![image-20230824143638295](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230824143638295.png)

 5、添加启动文件，打开新建的Start文件夹

![image-20230824144003831](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230824144003831.png)

![image-20230824144048307](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230824144048307.png)

![image-20230824144129815](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230824144129815.png)

 6、添加标准外设驱动，打开Library文件夹

![image-20230824144301314](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230824144301314.png)

![image-20230824144412810](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230824144412810.png)

 7、拷贝main文件到User文件夹下

![image-20230824144600528](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230824144600528.png)

 8、将文件添加到工程中

![image-20230824144800238](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230824144800238.png)

![image-20230824144924651](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230824144924651.png)

![image-20230824145013218](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230824145013218.png)

![image-20230824145045281](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230824145045281.png)

 9、添加文件路径

![image-20230824145330751](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230824145330751.png)

 10、选择调试器，勾选自动重启并运行

![image-20230824145530016](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230824145530016.png)

## 二、模块化编程

1、新建hardware文件夹，存放硬件的模块化程序。

2、以LED为例，在keil中新建LED.c和LED.h

3、在LED.c中编写功能函数

4、在LED.h中声明LED.c对外的接口

```c
#ifndef __LED_H
#define __LED_H
// 要声明的接口
#endif
```





## 三、中断

1、中断向量表

![image-20230828185443945](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230828185443945.png)

![image-20230828185452936](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230828185452936.png)

![image-20230828185501014](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230828185501014.png)

![image-20230828185506733](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230828185506733.png)

2、NVIC优先级分组

- NVIC的中断优先级由优先级寄存器的4位（0~15）决定，这4位可以进行切分，分为高n位的抢占优先级和低4-n位的响应优先级
- 抢占优先级高的可以中断嵌套，响应优先级高的可以优先排队，抢占优先级和响应优先级均相同的按中断号排队

| **分组方式** | **抢占优先级**  | **响应优先级**  |
| ------------ | --------------- | --------------- |
| 分组0        | 0位，取值为0    | 4位，取值为0~15 |
| 分组1        | 1位，取值为0~1  | 3位，取值为0~7  |
| 分组2        | 2位，取值为0~3  | 2位，取值为0~3  |
| 分组3        | 3位，取值为0~7  | 1位，取值为0~1  |
| 分组4        | 4位，取值为0~15 | 0位，取值为0    |

3、EXIT外部中断

- EXTI（Extern Interrupt）外部中断

- EXTI可以监测指定GPIO口的电平信号，当其指定的GPIO口产生电平变化时，EXTI将立即向NVIC发出中断申请，经过NVIC裁决后即可中断CPU主程序，使CPU执行EXTI对应的中断程序

- 支持的触发方式：上升沿/下降沿/双边沿/软件触发

- 支持的GPIO口：所有GPIO口，但相同的Pin不能同时触发中断

- 通道数：16个GPIO_Pin，外加PVD输出、RTC闹钟、USB唤醒、以太网唤醒

- 触发响应方式：中断响应/事件响应

![image-20230828185819836](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230828185819836.png)

4、配置EXIT中断（以霍尔传感器为例）

Hall.c

```c
#include "stm32f10x.h"                  // Device header

uint8_t SensorCount;

void Hall_Sensor_Init(void){
	// 开启外设时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	// 初始化GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	//初始化AFIO
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource14);
	
	//初始化EXIT
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line14;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStructure);
	
	//初始化NVIC，在misc中
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);   //优先级分组配置
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);	
}

uint8_t GetSensorCount(void){
  return SensorCount;
}

/*
  声明中断函数，在startup_stm32f10x_md.s中查看中断函数名
*/
void EXTI15_10_IRQHandler(void){
  //查看指定中断的标志位是否置位
	if(EXTI_GetITStatus(EXTI_Line14) == SET){
	  //执行中断
		/*如果出现数据乱跳的现象，可再次判断引脚电平，以避免抖动*/
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) == 0)
		{
			SensorCount ++;
		}
		//复位中断标志位
	  EXTI_ClearITPendingBit(EXTI_Line14);
	}
	
}
```

> 注意：要避免在主函数和中断函数中操作同一个外设，这可能会引起冲突。

## 四、定时器

> 定时指定时间

```c
#include "stm32f10x.h"                  // Device header

void Timer_Init(void){
	//开启TIM2的RCC外设时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	// 选择时钟源，默认为内部时钟
	TIM_InternalClockConfig(TIM12);
	
	//配置TIM2
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	// 输入信号的滤波，是按一定频率采样信号，当连续出现相同电平是认为信号稳定，这个参数是控制采样频率的分频
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	// 设置计数模式，这里使用向上计数
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	// 自动重装值
	TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;
	// 预分频值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;   //高级定时器使用，这里设为0
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	// 程序无法从0开始计数，因为初始化时会自动执行一个更新事件
	// 在这里清除标志位
	TIM_ClearFlag(TIM2,TIM_IT_Update);
	
  // 使能定时器
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
	// 配置NVIC
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM2,ENABLE);
}

//void TIM2_IRQHandler(void){
//  // 检查标志位
//	if(TIM_GetITStatus(TIM2,TIM_IT_Update) == SET){
//	  // 清除中断标志位
//		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
//	}
//}

```

> 在中断函数中，如果要使用跨文件的变量，可以在中断函数所在文件使用extern申明，这样系统会自动查找变量的位置，还可以将中断函数放在变量被定义的文件中。

### 4.1 PWM

![image-20230906113606461](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230906113606461.png)

![image-20230906113720610](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230906113720610.png)

```c
/*PWM驱动呼吸灯*/
#include "stm32f10x.h"                  // Device header

void PWM_Init(void){
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);  // 对OC1的默认引脚PA0进行重映射,重映射至PA15
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  //解除PA15原本的功能
	
	TIM_InternalClockConfig(TIM2);
	//配置定时器
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 100  - 1;     //ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 720 - 1;  //PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	
	//配PWM的输出引脚
	GPIO_InitTypeDef GPIO_InitStructure;
	//配置引脚为复用推挽输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//配置输出比较单元
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_Pulse = 0;   //CCR，改变占空比
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC1Init(TIM2,&TIM_OCInitStructure);
	
		// 定时器控制
	TIM_Cmd(TIM2,ENABLE);
	
} 
/*
更改占空比
*/
void PWM_ChangePulse(uint16_t pulse){
	TIM_SetCompare1(TIM2,pulse); 
}

```



### 4.2 输入捕获IC

![](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230907143607191.png)

![image-20230907143913193](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230907143913193.png)

![(C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230907143618057.png)

> 测周法测量转速和占空比

```c
#include "stm32f10x.h"                  // Device header

void IC_Init(void){
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	TIM_InternalClockConfig(TIM3);
	//配置定时器
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536  - 1;     // 最大计数值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;  //PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	
	//配置IC引脚PA6
	GPIO_InitTypeDef GPIO_InitStructure;
	//配置引脚为上拉输入
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0xf;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;  // 选择极性上升沿触发
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  // 直通或交叉
  TIM_PWMIConfig(TIM3,&TIM_ICInitStructure);
	
	// 选择触发源
	TIM_SelectInputTrigger(TIM3,TIM_TS_TI1FP1);
	// 选择从模式
	TIM_SelectSlaveMode(TIM3,TIM_SlaveMode_Reset);
	
		// 定时器控制
	TIM_Cmd(TIM3,ENABLE);
}
// 获取频率
uint16_t IC_GetFreq(void){
  return 1000000 / TIM_GetCapture1(TIM3);
}
// 获取占空比
uint16_t IC_GetDuty(void){
  return (TIM_GetCapture2(TIM3) + 1) * 100 / (TIM_GetCapture1(TIM3) + 1);
}

```



### 4.3 编码器接口

![image-20230907144105358](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230907144105358.png)

```c
#include "stm32f10x.h"                  // Device header

void Encoder_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;		//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;		//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0xF;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0xF;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	TIM_Cmd(TIM3, ENABLE);
}

int16_t Encoder_Get(void)
{
	int16_t Temp;
	Temp = TIM_GetCounter(TIM3);
	TIM_SetCounter(TIM3, 0);
	return Temp;
}

```

>在Timer中配置一个1s的中断
>
>```c
>#include "stm32f10x.h"                  // Device header
>
>void Timer_Init(void)
>{
>	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
>
>	
>	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
>	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
>	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
>	TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;
>	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;
>	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
>	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
>	
>	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
>	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
>	
>	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
>	
>	NVIC_InitTypeDef NVIC_InitStructure;
>	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
>	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
>	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
>	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
>	NVIC_Init(&NVIC_InitStructure);
>	
>	TIM_Cmd(TIM4, ENABLE);
>}
>
>/*
>void TIM2_IRQHandler(void)
>{
>	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
>	{
>		
>		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
>	}
>}
>*/
>
>```
>
>

> 在主函数中定义1秒中断的中断服务函数，此时获得Ecount值就是1秒内的counter值，也就是速度
>
> ```c
> void TIM4_IRQHandler(void)//定时器中断 1ms
> {
>     if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) 
>     {
>         Ecount = Encoder_Get();  
>         //count++;			
>     }
>     TIM_ClearITPendingBit(TIM4,TIM_IT_Update); 
> 	}
> ```

### 4.4 步进电机控制

 4.4.1 TB6600驱动器
![image-20230915174840161](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230915174840161.png)        

 通过拨码开关设定细分与电流

![image-20230915174928848](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230915174928848.png)



4. 4.2 共阴共阳接线法

![image-20230915175028870](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230915175028870.png)


共阴：

        驱动器  ——  stm32
    
        DIR- 与 PUL-  ——  GND
        DIR+ —— 方向引脚
        PUL+ —— 脉冲引脚

共阳：

        驱动器  ——  stm32
    
        DIR+ 与 PUL+  ——  +5V
        DIR- —— 方向引脚
        PUL- —— 脉冲引脚

4. 4.3 42步进电机

![image-20230915180153207](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230915180153207.png)

4.4.4 基本知识
        42步进电机的相数为2（A、B两相，每相并联支路数为2，每条支路串联2个线圈）、步距角为1.8°。

        ① 拍数（N=km）——每一次循环所包含的通电状态数（电机转过一个齿距角所需脉冲数）；
             单拍制（k=1）——拍数 = 相数；双拍制（k=2）——拍数 = 相数的两倍； 
    
        ② 相数（m）——即电机内部的线圈组数。如果使用细分驱动器，则相数将变得没有意义，只需在驱动器上改变细分数，就可以改变步距角；
    
        ④ 步距角（θs）——步进机通过一个电脉冲转子转过的角度；



4.4.5 转速细分控制
        电机的转速与脉冲频率成正比，电机转过的角度与脉冲数成正比。所以控制脉冲数和脉冲频率就可以精确调速。

         f：脉冲频率；θs：步距角；X：细分值；n：转速（rad/s）。
    
        若已知步距角=1.8°，细分值=32，想要达到1rad/s的转速（每秒转一圈），则脉冲频率=1*32*360/1.8=6400，即6400个脉冲为一转。

4.4.6 STM32控制

连线，这里使用共阴极连线

![image-20230915180752637](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230915180752637.png)

代码

mian.c

```c
#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "PWM.h"
#include "Motor.h"

int main(void){
	OLED_Init();
	PWM_Init();
	Motor_Init();
	Motor_Rotate(0);
	

  while(1){

	}
}

```

PWM.c

```c
#include "stm32f10x.h"                  // Device header

void PWM_Init(void){
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);  // 对OC1的默认引脚PA0进行重映射,重映射至PA15
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  //解除PA15原本的功能
	
	TIM_InternalClockConfig(TIM2);
	//配置定时器
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 100  - 1;     //ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 225 - 1;  //PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	
	//配PWM的输出引脚
	GPIO_InitTypeDef GPIO_InitStructure;
	//配置引脚为复用推挽输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//配置输出比较单元
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_Pulse = 50;   //CCR，改变占空比
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC1Init(TIM2,&TIM_OCInitStructure);
	
		// 定时器控制
	TIM_Cmd(TIM2,ENABLE);
	
} 


```

Motor.c

```c
#include "stm32f10x.h"                  // Device header

void Motor_Init(void){
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
  GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

void Motor_Rotate(uint8_t dir){
  if(dir == 0){
	  // 电机正转
		GPIO_SetBits(GPIOB,GPIO_Pin_3);
	}
	else if(dir == 1){
		// 电机反转
	  GPIO_ResetBits(GPIOB,GPIO_Pin_3);
	}
}

```



## 五、ADC

![image-20230910204242709](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230910204242709.png)

>- 开启RCC，ADC都在APB2
>- 配置分频器
>- 配置引脚为模拟输入，ADC引脚为PA0-PB1（STM32C8T6江科大开发板上的引脚顺序）
>- 配置注入通道
>- 初始化ADC
>- 开启控制
>- 校准

### 单通道

```c
#include "stm32f10x.h"                  // Device header

void AD_Init(void){
	// 开启时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	// 配置分频器,这个函数在rcc中
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	// 初始化GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	// 配置注入通道
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_55Cycles5);
	
	// 初始化ADC
	ADC_InitTypeDef ADC_InitStructure;
	// 单ADC模式
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	// 触发方式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	// 指定转换通道
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	// 连续转换模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;    // 若开启连续转换模式，只需在校准后触发转换一次
	// 扫描模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_Init(ADC1,&ADC_InitStructure);
	
	// 开启控制
	ADC_Cmd(ADC1,ENABLE);
	
	// 校准
	ADC_ResetCalibration(ADC1);
	// 获取重置校准寄存器的状态，状态寄存器为0时初始化完成
  while(ADC_GetResetCalibrationStatus(ADC1) == SET);   // 等于1就一直循环等待
	// 开始校准
  ADC_StartCalibration(ADC1);        
	// 获取校准状态，校准完成，硬件清除位
  while(ADC_GetCalibrationStatus(ADC1) == SET);
	
	// 使用连续转换模式，只需初始化时触发一次转换
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	
}

// 软件触发转换，并获取转换值
uint16_t AD_GetConvValue(void){
	// 软件触发转换
  ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	// 监测转换状态，ADC_FLAG_EOC: End of conversion flag   该位由硬件在(规则或注入)通道组转换结束时设置，由软件清除或由读取ADC_DR时清除
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC) == RESET);   // 开始转换后结束循环
	// 返回转换值
	return ADC_GetConversionValue(ADC1);	
}

```

### 多通道

```c
#include "stm32f10x.h"                  // Device header

/**
  * @brief  当配置为多通道若不及时将数据取走，那么后面的数据会覆盖之前的数据，但手动取数据无法确定
	每个通道的转换完成时间，只有配置成完成一个通道暂停一下，手动取数后继续
  */

void AD_Init(void){
	// 开启时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	// 配置分频器,这个函数在rcc中
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	// 初始化GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	// 配置注入通道
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_55Cycles5);
	
	// 初始化ADC
	ADC_InitTypeDef ADC_InitStructure;
	// 单ADC模式
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	// 触发方式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	// 指定转换通道
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	// 连续转换模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;    // 若开启连续转换模式，只需在校准后触发转换一次
	// 扫描模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_Init(ADC1,&ADC_InitStructure);
	
	// 开启控制
	ADC_Cmd(ADC1,ENABLE);
	
	// 校准
	ADC_ResetCalibration(ADC1);
	// 获取重置校准寄存器的状态，状态寄存器为0时初始化完成
  while(ADC_GetResetCalibrationStatus(ADC1) == SET);   // 等于1就一直循环等待
	// 开始校准
  ADC_StartCalibration(ADC1);        
	// 获取校准状态，校准完成，硬件清除位
  while(ADC_GetCalibrationStatus(ADC1) == SET);
	
}

// 软件触发转换，并获取转换值
uint16_t AD_GetConvValue(uint8_t ADC_Channel){
		// 配置规则通道
	ADC_RegularChannelConfig(ADC1,ADC_Channel,1,ADC_SampleTime_55Cycles5);
	// 软件触发转换
  ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	// 监测转换状态，ADC_FLAG_EOC: End of conversion flag   该位由硬件在(规则或注入)通道组转换结束时设置，由软件清除或由读取ADC_DR时清除
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC) == RESET);   // 开始转换后结束循环
	// 返回转换值
	return ADC_GetConversionValue(ADC1);	
}

```

### DMA

## 六、USART



### Tx

```c
#include "stm32f10x.h"                  // Device header

void Serial_Init(void){
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
	// 奇偶校验
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1,&USART_InitStructure);
	
	USART_Cmd(USART1,ENABLE);
	
}

void Serial_SendByte(uint8_t message){
  USART_SendData(USART1,message);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t length){
	uint16_t i;
	for(i=0; i < length; i++){
	  Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *str){
	uint8_t i;
  for(i = 0;str[i] != '\0' ; i++){
	  Serial_SendByte(str[i]);
	}
}

/*
将数字的每一位转变成字符发送
如num = 12345
1 = num / 10000 % 10
2 = num / 1000 % 10
3 = num / 100 % 10
.
.

*/
// 返回X^Y
uint16_t Serial_Pow(uint16_t x, uint16_t y){
  uint16_t result=1;
	while(y--){
	  result *= x;
	}
	return result;
}
void Serial_SendNumber(uint16_t num, uint16_t length){
  uint16_t i;
	for(i=0; i < length; i++){
	  Serial_SendByte(num / Serial_Pow(10,length - i - 1) % 10 + '0');
	}
}

```

> 移植printf的三种方法

![image-20230914171646313](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230914171646313.png)

- 重定向fputc函数，这是printf的底层函数

```c
#include <stdio.h>
int fputc(int ch,FILE *f){
  Serial_SendByte(ch);
	return ch;
}

printf("%d\n\r" , 12345);
```

- 使用sprintf

```c
	char String[100];
	sprintf(String, "Num=%d\r\n",666);
	Serial_SendString(String);
```

- 封装sprintf

```c
#include <stdarg.h>

void Serial_Printf(char *format,...){
  char String[100];
	va_list arg;
	va_start(arg,format);
	vsprintf(String,format,arg);
	va_end(arg);
	Serial_SendString(String);
}

Serial_Printf("Num=%d\r\n",666);
```

> - 关于使用printf打印中文，要选好Encoding，与串口调试助手相对应。
> - 在写pirntf("你好，世界！")编译器可能会报错
>
> ![image-20230914174220537](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230914174220537.png)
>
> ​    --no-multibyte-chars

### Rx

接收一个字节数据

```c
#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>

uint16_t Rdata;
uint8_t RdataFlag;

void Serial_Init(void){
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   // 推荐配置为上拉输入或浮空输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	// 奇偶校验
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1,&USART_InitStructure);
	
	// 使用中断
	USART_ITConfig(USART1,	USART_IT_RXNE,ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1,ENABLE);
	
}

void Serial_SendByte(uint8_t message){
  USART_SendData(USART1,message);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t length){
	uint16_t i;
	for(i=0; i < length; i++){
	  Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *str){
	uint8_t i;
  for(i = 0;str[i] != '\0' ; i++){
	  Serial_SendByte(str[i]);
	}
}

/*
将数字的每一位转变成字符发送
如num = 12345
1 = num / 10000 % 10
2 = num / 1000 % 10
3 = num / 100 % 10
.
.

*/
// 返回X^Y
uint16_t Serial_Pow(uint16_t x, uint16_t y){
  uint16_t result=1;
	while(y--){
	  result *= x;
	}
	return result;
}
void Serial_SendNumber(uint16_t num, uint16_t length){
  uint16_t i;
	for(i=0; i < length; i++){
	  Serial_SendByte(num / Serial_Pow(10,length - i - 1) % 10 + '0');
	}
}

// 重定向fputc
int fputc(int ch,FILE *f){
  Serial_SendByte(ch);
	return ch;
}

// 封装sprintf
void Serial_Printf(char *format,...){
  char String[100];
	va_list arg;
	va_start(arg,format);
	vsprintf(String,format,arg);
	va_end(arg);
	Serial_SendString(String);
}

// 数据读取标志位
uint8_t Serial_ReadFlag(void){
  	if(RdataFlag == 1){
			RdataFlag = 0;
		  return 1;
		}
		return 0;
}

// 获取接收到的数据
uint16_t Serial_GetRxData(void){
	
  return Rdata;
}
// USART1接收数据中断服务函数
void USART1_IRQHandler(void){
  if(USART_GetITStatus(USART1,USART_IT_RXNE) == SET){
		Rdata = USART_ReceiveData(USART1);
		RdataFlag = 1;
	  USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
}

```

接收数据包

Serial.c

```c
#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>

uint8_t Tdata[4];
uint8_t Rdata[4];
uint8_t RdataFlag;

void Serial_Init(void){
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   // 推荐配置为上拉输入或浮空输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	// 奇偶校验
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1,&USART_InitStructure);
	
	// 使用中断
	USART_ITConfig(USART1,	USART_IT_RXNE,ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1,ENABLE);
	
}

void Serial_SendByte(uint8_t message){
  USART_SendData(USART1,message);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t length){
	uint16_t i;
	for(i=0; i < length; i++){
	  Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *str){
	uint8_t i;
  for(i = 0;str[i] != '\0' ; i++){
	  Serial_SendByte(str[i]);
	}
}

/*
将数字的每一位转变成字符发送
如num = 12345
1 = num / 10000 % 10
2 = num / 1000 % 10
3 = num / 100 % 10
.
.

*/
// 返回X^Y
uint16_t Serial_Pow(uint16_t x, uint16_t y){
  uint16_t result=1;
	while(y--){
	  result *= x;
	}
	return result;
}
void Serial_SendNumber(uint16_t num, uint16_t length){
  uint16_t i;
	for(i=0; i < length; i++){
	  Serial_SendByte(num / Serial_Pow(10,length - i - 1) % 10 + '0');
	}
}

// 重定向fputc
int fputc(int ch,FILE *f){
  Serial_SendByte(ch);
	return ch;
}

// 封装sprintf
void Serial_Printf(char *format,...){
  char String[100];
	va_list arg;
	va_start(arg,format);
	vsprintf(String,format,arg);
	va_end(arg);
	Serial_SendString(String);
}

// 数据读取标志位
uint8_t Serial_ReadFlag(void){
  	if(RdataFlag == 1){
			RdataFlag = 0;
		  return 1;
		}
		return 0;
}

/**
  * @brief  发送数据，添加包头FF，包尾FE
  */
void Serial_SendDataPackage(void){
  Serial_SendByte(0xFF);
	Serial_SendArray(Tdata,4);
	Serial_SendByte(0xFE);
}

// USART1接收数据中断服务函数
void USART1_IRQHandler(void){
	// static 静态变量，只会初始化一次，函数退出，依然不变
	static uint8_t receive_state = 0;
	static uint8_t receive_data_count = 0;
  if(USART_GetITStatus(USART1,USART_IT_RXNE) == SET){
		uint8_t RdataF = USART_ReceiveData(USART1);

		// 状态机
		if(receive_state == 0){
			// 是否接收到包头
			if(RdataF == 0xFF){
			  receive_state = 1;
			}
		}
		else if(receive_state == 1){
			Rdata[receive_data_count] = RdataF;
			receive_data_count ++;
			if(receive_data_count >= 4){
				receive_data_count = 0;
				receive_state = 2;
			}
		}
		else if(receive_state == 2){
		  // 是否接收到包尾
			if(RdataF == 0xFE){
			  receive_state = 0;
				RdataFlag = 1;
			}
		}
	  USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
}

```

main.c

```c
#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Key.h"

int main(void){
	OLED_Init();
	Serial_Init();  
	KeyInit();
	Tdata[0] = 0x98;
	Tdata[1] = 0x97;
	Tdata[2] = 0x96;
	Tdata[3] = 0x95;
//	
//	Serial_SendDataPackage();
	OLED_ShowString(1,1,"Tdata");
	OLED_ShowString(3,1,"Rdata");

  while(1){
		if(ScanInput() == 1){
			Tdata[0] ++;
			Tdata[1] ++;
			Tdata[2] ++;
			Tdata[3] ++;
			
			Serial_SendDataPackage();
			
			OLED_ShowHexNum(2,1,Tdata[0],2);
			OLED_ShowHexNum(2,4,Tdata[1],2);
			OLED_ShowHexNum(2,7,Tdata[2],2);
			OLED_ShowHexNum(2,10,Tdata[3],2);
		}
    if(Serial_ReadFlag() == 1){
		  Serial_SendArray(Rdata,4);
			OLED_ShowHexNum(4,1,Rdata[0],2);
			OLED_ShowHexNum(4,4,Rdata[1],2);
			OLED_ShowHexNum(4,7,Rdata[2],2);
			OLED_ShowHexNum(4,10,Rdata[3],2);
		}
	}
}

```

