#include "control.h"	
#include "filter.h"
#include "usart.h"

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
u8 Flag_Target;
Bit_Flag Flag={0,0,0,0,0,1,0,0};	//sys.h文件中的位域，用于定义所有标识位

/**
	*	@brief  Configured TIM1 Timebase structure 
	*	@param	None
	* @retval None
	*	@remark Tout= ((arr+1)*(psc+1))/Tclk
	*					Tclk: TIM的输入时钟频率（单位：MHz）
	*					Tout: TIM溢出时间（单位：us）
	*/
void TIM1_Config(void)
{
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Prescaler=71;     //72000 000/(71+1)=1000 000Hz，T=1us
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //向上计数模式
  TIM_TimeBaseStructure.TIM_Period=49;        //1/(4999+1)=0.002Hz，即一个周期T=5ms
  TIM_TimeBaseStructure.TIM_ClockDivision=0;  //时钟不分割
//  TIM_TimeBaseStructure.TIM_RepetitionCounter=0; 
	
	NVIC_Config(0,1,TIM1_UP_IRQn,2);						//TIM1更新中断
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	TIM_ITConfig(TIM1, TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM1,ENABLE);
}

/**
	*	@brief	Use TIM1 time base to control data read and send function
	*	@param	None
	*	@retval None
	*	@remark 经过测试I2c读取一组数据需要2.5ms
	*/
void TIM1_UP_IRQHandler(void)
{   
	L_LED(1);		//测试中断执行时间
	if(TIM_GetITStatus(TIM1, TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		
		Encoder_Left=Read_Encoder(2);                                      //===读取编码器的值，因为两个电机的旋转了180度的，所以对其中一个取反，保证输出极性一致
		Encoder_Right=-Read_Encoder(3);                                      //===读取编码器的值
		Get_Angle(Way_Angle);                                               //===更新姿态		          
		Key();                                                              //===扫描按键状态 单击双击可以改变小车运行状态
		Balance_Pwm =balance(Angle_Balance,Gyro_Balance);                   //===平衡PID控制	
		Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);                  //===速度环PID控制	 记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
		Turn_Pwm    =turn(Encoder_Left,Encoder_Right,Gyro_Turn);            //===转向环PID控制     
		Moto1=Balance_Pwm-Velocity_Pwm;//+Turn_Pwm;                            //===计算左轮电机最终PWM
		Moto2=Balance_Pwm-Velocity_Pwm;//-Turn_Pwm;                            //===计算右轮电机最终PWM
		Xianfu_Pwm();                                                       //===PWM限幅
		if(Turn_Off(Angle_Balance,Voltage)==0)                              //===如果不存在异常
		Set_Pwm(Moto1,Moto2);                                               //===赋值给PWM寄存器
//		if(Flag)
	}   
	L_LED(0);	//测试中断执行时间
} 

/**************************************************************************
函数功能：直立PD控制
入口参数：角度、角速度
返回  值：直立控制PWM
**************************************************************************/
int balance(float Angle,float Gyro)
{  
	float Bias,kp=800,kd=1.8;
	int balance;
	Bias=Angle-ZHONGZHI;       //===求出平衡的角度中值 和机械相关
	balance=kp*Bias+Gyro*kd;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	return balance;
}

/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修Target_Velocity，比如，改成60就比较慢了
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
**************************************************************************/
int velocity(int encoder_left,int encoder_right)
{  
	static float Velocity,Encoder_Least,Encoder,Movement;
	static float Encoder_Integral,Target_Velocity=60;
	float kp=140,ki=0.7;
	//=============遥控前进后退部分=======================//               
	if(1==Flag.Front)    	Movement=Target_Velocity/((0==Flag.Speed)?2:1);	    //===前进标志位置1 
	else if(1==Flag.After)	Movement=-Target_Velocity/((0==Flag.Speed)?2:1);  //===后退标志位置1
	else  Movement=0;	
  //=============速度PI控制器=======================//	
	Encoder_Least =(Encoder_Left+Encoder_Right)-0;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
	Encoder *= 0.8;		                                                //===一阶低通滤波器       
	Encoder += Encoder_Least*0.2;	                                    //===一阶低通滤波器    
	Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
	Encoder_Integral=Encoder_Integral-Movement;                       //===接收遥控器数据，控制前进后退
	if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //===积分限幅
	if(Encoder_Integral<-10000)	Encoder_Integral=-10000;              //===积分限幅	
	Velocity=Encoder*kp+Encoder_Integral*ki;                          //===速度控制	
	if(Turn_Off(Angle_Balance,Voltage)==1||Flag.Stop==1)   Encoder_Integral=0; //===电机关闭后清除积分
	return Velocity;
}

/**************************************************************************
函数功能：转向控制  修改转向速度，请修改Turn_Limit即可
入口参数：左轮编码器、右轮编码器、Z轴陀螺仪
返回  值：转向控制PWM
**************************************************************************/
int turn(int encoder_left,int encoder_right,float gyro)//转向控制
{
 static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count;
	float Turn_Limit=50/((0==Flag.Speed)?2:1),Kp=30,Kd=0.8;     
	//=============遥控左右旋转部分=======================//
	if(1==Flag.Left||1==Flag.Right)   //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
	{
		if(++Turn_Count==1)
		Encoder_temp=myabs(encoder_left+encoder_right);
		Turn_Convert=50/Encoder_temp;
		if(Turn_Convert<0.6)Turn_Convert=0.6;
		if(Turn_Convert>3)Turn_Convert=3;
	}	
	else
	{
		Turn_Convert=0.9;
		Turn_Count=0;
		Encoder_temp=0;
	}			
	if(1==Flag.Left)	           Turn_Target-=Turn_Convert;
	else if(1==Flag.Right)	     Turn_Target+=Turn_Convert; 
	else Turn_Target=0;

	if(Turn_Target>Turn_Limit)  Turn_Target=Turn_Limit;    //===转向速度限幅
	if(Turn_Target<-Turn_Limit) Turn_Target=-Turn_Limit;
//	if(Flag.Front==1||Flag.After==1)  Kd=0.5;        
//	else Kd=0;   //转向的时候取消陀螺仪的纠正 有点模糊PID的思想
	//=============转向PD控制器=======================//
	Turn=Turn_Target*Kp +gyro*Kd;     //===结合Z轴陀螺仪进行PD控制
	return Turn;
}

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
	if(moto1<0)			AIN2=1,			AIN1=0;
	else 	          AIN2=0,			AIN1=1;
	PWMA=myabs(moto1);
	if(moto2<0)	BIN1=0,			BIN2=1;
	else        BIN1=1,			BIN2=0;
	PWMB=myabs(moto2);	
}

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{	
	int Limit=7100;    //===PWM满幅是7200 限制在7100
	if(Flag.Front==1)  	Moto1+=DIFFERENCE;  //DIFFERENCE是一个衡量平衡小车电机和机械安装差异的一个变量。直接作用于输出，让小车具有更好的一致性。
	if(Flag.After==1)  	Moto2-=DIFFERENCE;
	if(Moto1<-Limit) Moto1=-Limit;	
	if(Moto1>Limit)  Moto1=Limit;	
	if(Moto2<-Limit) Moto2=-Limit;	
	if(Moto2>Limit)  Moto2=Limit;		
	
}
/**************************************************************************
函数功能：按键修改小车运行状态 
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=Get_Key(); 
	if(tmp==SET_SINGLE)	Flag.Stop=!Flag.Stop;					//set键控制小车的启停
	else if(tmp==SET_LONG)														//set键长按设置滤波方式
	{
		if(2==Way_Angle)	Way_Angle=3;
		else Way_Angle=2;
	}
	else if(BN_LONG==tmp) Flag.Control=!Flag.Control;	//BN键长按设置遥控方式
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：倾角和电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off(float angle, int voltage)
{
	u8 temp;
	if(angle<-40||angle>40||1==Flag.Stop)//||voltage<1110)//===倾角大于40度关闭电机
	{	                                                 		//===Flag.Stop置1关闭电机
	temp=1;
	AIN1=0;                                            
	AIN2=0;
	BIN1=0;
	BIN2=0;
	}
	else
	temp=0;
	return temp;			
}
	
/**************************************************************************
函数功能：获取角度 三种算法经过我们的调校，都非常理想 
入口参数：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/
void Get_Angle(u8 way)
{ 
	float Accel_Y,Accel_X,Accel_Z,Gyro_X,Gyro_Z;
	Temperature=Read_Temperature();    //===读取MPU6050内置温度传感器数据，近似表示主板温度。
	if(way==1)                         //===DMP的读取在数据采集中断提醒的时候，严格遵循时序要求
	{	
		Read_DMP();                      //===读取加速度、角速度、倾角
		Angle_Balance=Pitch;             //===更新平衡倾角
		Gyro_Balance=gyro[1];            //===更新平衡角速度
		Gyro_Turn=gyro[2];               //===更新转向角速度
		Acceleration_Z=accel[2];         //===更新Z轴加速度计
	}			
	else
	{
		Gyro_X=I2C_Read_16(MPU6050_RA_GYRO_XOUT_H);   //读取Y轴陀螺仪
		Gyro_Z=I2C_Read_16(MPU6050_RA_GYRO_ZOUT_H);   //读取Z轴陀螺仪
		Accel_Y=I2C_Read_16(MPU6050_RA_ACCEL_YOUT_H); //读取X轴加速度计
		Accel_Z=I2C_Read_16(MPU6050_RA_ACCEL_ZOUT_H); //读取Z轴加速度计
		if(Gyro_X>32768)  Gyro_X-=65536;                       //数据类型转换  也可通过short强制类型转换
		if(Gyro_Z>32768)  Gyro_Z-=65536;                       //数据类型转换
		if(Accel_Y>32768) Accel_Y-=65536;                      //数据类型转换
		if(Accel_Z>32768) Accel_Z-=65536;                      //数据类型转换
		Gyro_Balance=-Gyro_X;                                  //更新平衡角速度
		Accel_X=atan2(-Accel_Y,Accel_Z)*180/PI;                //计算倾角	
		Gyro_X=Gyro_X/16.4;                                    //陀螺仪量程转换	
		if(Way_Angle==2)		  	Kalman_Filter(Accel_X,-Gyro_X);//卡尔曼滤波	
		else if(Way_Angle==3)   Yijielvbo(Accel_X,-Gyro_X);    //互补滤波
		Angle_Balance=angle;                                   //更新平衡倾角
		pitch=Accel_X;
		Gyro_Turn=Gyro_Z;                                      //更新转向角速度
		Acceleration_Z=Accel_Z;                                //===更新Z轴加速度计	
	}
}
/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	int temp;
	if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}



