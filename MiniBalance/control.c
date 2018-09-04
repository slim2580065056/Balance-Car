#include "control.h"	
#include "filter.h"
#include "usart.h"

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
u8 Flag_Target;
Bit_Flag Flag={0,0,0,0,0,1,0,0};	//sys.h�ļ��е�λ�����ڶ������б�ʶλ

/**
	*	@brief  Configured TIM1 Timebase structure 
	*	@param	None
	* @retval None
	*	@remark Tout= ((arr+1)*(psc+1))/Tclk
	*					Tclk: TIM������ʱ��Ƶ�ʣ���λ��MHz��
	*					Tout: TIM���ʱ�䣨��λ��us��
	*/
void TIM1_Config(void)
{
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Prescaler=71;     //72000 000/(71+1)=1000 000Hz��T=1us
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //���ϼ���ģʽ
  TIM_TimeBaseStructure.TIM_Period=49;        //1/(4999+1)=0.002Hz����һ������T=5ms
  TIM_TimeBaseStructure.TIM_ClockDivision=0;  //ʱ�Ӳ��ָ�
//  TIM_TimeBaseStructure.TIM_RepetitionCounter=0; 
	
	NVIC_Config(0,1,TIM1_UP_IRQn,2);						//TIM1�����ж�
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	TIM_ITConfig(TIM1, TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM1,ENABLE);
}

/**
	*	@brief	Use TIM1 time base to control data read and send function
	*	@param	None
	*	@retval None
	*	@remark ��������I2c��ȡһ��������Ҫ2.5ms
	*/
void TIM1_UP_IRQHandler(void)
{   
	L_LED(1);		//�����ж�ִ��ʱ��
	if(TIM_GetITStatus(TIM1, TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		
		Encoder_Left=Read_Encoder(2);                                      //===��ȡ��������ֵ����Ϊ�����������ת��180�ȵģ����Զ�����һ��ȡ������֤�������һ��
		Encoder_Right=-Read_Encoder(3);                                      //===��ȡ��������ֵ
		Get_Angle(Way_Angle);                                               //===������̬		          
		Key();                                                              //===ɨ�谴��״̬ ����˫�����Ըı�С������״̬
		Balance_Pwm =balance(Angle_Balance,Gyro_Balance);                   //===ƽ��PID����	
		Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);                  //===�ٶȻ�PID����	 ��ס���ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��
		Turn_Pwm    =turn(Encoder_Left,Encoder_Right,Gyro_Turn);            //===ת��PID����     
		Moto1=Balance_Pwm-Velocity_Pwm;//+Turn_Pwm;                            //===�������ֵ������PWM
		Moto2=Balance_Pwm-Velocity_Pwm;//-Turn_Pwm;                            //===�������ֵ������PWM
		Xianfu_Pwm();                                                       //===PWM�޷�
		if(Turn_Off(Angle_Balance,Voltage)==0)                              //===����������쳣
		Set_Pwm(Moto1,Moto2);                                               //===��ֵ��PWM�Ĵ���
//		if(Flag)
	}   
	L_LED(0);	//�����ж�ִ��ʱ��
} 

/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ����ٶ�
����  ֵ��ֱ������PWM
**************************************************************************/
int balance(float Angle,float Gyro)
{  
	float Bias,kp=800,kd=1.8;
	int balance;
	Bias=Angle-ZHONGZHI;       //===���ƽ��ĽǶ���ֵ �ͻ�е���
	balance=kp*Bias+Gyro*kd;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	return balance;
}

/**************************************************************************
�������ܣ��ٶ�PI���� �޸�ǰ�������ٶȣ�����Target_Velocity�����磬�ĳ�60�ͱȽ�����
��ڲ��������ֱ����������ֱ�����
����  ֵ���ٶȿ���PWM
**************************************************************************/
int velocity(int encoder_left,int encoder_right)
{  
	static float Velocity,Encoder_Least,Encoder,Movement;
	static float Encoder_Integral,Target_Velocity=60;
	float kp=140,ki=0.7;
	//=============ң��ǰ�����˲���=======================//               
	if(1==Flag.Front)    	Movement=Target_Velocity/((0==Flag.Speed)?2:1);	    //===ǰ����־λ��1 
	else if(1==Flag.After)	Movement=-Target_Velocity/((0==Flag.Speed)?2:1);  //===���˱�־λ��1
	else  Movement=0;	
  //=============�ٶ�PI������=======================//	
	Encoder_Least =(Encoder_Left+Encoder_Right)-0;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
	Encoder *= 0.8;		                                                //===һ�׵�ͨ�˲���       
	Encoder += Encoder_Least*0.2;	                                    //===һ�׵�ͨ�˲���    
	Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
	Encoder_Integral=Encoder_Integral-Movement;                       //===����ң�������ݣ�����ǰ������
	if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //===�����޷�
	if(Encoder_Integral<-10000)	Encoder_Integral=-10000;              //===�����޷�	
	Velocity=Encoder*kp+Encoder_Integral*ki;                          //===�ٶȿ���	
	if(Turn_Off(Angle_Balance,Voltage)==1||Flag.Stop==1)   Encoder_Integral=0; //===����رպ��������
	return Velocity;
}

/**************************************************************************
�������ܣ�ת�����  �޸�ת���ٶȣ����޸�Turn_Limit����
��ڲ��������ֱ����������ֱ�������Z��������
����  ֵ��ת�����PWM
**************************************************************************/
int turn(int encoder_left,int encoder_right,float gyro)//ת�����
{
 static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count;
	float Turn_Limit=50/((0==Flag.Speed)?2:1),Kp=30,Kd=0.8;     
	//=============ң��������ת����=======================//
	if(1==Flag.Left||1==Flag.Right)   //��һ������Ҫ�Ǹ�����תǰ���ٶȵ����ٶȵ���ʼ�ٶȣ�����С������Ӧ��
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

	if(Turn_Target>Turn_Limit)  Turn_Target=Turn_Limit;    //===ת���ٶ��޷�
	if(Turn_Target<-Turn_Limit) Turn_Target=-Turn_Limit;
//	if(Flag.Front==1||Flag.After==1)  Kd=0.5;        
//	else Kd=0;   //ת���ʱ��ȡ�������ǵľ��� �е�ģ��PID��˼��
	//=============ת��PD������=======================//
	Turn=Turn_Target*Kp +gyro*Kd;     //===���Z�������ǽ���PD����
	return Turn;
}

/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
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
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(void)
{	
	int Limit=7100;    //===PWM������7200 ������7100
	if(Flag.Front==1)  	Moto1+=DIFFERENCE;  //DIFFERENCE��һ������ƽ��С������ͻ�е��װ�����һ��������ֱ���������������С�����и��õ�һ���ԡ�
	if(Flag.After==1)  	Moto2-=DIFFERENCE;
	if(Moto1<-Limit) Moto1=-Limit;	
	if(Moto1>Limit)  Moto1=Limit;	
	if(Moto2<-Limit) Moto2=-Limit;	
	if(Moto2>Limit)  Moto2=Limit;		
	
}
/**************************************************************************
�������ܣ������޸�С������״̬ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=Get_Key(); 
	if(tmp==SET_SINGLE)	Flag.Stop=!Flag.Stop;					//set������С������ͣ
	else if(tmp==SET_LONG)														//set�����������˲���ʽ
	{
		if(2==Way_Angle)	Way_Angle=3;
		else Way_Angle=2;
	}
	else if(BN_LONG==tmp) Flag.Control=!Flag.Control;	//BN����������ң�ط�ʽ
}

/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������Ǻ͵�ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off(float angle, int voltage)
{
	u8 temp;
	if(angle<-40||angle>40||1==Flag.Stop)//||voltage<1110)//===��Ǵ���40�ȹرյ��
	{	                                                 		//===Flag.Stop��1�رյ��
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
�������ܣ���ȡ�Ƕ� �����㷨�������ǵĵ�У�����ǳ����� 
��ڲ�������ȡ�Ƕȵ��㷨 1��DMP  2�������� 3�������˲�
����  ֵ����
**************************************************************************/
void Get_Angle(u8 way)
{ 
	float Accel_Y,Accel_X,Accel_Z,Gyro_X,Gyro_Z;
	Temperature=Read_Temperature();    //===��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶ȡ�
	if(way==1)                         //===DMP�Ķ�ȡ�����ݲɼ��ж����ѵ�ʱ���ϸ���ѭʱ��Ҫ��
	{	
		Read_DMP();                      //===��ȡ���ٶȡ����ٶȡ����
		Angle_Balance=Pitch;             //===����ƽ�����
		Gyro_Balance=gyro[1];            //===����ƽ����ٶ�
		Gyro_Turn=gyro[2];               //===����ת����ٶ�
		Acceleration_Z=accel[2];         //===����Z����ٶȼ�
	}			
	else
	{
		Gyro_X=I2C_Read_16(MPU6050_RA_GYRO_XOUT_H);   //��ȡY��������
		Gyro_Z=I2C_Read_16(MPU6050_RA_GYRO_ZOUT_H);   //��ȡZ��������
		Accel_Y=I2C_Read_16(MPU6050_RA_ACCEL_YOUT_H); //��ȡX����ٶȼ�
		Accel_Z=I2C_Read_16(MPU6050_RA_ACCEL_ZOUT_H); //��ȡZ����ٶȼ�
		if(Gyro_X>32768)  Gyro_X-=65536;                       //��������ת��  Ҳ��ͨ��shortǿ������ת��
		if(Gyro_Z>32768)  Gyro_Z-=65536;                       //��������ת��
		if(Accel_Y>32768) Accel_Y-=65536;                      //��������ת��
		if(Accel_Z>32768) Accel_Z-=65536;                      //��������ת��
		Gyro_Balance=-Gyro_X;                                  //����ƽ����ٶ�
		Accel_X=atan2(-Accel_Y,Accel_Z)*180/PI;                //�������	
		Gyro_X=Gyro_X/16.4;                                    //����������ת��	
		if(Way_Angle==2)		  	Kalman_Filter(Accel_X,-Gyro_X);//�������˲�	
		else if(Way_Angle==3)   Yijielvbo(Accel_X,-Gyro_X);    //�����˲�
		Angle_Balance=angle;                                   //����ƽ�����
		pitch=Accel_X;
		Gyro_Turn=Gyro_Z;                                      //����ת����ٶ�
		Acceleration_Z=Accel_Z;                                //===����Z����ٶȼ�	
	}
}
/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	int temp;
	if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}



