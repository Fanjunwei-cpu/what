#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "lcd.h"
#include	"math.h"
#include "touch.h"
#include "key.h"

struct
{
	float	SetSpeed;
	float	ActualSpeed;
	float	err;
	float	last_err;
	float	Kp,Ki,Kd;
	float	integral;
	float	Voltage;
	float	umax;
	float	umin;
}pid;
void	pid_init(void)
{
	pid.SetSpeed=255;
	pid.ActualSpeed=0.0;
	pid.err=0.0;
	pid.last_err=0.0;
	pid.Voltage=0.0;
	pid.integral=0.0;
	pid.Kd=4.1;
	pid.Ki=3.3;
	pid.Kp=15.3;	//11.8
	pid.umax=70;
	pid.umin=10;
}
u16	value;
u8	N=1;
void	pid_realize(void)
{
	int index;
//	pid.SetSpeed=speed;
	pid.err =pid.SetSpeed-pid.ActualSpeed;
//	if(pid.ActualSpeed==30)
//	{
//		N=0;
//	}
//	else{N=1;}
	if(pid.ActualSpeed>pid.umax)
	{
		if(pid.err>40&&pid.err<20){index=0;}
		else	
		{
			index=1;
			if(pid.err<0)
			{
				pid.integral+=pid.err;
			}
		}	
	}
	else	if(pid.ActualSpeed<pid.umin)
	{
		if(pid.err>40 && pid.err<20){index=0;}	//abs(pid.err)>80
		else	
		{
			index=1;
			if(pid.err>0)
			{
				pid.integral+=pid.err;
			}
		}	
	}
	else
	{
		if(pid.err>40&&pid.err<20){index=0;}
		else	
		{
			index=1;
			pid.integral+=pid.err;
		}	
	}
	pid.Voltage=pid.Kp*pid.err+index*pid.Ki*pid.integral+pid.Kd*(pid.last_err-pid.err);

//	pid.last_err=pid.next_err;
	pid.last_err=pid.err;
	
}
u8 touch_task(void)
{
		tp_dev. scan(0);
		if(tp_dev.sta&TP_PRES_DOWN)			//触摸屏被按下
		{
			tp_dev. scan(0);
			delay_ms(10);
			if((tp_dev.x[0]>=1&&tp_dev.x[0]<=50)&&(tp_dev.y[0]<=160&&tp_dev.y[0]>=120))return	1;  //y240 200
			if((tp_dev.x[0]>=70&&tp_dev.x[0]<=120)&&(tp_dev.y[0]<=160&&tp_dev.y[0]>=120))return	2;   //y240 200
			if((tp_dev.x[0]>=140&&tp_dev.x[0]<=190)&&(tp_dev.y[0]<=160&&tp_dev.y[0]>=120))return	3;  //y240 200
			if((tp_dev.x[0]>=1&&tp_dev.x[0]<=50)&&(tp_dev.y[0]<=200&&tp_dev.y[0]>=160))return	4;   //300 260
			if((tp_dev.x[0]>=70&&tp_dev.x[0]<=120)&&(tp_dev.y[0]<=200&&tp_dev.y[0]>=160))return	5;
			if((tp_dev.x[0]>=140&&tp_dev.x[0]<=190)&&(tp_dev.y[0]<=200&&tp_dev.y[0]>=160))return	6;
		}
		delay_ms(10);
}
int main(void)
{    
	u8 key,pre;
	float temp;
	u8	M=0;
	u8 flag=0,flag_1=0;;
	u8 count=0;		
	u32	TIM5CH1_CAPTURE_VAL_FALL;	//输入捕获值(TIM2/TIM5是32位)
	u32	TIM5CH1_CAPTURE_VAL_Rise;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	uart_init(115200);//初始化串口波特率为115200
	LCD_Init();
	KEY_Init();
	tp_dev.init();
	//TIM5_CH1_Cap_Init(100000,84-1);
	TIM14_PWM_Init(500-1,84-1);	//84M/84=1Mhz的计数频率,重装载值500，所以PWM频率为 1M/500=2Khz.    
	pid_init(); 
	LCD_ShowString(35,40,200,16,16,"Kp:");
	LCD_ShowString(35,60,200,16,16,"Ki:");
	LCD_ShowString(35,80,200,16,16,"Kd:");
	LCD_ShowString(35,100,200,16,16,"set_speed:");
	LCD_ShowString(35,120,200,16,16,"speed:");
	LCD_ShowString(35,210,200,16,16,"Kd-");
	LCD_ShowString(105,210,200,16,16,"Ki-");
	LCD_ShowString(175,210,200,16,16,"Kp-");
	LCD_ShowString(35,270,200,16,16,"Kd+");
	LCD_ShowString(105,270,200,16,16,"Ki+");
	LCD_ShowString(175,270,200,16,16,"Kp+");
	LCD_ShowString(155,100,200,16,16,"r/s");
	LCD_DrawRectangle(20,200,70,240);
	LCD_DrawRectangle(90,200,140,240);
	LCD_DrawRectangle(160,200,210,240);
	LCD_DrawRectangle(20,260,70,300);
	LCD_DrawRectangle(90,260,140,300);
	LCD_DrawRectangle(160,260,210,300);
	POINT_COLOR = RED;
	LCD_PutString(40,10,"直流电机调速系统",16);
	POINT_COLOR = BLACK;
//	delay_ms(2000);
   while(1) //实现比较值从0-300递增，到300后从300-0递减，循环
	{
		pre=KEY_Scan(0);
		switch(pre)
		{
			case	1:pid.SetSpeed+=1;break;
			case	4:pid.SetSpeed-=1;break;
		}
		key=touch_task();
		switch(key)
		{
			case	1:pid.Kd+=0.1f;break;
			case	2:pid.Ki+=0.1f;break;
			case	3:pid.Kp+=0.1f;break;
			case	4:pid.Kd-=0.1f;break;
			case	5:pid.Ki-=0.1f;break;
			case	6:pid.Kp-=0.1f;break;
		}
		LCD_ShowFloatNum(75,40,pid.Kp,1,16,0x80);
		LCD_ShowFloatNum(75,60,pid.Ki,1,16,0x80);
		LCD_ShowFloatNum(75,80,pid.Kd,1,16,0x80);
		LCD_ShowFloatNum(115,100,pid.SetSpeed,1,16,0x80);
		TIM_SetCompare1(TIM14,255-pid.SetSpeed);	//修改比较值，修改占空比
		
//		if(flag==1||flag_1==1)
//		{
//			if(flag==1)
//			{
//				temp=(1000000)/(count*100000+TIM5CH1_CAPTURE_VAL_Rise);
//				if(temp<100)
//				{pid.ActualSpeed=temp;}
////				if(temp<60)
//				//	LCD_ShowNum(90,200,count,2,16);
//				//	LCD_ShowNum(90,180,TIM5CH1_CAPTURE_VAL_Rise,6,16);
//					LCD_Fill(90,120,240,136,WHITE);
////.						LCD_ShowFloatNum(90,120,(float)(1000000)/(count*100000+TIM5CH1_CAPTURE_VAL_Rise),2,16,0);
//				
//			}
//			if(flag_1==1)
//			{
//				pid.ActualSpeed=0;
//				LCD_Fill(90,120,240,136,WHITE);
//				LCD_ShowFloatNum(90,120,(float)0,2,16,0);
//			}
//		pid_realize();
////			LCD_ShowNum(40,140,count,4,16);
//			TIM5CH1_CAPTURE_VAL_Rise=0;
//			count=0;
//			flag=0;
//			flag_1=0;
//			if(N==0)
//			{
//				TIM_SetCompare1(TIM14,700+value);
//			}
//			else
//			{
//				TIM_SetCompare1(TIM14,700+pid.Voltage);
//				value=pid.Voltage;
//			}
//		}
	}
}
