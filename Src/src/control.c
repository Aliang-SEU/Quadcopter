#include "control.h"
#include "rc.h"
#include "mpu9250.h"
#include "pwm.h"
#include "math.h"

#define Gyro_G 	0.0610351f	  		//���ٶȱ�ɶ�
#define RtA 		57.324841	

PID PID_ROL_OUTER,PID_PIT_OUTER,PID_YAW_OUTER;
PID PID_ROL_INNER,PID_PIT_INNER,PID_YAW_INNER;

uint8_t ARMED = 1;		//������־	

int16_t MOTO1_PWM = 0;
int16_t MOTO2_PWM = 0;
int16_t MOTO3_PWM = 0;
int16_t MOTO4_PWM = 0;

struct _target Target;	//Ŀ����̬

//X�ַ���ģʽ
//˫�ջ�PID �⻷�Ƕ� �ڻ����ٶ� �⻷��PID���� �ڻ�ʹ��PD����
//�������Ϊ��̬ŷ����
void CONTROL(float roll, float pitch, float yaw)
{
	uint16_t moto1=0,moto2=0,moto3=0,moto4=0;
	static float roll_old,pitch_old,yaw_old;
	static uint8_t cnt=0;	//���Լ���
	float deviation_pitch,deviation_roll,deviation_yaw;
	
	static int16_t Last_GYRO_X=0,Last_GYRO_Y=0,Last_GYRO_Z=0;
	static float Outer_Pitch_Integer=0,Outer_Roll_Integer=0;	//�⻷����
	
	//�ڻ����Ƶ�Ƶ�����⻷��2�� 
	//�⻷����
	if(cnt >= 2)
	{
		  //****pitch��*************************
			deviation_pitch = Target.Pitch - pitch;
			Outer_Pitch_Integer += deviation_pitch;
		
			//�����޷�
			if(Outer_Pitch_Integer > PID_PIT_OUTER.IMAX)  Outer_Pitch_Integer = PID_PIT_OUTER.IMAX;
	    else if(Outer_Pitch_Integer < -PID_PIT_OUTER.IMAX)  Outer_Pitch_Integer = -PID_PIT_OUTER.IMAX;
			
			PID_PIT_OUTER.OUT = PID_PIT_OUTER.P * deviation_pitch + PID_PIT_OUTER.I * Outer_Pitch_Integer + PID_PIT_OUTER.D * (deviation_pitch - pitch_old);
			pitch_old = deviation_pitch;
			
			//****roll��*************************
			deviation_roll = Target.Roll - roll;
			Outer_Roll_Integer += deviation_roll;
			
			if(Outer_Roll_Integer > PID_ROL_OUTER.IMAX) Outer_Roll_Integer = PID_ROL_OUTER.IMAX;
			if(Outer_Roll_Integer < -PID_ROL_OUTER.IMAX) Outer_Roll_Integer = -PID_ROL_OUTER.IMAX;
			
			PID_ROL_OUTER.OUT = PID_ROL_OUTER.P * deviation_roll + PID_ROL_OUTER.I * Outer_Roll_Integer + PID_ROL_OUTER.D * (deviation_roll - roll_old);
			roll_old = deviation_roll;
		
			//****yaw��****************************
			if((Target.Yaw - yaw)>180 || (Target.Yaw - Target.Yaw)<-180)
			{
				if(Target.Yaw>0 && yaw<0)  deviation_yaw= (-180 - yaw) +(Target.Yaw - 180);
				if(Target.Yaw<0 && yaw>0)  deviation_yaw= (180 - yaw) +(Target.Yaw + 180);
			}
			else  deviation_yaw = Target.Yaw - yaw;
			
			PID_YAW_OUTER.OUT = PID_YAW_OUTER.P * deviation_yaw + PID_YAW_OUTER.D * (deviation_yaw - yaw_old);
			yaw_old = deviation_yaw;
			
		cnt=0;
	}
	cnt++;
	//�ڻ����ٶ�PD����
		//*******rol�����*************************************
		//����������ֵ
		PID_ROL_INNER.pout = PID_ROL_INNER.P * (PID_ROL_OUTER.OUT-MPU9250_GYRO_LAST.X*Gyro_G);
		//����΢�����ֵ
		PID_ROL_INNER.dout = PID_ROL_INNER.D * (Last_GYRO_X - MPU9250_GYRO_LAST.X);

		//*******pit�����*************************************
		//����������ֵ
		PID_PIT_INNER.pout = PID_PIT_INNER.P * (PID_PIT_OUTER.OUT-MPU9250_GYRO_LAST.Y*Gyro_G);
		//����΢�����ֵ
		PID_PIT_INNER.dout = PID_PIT_INNER.D * (Last_GYRO_Y - MPU9250_GYRO_LAST.Y);
	
		//*******yaw�����****************************************
		//����������ֵ
		PID_YAW_INNER.pout = PID_YAW_INNER.P * (PID_YAW_INNER.OUT-MPU9250_GYRO_LAST.Z*Gyro_G);
		//����΢�����ֵ
		PID_YAW_INNER.dout = PID_YAW_INNER.D * (Last_GYRO_Z - MPU9250_GYRO_LAST.Z);
	
		PID_ROL_INNER.OUT = PID_ROL_INNER.pout + PID_ROL_INNER.iout;
		PID_PIT_INNER.OUT = PID_PIT_INNER.pout + PID_PIT_INNER.iout;
		PID_YAW_INNER.OUT = PID_YAW_INNER.pout + PID_YAW_INNER.iout;
		
		//������ʷֵ
		Last_GYRO_X = MPU9250_GYRO_LAST.X;
		Last_GYRO_Y = MPU9250_GYRO_LAST.Y;
		Last_GYRO_Z = MPU9250_GYRO_LAST.Z;
		
		//���ſ���
		if(Rc_Get.THROTTLE>RC_MINCHECK)
		{
			int16_t date_throttle	= (Rc_Get.THROTTLE)/cos(roll/RtA)/cos(pitch/RtA);
			moto1 = date_throttle - PID_ROL_INNER.OUT - PID_PIT_INNER.OUT + PID_YAW_INNER.OUT;
			moto2 = date_throttle + PID_ROL_INNER.OUT - PID_PIT_INNER.OUT - PID_YAW_INNER.OUT;
			moto3 = date_throttle + PID_ROL_INNER.OUT + PID_PIT_INNER.OUT + PID_YAW_INNER.OUT;
			moto4 = date_throttle + PID_ROL_INNER.OUT - PID_PIT_INNER.OUT - PID_YAW_INNER.OUT;
				
			Moto_PwmRflash(moto1,moto2,moto3,moto4);
		}
		else
		{
			moto1 = 0;
			moto2 = 0;
			moto3 = 0;
			moto4 = 0;
			Outer_Roll_Integer = 0;
			Outer_Pitch_Integer = 0;
			Moto_PwmRflash(1000,1000,1000,1000);
		}
}
