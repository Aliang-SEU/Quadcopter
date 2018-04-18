#include "uart_communication.h"
#include "Attitude_Calc.h"
#include "math.h"
#include "mpu9250.h"
#include "uart1.h"
#include "rc.h"
#include "control.h"

uint8_t Data_to_Receive[32];//接收到的数据
uint8_t Data_to_Send[32];//需要发送的数据

//发送传感器的状态
void UART_Send_Status(void)
{
	uint8_t _cnt=0;
	uint8_t sum=0;
	uint8_t i=0;
	int16_t _temp=0;
	//帧头、功能字、长度
	Data_to_Send[_cnt++]=0xaa;
	Data_to_Send[_cnt++]=0xaa;
	Data_to_Send[_cnt++]=0x01;
	Data_to_Send[_cnt++]=0x00;
	
	//飞控姿态
	_temp = (int16_t)(Q_ANGLE.roll*100); //roll
	Data_to_Send[_cnt++]=BYTE1(_temp);
	Data_to_Send[_cnt++]=BYTE0(_temp);
	
	_temp = (int16_t)(Q_ANGLE.pitch*100);//pit
	Data_to_Send[_cnt++]=BYTE1(_temp);
	Data_to_Send[_cnt++]=BYTE0(_temp);
 	
	_temp = (int16_t)(Q_ANGLE.yaw*100);//YAW
 	Data_to_Send[_cnt++]=BYTE1(_temp);
 	Data_to_Send[_cnt++]=BYTE0(_temp);
	 //超声波高度
	Data_to_Send[_cnt++]=0;
 	Data_to_Send[_cnt++]=0;
	//气压计高度数据
	Data_to_Send[_cnt++]=0;
 	Data_to_Send[_cnt++]=0;
	Data_to_Send[_cnt++]=0;
 	Data_to_Send[_cnt++]=0;

 	Data_to_Send[3]=_cnt-4;
	for(i=0;i<_cnt;i++)
		sum += Data_to_Send[i];
	Data_to_Send[_cnt++]=sum;
	UART1_Send(Data_to_Send,_cnt);
}
void UART_Send_Sensor(void)
{
	uint8_t _cnt=0;
	uint8_t i=0;
	uint8_t sum = 0;
	
	Data_to_Send[_cnt++]=0xaa;
	Data_to_Send[_cnt++]=0xaa;
	Data_to_Send[_cnt++]=0x02;
	Data_to_Send[_cnt++]=0x12;
	Data_to_Send[_cnt++]=BYTE1(Accel.X);
	Data_to_Send[_cnt++]=BYTE0(Accel.X);
	Data_to_Send[_cnt++]=BYTE1(Accel.Y);
	Data_to_Send[_cnt++]=BYTE0(Accel.Y);
	Data_to_Send[_cnt++]=BYTE1(Accel.Z);
	Data_to_Send[_cnt++]=BYTE0(Accel.Z);
	Data_to_Send[_cnt++]=BYTE1(MPU9250_GYRO_LAST.X);
	Data_to_Send[_cnt++]=BYTE0(MPU9250_GYRO_LAST.X);
	Data_to_Send[_cnt++]=BYTE1(MPU9250_GYRO_LAST.Y);
	Data_to_Send[_cnt++]=BYTE0(MPU9250_GYRO_LAST.Y);
	Data_to_Send[_cnt++]=BYTE1(MPU9250_GYRO_LAST.Z);
	Data_to_Send[_cnt++]=BYTE0(MPU9250_GYRO_LAST.Z);
	Data_to_Send[_cnt++]=BYTE1(MPU9250_MAG_LAST.X);
	Data_to_Send[_cnt++]=BYTE0(MPU9250_MAG_LAST.X);
	Data_to_Send[_cnt++]=BYTE1(MPU9250_MAG_LAST.Y);
	Data_to_Send[_cnt++]=BYTE0(MPU9250_MAG_LAST.Y);
	Data_to_Send[_cnt++]=BYTE1(MPU9250_MAG_LAST.Z);
	Data_to_Send[_cnt++]=BYTE0(MPU9250_MAG_LAST.Z);
	
	for(i=0;i<_cnt;i++)
		sum += Data_to_Send[i];
	Data_to_Send[_cnt++]=sum;
	UART1_Send(Data_to_Send,_cnt);
}
//发送遥控器数据
void UART_Send_RCDATA()
{
	uint8_t _cnt=0;
	uint8_t i=0;
	uint8_t sum = 0;
	
	Data_to_Send[_cnt++]=0xaa;
	Data_to_Send[_cnt++]=0xaa;
	Data_to_Send[_cnt++]=0x03;
	Data_to_Send[_cnt++]=0x00;
	Data_to_Send[_cnt++]=BYTE1(Rc_Get.THROTTLE);
	Data_to_Send[_cnt++]=BYTE0(Rc_Get.THROTTLE);
	Data_to_Send[_cnt++]=BYTE1(Rc_Get.YAW);
	Data_to_Send[_cnt++]=BYTE0(Rc_Get.YAW);
	Data_to_Send[_cnt++]=BYTE1(Rc_Get.ROLL);
	Data_to_Send[_cnt++]=BYTE0(Rc_Get.ROLL);
	Data_to_Send[_cnt++]=BYTE1(Rc_Get.PITCH);
	Data_to_Send[_cnt++]=BYTE0(Rc_Get.PITCH);
	Data_to_Send[_cnt++]=BYTE1(Rc_Get.AUX1);
	Data_to_Send[_cnt++]=BYTE0(Rc_Get.AUX1);
	Data_to_Send[_cnt++]=BYTE1(Rc_Get.AUX2);
	Data_to_Send[_cnt++]=BYTE0(Rc_Get.AUX2);
	Data_to_Send[_cnt++]=BYTE1(Rc_Get.AUX3);
	Data_to_Send[_cnt++]=BYTE0(Rc_Get.AUX3);
	Data_to_Send[_cnt++]=BYTE1(Rc_Get.AUX4);
	Data_to_Send[_cnt++]=BYTE0(Rc_Get.AUX4);
	Data_to_Send[_cnt++]=BYTE1(Rc_Get.AUX5);
	Data_to_Send[_cnt++]=BYTE0(Rc_Get.AUX5);
	Data_to_Send[_cnt++]=0;
	
	Data_to_Send[3]=_cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += Data_to_Send[i];
	Data_to_Send[_cnt++]=sum;
	UART1_Send(Data_to_Send,_cnt);
}
//发送油门数据数据
void UART_Send_MOTO()
{
	uint8_t _cnt=0;
	uint8_t i=0;
	uint8_t sum = 0;

	Data_to_Send[_cnt++]=0xaa;
	Data_to_Send[_cnt++]=0xaa;
	Data_to_Send[_cnt++]=0x06;
	Data_to_Send[_cnt++]=0x00;
	Data_to_Send[_cnt++]=BYTE1(MOTO1_PWM);
	Data_to_Send[_cnt++]=BYTE0(MOTO1_PWM);
	Data_to_Send[_cnt++]=BYTE1(MOTO1_PWM);
	Data_to_Send[_cnt++]=BYTE0(MOTO1_PWM);
	Data_to_Send[_cnt++]=BYTE1(MOTO1_PWM);
	Data_to_Send[_cnt++]=BYTE0(MOTO1_PWM);
	Data_to_Send[_cnt++]=BYTE1(MOTO1_PWM);
	Data_to_Send[_cnt++]=BYTE0(MOTO1_PWM);

	Data_to_Send[3]=_cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += Data_to_Send[i];
	Data_to_Send[_cnt++]=sum;
	UART1_Send(Data_to_Send,_cnt);
}
//发送PID参数到上位机
void UART_Send_PID()
{
	uint8_t _cnt=0;
	uint8_t i=0;
	uint8_t sum = 0;
	int16_t temp=0;
	
	Data_to_Send[_cnt++]=0xaa;
	Data_to_Send[_cnt++]=0xaa;
	Data_to_Send[_cnt++]=0x10;
	Data_to_Send[_cnt++]=0x00;

	temp = (int16_t)(PID_ROL_INNER.P*100);
	Data_to_Send[_cnt++]=BYTE1(temp);
	Data_to_Send[_cnt++]=BYTE0(temp);
	temp = (int16_t)(PID_ROL_INNER.I*1000);
	Data_to_Send[_cnt++]=BYTE1(temp);
	Data_to_Send[_cnt++]=BYTE0(temp);
	temp = (int16_t)(PID_ROL_INNER.D*100);
	Data_to_Send[_cnt++]=BYTE1(temp);
	Data_to_Send[_cnt++]=BYTE0(temp);
	temp = (int16_t)(PID_PIT_INNER.P*100);
	Data_to_Send[_cnt++]=BYTE1(temp);
	Data_to_Send[_cnt++]=BYTE0(temp);
	temp = (int16_t)(PID_PIT_INNER.I*1000);
	Data_to_Send[_cnt++]=BYTE1(temp);
	Data_to_Send[_cnt++]=BYTE0(temp);
	temp = (int16_t)(PID_PIT_INNER.D*100);
	Data_to_Send[_cnt++]=BYTE1(temp);
	Data_to_Send[_cnt++]=BYTE0(temp);
	temp = (int16_t)(PID_YAW_INNER.P*100);
	Data_to_Send[_cnt++]=BYTE1(temp);
	Data_to_Send[_cnt++]=BYTE0(temp);
	temp = (int16_t)(PID_YAW_INNER.I*1000);
	Data_to_Send[_cnt++]=BYTE1(temp);
	Data_to_Send[_cnt++]=BYTE0(temp);
	temp = (int16_t)(PID_YAW_INNER.D*100);
	Data_to_Send[_cnt++]=BYTE1(temp);
	Data_to_Send[_cnt++]=BYTE0(temp);

	Data_to_Send[3]=_cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += Data_to_Send[i];
	Data_to_Send[_cnt++]=sum;
	UART1_Send(Data_to_Send,_cnt);
}

