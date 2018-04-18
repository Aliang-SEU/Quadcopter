#include "stm32f10x.h"
#include "math.h"
#include "mpu9250.h"

typedef struct data
{
	float x;
	float y;
	float z;
}Data;

typedef struct zitai
{
	float yaw;
	float roll;
	float pitch;
}ZiTai;

extern S_INT16_XYZ Low_Pass(void);
extern S_INT16_XYZ Accel,Gyro;
extern ZiTai Q_ANGLE;			//四元数计算出的角度
extern void Prepare_Data(void);	
extern void Get_Attitude(void);
extern void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
