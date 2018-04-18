/****************************************
						极光V1.0 姿态解算(互补滤波+四元数算法)
						修改时间2015/07/29
***************************************/
#include "Attitude_Calc.h"
#include "math.h"
#include "uart1.h"
#include "oled.h"
#include "MPU9250.h"

//四元数计算
#define RtA 		57.324841f				//弧度到角度
#define AtR    	0.0174533f				//度到角度
#define Acc_G 	0.0011963f   			//加速度变成G 4g
#define Gyro_G 	0.0610351f	  		//角速度变成度
#define Gyro_Gr	0.0010653f		  	//2000	
#define FILTER_NUM 8

S_INT16_XYZ ACC_AVG;			//平均值滤波后的ACC
S_INT16_XYZ Accel,Gyro;
Data Gyr;
Data GYRO_I;				//陀螺仪积分
Data EXP_ANGLE;		//期望角度
Data DIF_ANGLE;		//期望角度与实际角度差
ZiTai Q_ANGLE;			//四元数计算出的角度

int16_t	ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];	//加速度滑动窗口滤波数组

void Prepare_Data(void)	
{
	static uint8_t filter_cnt=0;
	int32_t temp1=0,temp2=0,temp3=0;
	uint8_t i;
	
	ACC_X_BUF[filter_cnt] = MPU9250_ACC_LAST.X-ACC_OFFSET.X;//更新滑动窗口数组
	ACC_Y_BUF[filter_cnt] = MPU9250_ACC_LAST.Y-ACC_OFFSET.Y;
	ACC_Z_BUF[filter_cnt] = MPU9250_ACC_LAST.Z-ACC_OFFSET.Z;
	
	for(i=0;i<FILTER_NUM;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}
	ACC_AVG.X = temp1 / FILTER_NUM;
	ACC_AVG.Y = temp2 / FILTER_NUM;
	ACC_AVG.Z = temp3 / FILTER_NUM;
	
	Accel.X = ACC_AVG.X;	//滤波后的数据可以使用
	Accel.Y = ACC_AVG.Y; 
	Accel.Z = ACC_AVG.Z;
	
	filter_cnt++;
	if(filter_cnt==FILTER_NUM)	filter_cnt=0;
	
	MPU9250_GYRO_LAST.X-=GYRO_OFFSET.X;
	MPU9250_GYRO_LAST.Y-=GYRO_OFFSET.Y;
	MPU9250_GYRO_LAST.Z-=GYRO_OFFSET.Z;
	MPU9250_MAG_LAST.X-=MAG_OFFSET.X;
	MPU9250_MAG_LAST.Y-=MAG_OFFSET.Y;
	MPU9250_MAG_LAST.Z-=MAG_OFFSET.Z;
	
	MPU9250_GYRO_LAST = Low_Pass();
}
//一阶数字低通滤波器
S_INT16_XYZ Low_Pass(void)
{
  S_INT16_XYZ Result={0,0,0};
  static S_INT16_XYZ Last_Result={0,0,0};
  static float a=0.8;
  
  Result.X = (int16_t)((1-a)*MPU9250_GYRO_LAST.X+a*Last_Result.X);
  Result.Y = (int16_t)((1-a)*MPU9250_GYRO_LAST.Y+a*Last_Result.Y);
  Result.Z = (int16_t)((1-a)*MPU9250_GYRO_LAST.Z+a*Last_Result.Z);
 
  Last_Result = Result;
  return Result;
}
void Get_Attitude(void)
{
	 AHRSupdate(MPU9250_GYRO_LAST.X*Gyro_Gr,
						MPU9250_GYRO_LAST.Y*Gyro_Gr,
						MPU9250_GYRO_LAST.Z*Gyro_Gr,
						ACC_AVG.X,ACC_AVG.Y,ACC_AVG.Z,
						MPU9250_MAG_LAST.X,MPU9250_MAG_LAST.Y,
						MPU9250_MAG_LAST.Z);	//转成弧度
}

//四元数法融合数据
#define Pi 3.1415926f 	
#define Kp  12.0f				//比例增益控制的收敛速度，加速度计
#define Ki  0.005f			//积分增益控制的陀螺仪的偏差的收敛速度
#define halfT  0.001f		//一半的样本周期 500hz 所以为0.001ms

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;        // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;       // scaled integral error

void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;

  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;          
  
  //   [   1-2*q3*q3-2*q1*q1                      2*q1*q2-2*q0*q3      ]
  //   [   2*q2*q3-2*q0*q1     1-2*q2*q2-2*q1*q1  2*q1*q3+2*q0*q2      ]
  //   [   2*q1*q2+2*q0*q3                        1-2*q2*q2-2*q3*q3    ]
  
  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax/ norm;
  ay = ay/ norm;
  az = az/ norm;
  norm = sqrt(mx*mx + my*my + mz*mz);          
  mx = mx/ norm;
  my = my/ norm;
  mz = mz/ norm;         
        
  hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);         
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;        
            
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
        
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
        
  exInt = exInt + ex*Ki;
  eyInt = eyInt + ey*Ki;
  ezInt = ezInt + ez*Ki;
        
  gx = gx + Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;
        
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
        
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
  
  Q_ANGLE.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3; // yaw 
  Q_ANGLE.pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
  Q_ANGLE.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
	
}

