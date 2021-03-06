/****************************************
						极光V1.0 卡尔曼滤波(可以使用)
						修改时间2015/07/29
***************************************/
#include "kalman.h"
#include "mpu9250.h"
#include "math.h"

float Accel_x;	     //X轴加速度值暂存
float Accel_y;	     //Y轴加速度值暂存
float Accel_z;	     //Z轴加速度值暂存

float Gyro_x;		 //X轴陀螺仪数据暂存
float Gyro_y;        //Y轴陀螺仪数据暂存
float Gyro_z;		 //Z轴陀螺仪数据暂存

//float Angle_gy;    //由角速度计算的倾斜角度
float Angle_x_temp;  //由加速度计算的x倾斜角度
float Angle_y_temp;  //由加速度计算的y倾斜角度

float Angle_X_Final; //X最终倾斜角度
float Angle_Y_Final; //Y最终倾斜角度

//角度计算
void Angle_Calcu(void)	 
{
	//范围为2g时，换算关系：16384 LSB/g
	//deg = rad*180/3.14
	float x,y,z;
	
	Accel_x = MPU9250_ACC_LAST.X; //x轴加速度值暂存
	Accel_y =	MPU9250_ACC_LAST.Y; //y轴加速度值暂存
	Accel_z = MPU9250_ACC_LAST.Z; //z轴加速度值暂存
	Gyro_x  = MPU9250_GYRO_LAST.X;  //x轴陀螺仪值暂存
	Gyro_y  = MPU9250_GYRO_LAST.Y;  //y轴陀螺仪值暂存
	Gyro_z  = MPU9250_GYRO_LAST.Z;  //z轴陀螺仪值暂存
	
	//处理x轴加速度
	if(Accel_x<32768) x=Accel_x/16384;
	else              x=1-(Accel_x-49152)/16384;
	
	//处理y轴加速度
	if(Accel_y<32768) y=Accel_y/16384;
	else              y=1-(Accel_y-49152)/16384;
	
	//处理z轴加速度
	if(Accel_z<32768) z=Accel_z/16384;
	else              z=(Accel_z-49152)/16384;

	//用加速度计算三个轴和水平面坐标系之间的夹角
	Angle_x_temp=(atan(y/z))*180/3.14;
	Angle_y_temp=(atan(x/z))*180/3.14;

	//角度的正负号											
	if(Accel_x<32768) Angle_y_temp = +Angle_y_temp;
	if(Accel_x>32768) Angle_y_temp = -Angle_y_temp;
	if(Accel_y<32768) Angle_x_temp = +Angle_x_temp;
	if(Accel_y>32768) Angle_x_temp = -Angle_x_temp;
	if(Accel_z<32768) {}
	if(Accel_z>32768) {}
	
	//角速度
	//向前运动
	if(Gyro_x<32768) Gyro_x=-(Gyro_x/16.4);//范围为1000deg/s时，换算关系：16.4 LSB/(deg/s)
	//向后运动
	if(Gyro_x>32768) Gyro_x=+(65535-Gyro_x)/16.4;
	//向前运动
	if(Gyro_y<32768) Gyro_y=-(Gyro_y/16.4);//范围为1000deg/s时，换算关系：16.4 LSB/(deg/s)
	//向后运动
	if(Gyro_y>32768) Gyro_y=+(65535-Gyro_y)/16.4;
	//向前运动
	if(Gyro_z<32768) Gyro_z=-(Gyro_z/16.4);//范围为1000deg/s时，换算关系：16.4 LSB/(deg/s)
	//向后运动
	if(Gyro_z>32768) Gyro_z=+(65535-Gyro_z)/16.4;
	
	//Angle_gy = Angle_gy + Gyro_y*0.025;  //角速度积分得到倾斜角度.越大积分出来的角度越大
	Kalman_Filter_X(Angle_x_temp,Gyro_x);  //卡尔曼滤波计算X倾角
	Kalman_Filter_Y(Angle_y_temp,Gyro_y);  //卡尔曼滤波计算Y倾角															  
} 

//卡尔曼参数		
float Q_angle = 0.001;  
float Q_gyro  = 0.003;
float R_angle = 0.5;
float dt      = 0.01;//dt为kalman滤波器采样时间;
char  C_0     = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };

void Kalman_Filter_X(float Accel,float Gyro) //卡尔曼函数		
{
	Angle_X_Final += (Gyro - Q_bias) * dt; //先验估计
	
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]= -PP[1][1];
	Pdot[2]= -PP[1][1];
	Pdot[3]= Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle_X_Final;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle_X_Final += K_0 * Angle_err;	 //后验估计
	Q_bias        += K_1 * Angle_err;	 //后验估计
	Gyro_x         = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
}

void Kalman_Filter_Y(float Accel,float Gyro) //卡尔曼函数		
{
	Angle_Y_Final += (Gyro - Q_bias) * dt; //先验估计
	
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]=- PP[1][1];
	Pdot[2]=- PP[1][1];
	Pdot[3]=Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle_Y_Final;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle_Y_Final	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	Gyro_y   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
}
