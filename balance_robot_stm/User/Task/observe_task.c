/**
  *********************************************************************
  * @file      observe_task.c/h
  * @brief     该任务是对机体运动速度估计，用于抑制打滑
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "observe_task.h"
#include "kalman_filter.h"
#include "cmsis_os.h"
#include "Motor.h"
#include "bsp_dwt.h"
#include "user_lib.h"

Ordinary_Least_Squares_t v_smoother; //低通滤波

KalmanFilter_t vaEstimateKF;	   // 卡尔曼滤波器结构体

float vaEstimateKF_F[4] = {1.0f, 0.003f, 
                           0.0f, 1.0f};	   // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {0.5f, 0.0f, 
                           0.0f, 0.5f};    // Q矩阵初始值

float vaEstimateKF_R[4] = {100.0f, 0.0f, 
                            0.0f,  100.0f}; 	
														
float vaEstimateKF_K[4];
													 
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// 设置矩阵H为常量
														 															 
extern INS_t INS;		
extern chassis_t chassis_move;																 															 
																 
extern vmc_leg_t right;			
extern vmc_leg_t left;	

float vel_acc[2]; 
uint32_t OBSERVE_TIME=3;//任务周期是3ms		
uint32_t OBSERVE_TIME_DWT; //dwt获取的系统时间
float OBSERVE_dt;																 
void 	Observe_task(void)
{
	while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}
	static float wr,wl=0.0f;
	static float vrb,vlb=0.0f;
	static float aver_v=0.0f;
	static float v_origin=0.0f;
	OLS_Init(&v_smoother, 30);
		
	xvEstimateKF_Init(&vaEstimateKF);
	
  while(1)
	{  
		OBSERVE_dt = DWT_GetDeltaT(&OBSERVE_TIME_DWT);//获取系统时间
		wr= -DM_6215_Motor_right.Data.Velocity - INS.Gyro[0] + right_vmc.d_alpha;//右边驱动轮转子相对大地角速度
		vrb=wr*0.0603f+ right_vmc.L0*right_vmc.d_theta*arm_cos_f32(right_vmc.theta) + right_vmc.d_L0*arm_sin_f32(right_vmc.theta);//机体b系的速度
		
		wl= DM_6215_Motor_left.Data.Velocity - INS.Gyro[0] + left_vmc.d_alpha;//左边驱动轮转子相对大地角速度
		vlb=wl*0.0603f+ left_vmc.L0*left_vmc.d_theta*arm_cos_f32(left_vmc.theta) + left_vmc.d_L0*arm_sin_f32(left_vmc.theta);//机体b系的速度
		
		aver_v=(vlb-vrb)/2.0f;//取平均
    xvEstimateKF_Update(&vaEstimateKF,INS.MotionAccel_n[1],aver_v);
		
		//原地自转的过程中v_filter和x_filter应该都是为0
		chassis_move.v_kfilter=vel_acc[0];//得到卡尔曼滤波后的速度
		chassis_move.x_kfilter=chassis_move.x_filter+chassis_move.v_filter*OBSERVE_dt;
		
		//如果想直接用轮子速度，不做融合的话可以这样
		v_origin = (-DM_6215_Motor_right.Data.Velocity + DM_6215_Motor_left.Data.Velocity)*(0.0603f)/2.0f;//0.0603是轮子半径，电机反馈的是角速度，乘半径后得到线速度，数学模型中定义的是轮子顺时针为正，所以要乘个负号
		chassis_move.v_filter = OLS_Smooth(&v_smoother, OBSERVE_dt, v_origin);
		chassis_move.x_filter = chassis_move.x_filter + chassis_move.v_filter*OBSERVE_dt;
		
		osDelay(OBSERVE_TIME);
	}
}

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// 状态向量2维 没有控制量 测量向量2维
	
		memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{   	
    //卡尔曼滤波器测量值更新
    EstimateKF->MeasuredVector[0] =	vel;//测量速度
    EstimateKF->MeasuredVector[1] = acc;//测量加速度
    		
    //卡尔曼滤波器更新函数
    Kalman_Filter_Update(EstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++)
    {
      vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}


