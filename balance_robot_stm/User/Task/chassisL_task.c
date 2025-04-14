/**
  *********************************************************************
  * @file      chassisL_task.c/h
  * @brief     该任务控制左半部分的电机，分别是两个DM4310和一个DM6215，这三个电机挂载在can2总线上
	*						 从底盘上往下看，左上角的DM4310发送id为8、接收id为4，
	*						 左下角的DM4310发送id为6、接收id为3，
	*						 左边DM轮毂电机发送id为1、接收id为0。
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "chassisL_task.h"
#include "bsp_dwt.h"
#include "user_lib.h"
vmc_leg_t left_vmc;

float LQR_K_L[12]={ 
   -1.6700,   -0.1876,   -0.7057,   -0.7484,    2.3083,    0.2481,
    1.0421,    0.0576,    0.1998,    0.2066,   23.3252,    1.0649};

//dm
//float LQR_K_L[12]={ 
//-2.1954,   -0.2044  , -0.8826,   -1.3245,    1.2784  ,  0.1112,
//    2.5538,   0.2718  ,  1.5728  ,  2.2893  , 12.1973 ,   0.4578};

//extern float Poly_Coefficient[12][4];
		
PidTypeDef LegL_Pid;
extern INS_t INS;
Ordinary_Least_Squares_t DThetaL_smoother; //dtheta滤波器
uint32_t CHASSL_TIME_DWT; //dwt获取的系统时间
float CHASSL_dt;
uint32_t CHASSL_TIME=1;				
void ChassisL_task(void)
{
  while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}	
  ChassisL_init(&chassis_move,&left_vmc,&LegL_Pid);//初始化左边两个关节电机和左边轮毂电机的id和控制模式、初始化腿部
	OLS_Init(&DThetaL_smoother, 50);
	while(1)
	{	
		CHASSL_dt = DWT_GetDeltaT(&CHASSL_TIME_DWT);//获取系统时间
		chassisL_feedback_update(&chassis_move,&left_vmc,&INS);//更新数据
		chassisL_control_loop(&chassis_move,&left_vmc,&INS,LQR_K_L,&LegL_Pid);//控制计算
   		
    if(chassis_move.start_flag==1)	
		{
			DM_Motor_CAN_TxMessage_4310(&FDCAN1_TxFrame, &DM_4310_Motor_leftfront, MIT_Mode, 0.0f, 0.0f,0.0f, 0.0f, left_vmc.T1);// LeftFront
			osDelay(CHASSL_TIME);
			DM_Motor_CAN_TxMessage_4310(&FDCAN1_TxFrame, &DM_4310_Motor_leftback, MIT_Mode, 0.0f, 0.0f,0.0f, 0.0f, left_vmc.T2);// LeftBack
			osDelay(CHASSL_TIME);
			DM_Motor_CAN_TxMessage_6215(&FDCAN1_TxFrame, &DM_6215_Motor_left, MIT_Mode, 0.0f, 0.0f,0.0f, 0.0f, left_vmc.T);
			osDelay(CHASSL_TIME);
		}
		else if(chassis_move.start_flag==0)	
		{
			DM_Motor_CAN_TxMessage_4310(&FDCAN1_TxFrame, &DM_4310_Motor_leftfront, MIT_Mode, 0.0f, 0.0f,0.0f, 0.0f, 0.0f);
			osDelay(CHASSL_TIME);
			DM_Motor_CAN_TxMessage_4310(&FDCAN1_TxFrame, &DM_4310_Motor_leftback, MIT_Mode, 0.0f, 0.0f,0.0f, 0.0f, 0.0f);
			osDelay(CHASSL_TIME);
			DM_Motor_CAN_TxMessage_6215(&FDCAN1_TxFrame, &DM_6215_Motor_left, MIT_Mode, 0.0f, 0.0f,0.0f, 0.0f, 0.0f);
			osDelay(CHASSL_TIME);
		}
	}
}

void ChassisL_init(chassis_t *chassis,vmc_leg_t *vmc,PidTypeDef *legl)
{
  const static float legl_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};
	VMC_init(vmc);//给杆长赋值
	PID_init(legl, PID_POSITION,legl_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//腿长pid
}

void chassisL_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins)
{
	
  vmc->phi1 = pi - DM_4310_Motor_leftfront.Data.Position;
	vmc->phi4 = -DM_4310_Motor_leftback.Data.Position;
	vmc->d_theta_smooth = OLS_Smooth(&DThetaL_smoother, CHASSL_dt, vmc->d_theta);
	
}

extern uint8_t right_flag;
uint8_t left_flag;
void chassisL_control_loop(chassis_t *chassis,vmc_leg_t *vmcl,INS_t *ins,float *LQR_K,PidTypeDef *leg)
{
	VMC_calc_1_left(vmcl,ins,CHASSL_dt);//计算theta和d_theta给lqr用，同时也计算左腿长L0,该任务控制周期是3*0.001秒
	
//	for(int i=0;i<12;i++)
//	{
//		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcl->L0 );	
//	}
			
	vmcl->T=(LQR_K[0]*(0.0f - vmcl->theta)
																					+LQR_K[1]*(0.0f - vmcl->d_theta_smooth)
																					+LQR_K[2]*(chassis->x_set - chassis->x_filter)
																					+LQR_K[3]*(chassis->v_set - chassis->v_filter)
																					+LQR_K[4]*(0.0f - chassis->Pitch_smooth)
																					+LQR_K[5]*(0.0f - chassis->DPitch_smooth));
	
	//右边髋关节输出力矩				
	vmcl->Tp=(LQR_K[6]*(0.0f - vmcl->theta)
					+LQR_K[7]*(0.0f - vmcl->d_theta_smooth)
					+LQR_K[8]*(chassis->x_set - chassis->x_filter)
					+LQR_K[9]*(chassis->v_set - chassis->v_filter)
					+LQR_K[10]*(0.0f - chassis->Pitch_smooth)
					+LQR_K[11]*(0.0f - chassis->DPitch_smooth));
	 		
	vmcl->T = vmcl->T - chassis->turn_T;	//轮毂电机输出力矩，yaw补偿取负
	mySaturate(&vmcl->T,-1.0f,1.0f);	
	vmcl->Tp = vmcl->Tp - chassis->leg_tp;//髋关节输出力矩，防劈叉补偿取负
	vmcl->F0=10.5f/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0, chassis->leg_right_set) - chassis->now_roll_set;//前馈+pd，这里ROLL补偿的正负方向需要测试
//	vmcl->F0=+ PID_Calc(leg,vmcl->L0, chassis->leg_right_set) - chassis->now_roll_set;//测试腿长控制用

//	jump_loop_l(chassis,vmcl,leg); 	

	left_flag=ground_detectionL(vmcl,ins);//左腿离地检测
//	
//	 if(chassis->recover_flag==0)	
//	 {//倒地自起不需要检测是否离地
//		if(left_flag==1&&right_flag==1&&vmcl->leg_flag==0)
//		{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
//			chassis->wheel_motor[1].wheel_T=0.0f;
//			vmcl->Tp=LQR_K[6]*(vmcl->theta-0.0f)+ LQR_K[7]*(vmcl->d_theta-0.0f);
//			
//			chassis->x_filter=0.0f;//对位移清零
//			chassis->x_set=chassis->x_filter;
//			chassis->turn_set=chassis->total_yaw;
//			vmcl->Tp=vmcl->Tp+chassis->leg_tp;		
//		}
//		else
//		{//没有离地
//			vmcl->leg_flag=0;//置为0			
//		}
//	 }
//	 else if(chassis->recover_flag==1)
//	 {
//		 vmcl->Tp=0.0f;
//	 }
	
	mySaturate(&vmcl->F0,-100.0f,100.0f);//限幅 
	
	VMC_calc_2(vmcl);//计算期望的关节输出力矩
	
  //额定扭矩
  mySaturate(&vmcl->torque_set[1],-3.0f,3.0f);	
	mySaturate(&vmcl->torque_set[0],-3.0f,3.0f);	
			
	//根据电机输出方向左边需要取负号
	vmcl->T1 = -vmcl->torque_set[0];
	vmcl->T2 = -vmcl->torque_set[1];
}
void jump_loop_l(chassis_t *chassis,vmc_leg_t *vmcl,PidTypeDef *leg)
{
	if(chassis->jump_flag == 1)
	{
		if(chassis->jump_status_l == 0)
		{
			vmcl->F0= Mg/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0,0.07f) ;//前馈+pd
			if(vmcl->L0<0.1f)
			{
				chassis->jump_time_l++;
			}
		}
		else if(chassis->jump_status_l == 1)
		{
			vmcl->F0= Mg/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0,0.4f) ;//前馈+pd
			if(vmcl->L0>0.16f)
			{
				chassis->jump_time_l++;
			}
		}
		else if(chassis->jump_status_l == 2)
		{
			vmcl->F0=Mg/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0,chassis->leg_right_set) ;//前馈+pd
			if(vmcl->L0<(chassis->leg_right_set+0.01f))
			{
				chassis->jump_time_l++;
			}
		}
		else
		{
			vmcl->F0=Mg/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0,chassis->leg_left_set) ;//前馈+pd
		}

	}
	else
	{
		vmcl->F0=Mg/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0,chassis->leg_left_set) + chassis->now_roll_set;//前馈+pd
	}

}

