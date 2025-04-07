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
#include "bsp_can.h"
#include "VMC_calc.h"
#include "INS_task.h"
#include "cmsis_os.h"
#include "pid.h"

vmc_leg_t left_vmc;

float LQR_K_L[12]={ 
   -1.7323 ,  -0.2163  , -0.6886 ,  -0.8960 ,    3.0823  ,  0.3612,
    1.8842,   0.2119  ,   0.7182  ,  0.9088  , 16.7720 ,  0.7983};

//dm
//float LQR_K_L[12]={ 
//-2.1954,   -0.2044  , -0.8826,   -1.3245,    1.2784  ,  0.1112,
//    2.5538,   0.2718  ,  1.5728  ,  2.2893  , 12.1973 ,   0.4578};

//extern float Poly_Coefficient[12][4];
		
PidTypeDef LegL_Pid;
extern INS_t INS;

uint32_t CHASSL_TIME=1;				
void ChassisL_task(void)
{
  while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}	
  ChassisL_init(&chassis_move,&left_vmc,&LegL_Pid);//初始化左边两个关节电机和左边轮毂电机的id和控制模式、初始化腿部
	
	while(1)
	{	
		chassisL_feedback_update(&chassis_move,&left_vmc,&INS);//更新数据
		
		chassisL_control_loop(&chassis_move,&left_vmc,&INS,LQR_K_L,&LegL_Pid);//控制计算
   		
    if(chassis_move.start_flag==1)	
		{
			DM_Motor_CAN_TxMessage_4310(&FDCAN1_TxFrame, &chassis_move.joint_motor[3], MIT_Mode, 0.0f, 0.0f,0.0f, 0.0f, chassis_move.joint_motor[3].Control.Torque);// LeftFront
			osDelay(CHASSL_TIME);
			DM_Motor_CAN_TxMessage_4310(&FDCAN1_TxFrame, &chassis_move.joint_motor[2], MIT_Mode, 0.0f, 0.0f,0.0f, 0.0f, chassis_move.joint_motor[2].Control.Torque);// LeftBack
			osDelay(CHASSL_TIME);
//			DM_Motor_CAN_TxMessage_6215(&FDCAN1_TxFrame, &chassis_move.wheel_motor[1], MIT_Mode, 0.0f, 0.0f,0.0f, 0.0f, chassis_move.wheel_motor[1].Control.Torque);
			osDelay(CHASSL_TIME);
		}
		else if(chassis_move.start_flag==0)	
		{
			DM_Motor_CAN_TxMessage_4310(&FDCAN1_TxFrame, &chassis_move.joint_motor[3], MIT_Mode, 0.0f, 0.0f,0.0f, 0.0f, 0.0f);
			osDelay(CHASSL_TIME);
			DM_Motor_CAN_TxMessage_4310(&FDCAN1_TxFrame, &chassis_move.joint_motor[2], MIT_Mode, 0.0f, 0.0f,0.0f, 0.0f, 0.0f);
			osDelay(CHASSL_TIME);
			DM_Motor_CAN_TxMessage_6215(&FDCAN1_TxFrame, &chassis_move.wheel_motor[1], MIT_Mode, 0.0f, 0.0f,0.0f, 0.0f, 0.0f);
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
		//将电机反馈值获取到底盘结构体中
	chassis->joint_motor[3] = DM_4310_Motor_leftfront;
	chassis->joint_motor[2] = DM_4310_Motor_leftback;
	chassis->wheel_motor[1] = DM_6215_Motor_left;
	
  vmc->phi1 = pi-chassis->joint_motor[3].Data.Position;
	vmc->phi4 = -chassis->joint_motor[2].Data.Position;
		
//	chassis->myPithL=0.0f-ins->Pitch;
//	chassis->myPithGyroL=0.0f-ins->Gyro[0];
	
}

extern uint8_t right_flag;
uint8_t left_flag;
void chassisL_control_loop(chassis_t *chassis,vmc_leg_t *vmcl,INS_t *ins,float *LQR_K,PidTypeDef *leg)
{
	VMC_calc_1_left(vmcl,ins,((float)CHASSL_TIME)*3.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算左腿长L0,该任务控制周期是3*0.001秒
	
//	for(int i=0;i<12;i++)
//	{
//		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcl->L0 );	
//	}
			
	chassis->wheel_motor[1].Control.Torque=(LQR_K[0]*(0.0f - vmcl->theta)
																					+LQR_K[1]*(0.0f - vmcl->d_theta)
																					+LQR_K[2]*(chassis->x_set - chassis->x_filter)
																					+LQR_K[3]*(chassis->v_set - chassis->v_filter)
																					+LQR_K[4]*(0.0f - chassis->myPithR)
																					+LQR_K[5]*(0.0f - chassis->myPithGyroR));
	
	//右边髋关节输出力矩				
	vmcl->Tp=(LQR_K[6]*(0.0f - vmcl->theta)
					+LQR_K[7]*(0.0f - vmcl->d_theta)
					+LQR_K[8]*(chassis->x_set - chassis->x_filter)
					+LQR_K[9]*(chassis->v_set - chassis->v_filter)
					+LQR_K[10]*(0.0f - chassis->myPithR)
					+LQR_K[11]*(0.0f - chassis->myPithGyroR));
	 		
	chassis->wheel_motor[1].Control.Torque = chassis->wheel_motor[1].Control.Torque + chassis->turn_T;	//轮毂电机输出力矩，yaw补偿取负
	//根据电机正方向输出值取负号：
	chassis->wheel_motor[1].Control.Torque = -chassis->wheel_motor[1].Control.Torque;
	mySaturate(&chassis->wheel_motor[1].Control.Torque,-1.0f,1.0f);	
	vmcl->Tp = vmcl->Tp + chassis->leg_tp;//髋关节输出力矩，防劈叉补偿取负
	vmcl->F0=10.5f/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0, chassis->leg_right_set) - chassis->now_roll_set;//前馈+pd，这里ROLL补偿的正负方向需要测试
//	vmcl->F0=+ PID_Calc(leg,vmcl->L0, chassis->leg_right_set) - chassis->now_roll_set;//测试腿长控制用

//	jump_loop_l(chassis,vmcl,leg); 	

//	left_flag=ground_detectionL(vmcl,ins);//左腿离地检测
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
	chassis->joint_motor[3].Control.Torque = -vmcl->torque_set[0];
	chassis->joint_motor[2].Control.Torque = -vmcl->torque_set[1];
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

