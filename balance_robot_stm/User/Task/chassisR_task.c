/**
  *********************************************************************
  * @file      chassisR_task.c/h
  * @brief     该任务控制右半部分的电机，分别是两个DM4310和一个DM6215，这三个电机挂载在can1总线上
	*						 从底盘上往下看，右上角的DM4310发送id为6、接收id为3，
	*						 右下角的DM4310发送id为8、接收id为4，
	*						 右边DM轮毂电机发送id为1、接收id为0。
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
	
#include "chassisR_task.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "bsp_dwt.h"

float LQR_K_R[12]={ 
   -1.6700,   -0.1876,   -0.7057,   -0.7484,    2.3083,    0.2481,
    1.0421,    0.0576,    0.1998,    0.2066,   23.3252,    1.0649};

//DM
//float LQR_K_R[12]={ 
//-2.1954,   -0.2044  , -0.8826,   -1.3245,    1.2784  ,  0.1112,
//    2.5538,   0.2718  ,  1.5728  ,  2.2893  , 12.1973 ,   0.4578};

//float Poly_Coefficient[12][4]={	{-46.8734500193858,	32.4883608520456,	-17.0885922501780	,-0.536065047533883},
//																{1.48246010807197,	-2.36607779875741,	-1.53521740398882,	-0.0213682479756981},
//																{-44.9737664784523,	25.1746395712299,	-4.80854925156531,	-2.84031633786197},
//																{-4.93762493870657,	3.91974634758477,	-2.79459683826947,	-1.00823815038500},
//																{-194.609164251463,	117.144266080074,	-25.3701698043778	,2.32296290792753},
//																{-10.7217041372824,	6.59258111237609,	-1.47162116809482,	0.166761140976817},
//																{-141.430351511940,	86.5035702356333,	-19.2831262431850,	2.08471526597486},
//																{-12.4480584697546,	7.57666027947902,	-1.66833442186198	,0.196027465825959},
//																{-646.233100548413,	386.625379822743,	-82.6337764346849,	6.98716202318258},
//																{-227.994193295510,	135.591639019836,	-28.7547957179339	,2.43623020558302},
//																{205.536462887607,	-116.019511024166	,22.4916342449141,9.77817098877540},
//																{15.8984823843716	,-9.14713791307948,	1.83367299366907	,0.420742565153764}};
	
//float Poly_Coefficient[12][4]={	{-88.3079710751263,	68.9068310796955,	-30.0003802287502,	-0.197774178106864},
//																{1.52414598059982	,-1.09343038036609,	-2.82688593867512,	0.0281973842051861},
//																{-21.8700750609220	,12.7421672466682,	-2.58779676995074	,-0.750848242540331},
//																{-29.3271263750692,	17.6067629457167,	-4.23484645974363	,-1.08976980288501},
//																{-147.771748892911,	94.0665615939814,	-22.5139626085997	,2.53224765312440},
//																{-6.72857056332562,	4.46216499907277,	-1.14328671767927	,0.176775242328476},
//																{-43.1495035855057,	35.1427890165576,	-12.7617044245710	,3.36940801739176},
//																{4.14428184617563,	-2.56933858132474,	0.479050092243477	,0.248175261724735},
//																{-229.898177881547	,144.949258291255	,-33.9196587052128,	3.44291788865558},
//																{-329.509693153293,	207.219295206736,	-48.3799707459102	,4.952560575479143},
//																{380.589246401548,	-223.660017597103	,46.1696952431268	,9.82308882692083},
//																{26.1010681824798	,-15.7241310513153	,3.39175554658673	,0.278568898146322}};
//三次多项式拟合系数
//float Poly_Coefficient[12][4]={	{-92.0699600773357	,71.4868555011764	,-30.6921933953314,	-0.0958002007084524},
//																{1.60098559368738,	-1.13274122580887	,-2.82116637582756	,0.0295182225494464},
//																{-21.1867196303270,	12.3610554853386,	-2.51566924070518	,-0.755562691415545},
//																{-30.6461230131359,	18.4403464720723,	-4.42893895932222	,-1.07006891622098},
//																{-141.783593718613,	90.5321293186876,	-21.7824629497436	,2.47606043845602},
//																{-6.49940206537698,	4.32462034853376	,-1.11404205284405,	0.174401976130811},
//																{3.09605574012049	,7.37880709057633	,-6.81650374507351	,2.87205502564828},
//																{4.83479253102295	,-3.01643745917309,	0.586384046364414	,0.237251571342193},
//																{-224.130470954818	,141.560444143461	,-33.2264057886601,	3.39152168796696},
//																{-313.237104933888,	197.784837724870	,-46.4812453696717,	4.81419185198688},
//																{360.850043501824,	-212.560933205359	,44.0392588918109 ,	9.96626295435711},
//																{25.0961164366780	,-15.1533421691331,	3.28051567574422	,0.286235660771812}};

extern vmc_leg_t left_vmc;	
extern INS_t INS;													
chassis_t chassis_move;
vmc_leg_t right_vmc;
															
PidTypeDef LegR_Pid;//右腿的腿长pd
PidTypeDef Tp_Pid;//防劈叉补偿pd
PidTypeDef Turn_Pid;//转向pd
PidTypeDef RollR_Pid; //ROLL补偿pd
Ordinary_Least_Squares_t Pitch_smoother; //pitch滤波器
Ordinary_Least_Squares_t DPitch_smoother; //dpitch滤波器
Ordinary_Least_Squares_t DThetaR_smoother; //dtheta滤波器
uint32_t CHASSR_TIME_DWT; //dwt获取的系统时间
float CHASSR_dt;
uint32_t CHASSR_TIME=1;	//任务延时			
void ChassisR_task(void)
{
	while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}
	chassis_move.start_flag = 0;// 底盘未就绪
	DM_Motor_Command(&FDCAN1_TxFrame, &DM_4310_Motor_leftfront, Motor_Disable);
	DM_Motor_Command(&FDCAN2_TxFrame, &DM_4310_Motor_rightfront, Motor_Disable);
	osDelay(1);	
	DM_Motor_Command(&FDCAN1_TxFrame, &DM_4310_Motor_leftback, Motor_Disable);
	DM_Motor_Command(&FDCAN2_TxFrame, &DM_4310_Motor_rightback, Motor_Disable);
	osDelay(1);	
	DM_Motor_Command(&FDCAN1_TxFrame, &DM_6215_Motor_left, Motor_Disable);
	DM_Motor_Command(&FDCAN2_TxFrame, &DM_6215_Motor_right, Motor_Disable);	
  ChassisR_init(&chassis_move,&right_vmc,&LegR_Pid);//初始化右边两个关节电机和右边轮毂电机的id和控制模式、初始化腿部
  Pensation_init(&Tp_Pid,&Turn_Pid);//补偿pid初始化
//	roll_pid_init(&RollR_Pid);
	OLS_Init(&Pitch_smoother, 10);
	OLS_Init(&DPitch_smoother, 3);
	OLS_Init(&DThetaR_smoother, 50);
	while(1)
	{	
		CHASSR_dt = DWT_GetDeltaT(&CHASSR_TIME_DWT);//获取系统时间
		chassisR_feedback_update(&chassis_move,&right_vmc,&INS);//更新数据
	  chassisR_control_loop(&chassis_move,&right_vmc,&INS,LQR_K_R,&LegR_Pid);//控制计算
   
		if(chassis_move.start_flag==1)	
		{
			DM_Motor_CAN_TxMessage_4310(&FDCAN2_TxFrame, &DM_4310_Motor_rightfront, MIT_Mode, 0.0f, 0.0f,0.0f, 0.0f, right_vmc.T1);
			osDelay(CHASSR_TIME);
			DM_Motor_CAN_TxMessage_4310(&FDCAN2_TxFrame, &DM_4310_Motor_rightback, MIT_Mode, 0.0f, 0.0f,0.0f, 0.0f, right_vmc.T2);
			osDelay(CHASSR_TIME);
			DM_Motor_CAN_TxMessage_6215(&FDCAN2_TxFrame, &DM_6215_Motor_right, MIT_Mode, 0.0f, 0.0f,0.0f, 0.0f, right_vmc.T);
			osDelay(CHASSR_TIME);
		}
		else if(chassis_move.start_flag==0)	
		{
			DM_Motor_CAN_TxMessage_4310(&FDCAN2_TxFrame, &DM_4310_Motor_rightfront, MIT_Mode, 0.0f, 0.0f,0.0f, 0.0f, 0.0f);
			osDelay(CHASSR_TIME);
			DM_Motor_CAN_TxMessage_4310(&FDCAN2_TxFrame, &DM_4310_Motor_rightback, MIT_Mode, 0.0f, 0.0f,0.0f, 0.0f, 0.0f);
			osDelay(CHASSR_TIME);
			DM_Motor_CAN_TxMessage_6215(&FDCAN2_TxFrame, &DM_6215_Motor_right, MIT_Mode, 0.0f, 0.0f,0.0f, 0.0f, 0.0f);
			osDelay(CHASSR_TIME);
		}
	
	}
}

void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc,PidTypeDef *legr)
{
  const static float legr_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD}; //腿长PID参数
	VMC_init(vmc);//给杆长赋值
	chassis->leg_set = 0.10;
//	chassis->leg_set = 0.18;//测试腿长PID使用
	chassis->leg_right_set = chassis->leg_set;
	chassis->leg_left_set = chassis->leg_set;
	PID_init(legr, PID_POSITION,legr_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//腿长PID初始化
}

void Pensation_init(PidTypeDef *Tp,PidTypeDef *turn)
{//补偿pid初始化：防劈叉补偿、偏航角补偿

	const static float tp_pid[3] = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
	const static float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};
	
	PID_init(Tp, PID_POSITION, tp_pid, TP_PID_MAX_OUT,TP_PID_MAX_IOUT);
	PID_init(turn, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);

}

void roll_pid_init(PidTypeDef *roll_pid)
{
	const static float roll[3] = {ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD};
	
	PID_init(roll_pid, PID_POSITION, roll, ROLL_PID_MAX_OUT,ROLL_PID_MAX_IOUT);
}
void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins)
{
	
  vmc->phi1 = pi + DM_4310_Motor_rightfront.Data.Position;
	vmc->phi4 = DM_4310_Motor_rightback.Data.Position;
		
	chassis->myPith = 0.0f-ins->Pitch; //仿真中抬头为正，原数据是低头为正
	chassis->myPithGyro = 0.0f-ins->Gyro[0]; //ins-Gyro0是低头增加
	chassis->Pitch_smooth = OLS_Smooth(&Pitch_smoother, CHASSR_dt, chassis->myPith); //最小二乘平滑滤波处理
	chassis->DPitch_smooth = OLS_Smooth(&DPitch_smoother, CHASSR_dt, chassis->myPithGyro);
	vmc->d_theta_smooth = OLS_Smooth(&DThetaR_smoother, CHASSR_dt, vmc->d_theta);
	
	chassis->total_yaw=ins->YawTotalAngle;
//	chassis->roll=-ins->Roll;
	chassis->theta_err =  vmc->theta - left_vmc.theta; //右-左
	
//	if(ins->Pitch<(3.1415926f/6.0f)&&ins->Pitch>(-3.1415926f/6.0f)) //30度
//	{//根据pitch角度判断倒地自起是否完成
//		chassis->recover_flag=0;
//	}
}
uint32_t count_roll = 0; 
uint8_t right_flag=0;
extern uint8_t left_flag;
void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,PidTypeDef *leg)
{
	VMC_calc_1_right(vmcr,ins,CHASSR_dt);//计算theta和d_theta给lqr用，同时也计算右腿长L0,该任务控制周期是3*0.001秒
	
	for(int i=0;i<12;i++)
	{
		// 进行完定腿长测试后再测试变腿长
//		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcr->L0 );	
		// 定腿长0.12m
	}
		
	chassis->turn_T = PID_Calc(&Turn_Pid, chassis->total_yaw, chassis->turn_set);//yaw轴pid计算
//  chassis->turn_T=Turn_Pid.Kp*(chassis->turn_set - chassis->total_yaw) - Turn_Pid.Kd*ins->Gyro[2];//这样计算更稳一点
	chassis->leg_tp=PID_Calc(&Tp_Pid, chassis->theta_err,0.0f);//防劈叉pid计算
	
	vmcr->T=(LQR_K[0]*(0.0f - vmcr->theta)
																					+LQR_K[1]*(0.0f - vmcr->d_theta_smooth)
																					+LQR_K[2]*(chassis->x_set - chassis->x_filter)
																					+LQR_K[3]*(chassis->v_set - chassis->v_filter)
																					+LQR_K[4]*(0.0f - chassis->Pitch_smooth)
																					+LQR_K[5]*(0.0f - chassis->DPitch_smooth));
	
	//右边髋关节输出力矩				
	vmcr->Tp=(LQR_K[6]*(0.0f - vmcr->theta)
					+LQR_K[7]*(0.0f - vmcr->d_theta_smooth)
					+LQR_K[8]*(chassis->x_set - chassis->x_filter)
					+LQR_K[9]*(chassis->v_set - chassis->v_filter)
					+LQR_K[10]*(0.0f - chassis->Pitch_smooth)
					+LQR_K[11]*(0.0f - chassis->DPitch_smooth));
				
	vmcr->T = - (vmcr->T + chassis->turn_T);	//轮毂电机输出力矩  根据电机正方向输出值取负号：
	mySaturate(&vmcr->T, -1.0f, 1.0f);
	vmcr->Tp = vmcr->Tp + chassis->leg_tp;//髋关节输出力矩


//	chassis->now_roll_set = PID_Calc(&RollR_Pid,chassis->roll,chassis->roll_set);


	vmcr->F0=10.5f/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0, chassis->leg_right_set) + chassis->now_roll_set;//前馈+pd，这里ROLL补偿的正负方向需要测试
//	vmcr->F0= PID_Calc(leg,vmcr->L0, chassis->leg_right_set) + chassis->now_roll_set;//测试腿长控制用
//	jump_loop_r(chassis,vmcr,leg);
		
	right_flag=ground_detectionR(vmcr,ins);//右腿离地检测
//	 
//	 if(chassis->recover_flag==0)		
//	 {//倒地自起不需要检测是否离地	 
//		if(right_flag==1&&left_flag==1&&vmcr->leg_flag==0)
//		{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
//				chassis->wheel_motor[0].wheel_T=0.0f;
//				vmcr->Tp=LQR_K[6]*(vmcr->theta-0.0f)+ LQR_K[7]*(vmcr->d_theta-0.0f);

//				chassis->x_filter=0.0f;
//				chassis->x_set=chassis->x_filter;
//				chassis->turn_set=chassis->total_yaw;
//				vmcr->Tp=vmcr->Tp+chassis->leg_tp;		
//		}
//		else
//		{//没有离地
//			vmcr->leg_flag=0;//置为0
//			
//		}
//	 }
//	 else if(chassis->recover_flag==1)
//	 {
//		 vmcr->Tp=0.0f;
//	 }	 
//	 
	mySaturate(&vmcr->F0,-150.0f,150.0f);//限幅 
	
	VMC_calc_2(vmcr);//计算期望的关节输出力矩

	//额定扭矩
  mySaturate(&vmcr->torque_set[1],-3.0f,3.0f);	
	mySaturate(&vmcr->torque_set[0],-3.0f,3.0f);		
	
	vmcr->T1 = vmcr->torque_set[0];
	vmcr->T2 = vmcr->torque_set[1];
	
}

void mySaturate(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}


void jump_loop_r(chassis_t *chassis,vmc_leg_t *vmcr,PidTypeDef *leg)
{
	if(chassis->jump_flag == 1)
	{
		if(chassis->jump_status_r == 0)
		{
			vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,0.07f) ;//前馈+pd
			if(vmcr->L0<0.1f)
			{
				chassis->jump_time_r++;
			}
			if(chassis->jump_time_r>=10&&chassis->jump_time_l>=10)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 1;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 1;
			}
		}
		else if(chassis->jump_status_r == 1)
		{
			vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,0.4f) ;//前馈+pd
			if(vmcr->L0>0.16f)
			{
				chassis->jump_time_r++;
			}
			if(chassis->jump_time_r>=10&&chassis->jump_time_l>=10)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 2;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 2;
			}
		}
		else if(chassis->jump_status_r == 2)
		{
			vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,chassis->leg_right_set) ;//前馈+pd
			if(vmcr->L0<(chassis->leg_right_set+0.01f))
			{
				chassis->jump_time_r++;
			}
			if(chassis->jump_time_r>=10&&chassis->jump_time_l>=10)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 3;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 3;
			}
		}
		else
		{
			vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,chassis->leg_right_set) ;//前馈+pd
		}

		if(chassis->jump_status_r == 3&&chassis->jump_status_l == 3)
		{
			chassis->jump_flag = 0;
			chassis->jump_time_r = 0;
			chassis->jump_status_r = 0;
			chassis->jump_time_l = 0;
			chassis->jump_status_l = 0;
		}
	}
	else
	{
		vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,chassis->leg_right_set) - chassis->now_roll_set;//前馈+pd
	}

}

