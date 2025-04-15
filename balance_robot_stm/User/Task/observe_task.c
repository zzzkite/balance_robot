/**
  *********************************************************************
  * @file      observe_task.c/h
  * @brief     �������ǶԻ����˶��ٶȹ��ƣ��������ƴ�
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

Ordinary_Least_Squares_t v_smoother; //��ͨ�˲�

KalmanFilter_t vaEstimateKF;	   // �������˲����ṹ��

float vaEstimateKF_F[4] = {1.0f, 0.003f, 
                           0.0f, 1.0f};	   // ״̬ת�ƾ��󣬿�������Ϊ0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // �������Э�����ʼֵ

float vaEstimateKF_Q[4] = {0.5f, 0.0f, 
                           0.0f, 0.5f};    // Q�����ʼֵ

float vaEstimateKF_R[4] = {100.0f, 0.0f, 
                            0.0f,  100.0f}; 	
														
float vaEstimateKF_K[4];
													 
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// ���þ���HΪ����
														 															 
extern INS_t INS;		
extern chassis_t chassis_move;																 															 
																 
extern vmc_leg_t right;			
extern vmc_leg_t left;	

float vel_acc[2]; 
uint32_t OBSERVE_TIME=3;//����������3ms		
uint32_t OBSERVE_TIME_DWT; //dwt��ȡ��ϵͳʱ��
float OBSERVE_dt;																 
void 	Observe_task(void)
{
	while(INS.ins_flag==0)
	{//�ȴ����ٶ�����
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
		OBSERVE_dt = DWT_GetDeltaT(&OBSERVE_TIME_DWT);//��ȡϵͳʱ��
		wr= -DM_6215_Motor_right.Data.Velocity - INS.Gyro[0] + right_vmc.d_alpha;//�ұ�������ת����Դ�ؽ��ٶ�
		vrb=wr*0.0603f+ right_vmc.L0*right_vmc.d_theta*arm_cos_f32(right_vmc.theta) + right_vmc.d_L0*arm_sin_f32(right_vmc.theta);//����bϵ���ٶ�
		
		wl= DM_6215_Motor_left.Data.Velocity - INS.Gyro[0] + left_vmc.d_alpha;//���������ת����Դ�ؽ��ٶ�
		vlb=wl*0.0603f+ left_vmc.L0*left_vmc.d_theta*arm_cos_f32(left_vmc.theta) + left_vmc.d_L0*arm_sin_f32(left_vmc.theta);//����bϵ���ٶ�
		
		aver_v=(vlb-vrb)/2.0f;//ȡƽ��
    xvEstimateKF_Update(&vaEstimateKF,INS.MotionAccel_n[1],aver_v);
		
		//ԭ����ת�Ĺ�����v_filter��x_filterӦ�ö���Ϊ0
		chassis_move.v_kfilter=vel_acc[0];//�õ��������˲�����ٶ�
		chassis_move.x_kfilter=chassis_move.x_filter+chassis_move.v_filter*OBSERVE_dt;
		
		//�����ֱ���������ٶȣ������ںϵĻ���������
		v_origin = (-DM_6215_Motor_right.Data.Velocity + DM_6215_Motor_left.Data.Velocity)*(0.0603f)/2.0f;//0.0603�����Ӱ뾶������������ǽ��ٶȣ��˰뾶��õ����ٶȣ���ѧģ���ж����������˳ʱ��Ϊ��������Ҫ�˸�����
		chassis_move.v_filter = OLS_Smooth(&v_smoother, OBSERVE_dt, v_origin);
		chassis_move.x_filter = chassis_move.x_filter + chassis_move.v_filter*OBSERVE_dt;
		
		osDelay(OBSERVE_TIME);
	}
}

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// ״̬����2ά û�п����� ��������2ά
	
		memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{   	
    //�������˲�������ֵ����
    EstimateKF->MeasuredVector[0] =	vel;//�����ٶ�
    EstimateKF->MeasuredVector[1] = acc;//�������ٶ�
    		
    //�������˲������º���
    Kalman_Filter_Update(EstimateKF);

    // ��ȡ����ֵ
    for (uint8_t i = 0; i < 2; i++)
    {
      vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}


