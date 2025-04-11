#ifndef __CHASSISL_TASK_H
#define __CHASSISL_TASK_H

#include "main.h"
#include "Motor.h"
#include "VMC_calc.h"
#include "bsp_can.h"
#include "INS_task.h"
#include "cmsis_os.h"
#include "pid.h"
#include "chassisR_task.h"

extern vmc_leg_t left_vmc;	
extern void ChassisL_task(void);
extern void ChassisL_init(chassis_t *chassis,vmc_leg_t *vmc,PidTypeDef *legl);
extern void chassisL_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins);
extern void chassisL_control_loop(chassis_t *chassis,vmc_leg_t *vmcl,INS_t *ins,float *LQR_K,PidTypeDef *leg);
void jump_loop_l(chassis_t *chassis,vmc_leg_t *vmcl,PidTypeDef *leg);
#endif



