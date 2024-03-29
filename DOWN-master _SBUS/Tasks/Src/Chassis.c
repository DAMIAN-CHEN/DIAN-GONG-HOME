//
// Created by 14685 on 2022/7/16.
//

#include "Chassis.h"
#include "bsp_uart.h"
#include "motor.h"
#include "bsp_can.h"
#include "keyboard.h"

ChassisTypeDef chassis;
MotorTypeDef chassis_motor[4];
//底盘跟随云台PID结构体
PIDTypeDef rotate_follow;
/* 底盘电机期望转速(rpm) */
int16_t chassis_moto_speed_ref[4];
/* 底盘电机电流 */
int16_t chassis_moto_current[4];

//底盘陀螺转速系数
char spin_flag=0;
float yaw_relative_angle=0;


void Chassis_Task(void const * argument){

    Chassis_Init_param();
    chassis.ctrl_mode = CHASSIS_STOP;
    uint32_t chassis_wake_time = osKernelSysTick();

    while (1){
        //根据状态机切换底盘模式
        Chassis_Get_mode();

        switch (chassis.ctrl_mode) {
            case CHASSIS_FOLLOW_GIMBAL:{
                Chassis_Follow_control();
            }break;

            case CHASSIS_OPEN_LOOP:{
                Chassis_Open_control();
            }break;

            case CHASSIS_SPIN:{
                Chassis_Spin_control();
            }break;

            case CHASSIS_FLY:{
                Chassis_Fly_control();
            }break;

            case CHASSIS_STOP:{
                Chassis_Stop_control();
            }break;

            case CHASSIS_RELAX:{
                Chassis_Relax_control();
            }break;
        }
        osDelayUntil(&chassis_wake_time, CHASSIS_PERIOD);
    }
}

void Chassis_Init_param(void)
{
    //挂起底盘任务
    //osThreadSuspend(NULL);
    //底盘PID参数设置
    for (int i = 0; i < 4; i++)
    {
        PID_Init(
                &chassis_motor[i].PID_Velocity, 16000, 8000, 1, 8, 15, 0, 500, 100, 0.001, 0, 1, Integral_Limit | OutputFilter);
        chassis_motor[i].Max_Out = 8000;
    }
    //底盘跟随PID参数设置
    PID_Init(&rotate_follow, 1500, 300, 0, 10, 10, 0, 300,
             100, 0, 0.01, 5,
             Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | DerivativeFilter);
}

void Chassis_Get_mode(void){
    chassis.last_mode = chassis.ctrl_mode;

    if (rc.sw1 == RC_UP){
        chassis.ctrl_mode = CHASSIS_OPEN_LOOP;
    }
    if (rc.sw1 == RC_DN){
        chassis.ctrl_mode = CHASSIS_STOP;
    }
    if (rc.sw2 == RC_DN){
        chassis.ctrl_mode = CHASSIS_RELAX;
    }
}

void Chassis_Relax_control(void){
    static uint16_t data[8];
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    Chassis_Send_current(data);
}

void Chassis_Stop_control(void){
    chassis.vx = 0;
    chassis.vy = 0;
    chassis.vw = 0;
    Chassis_Custom_control();
}

void Chassis_Fly_control(void){
    int16_t   chassis_fly_current[]={-200, 200, 200, -200}; //底盘飞坡模式时，所有电机电流值最大16384
    chassis.vx = 0;
    chassis.vy = 0;
    chassis.vw = PID_Calculate(&rotate_follow, yaw_relative_angle,0);
    Chassis_Custom_control();
    if(yaw_relative_angle<=0.3){   //底盘和云台对准完毕  //TODO
        Chassis_Send_current(chassis_fly_current);  //将四个电机的最大电流发送给电调
    }
}

void Chassis_Open_control(void){
    Chassis_Get_control_information();
//    chassis.vw = 0;  //TODO
    Chassis_Custom_control();
}

void Chassis_Follow_control(void){
    Chassis_Get_control_information();
    chassis.vw = PID_Calculate(&rotate_follow, yaw_relative_angle,0);
    if ((abs(rc.ch1) <= 10)
        && (abs(rc.ch2) <= 10)
        && (abs(rc.ch3) <= 10)
        && (abs(rc.ch4) <= 10)
        && (abs(rc.mouse.x) <= 1)
        && (abs(rc.mouse.y) <= 1)){
        chassis.vw = 0;
    }
    Chassis_Top_handle();
    Chassis_Custom_control();
}

void Chassis_Spin_control(void){
    Chassis_Get_control_information();
    if(rc.kb.bit.E)
    spin_flag++;
    if(rc.kb.bit.E && rc.kb.bit.SHIFT)
        spin_flag=0;
    chassis.vw=10*spin_flag;   //陀螺旋转速度
    Chassis_Top_handle();
    Chassis_Custom_control();
}


void Chassis_Send_current(int16_t current[]){
    static uint8_t data[8];
    data[0] = current[0] >> 8;
    data[1] = current[0];
    data[2] = current[1] >> 8;
    data[3] = current[1];
    data[4] = current[2] >> 8;
    data[5] = current[2];
    data[6] = current[3] >> 8;
    data[7] = current[3];
    CAN_Send(CHASSIS_CAN, CAN_CHASSIS_ID, data);
}

/* 底盘控制信号获取 */
void Chassis_Get_control_information(void){
    pc_kb_hook();
    //遥控器以及鼠标对底盘的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s)
   /* chassis.vx = rc.ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_MAX_VALUE * MAX_CHASSIS_VX_SPEED + km.vx * CHASSIS_PC_MOVE_RATIO_X; //麦克纳姆轮
    chassis.vy = rc.ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_MAX_VALUE * MAX_CHASSIS_VY_SPEED + km.vy * CHASSIS_PC_MOVE_RATIO_Y;
    chassis.vw = rc.ch4 * CHASSIS_RC_MOVE_RATIO_R / RC_MAX_VALUE * MAX_CHASSIS_VR_SPEED + rc.mouse.x * CHASSIS_PC_MOVE_RATIO_R;*/

    chassis.vx = rc.ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_MAX_VALUE * MAX_CHASSIS_VX_SPEED + km.vx * CHASSIS_PC_MOVE_RATIO_X; //全向轮
    chassis.vy = -(rc.ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_MAX_VALUE * MAX_CHASSIS_VY_SPEED + km.vy * CHASSIS_PC_MOVE_RATIO_Y);
    chassis.vw = rc.ch4 * CHASSIS_RC_MOVE_RATIO_R / RC_MAX_VALUE * MAX_CHASSIS_VR_SPEED + rc.mouse.x * CHASSIS_PC_MOVE_RATIO_R;
}


/* 底盘运动的速度分解，以及电机转速的闭环控制 */
void Chassis_Custom_control(void){
    //底盘速度分解，计算底盘电机转速
    Chassis_Calc_moto_speed(chassis.vx, chassis.vy, chassis.vw, chassis_moto_speed_ref);
    //闭环计算底盘轮子电机电流
    Chassis_Calculate_close_loop();
    //将计算好的电流值发送给电调
    Chassis_Send_current(chassis_moto_current);
}

/* 底盘的运动分解处理 */
void Chassis_Calc_moto_speed(float vx, float vy, float vw, int16_t speed[]){
    static float rotate_ratio_f = ((WHEELBASE+WHEELTRACK)/2.0f - GIMBAL_OFFSET)/RADIAN_COEF;
    static float rotate_ratio_b = ((WHEELBASE+WHEELTRACK)/2.0f + GIMBAL_OFFSET)/RADIAN_COEF;
    static float wheel_rpm_ratio = 60.0f/(PERIMETER*CHASSIS_DECELE_RATIO);

    int16_t wheel_rpm[4];
    float max = 0;

    //限制底盘各方向速度
    VAL_LIMIT(vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
    VAL_LIMIT(vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
    VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s

   /* wheel_rpm[0] = -(+vx -vy - vw * rotate_ratio_f) * wheel_rpm_ratio;   //麦克纳姆轮
    wheel_rpm[1] = (-vx -vy + vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[2] = -(-vx - vy - vw * rotate_ratio_b) * wheel_rpm_ratio;
    wheel_rpm[3] = (+vx - vy + vw * rotate_ratio_b) * wheel_rpm_ratio;*/

    wheel_rpm[0] = (+vx + vy + 0.03f * vw * (LENGTH_A+LENGTH_B)) * wheel_rpm_ratio*0.8f; //left//x全向轮
    wheel_rpm[1] = (+vx - vy + 0.03f * vw * (LENGTH_A+LENGTH_B)) * wheel_rpm_ratio*0.8f; //forward
    wheel_rpm[2] = (-vx - vy + 0.03f * vw * (LENGTH_A+LENGTH_B)) * wheel_rpm_ratio*0.8f; //right
    wheel_rpm[3] = (-vx + vy + 0.03f * vw * (LENGTH_A+LENGTH_B)) * wheel_rpm_ratio*0.8f; //back
    //限制每个轮的转速，找出最大值
    for (uint8_t i = 0; i < 4; i++){
        if (abs(wheel_rpm[i]) > max)
            max = abs(wheel_rpm[i]);
    }
    //若超速，给每个轮的速度乘以相同比例

    if (max > MAX_WHEEL_RPM){
        float rate = MAX_WHEEL_RPM / max;
        for (uint8_t i = 0; i < 4; i++)
            wheel_rpm[i] *= rate;
    }

#ifdef POWER_CONTORL
    //功率限制
	//按缓冲功率减小的程度给每个轮的速度乘以相同比例
	if(chassis_power > POWER_LIMIT)
	{
		float r = power_buffer * power_buffer/3600;
		for(int i=0; i<4; i++){
			wheel_rpm[i] *= r;
		}
	}
#endif
    memcpy(speed, wheel_rpm, 4*sizeof(int16_t));
}

void Chassis_Calculate_close_loop(void){
    for (int i = 0; i < 4; i++)
    {
        chassis_moto_current[i]=Motor_Speed_Calculate(&chassis_motor[i], chassis_motor[i].Velocity_RPM, chassis_moto_speed_ref[i]);
    }
}

/* 底盘陀螺处理 */
void Chassis_Top_handle(void){
    float rad=-(yaw_relative_angle/RADIAN_COEF);
    float a=cosf(rad),b=sinf(rad);
    float temp_x=a*chassis.vx+b*chassis.vy;
    float temp_y=-b*chassis.vx+a*chassis.vy;
    chassis.vx=temp_x;
    chassis.vy=temp_y;
}
