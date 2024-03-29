/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

/**
  *********************** (C) COPYRIGHT 2018 DJI **********************
  * @update
  * @history
  * Version     Date              Author           Modification
  * V1.0.0      January-15-2018   ric.luo
  * @verbatim
  *********************** (C) COPYRIGHT 2018 DJI **********************
  */ 
 



#ifndef __SYS_H__
#define __SYS_H__


/*************************课程模块设置*******************************/
///* 可用的课程模块选项有 */
//enum
//{
//  ALONE_MOTO_TEST,    //单独电机测试
//  ALONE_CHASSIS,      //单独底盘机构
//  ALONE_GIMBAL,       //单独云台机构
//  ALONE_SHOOT,        //单独发射机构
//  COMPLETE_STRUCTURE, //整车结构模式
//};
///* 将上面课程模块使用如下的宏定义使能 */
//#define INFANTRY_STRUCT COMPLETE_STRUCTURE
/* 4轮车，如果使用3轮车注释掉此宏定义 */
#define FOUR_WHEEL

/*************************底盘速度设置*******************************/

/* 遥控器模式下的底盘最大速度限制 */
/* 底盘平移速度 */
#define CHASSIS_RC_MOVE_RATIO_X 1.0f
/* 底盘前进速度 */
#define CHASSIS_RC_MOVE_RATIO_Y 0.8f
/* 底盘旋转速度，只在底盘开环模式下使用 */
#define CHASSIS_RC_MOVE_RATIO_R 0.8f

/* 鼠标键盘模式下的底盘最大速度限制 */
/* 底盘平移速度 */
#define CHASSIS_PC_MOVE_RATIO_X 0.5f
/* 底盘前进速度 */
#define CHASSIS_PC_MOVE_RATIO_Y 1.0f
/* 底盘旋转速度，只在底盘开环模式下使用 */
#define CHASSIS_PC_MOVE_RATIO_R 5.0f

/*************************云台速度设置*******************************/

/* 遥控器模式下的云台速度限制 */
/* 云台pitch轴速度 */
#define GIMBAL_RC_MOVE_RATIO_PIT 0.5f
/* 云台yaw轴速度 */
#define GIMBAL_RC_MOVE_RATIO_YAW 0.5f
/* 鼠标键盘模式下的云台速度限制 */
/* 云台pitch轴速度 */
#define GIMBAL_PC_MOVE_RATIO_PIT 0.8f
/* 云台yaw轴速度 */
#define GIMBAL_PC_MOVE_RATIO_YAW 1.0f







/*************************下面是系统接口设置，请勿改动**************************/


/* 遥控器的最大行程 */
#define RC_MAX_VALUE      660.0f

#define RC_RATIO          0.002f
#define KB_RATIO          0.02f



/********** 4轮底盘信息 **********/
/* 底盘轮距(mm) */
#define WHEELTRACK     300
/* 底盘轴距(mm) */
#define WHEELBASE      388
/* 云台偏移(mm) */
#define GIMBAL_OFFSET  0


/* 底盘轮子周长(mm) */
#define PERIMETER      484

/******** 底盘电机使用3508 *******/
/* 3508底盘电机减速比 */
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)
/* 单个电机速度极限，单位是分钟每转 */
#define MAX_WHEEL_RPM        9000   //8347rpm = 3500mm/s


/******** 底盘最大速度设置 *******/
/* 底盘移动最大速度，单位是毫米每秒 */
#define MAX_CHASSIS_VX_SPEED 5000
#define MAX_CHASSIS_VY_SPEED 5000
/* 底盘旋转最大速度，单位是度每秒 */
#define MAX_CHASSIS_VR_SPEED 360

/* yaw轴最大转角 */
#define YAW_ANGLE_MAX        80
/* yaw轴最小转角 */
#define YAW_ANGLE_MIN        -80
/* pitch轴最大仰角 */
#define PIT_ANGLE_MAX        32.0f
/* pitch轴最大俯角 */
#define PIT_ANGLE_MIN        -31.5f



/* 串口数据相关 */
#define MAX_DMA_COUNT        	200
#define DBUS_FRAME_SIZE      	18
#define BLUETOOTH_FRAME_SIZE  20
#define NUC_FRAME_SIZE  15
#define REFEREE_FRAME_SIZE 50

/* 常用的一些物理系数 */
/* 角度转弧度系数 */
#define RADIAN_COEF          57.3f

/* 极值限制函数宏定义 */
#define VAL_LIMIT(val, min, max)\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\

#endif
