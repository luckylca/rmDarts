#ifndef STATUS_H
#define STATUS_H

// 初始化状态
extern int gimbalBeginInit;
extern int chassisBeginInit;
extern int shootBeginInit;
extern int robotcmdBeginInit;

// 云台步骤状态
extern int gimbal_Step_1_AngleReached;
extern int gimbal_Step_2_AngleReached;
extern int gimbal_Step_3_AngleReached;
extern int gimbal_Step_4_AngleReached;

// 底盘步骤状态
extern int chassis_Step_1_AngleReached;
extern int chassis_Step_2_AngleReached;
extern int chassis_Step_3_AngleReached;
extern int chassis_Step_4_AngleReached;

// 发射机构步骤状态
extern int shoot_Step_1_ChargeLoaderReached;
extern int shoot_Step_2_ChargeLoaderReached;
extern int shoot_Step_3_ChargeLoaderReached;
extern int shoot_Step_4_ChargeLoaderReached;

extern int shoot_Step_1_RotateChangeDartReached;
extern int shoot_Step_2_RotateChangeDartReached;
extern int shoot_Step_3_RotateChangeDartReached;
extern int shoot_Step_4_RotateChangeDartReached;

#endif // STATUS_H