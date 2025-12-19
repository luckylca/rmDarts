//这里放所有的状态参数，统一使用全局变量进行管理
//命名采用驼峰命名法，application+step+状态，0 代表未完成，1 代表完成，例如 gimbalStep1RotateChangeDart

int gimbalBeginInit = 0;
int chassisBeginInit = 0;
int shootBeginInit = 0;
int robotcmdBeginInit = 0;

int gimbal_Step_1_AngleReached = 0;
int gimbal_Step_2_AngleReached = 0;
int gimbal_Step_3_AngleReached = 0;
int gimbal_Step_4_AngleReached = 0;

int chassis_Step_1_AngleReached = 0;
int chassis_Step_2_AngleReached = 0;
int chassis_Step_3_AngleReached = 0;
int chassis_Step_4_AngleReached = 0;

int shoot_Step_1_ChargeLoaderReached = 0;
int shoot_Step_2_ChargeLoaderReached = 0;
int shoot_Step_3_ChargeLoaderReached = 0;
int shoot_Step_4_ChargeLoaderReached = 0;
int shoot_Step_1_RotateChangeDartReached = 0;
int shoot_Step_2_RotateChangeDartReached = 0;
int shoot_Step_3_RotateChangeDartReached = 0;
int shoot_Step_4_RotateChangeDartReached = 0;
