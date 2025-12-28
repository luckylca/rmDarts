#include "gimbal.h"
#include "dji_motor.h"
#include "encoder.h"
#include "general_def.h"
#include "ins_task.h"
#include "jz_motor.h"
#include "message_center.h"
#include "robot_def.h"
#include "user_lib.h"
#include "controller.h"
#include "status.h"
static DJIMotorInstance *yaw_motor;
static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static EncoderInstance *encoder;
extern int16_t cyy_test_data;
float cyy_target_angle = 0;
float cyy_init_angle = 0;
int init_flag = 0;
int cyy_target_todo = 1;

void GimbalInit()
{
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config =
            {
                .can_handle = &hcan1,
                .tx_id = 7,
            },
        .controller_param_init_config =
            {
                .angle_PID =
                    {
                        .Kp = 20, // 10
                        .Ki = 0,
                        .Kd = 1,
                        .MaxOut = 500000,
                    },
                .speed_PID =
                    {
                        .Kp = 15, // 10
                        .Ki = 1,  // 1
                        .Kd = 0,
                        .Improve = PID_Integral_Limit,
                        .IntegralLimit = 5000,
                        .MaxOut = 5000,
                    },
                .current_PID =
                    {
                        .Kp = 0.5, // 0.7
                        .Ki = 0.1, // 0.1
                        .Kd = 0,
                        .Improve = PID_Integral_Limit,
                        .IntegralLimit = 5000,
                        .MaxOut = 5000,
                    },
            },
        .controller_setting_init_config =
            {
                .angle_feedback_source = MOTOR_FEED,
                .speed_feedback_source = MOTOR_FEED,
                .outer_loop_type =
                    SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
                .close_loop_type = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
                .motor_reverse_flag =
                    MOTOR_DIRECTION_NORMAL, //  MOTOR_DIRECTION_NORMAL
                                            //  MOTOR_DIRECTION_REVERSE
            },
        .motor_type = M2006,
        .storage_type = NO_STORAGE};
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);

    SPI_Init_Config_s encoder_spi_config = {
        .spi_handle = &hspi2,
        .GPIOx = GPIOB,
        .cs_pin = GPIO_PIN_12,
        .spi_work_mode = SPI_BLOCK_MODE,
        .callback = NULL,
        .id = NULL,
    };
    encoder = EncoderInit(&encoder_spi_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

void GimbalTask()
{

    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    switch (gimbal_cmd_recv.gimbal_mode)
    {
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        break;
    case GIMBAL_TEST:
        DJIMotorEnable(yaw_motor);
        DJIMotorOuterLoop(yaw_motor, ANGLE_LOOP);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // 这里大概率还需要调节系数
        break;
    case AUTO_DART:
        /*
        // 这一部分是将视觉数据转换为电机角度
        DJIMotorEnable(yaw_motor);
        if (cyy_target_todo == 1) {
          cyy_target_angle -= 0.1 * (cyy_test_data - 320) / 64.0f;
          cyy_target_angle = float_constrain(cyy_target_angle, -20.0, 20.0);
          cyy_target_todo = 0;
        }
        if ((-yaw_motor->measure.total_angle - cyy_target_angle < 1) &&
            (-yaw_motor->measure.total_angle - cyy_target_angle > -1)) {
          cyy_target_todo = 1;
        } else {
          cyy_target_todo = 0;
        }
        // cyy_target_angle += cyy_test_data * 0.000004;
        // cyy_target_angle = float_constrain(cyy_target_angle, -170.0, 170.0);
        DJIMotorSetRef(yaw_motor, cyy_target_angle);
        */

        // 第一发镖
        {
            uint8_t cur = DartSys.currentStep;
            // 防止数组越界
            if (cur >= 4)
                return;

            if (cur == 0)
            {
                if (!DART_CHECK_BIT(0, FLAG_GIMBAL_AIMED))
                {
                    // gimbal定位到目标位置
                    DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
                    if (yaw_motor->measure.total_angle == gimbal_cmd_recv.yaw)
                    { // 到达位置后设置标志位
                        DART_SET_BIT(0, FLAG_GIMBAL_AIMED);
                    }
                }
                return;
            }

            // 第二发镖
            if (cur == 1)
            {
                if (!DART_CHECK_BIT(1, FLAG_GIMBAL_AIMED))
                {
                    // gimbal定位到目标位置
                    DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
                    if (yaw_motor->measure.total_angle == gimbal_cmd_recv.yaw)
                    { // 到达位置后设置标志位
                        DART_SET_BIT(1, FLAG_GIMBAL_AIMED);
                    }
                }
                return;
            }

            // 第三发镖
            if (cur == 2)
            {
                if (!DART_CHECK_BIT(2, FLAG_GIMBAL_AIMED))
                {
                    // gimbal定位到目标位置
                    DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
                    if (yaw_motor->measure.total_angle == gimbal_cmd_recv.yaw)
                    { // 到达位置后设置标志位
                        DART_SET_BIT(2, FLAG_GIMBAL_AIMED);
                    }
                }
                return;
            }

            // 第四发镖
            if (cur == 3)
            {
                if (!DART_CHECK_BIT(3, FLAG_GIMBAL_AIMED))
                {
                    // gimbal定位到目标位置
                    DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
                    if (yaw_motor->measure.total_angle == gimbal_cmd_recv.yaw)
                    { // 到达位置后设置标志位
                        DART_SET_BIT(3, FLAG_GIMBAL_AIMED);
                    }
                }
                return;
            }
        }
        break;
    default:
        break;
    }
    gimbal_feedback_data.yaw_motor_single_round_angle =
        yaw_motor->measure.angle_single_round;
    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}