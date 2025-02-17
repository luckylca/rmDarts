#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机械臂任务
 * 
 */
void uart1Task();

/**
 * @brief 拉力传感器任务
 * 
 */
void uart6Task();

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 * 
 */
void RobotCMDTask();

#endif // !ROBOT_CMD_H