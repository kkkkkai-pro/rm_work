/*
 * control_task.h
 *
 * 串级 PID 控制任务（FreeRTOS 任务，周期 1 ms）：
 *   外环：角度环  -> 输出目标速度 (rpm)
 *   内环：速度环  -> 输出电流值
 * 任务函数名 MotorControlTask 与 CubeMX 里创建的
 * 任务入口名保持一致。
 */
#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void MotorControlTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_TASK_H */
