/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       voltage_task.c/h
  *             percentage.24��Դ��ѹADC����,��ȡ��ѹ���Ҽ�������ٷֱ�.
  *             ����Դ��ֱ��������,���޸�VOLTAGE_DROP
  */
#ifndef VOLTAGE_TASK_H
#define VOLTAGE_TASK_H
#include "struct_typedef.h"

/**
  * @brief          ��Դ�����ͼ����Դ�ٷֱ�
  * @param[in]      pvParameters: NULL
  */
extern void battery_voltage_task(void const *argument);

/**
  * @brief          ��ȡ����
  * @param[in]      void
  * @retval         ����, ��λ 1, 1 = 1%
  */
extern uint16_t get_battery_percentage(void);
#endif
