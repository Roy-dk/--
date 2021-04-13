/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       detect_task.c/h
  * @brief      detect error task, judged by receiving data time. provide detect
                hook function, error exist function.
  *             ���������� ͨ����������ʱ�����ж�.�ṩ ��⹳�Ӻ���,������ں���.

    ���Ҫ���һ�����豸
    1.��һ����detect_task.h������豸������errorList�������
    enum errorList
    {
        ...
        XXX_TOE,    //���豸
        ERROR_LIST_LENGHT,
    };
    2.��detect_init����,���offlineTime, onlinetime, priority����
        uint16_t set_item[ERROR_LIST_LENGHT][3] =
        {
            ...
            {n,n,n}, //XX_TOE
        };
    3.�����data_is_error_fun ,solve_lost_fun,solve_data_error_fun��������ֵ������ָ��
    4.��XXX_TOE�豸��������ʱ��, ��Ӻ���detect_hook(XXX_TOE).

  */

#ifndef DETECT_TASK_H
#define DETECT_TASK_H
#include "struct_typedef.h"

#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 10

//�������Լ���Ӧ�豸˳��
enum errorList
{
  DBUS_TOE = 0,
  CHASSIS_MOTOR1_TOE,
  CHASSIS_MOTOR2_TOE,
  CHASSIS_MOTOR3_TOE,
  CHASSIS_MOTOR4_TOE,
  YAW_GIMBAL_MOTOR_TOE,
  PITCH_GIMBAL_MOTOR_TOE,
  TRIGGER_MOTOR_TOE,
  BOARD_GYRO_TOE,
  BOARD_ACCEL_TOE,
  BOARD_MAG_TOE,
  REFEREE_TOE,
  RM_IMU_TOE,
  OLED_TOE,
  ERROR_LIST_LENGHT,
};

typedef __packed struct
{
  uint32_t new_time;
  uint32_t last_time;
  uint32_t lost_time;
  uint32_t work_time;
  uint16_t set_offline_time : 12;
  uint16_t set_online_time : 12;
  uint8_t enable : 1;
  uint8_t priority : 4;
  uint8_t error_exist : 1;
  uint8_t is_lost : 1;
  uint8_t data_is_error : 1;

  fp32 frequency;
  bool_t (*data_is_error_fun)(void);
  void (*solve_lost_fun)(void);
  void (*solve_data_error_fun)(void);
} error_t;

/**
  * @brief          detect task
  */
/**
  * @brief          �������
  */
extern void detect_task(void const *pvParameters);

/**
  * @brief          ��ȡ�豸��Ӧ�Ĵ���״̬
  * @param[in]      toe:�豸Ŀ¼
  * @retval         true(����) ����false(û����)
  */
extern bool_t toe_is_error(uint8_t err);

/**
  * @brief          ��¼ʱ��
  * @param[in]      toe:�豸Ŀ¼
  */
extern void detect_hook(uint8_t toe);

/**
  * @brief          �õ������б�
  * @param[in]      none
  * @retval         error_list��ָ��
  */
extern const error_t *get_error_list_point(void);

#endif
