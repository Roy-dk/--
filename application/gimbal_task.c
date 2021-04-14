/**
  * @brief      �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
*/

#include "gimbal_task.h"

#include "main.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "shoot.h"
#include "pid.h"

//�������ֵ���� 0��8191
#define ecd_format(ecd)    \
  {                        \
    if ((ecd) > ECD_RANGE) \
      (ecd) -= ECD_RANGE;  \
    else if ((ecd) < 0)    \
      (ecd) += ECD_RANGE;  \
  }

#define gimbal_total_pid_clear(gimbal_clear)                                               \
  {                                                                                        \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
    PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                           \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
    PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
  }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

/**
  * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     init:"gimbal_control"����ָ��.
  */
static void gimbal_init(gimbal_control_t *init);

/**
  * @brief          ������̨����ģʽ����Ҫ��'gimbal_behaviour_mode_set'�����иı�
  * @param[out]     gimbal_set_mode:"gimbal_control"����ָ��.
  */
static void gimbal_set_mode(gimbal_control_t *set_mode);

/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     gimbal_feedback_update:"gimbal_control"����ָ��.
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update);

/**
  * @brief          ��̨ģʽ�ı䣬��Щ������Ҫ�ı䣬�������yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰyaw�Ƕ�
  * @param[out]     mode_change:"gimbal_control"����ָ��.
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);

/**
  * @brief          ����ecd��offset_ecd֮�����ԽǶ�
  * @param[in]      ecd: �����ǰ����
  * @param[in]      offset_ecd: �����ֵ����
  * @retval         ��ԽǶȣ���λrad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

/**
  * @brief          ������̨�����趨ֵ������ֵ��ͨ��gimbal_behaviour_control_set�������õ�
  * @param[out]     gimbal_set_control:"gimbal_control"����ָ��.
  */
static void gimbal_set_control(gimbal_control_t *set_control);

/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     gimbal_control_loop:"gimbal_control"����ָ��.
  */
static void gimbal_control_loop(gimbal_control_t *control_loop);

/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor, int type);

/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor, int type);

/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_RAW������ֱֵ�ӷ��͵�CAN����.
  * @param[out]     gimbal_motor:yaw�������pitch���
  */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);

/**
  * @brief          ��GIMBAL_MOTOR_GYROģʽ�����ƽǶ��趨,��ֹ�������
  * @param[out]     gimbal_motor:yaw�������pitch���
  */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add, int type);

/**
  * @brief          ��GIMBAL_MOTOR_ENCONDEģʽ�����ƽǶ��趨,��ֹ�������
  * @param[out]     gimbal_motor:yaw�������pitch���
  */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

/**
  * @brief          ��̨�Ƕ�PID��ʼ��, ��Ϊ�Ƕȷ�Χ��(-pi,pi)��������PID.c��PID
  * @param[out]     pid:��̨PIDָ��
  * @param[in]      maxout: pid������
  * @param[in]      intergral_limit: pid���������
  * @param[in]      kp: pid kp
  * @param[in]      ki: pid ki
  * @param[in]      kd: pid kd
  */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);

/**
  * @brief          ��̨PID��������pid��out,iout
  * @param[out]     pid_clear:"gimbal_control"����ָ��.
  */
static void gimbal_PID_clear(gimbal_PID_t *pid_clear);

/**
  * @brief          ��̨�Ƕ�PID����, ��Ϊ�Ƕȷ�Χ��(-pi,pi)��������PID.c��PID
  * @param[out]     pid:��̨PIDָ��
  * @param[in]      get: �Ƕȷ���
  * @param[in]      set: �Ƕ��趨
  * @param[in]      error_delta: ���ٶ�
  * @retval         pid ���
  */
static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

/**
  * @brief          ��̨У׼����
  * @param[in]      gimbal_cali: У׼����
  * @param[out]     yaw_offset:yaw�����̨��ֵ
  * @param[out]     pitch_offset:pitch �����̨��ֵ
  * @param[out]     max_yaw:yaw �������е�Ƕ�
  * @param[out]     min_yaw: yaw �����С��е�Ƕ�
  * @param[out]     max_pitch: pitch �������е�Ƕ�
  * @param[out]     min_pitch: pitch �����С��е�Ƕ�
  */
static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

/**********************************************************************************************************************/

#if GIMBAL_TEST_MODE
//j-scope ����pid����
static void J_scope_gimbal_test(void);
#endif

//��̨���������������
gimbal_control_t gimbal_control;

//���͵ĵ������
static int16_t yaw_can_set_current = 0, pitch_can_set_current = 0, shoot_can_set_current = 0;

/**
  * @brief          ��̨���񣬼�� GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: ��
  */

void gimbal_task(void const *pvParameters)
{
  //�ȴ������������������������
  //wait a time
  vTaskDelay(GIMBAL_TASK_INIT_TIME);
  //gimbal init
  //��̨��ʼ��
  gimbal_init(&gimbal_control);
  //shoot init
  //�����ʼ��
  shoot_init();
  //wait for all motor online
  //�жϵ���Ƿ�����
  while (toe_is_error(YAW_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
  {
    vTaskDelay(GIMBAL_CONTROL_TIME);
    gimbal_feedback_update(&gimbal_control); //��̨���ݷ���
  }

  while (1)
  {
    gimbal_set_mode(&gimbal_control);                    //������̨����ģʽ
    gimbal_mode_change_control_transit(&gimbal_control); //����ģʽ�л� �������ݹ���
    gimbal_feedback_update(&gimbal_control);             //��̨���ݷ���
    gimbal_set_control(&gimbal_control);                 //������̨������
    gimbal_control_loop(&gimbal_control);                //��̨����PID����
    shoot_can_set_current = shoot_control_loop();        //����������ѭ��
#if YAW_TURN
    yaw_can_set_current = -gimbal_control.gimbal_yaw_motor.given_current;
#else
    yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
#endif

#if PITCH_TURN
    pitch_can_set_current = -gimbal_control.gimbal_pitch_motor.given_current;
#else
    pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
#endif

    if (!(toe_is_error(YAW_GIMBAL_MOTOR_TOE) && toe_is_error(PITCH_GIMBAL_MOTOR_TOE) && toe_is_error(TRIGGER_MOTOR_TOE)))
    {
      if (toe_is_error(DBUS_TOE))
      {
        CAN_cmd_gimbal(0, 0, 0, 0);
      }
      else
      {
        CAN_cmd_gimbal(yaw_can_set_current, pitch_can_set_current, shoot_can_set_current, 0);
      }
    }

#if GIMBAL_TEST_MODE
    J_scope_gimbal_test();
#endif

    vTaskDelay(GIMBAL_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
    gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
  }
}

/**
  * @brief          ��̨У׼���ã���У׼����̨��ֵ�Լ���С����е��ԽǶ�
  * @param[in]      yaw_offse:yaw ��ֵ
  * @param[in]      pitch_offset:pitch ��ֵ
  * @param[in]      max_yaw:max_yaw:yaw �����ԽǶ�
  * @param[in]      min_yaw:yaw ��С��ԽǶ�
  * @param[in]      max_yaw:pitch �����ԽǶ�
  * @param[in]      min_yaw:pitch ��С��ԽǶ�
  * @retval         ���ؿ�
  * @waring         �������ʹ�õ�gimbal_control ��̬�������º�������������ͨ��ָ�븴��
  */
void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
  gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
  gimbal_control.gimbal_yaw_motor.max_relative_angle = max_yaw;
  gimbal_control.gimbal_yaw_motor.min_relative_angle = min_yaw;

  gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
  gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch;
  gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch;
}

/**
  * @brief          ��̨У׼���㣬��У׼��¼����ֵ,��� ��Сֵ����
  * @param[out]     yaw ��ֵ ָ��
  * @param[out]     pitch ��ֵ ָ��
  * @param[out]     yaw �����ԽǶ� ָ��
  * @param[out]     yaw ��С��ԽǶ� ָ��
  * @param[out]     pitch �����ԽǶ� ָ��
  * @param[out]     pitch ��С��ԽǶ� ָ��
  * @retval         ����1 ����ɹ�У׼��ϣ� ����0 ����δУ׼��
  * @waring         �������ʹ�õ�gimbal_control ��̬�������º�������������ͨ��ָ�븴��
  */
bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
  if (gimbal_control.gimbal_cali.step == 0)
  {
    gimbal_control.gimbal_cali.step = GIMBAL_CALI_START_STEP;
    //�������ʱ������ݣ���Ϊ��ʼ���ݣ����ж������Сֵ
    gimbal_control.gimbal_cali.max_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
    gimbal_control.gimbal_cali.max_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
    gimbal_control.gimbal_cali.max_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
    gimbal_control.gimbal_cali.max_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
    gimbal_control.gimbal_cali.min_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
    gimbal_control.gimbal_cali.min_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
    gimbal_control.gimbal_cali.min_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
    gimbal_control.gimbal_cali.min_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
    return 0;
  }
  else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP)
  {
    calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
    (*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
    (*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
    (*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
    (*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
    gimbal_control.gimbal_yaw_motor.offset_ecd = *yaw_offset;
    gimbal_control.gimbal_yaw_motor.max_relative_angle = *max_yaw;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = *min_yaw;
    gimbal_control.gimbal_pitch_motor.offset_ecd = *pitch_offset;
    gimbal_control.gimbal_pitch_motor.max_relative_angle = *max_pitch;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = *min_pitch;
    gimbal_control.gimbal_cali.step = 0;
    return 1;
  }
  else
  {
    return 0;
  }
}

/**
  * @brief          ��̨У׼���㣬��У׼��¼����ֵ,��� ��Сֵ
  * @param[out]     yaw ��ֵ ָ��
  * @param[out]     pitch ��ֵ ָ��
  * @param[out]     yaw �����ԽǶ� ָ��
  * @param[out]     yaw ��С��ԽǶ� ָ��
  * @param[out]     pitch �����ԽǶ� ָ��
  * @param[out]     pitch ��С��ԽǶ� ָ��
  */
static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
  if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
  {
    return;
  }

  int16_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;

#if YAW_TURN
  temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;

  if (temp_ecd < 0)
  {
    temp_ecd += ecd_range;
  }
  temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);

  ecd_format(temp_ecd);
  *yaw_offset = temp_ecd;
  *max_yaw = -motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
  *min_yaw = -motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#else

  temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

  if (temp_ecd < 0)
  {
    temp_ecd += ECD_RANGE;
  }
  temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);

  ecd_format(temp_ecd);
  *yaw_offset = temp_ecd;
  *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
  *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#endif

#if PITCH_TURN

  temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
  temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
  temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
  temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;

  ecd_format(temp_max_ecd);
  ecd_format(temp_min_ecd);

  temp_ecd = temp_max_ecd - temp_min_ecd;

  if (temp_ecd > HALF_ECD_RANGE)
  {
    temp_ecd -= ECD_RANGE;
  }
  else if (temp_ecd < -HALF_ECD_RANGE)
  {
    temp_ecd += ECD_RANGE;
  }

  if (temp_max_ecd > temp_min_ecd)
  {
    temp_min_ecd += ECD_RANGE;
  }

  temp_ecd = temp_max_ecd - temp_ecd / 2;

  ecd_format(temp_ecd);

  *pitch_offset = temp_ecd;

  *max_pitch = -motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
  *min_pitch = -motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);

#else
  temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
  temp_max_ecd = gimbal_cali->max_pitch_ecd - temp_ecd;
  temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
  temp_min_ecd = gimbal_cali->min_pitch_ecd - temp_ecd;

  ecd_format(temp_max_ecd);
  ecd_format(temp_min_ecd);

  temp_ecd = temp_max_ecd - temp_min_ecd;

  if (temp_ecd > HALF_ECD_RANGE)
  {
    temp_ecd -= ECD_RANGE;
  }
  else if (temp_ecd < -HALF_ECD_RANGE)
  {
    temp_ecd += ECD_RANGE;
  }

  temp_ecd = temp_max_ecd - temp_ecd / 2;

  ecd_format(temp_ecd);

  *pitch_offset = temp_ecd;

  *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
  *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
#endif
}

/**
  * @brief          ����yaw �������ָ��
  * @param[in]      none
  * @retval         yaw���ָ��
  */
const gimbal_motor_t *get_yaw_motor_point(void)
{
  return &gimbal_control.gimbal_yaw_motor;
}

/**
  * @brief          ����pitch �������ָ��
  * @param[in]      none
  * @retval         pitch
  */
const gimbal_motor_t *get_pitch_motor_point(void)
{
  return &gimbal_control.gimbal_pitch_motor;
}

gimbal_motor_t *yaw_motor;

/**
  * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     init:"gimbal_control"����ָ��.
  */
static void gimbal_init(gimbal_control_t *init)
{

  static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
	static const fp32 Pitch_curr_pid[3] = {1, 0.05, 0};
  static const fp32 Yaw_speed_pid[3] = {100, 25, 0};
  static const fp32 Yaw_curr_pid[3] = {1, 0.05, 0};
	
  //�������ָ���ȡ
  init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
  init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
  //����������ָ���ȡ
  init->gimbal_INT_angle_point = get_INS_angle_point();
  init->gimbal_INT_gyro_point = get_gyro_data_point();
  //ң��������ָ���ȡ
  init->gimbal_rc_ctrl = get_remote_control_point();
  //��ʼ�����ģʽ
  init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
  init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
  //��ʼ��yaw���pid
  gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
  gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
  PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
  PID_init(&init->gimbal_yaw_motor.gimbal_motor_curr_pid, PID_POSITION, Yaw_curr_pid, 30000, 30000);
  yaw_motor = &init->gimbal_yaw_motor;
  //��ʼ��pitch���pid
  gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
  gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
  PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
	PID_init(&init->gimbal_pitch_motor.gimbal_motor_curr_pid, PID_POSITION, Pitch_curr_pid, 30000, 30000);
  //�������PID
  gimbal_total_pid_clear(init);

  gimbal_feedback_update(init);

  init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
  init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
  init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;

  init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
  init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
  init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
}

/**
  * @brief          ������̨����ģʽ����Ҫ��'gimbal_behaviour_mode_set'�����иı�
  * @param[out]     gimbal_set_mode:"gimbal_control"����ָ��.
  */
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
  if (set_mode == NULL)
  {
    return;
  }
  gimbal_behaviour_mode_set(set_mode);
}

/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     gimbal_feedback_update:"gimbal_control"����ָ��.
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
  if (feedback_update == NULL)
  {
    return;
  }
  //��̨���ݸ���
  feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);

#if PITCH_TURN
  feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                  feedback_update->gimbal_pitch_motor.offset_ecd);
#else

  feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                 feedback_update->gimbal_pitch_motor.offset_ecd);
#endif

  feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

  feedback_update->gimbal_yaw_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);

#if YAW_TURN
  feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                feedback_update->gimbal_yaw_motor.offset_ecd);

#else
  feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                               feedback_update->gimbal_yaw_motor.offset_ecd);
#endif
  feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET)) - arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
}

/**
  * @brief          ����ecd��offset_ecd֮�����ԽǶ�
  * @param[in]      ecd: �����ǰ����
  * @param[in]      offset_ecd: �����ֵ����
  * @retval         ��ԽǶȣ���λrad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
  int32_t relative_ecd = ecd - offset_ecd;
  if (relative_ecd > HALF_ECD_RANGE)
  {
    relative_ecd -= ECD_RANGE;
  }
  else if (relative_ecd < -HALF_ECD_RANGE)
  {
    relative_ecd += ECD_RANGE;
  }

  return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
  * @brief          ��̨ģʽ�ı䣬��Щ������Ҫ�ı䣬�������yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰyaw�Ƕ�
  * @param[out]     gimbal_mode_change:"gimbal_control"����ָ��.
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
  if (gimbal_mode_change == NULL)
  {
    return;
  }
  //yaw���״̬���л���������
  if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
  {
    gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
  }
  else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
  {
    gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
  }
  else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
  {
    gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
  }
  gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

  //pitch���״̬���л���������
  if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
  {
    gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
  }
  else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
  {
    gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
  }
  else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
  {
    gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
  }

  gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}

/**
  * @brief          ������̨�����趨ֵ������ֵ��ͨ��gimbal_behaviour_control_set�������õ�
  * @param[out]     gimbal_set_control:"gimbal_control"����ָ��.
  */
static void gimbal_set_control(gimbal_control_t *set_control)
{
  if (set_control == NULL)
  {
    return;
  }

  fp32 add_yaw_angle = 0.0f;
  fp32 add_pitch_angle = 0.0f;

  gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
  //yaw���ģʽ����
  if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
  {
    //rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
    set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
  }
  else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
  {
    //gyroģʽ�£������ǽǶȿ���
    gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle, 1);
  }
  else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
  {
    //encondeģʽ�£��������Ƕȿ���
    gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
  }

  //pitch���ģʽ����
  if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
  {
    //rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
    set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
  }
  else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
  {
    //gyroģʽ�£������ǽǶȿ���
    //gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle,0);
    gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
  }
  else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
  {
    //encondeģʽ�£��������Ƕȿ���
    gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
  }
}

/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add, int type)
{
  static fp32 bias_angle;
  static fp32 angle_set;
  if (gimbal_motor == NULL || type == 0)
  {
    return;
  }
  //now angle error
  //��ǰ�������Ƕ�
  bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
  //relative angle + angle error + add_angle > max_relative angle
  //��̨��ԽǶ�+ ���Ƕ� + �����Ƕ� ������� ����е�Ƕ�
  /*	delete limit 
  if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
  {
    //�����������е�Ƕȿ��Ʒ���
    if (add > 0.0f)
    {
      //calculate max add_angle
      //�����һ��������ӽǶȣ�
      add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
    }
  }
  else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
  {
    if (add < 0.0f)
    {
      add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
    }
  }*/
  angle_set = gimbal_motor->absolute_angle_set;
  gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}

/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
  if (gimbal_motor == NULL)
  {
    return;
  }
  gimbal_motor->relative_angle_set += add;
  //�Ƿ񳬹���� ��Сֵ
  if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
  {
    gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
  }
  else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
  {
    gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
  }
}

/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     gimbal_control_loop:"gimbal_control"����ָ��.
  */
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
  if (control_loop == NULL)
  {
    return;
  }

  if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
  {
    gimbal_motor_raw_angle_control(&control_loop->gimbal_yaw_motor);
  }
  else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
  {
    gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor, TYPE_YAW);
  }
  else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
  {
    gimbal_motor_relative_angle_control(&control_loop->gimbal_yaw_motor, TYPE_YAW);
  }

  if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
  {
    gimbal_motor_raw_angle_control(&control_loop->gimbal_pitch_motor);
  }
  else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
  {
    //gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_motor, 1);
    gimbal_motor_relative_angle_control(&control_loop->gimbal_pitch_motor, TYPE_PITCH);
  }
  else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
  {
    gimbal_motor_relative_angle_control(&control_loop->gimbal_pitch_motor, TYPE_PITCH);
  }
}

/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor, int type)
{
  if (gimbal_motor == NULL)
  {
    return;
  }
  //�ǶȻ����ٶȻ�������������pid����
  static int cnt1 = 0;
  cnt1++;
  if ((cnt1 % 10) == 0)
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
  if (type == TYPE_YAW)
  {
    static float curr_yaw = 0;
    if ((cnt1 % 5) == 0)
    {
      curr_yaw = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->gimbal_motor_measure->speed_rpm, gimbal_motor->motor_gyro_set);
    }
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_curr_pid, gimbal_motor->gimbal_motor_measure->given_current, curr_yaw);
  }
  else
  {
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
  }
  //����ֵ��ֵ
  gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

//fp32 absolute_angle_zero = 0;
//int16_t sw = 0;
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor, int type)
{
	static int cnt2 = 0;
  if (gimbal_motor == NULL)
  {
    return;
  }

  //�ǶȻ����ٶȻ�����pid����
  if (type == TYPE_PITCH)
  {
		static float curr_pitch = 0;
		if(cnt2 % 10 == 0)
			gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
		if(cnt2 % 5 == 0)
			curr_pitch = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->gimbal_motor_measure->speed_rpm, gimbal_motor->motor_gyro_set);
		gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_curr_pid, gimbal_motor->gimbal_motor_measure->given_current, curr_pitch);

  }
  if (type == TYPE_YAW)
  {
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
  }
	
  //gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
  
	//����ֵ��ֵ
  gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_RAW������ֱֵ�ӷ��͵�CAN����.
  * @param[out]     gimbal_motor:yaw�������pitch���
  */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor)
{
  if (gimbal_motor == NULL)
  {
    return;
  }
  gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
  gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

#if GIMBAL_TEST_MODE
int32_t yaw_ins_int_1000, pitch_ins_int_1000;
int32_t yaw_ins_set_1000, pitch_ins_set_1000;
int32_t pitch_relative_set_1000, pitch_relative_angle_1000;
int32_t yaw_speed_int_1000, pitch_speed_int_1000;
int32_t yaw_speed_set_int_1000, pitch_speed_set_int_1000;
static void J_scope_gimbal_test(void)
{
  yaw_ins_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle * 1000);
  yaw_ins_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle_set * 1000);
  yaw_speed_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro * 1000);
  yaw_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_set * 1000);

  pitch_ins_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle * 1000);
  pitch_ins_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle_set * 1000);
  pitch_speed_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro * 1000);
  pitch_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro_set * 1000);
  pitch_relative_angle_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle * 1000);
  pitch_relative_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle_set * 1000);
}

#endif

/**
  * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     gimbal_init:"gimbal_control"����ָ��.
  */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
  if (pid == NULL)
  {
    return;
  }
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;

  pid->err = 0.0f;
  pid->get = 0.0f;

  pid->max_iout = max_iout;
  pid->max_out = maxout;
}

static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
  fp32 err;
  if (pid == NULL)
  {
    return 0.0f;
  }
  pid->get = get;
  pid->set = set;

  err = set - get;
  pid->err = rad_format(err);
  pid->Pout = pid->kp * pid->err;
  pid->Iout += pid->ki * pid->err;
  pid->Dout = pid->kd * error_delta;
  abs_limit(&pid->Iout, pid->max_iout);
  pid->out = pid->Pout + pid->Iout + pid->Dout;
  abs_limit(&pid->out, pid->max_out);
  return pid->out;
}

/**
  * @brief          ��̨PID��������pid��out,iout
  * @param[out]     gimbal_pid_clear:"gimbal_control"����ָ��.
  */
static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
  if (gimbal_pid_clear == NULL)
  {
    return;
  }
  gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
  gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}
