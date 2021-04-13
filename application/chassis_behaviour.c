/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_behaviour.c/h
  *             ����ң������ֵ������������Ϊ��
    ���Ҫ���һ���µ���Ϊģʽ
    1.���ȣ���chassis_behaviour.h�ļ��У� ���һ������Ϊ������ chassis_behaviour_e
    erum
    {  
        ...
        ...
        CHASSIS_XXX_XXX, // ����ӵ�
    }chassis_behaviour_e,

    2. ʵ��һ���µĺ��� chassis_xxx_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t * chassis )
        "vx,vy,wz" �����ǵ����˶�����������
        ��һ������: 'vx' ͨ�����������ƶ�,��ֵ ǰ���� ��ֵ ����
        �ڶ�������: 'vy' ͨ�����ƺ����ƶ�,��ֵ ����, ��ֵ ����
        ����������: 'wz' �����ǽǶȿ��ƻ�����ת�ٶȿ���
        ������µĺ���, ���ܸ� "vx","vy",and "wz" ��ֵ��Ҫ���ٶȲ���
    3.  ��"chassis_behaviour_mode_set"��������У�����µ��߼��жϣ���chassis_behaviour_mode��ֵ��CHASSIS_XXX_XXX
        �ں���������"else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)" ,Ȼ��ѡ��һ�ֵ��̿���ģʽ
        4��:
        CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'�ǽǶȿ��� ��̨�͵��̵���ԽǶ�
        �����������"xxx_angle_set"������'wz'
        CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'�ǽǶȿ��� ���̵������Ǽ�����ľ��ԽǶ�
        �����������"xxx_angle_set"
        CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'����ת�ٶȿ���
        CHASSIS_VECTOR_RAW : ʹ��'vx' 'vy' and 'wz'ֱ�����Լ�������ֵĵ���ֵ������ֵ��ֱ�ӷ��͵�can ������
    4.  ��"chassis_behaviour_control_set" ������������
        else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)
        {
            chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
        }
  */

#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"

#include "gimbal_behaviour.h"

/**
  * @brief          ������������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����Ϲʶ����趨ֵ������Ϊ0
  * @param[in]      vx_setǰ�����ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      vy_set���ҵ��ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      wz_set��ת���ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          ���̲��ƶ�����Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set��ת���ٶȣ���ת�ٶ��ǿ��Ƶ��̵ĵ��̽��ٶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          ���̸�����̨����Ϊ״̬���£�����ģʽ�Ǹ�����̨�Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      angle_set��������̨���Ƶ�����ԽǶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          ���̸������yaw����Ϊ״̬���£�����ģʽ�Ǹ�����̽Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      angle_set�������õ�yaw����Χ -PI��PI
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          ���̲�����Ƕȵ���Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�������ת�ٶ��ɲ���ֱ���趨
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set�������õ���ת�ٶ�,��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          ���̿�������Ϊ״̬���£�����ģʽ��rawԭ��״̬���ʶ��趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶȣ���ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set ��ת�ٶȣ� ��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
  * @param[in]      chassis_move_rc_to_vector��������
  */

static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

//highlight, the variable chassis behaviour mode
//���⣬���������Ϊģʽ����
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;

/**
  * @brief          ͨ���߼��жϣ���ֵ"chassis_behaviour_mode"������ģʽ
  * @param[in]      chassis_move_mode: ��������
  */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
  if (chassis_move_mode == NULL)
  {
    return;
  }

  //remote control  set chassis behaviour mode
  //ң��������ģʽ
  if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
  {
    //can change to CHASSIS_ZERO_FORCE,CHASSIS_NO_MOVE,CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,
    //CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW,CHASSIS_NO_FOLLOW_YAW,CHASSIS_OPEN
    chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
  }
  else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
  {
    chassis_behaviour_mode = CHASSIS_NO_MOVE;
  }
  else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
  {
    chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW; //00000000000000000
  }

  //when gimbal in some mode, such as init mode, chassis must's move
  //����̨��ĳЩģʽ�£����ʼ���� ���̲���
  if (gimbal_cmd_to_chassis_stop())
  {
    chassis_behaviour_mode = CHASSIS_NO_MOVE;
  }

  //add your own logic to enter the new mode
  //����Լ����߼��жϽ�����ģʽ

  //accord to beheviour mode, choose chassis control mode
  //������Ϊģʽѡ��һ�����̿���ģʽ
  if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
  {
    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
  }
  else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
  {
    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
  }
  else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
  {
    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
  }
  else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
  {
    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;
  }
  else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
  {
    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
  }
  else if (chassis_behaviour_mode == CHASSIS_OPEN)
  {
    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
  }
}

/**
  * @brief          ���ÿ�����.���ݲ�ͬ���̿���ģʽ��������������Ʋ�ͬ�˶�.������������棬����ò�ͬ�Ŀ��ƺ���.
  * @param[out]     vx_set, ͨ�����������ƶ�.
  * @param[out]     vy_set, ͨ�����ƺ����ƶ�.
  * @param[out]     wz_set, ͨ��������ת�˶�.
  * @param[in]      chassis_move_rc_to_vector,  ��������������Ϣ.
  */

void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

  if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
  {
    chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
  }
  else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
  {
    chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
  }
  else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
  {
    chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
  }
  else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
  {
    chassis_engineer_follow_chassis_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
  }
  else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
  {
    chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
  }
  else if (chassis_behaviour_mode == CHASSIS_OPEN)
  {
    chassis_open_set_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
  }
}

/**
  * @brief          ������������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����Ϲʶ����趨ֵ������Ϊ0
  * @param[in]      vx_setǰ�����ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      vy_set���ҵ��ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      wz_set��ת���ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  *vx_can_set = 0.0f;
  *vy_can_set = 0.0f;
  *wz_can_set = 0.0f;
}

/**
  * @brief          ���̲��ƶ�����Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set��ת���ٶȣ���ת�ٶ��ǿ��Ƶ��̵ĵ��̽��ٶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  *vx_set = 0.0f;
  *vy_set = 0.0f;
  *wz_set = 0.0f;
}

/**
  * @brief          ���̸�����̨����Ϊ״̬���£�����ģʽ�Ǹ�����̨�Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      angle_set��������̨���Ƶ�����ԽǶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  //channel value and keyboard value change to speed set-point, in general
  //ң������ͨ��ֵ�Լ����̰��� �ó� һ������µ��ٶ��趨ֵ
  chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

  //swing angle is generated by sin function, swing_time is the input time of sin
  //ҡ�ڽǶ�������sin�������ɣ�swing_time ��sin����������ֵ
  static fp32 swing_time = 0.0f;

  static fp32 swing_angle = 0.0f;
  //max_angle is the max angle that chassis will ratate
  //max_angle ��sin�����ķ�ֵ
  static fp32 max_angle = SWING_NO_MOVE_ANGLE;
  //swing_time  plus the add_time in one control cycle
  //swing_time ��һ�����������ڣ����� add_time
  static fp32 const add_time = PI * 0.5f * configTICK_RATE_HZ / CHASSIS_CONTROL_TIME_MS;

  static uint8_t swing_flag = 0;

  //judge if swing
  //�ж��Ƿ�Ҫҡ��
  if (chassis_move_rc_to_vector->chassis_RC->key.v & SWING_KEY)
  {
    if (swing_flag == 0)
    {
      swing_flag = 1;
      swing_time = 0.0f;
    }
  }
  else
  {
    swing_flag = 0;
  }

  //judge if keyboard is controlling the chassis, if yes, reduce the max_angle
  //�жϼ��������ǲ����ڿ��Ƶ����˶����������˶���Сҡ�ڽǶ�
  if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY ||
      chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
  {
    max_angle = SWING_MOVE_ANGLE;
  }
  else
  {
    max_angle = SWING_NO_MOVE_ANGLE;
  }

  if (swing_flag)
  {
    swing_angle = max_angle * arm_sin_f32(swing_time);
    swing_time += add_time;
  }
  else
  {
    swing_angle = 0.0f;
  }
  //swing_time  range [0, 2*PI]
  //sin����������2pi
  if (swing_time > 2 * PI)
  {
    swing_time -= 2 * PI;
  }

  *angle_set = swing_angle;
}

/**
  * @brief          ���̸������yaw����Ϊ״̬���£�����ģʽ�Ǹ�����̽Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      angle_set�������õ�yaw����Χ -PI��PI
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

  *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set - CHASSIS_ANGLE_Z_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL]);
}

/**
  * @brief          ���̲�����Ƕȵ���Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�������ת�ٶ��ɲ���ֱ���趨
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set�������õ���ת�ٶ�,��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
  *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}

/**
  * @brief          ���̿�������Ϊ״̬���£�����ģʽ��rawԭ��״̬���ʶ��趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶȣ���ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set ��ת�ٶȣ� ��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
  * @param[in]      chassis_move_rc_to_vector��������
  */

static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  *vx_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  *vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  *wz_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  return;
}
