/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  *             ��Ҫ����������bmi088��������ist8310�������̬���㣬�ó�ŷ���ǣ�
  *             �ṩͨ��bmi088��data ready �ж�����ⲿ�������������ݵȴ��ӳ�
  *             ͨ��DMA��SPI�����ԼCPUʱ��.

  */

#include "INS_task.h"

#include "main.h"

#include "cmsis_os.h"

#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "pid.h"
#include "ahrs.h"

#include "calibrate_task.h"
#include "detect_task.h"

#define IMU_temp_PWM(pwm) imu_pwm_set(pwm) //pwm����

#define BMI088_BOARD_INSTALL_SPIN_MATRIX \
  {0.0f, 1.0f, 0.0f},                    \
      {-1.0f, 0.0f, 0.0f},               \
  {                                      \
    0.0f, 0.0f, 1.0f                     \
  }

#define IST8310_BOARD_INSTALL_SPIN_MATRIX \
  {1.0f, 0.0f, 0.0f},                     \
      {0.0f, 1.0f, 0.0f},                 \
  {                                       \
    0.0f, 0.0f, 1.0f                      \
  }

/**
  * @brief          ��ת������,���ٶȼƺʹ�����,��������Ư,��Ϊ�豸�в�ͬ��װ��ʽ
  * @param[out]     gyro: ������Ư����ת
  * @param[out]     accel: ������Ư����ת
  * @param[out]     mag: ������Ư����ת
  * @param[in]      bmi088: �����Ǻͼ��ٶȼ�����
  * @param[in]      ist8310: ����������
  */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);

/**
  * @brief          ����bmi088���¶�
  * @param[in]      temp:bmi088���¶�
  */
static void imu_temp_control(fp32 temp);

/**
  * @brief          ����imu_update_flag��ֵ����SPI DMA
  * @param[in]      temp:bmi088���¶�
  */
static void imu_cmd_spi_dma(void);

extern SPI_HandleTypeDef hspi1;

static TaskHandle_t INS_task_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};

volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;

bmi088_real_data_t bmi088_real_data;
fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];

ist8310_real_data_t ist8310_real_data;
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
fp32 mag_cali_offset[3];

static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_type_def imu_temp_pid;

static const float timing_time = 0.001f; //tast run time , unit s.�������е�ʱ�� ��λ s

//���ٶȼƵ�ͨ�˲�
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f}; //euler angle, unit rad.ŷ���� ��λ rad

/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  */
/**
  * @brief          imu����, ��ʼ�� bmi088, ist8310, ����ŷ����
  * @param[in]      pvParameters: NULL
  */

void INS_task(void const *pvParameters)
{
  //wait a time
  osDelay(INS_TASK_INIT_TIME);
  while (BMI088_init())
  {
    osDelay(100);
  }
  while (ist8310_init())
  {
    osDelay(100);
  }

  BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
  //rotate and zero drift
  imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

  PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
  AHRS_init(INS_quat, INS_accel, INS_mag);

  accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
  accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
  accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
  //get the handle of task
  //��ȡ��ǰ�������������
  INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

  //set spi frequency
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

  SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

  imu_start_dma_flag = 1;

  while (1)
  {
    //wait spi DMA tansmit done
    //�ȴ�SPI DMA����
    while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
    {
    }

    if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
      gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
      BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
    }

    if (accel_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
      accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
      BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
    }

    if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
      accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
      BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
      imu_temp_control(bmi088_real_data.temp);
    }

    //rotate and zero drift
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

    //���ٶȼƵ�ͨ�˲�
    //accel low-pass filter
    accel_fliter_1[0] = accel_fliter_2[0];
    accel_fliter_2[0] = accel_fliter_3[0];

    accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

    accel_fliter_1[1] = accel_fliter_2[1];
    accel_fliter_2[1] = accel_fliter_3[1];

    accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

    accel_fliter_1[2] = accel_fliter_2[2];
    accel_fliter_2[2] = accel_fliter_3[2];

    accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

    AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag);
    get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);

    //because no use ist8310 and save time, no use
    if (mag_update_flag &= 1 << IMU_DR_SHFITS)
    {
      mag_update_flag &= ~(1 << IMU_DR_SHFITS);
      mag_update_flag |= (1 << IMU_SPI_SHFITS);
      //            ist8310_read_mag(ist8310_real_data.mag);
    }
  }
}

/**
  * @brief          ��ת������,���ٶȼƺʹ�����,��������Ư,��Ϊ�豸�в�ͬ��װ��ʽ
  * @param[out]     gyro: ������Ư����ת
  * @param[out]     accel: ������Ư����ת
  * @param[out]     mag: ������Ư����ת
  * @param[in]      bmi088: �����Ǻͼ��ٶȼ�����
  * @param[in]      ist8310: ����������
  */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
  for (uint8_t i = 0; i < 3; i++)
  {
    gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
    accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
    mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
  }
}

/**
  * @brief          ����bmi088���¶�
  * @param[in]      temp:bmi088���¶�
  */
static void imu_temp_control(fp32 temp)
{
  uint16_t tempPWM;
  static uint8_t temp_constant_time = 0;
  if (first_temperate)
  {
    PID_calc(&imu_temp_pid, temp, get_control_temperature());
    if (imu_temp_pid.out < 0.0f)
    {
      imu_temp_pid.out = 0.0f;
    }
    tempPWM = (uint16_t)imu_temp_pid.out;
    IMU_temp_PWM(tempPWM);
  }
  else
  {
    //��û�дﵽ���õ��¶ȣ�һֱ����ʼ���
    //in beginning, max power
    if (temp > get_control_temperature())
    {
      temp_constant_time++;
      if (temp_constant_time > 200)
      {
        //�ﵽ�����¶ȣ�������������Ϊһ������ʣ���������
        //
        first_temperate = 1;
        imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
      }
    }

    IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
  }
}

/**
  * @brief          ������������Ư
  * @param[out]     gyro_offset:������Ư
  * @param[in]      gyro:���ٶ�����
  * @param[out]     offset_time_count: �Զ���1
  */
void gyro_offset_calc(fp32 gyro_offset[3], fp32 gyro[3], uint16_t *offset_time_count)
{
  if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
  {
    return;
  }

  gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
  gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
  gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
  (*offset_time_count)++;
}

/**
  * @brief          У׼������
  * @param[out]     �����ǵı������ӣ�1.0fΪĬ��ֵ�����޸�
  * @param[out]     �����ǵ���Ư���ɼ������ǵľ�ֹ�������Ϊoffset
  * @param[out]     �����ǵ�ʱ�̣�ÿ����gyro_offset���û��1,
  */
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
  if (*time_count == 0)
  {
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
  }
  gyro_offset_calc(gyro_offset, INS_gyro, time_count);

  cali_offset[0] = gyro_offset[0];
  cali_offset[1] = gyro_offset[1];
  cali_offset[2] = gyro_offset[2];
  cali_scale[0] = 1.0f;
  cali_scale[1] = 1.0f;
  cali_scale[2] = 1.0f;
}

/**
  * @brief          У׼���������ã�����flash���������ط�����У׼ֵ
  * @param[in]      �����ǵı������ӣ�1.0fΪĬ��ֵ�����޸�
  * @param[in]      �����ǵ���Ư
  */
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
  gyro_cali_offset[0] = cali_offset[0];
  gyro_cali_offset[1] = cali_offset[1];
  gyro_cali_offset[2] = cali_offset[2];
  gyro_offset[0] = gyro_cali_offset[0];
  gyro_offset[1] = gyro_cali_offset[1];
  gyro_offset[2] = gyro_cali_offset[2];
}

/**
  * @brief          ��ȡ��Ԫ��
  * @param[in]      none
  * @retval         INS_quat��ָ��
  */
const fp32 *get_INS_quat_point(void)
{
  return INS_quat;
}

/**
  * @brief          ��ȡŷ����, 0:yaw, 1:pitch, 2:roll ��λ rad
  * @param[in]      none
  * @retval         INS_angle��ָ��
  */
const fp32 *get_INS_angle_point(void)
{
  return INS_angle;
}

/**
  * @brief          ��ȡ���ٶ�,0:x��, 1:y��, 2:roll�� ��λ rad/s
  * @param[in]      none
  * @retval         INS_gyro��ָ��
  */
extern const fp32 *get_gyro_data_point(void)
{
  return INS_gyro;
}

/**
  * @brief          ��ȡ���ٶ�,0:x��, 1:y��, 2:roll�� ��λ m/s2
  * @param[in]      none
  * @retval         INS_accel��ָ��
  */
extern const fp32 *get_accel_data_point(void)
{
  return INS_accel;
}

/**
  * @brief          ��ȡ���ٶ�,0:x��, 1:y��, 2:roll�� ��λ ut
  * @param[in]      none
  * @retval         INS_mag��ָ��
  */
extern const fp32 *get_mag_data_point(void)
{
  return INS_mag;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == INT1_ACCEL_Pin)
  {
    detect_hook(BOARD_ACCEL_TOE);
    accel_update_flag |= 1 << IMU_DR_SHFITS;
    accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
    if (imu_start_dma_flag)
    {
      imu_cmd_spi_dma();
    }
  }
  else if (GPIO_Pin == INT1_GYRO_Pin)
  {
    detect_hook(BOARD_GYRO_TOE);
    gyro_update_flag |= 1 << IMU_DR_SHFITS;
    if (imu_start_dma_flag)
    {
      imu_cmd_spi_dma();
    }
  }
  else if (GPIO_Pin == DRDY_IST8310_Pin)
  {
    detect_hook(BOARD_MAG_TOE);
    mag_update_flag |= 1 << IMU_DR_SHFITS;
  }
  else if (GPIO_Pin == GPIO_PIN_0)
  {

    //wake up the task
    //��������
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
      static BaseType_t xHigherPriorityTaskWoken;
      vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}

/**
  * @brief          ����imu_update_flag��ֵ����SPI DMA
  * @param[in]      temp:bmi088���¶�
  */
static void imu_cmd_spi_dma(void)
{

  //���������ǵ�DMA����
  if ((gyro_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
  {
    gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
    gyro_update_flag |= (1 << IMU_SPI_SHFITS);

    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
    SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
    return;
  }
  //�������ٶȼƵ�DMA����
  if ((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
  {
    accel_update_flag &= ~(1 << IMU_DR_SHFITS);
    accel_update_flag |= (1 << IMU_SPI_SHFITS);

    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
    SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
    return;
  }

  if ((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
  {
    accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
    accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
    SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
    return;
  }
}

void DMA2_Stream2_IRQHandler(void)
{

  if (DMA2->LISR & (1 << 21))
  {
    DMA2->LIFCR = (uint32_t)((DMA_LISR_FEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_TEIF0 | DMA_LISR_HTIF0 | DMA_LISR_TCIF0) << 21);

    //gyro read over
    //�����Ƕ�ȡ���
    if (gyro_update_flag & (1 << IMU_SPI_SHFITS))
    {
      gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
      gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

      HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
    }

    //accel read over
    //���ٶȼƶ�ȡ���
    if (accel_update_flag & (1 << IMU_SPI_SHFITS))
    {
      accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
      accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

      HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
    }
    //temperature read over
    //�¶ȶ�ȡ���
    if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
    {
      accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
      accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

      HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
    }

    imu_cmd_spi_dma();

    if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
      __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
    }
  }
}
