/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    // Parámetros Físicos
    float masa, gravedad, f_aero, tau_motor;
    float inercia_xx, inercia_yy, inercia_zz;
    float alpha, beta;

    // Estado Real (Traslación y Rotación)
    float x, y, z;
    float vx, vy, vz;
    float phi, theta, psi;
    float p, q, r;

    // Sensores y Referencias
    float z_medida, z_filtrada;
    float x_ref, y_ref, z_ref;

    // Salidas PID y Motores
    float theta_deseado, phi_deseado;
    float U1_real, U1_deseado, U2, U3, U4;
    float angulo_max;

    // Memoria PID Z
    float kp_z, ki_z, kd_z;
    float err_z_acum, err_z_prev;

    // Memoria PID XY
    float kp_xy, ki_xy, kd_xy;
    float err_x_acum, err_x_prev;
    float err_y_acum, err_y_prev;

    // Memoria PID Actitud
    float kp_att, ki_att, kd_att;
    float err_phi_acum, err_phi_prev;
    float err_theta_acum, err_theta_prev;
    float err_psi_acum, err_psi_prev;
} Drone_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
Drone_t drone;
volatile uint8_t received_command = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */
  drone.masa = 1.2f;
  drone.gravedad = 9.81f;
  drone.inercia_xx = 0.01f;
  drone.inercia_yy = 0.01f;
  drone.inercia_zz = 0.01f;
  drone.f_aero = 0.1f;
  drone.tau_motor = 0.05f;

  float dt_nav = 0.01f;
  drone.alpha = dt_nav / (drone.tau_motor + dt_nav);
  drone.beta = 0.1f;

  drone.z = 10.0f;
  drone.z_filtrada = 10.0f;
  drone.U1_real = drone.masa * drone.gravedad;

  drone.x_ref = 5.0f;
  drone.y_ref = 5.0f;
  drone.z_ref = 5.0f;
  drone.angulo_max = 0.3f;

  drone.kp_z = 6.0f;    drone.ki_z = 0.0f;    drone.kd_z = 5.5f;
  drone.kp_xy = 0.04f;  drone.ki_xy = 0.0f;   drone.kd_xy = 0.4f;
  drone.kp_att = 2.5f;  drone.ki_att = 0.0f;  drone.kd_att = 0.3f;

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// RECEPCIÓN DE COMANDO POR USB DESDE EL PC
	if (received_command != 0) {
	  switch(received_command) {
	    case 'w': case 'W': drone.x_ref += 2.0f; break; // Adelante
	    case 's': case 'S': drone.x_ref -= 2.0f; break; // Atrás
	    case 'a': case 'A': drone.y_ref += 2.0f; break; // Izquierda
	    case 'd': case 'D': drone.y_ref -= 2.0f; break; // Derecha
	    case 'r': case 'R': drone.z_ref += 2.0f; break; // Subir (Elevar)
	    case 'f': case 'F': drone.z_ref -= 2.0f; break; // Bajar (Frenar)
	  }
	  received_command = 0;
	}

    // ENVIO DE INFORMACIÓN POR USB AL PC
	// Creamos buffer para guardar el mensaje
	char tx_buffer[64];
	// Formateamos las variables float a texto con 2 decimales
	uint16_t len = snprintf(tx_buffer, sizeof(tx_buffer), "X:%.2f, Y:%.2f, Z:%.2f\r\n", drone.x, drone.y, drone.z);
	// Enviamos el mensaje por el USB
	CDC_Transmit_FS((uint8_t*)tx_buffer, len);
	// Pausa de 50 ms para no saturar el bus USB ni colgar el PC
	HAL_Delay(50);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 169;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  if (htim->Instance == TIM6) {

    float dt_att = 0.001f; // 1 ms

	// LECTURA DE SENSORES SIMULADOS Y FILTRADO
	// Ruido blanco entre -0.2 y 0.2
	float ruido = (((float)rand() / (float)RAND_MAX) * 0.4f) - 0.2f;
	drone.z_medida = drone.z + ruido;
	drone.z_filtrada = (1.0f - drone.beta) * drone.z_filtrada + drone.beta * drone.z_medida;

	// LAZO EXTERNO: NAVEGACIÓN
	static uint8_t div_nav = 0;
	if (++div_nav >= 10) {
      div_nav = 0;
	  float dt_nav = 0.01f;

	  // PID Altitud (Z)
	  float err_z = drone.z_ref - drone.z_filtrada;
	  drone.err_z_acum += err_z * dt_nav;
	  float vel_err_z = (err_z - drone.err_z_prev) / dt_nav;
	  float fuerza_pid = drone.kp_z * err_z + drone.err_z_acum * drone.ki_z + vel_err_z * drone.kd_z;
	  drone.err_z_prev = err_z;
	  drone.U1_deseado = (drone.masa * drone.gravedad) + fuerza_pid;

	  // PID Navegación X -> Theta (Pitch)
	  float err_x = drone.x_ref - drone.x;
	  drone.err_x_acum += err_x * dt_nav;
	  float vel_err_x = (err_x - drone.err_x_prev) / dt_nav;
	  drone.theta_deseado = drone.kp_xy * err_x + drone.err_x_acum * drone.ki_xy + vel_err_x * drone.kd_xy;
	  drone.err_x_prev = err_x;

	  // PID Navegación Y -> Phi (Roll)
	  float err_y = drone.y_ref - drone.y;
	  drone.err_y_acum += err_y * dt_nav;
	  float vel_err_y = (err_y - drone.err_y_prev) / dt_nav;
	  drone.phi_deseado = -(drone.kp_xy * err_y + drone.err_y_acum * drone.ki_xy + vel_err_y * drone.kd_xy);
	  drone.err_y_prev = err_y;

	  // Saturación de Seguridad
	  if(drone.theta_deseado > drone.angulo_max) drone.theta_deseado = drone.angulo_max;
	  else if(drone.theta_deseado < -drone.angulo_max) drone.theta_deseado = -drone.angulo_max;

	  if(drone.phi_deseado > drone.angulo_max) drone.phi_deseado = drone.angulo_max;
	  else if(drone.phi_deseado < -drone.angulo_max) drone.phi_deseado = -drone.angulo_max;

	  // Límite de Motores
	  if (drone.U1_deseado <= 0.0f) drone.U1_deseado = 0.0f;
	  else if (drone.U1_deseado > 3.0f * drone.masa * drone.gravedad) drone.U1_deseado = 3.0f * drone.masa * drone.gravedad;
	}

	// RETARDO FÍSICO DE LOS MOTORES
	drone.U1_real += drone.alpha * (drone.U1_deseado - drone.U1_real);

	// LAZO INTERNO: ACTITUD
	// PID Roll
	float err_phi = drone.phi_deseado - drone.phi;
	drone.err_phi_acum += err_phi * dt_att;
	float vel_err_phi = (err_phi - drone.err_phi_prev) / dt_att;
	drone.U2 = drone.kp_att * err_phi + drone.err_phi_acum * drone.ki_att + vel_err_phi * drone.kd_att;
	drone.err_phi_prev = err_phi;

	// PID Pitch
	float err_theta = drone.theta_deseado - drone.theta;
	drone.err_theta_acum += err_theta * dt_att;
	float vel_err_theta = (err_theta - drone.err_theta_prev) / dt_att;
	drone.U3 = drone.kp_att * err_theta + drone.err_theta_acum * drone.ki_att + vel_err_theta * drone.kd_att;
	drone.err_theta_prev = err_theta;

	// PID Yaw
	float err_psi = 0.0f - drone.psi;
	drone.err_psi_acum += err_psi * dt_att;
	float vel_err_psi = (err_psi - drone.err_psi_prev) / dt_att;
	drone.U4 = drone.kp_att * err_psi + drone.err_psi_acum * drone.ki_att + vel_err_psi * drone.kd_att;
	drone.err_psi_prev = err_psi;

	// FÍSICA NEWTONIANA
	float dp = (drone.U2 + (drone.inercia_yy - drone.inercia_zz) * drone.q * drone.r) / drone.inercia_xx;
	float dq = (drone.U3 + (drone.inercia_zz - drone.inercia_xx) * drone.p * drone.r) / drone.inercia_yy;
	float dr = (drone.U4 + (drone.inercia_xx - drone.inercia_yy) * drone.p * drone.q) / drone.inercia_zz;

	float dphi = drone.p + drone.q * sinf(drone.phi) * tanf(drone.theta) + drone.r * cosf(drone.phi) * tanf(drone.theta);
	float dtheta = drone.q * cosf(drone.phi) - drone.r * sinf(drone.phi);
	float dpsi = drone.q * sinf(drone.phi) / cosf(drone.theta) + drone.r * cosf(drone.phi) / cosf(drone.theta);

	float ax = (drone.U1_real / drone.masa) * (cosf(drone.phi) * sinf(drone.theta) * cosf(drone.psi) + sinf(drone.phi) * sinf(drone.psi)) - (drone.f_aero / drone.masa) * drone.vx;
	float ay = (drone.U1_real / drone.masa) * (cosf(drone.phi) * sinf(drone.theta) * sinf(drone.psi) - sinf(drone.phi) * cosf(drone.psi)) - (drone.f_aero / drone.masa) * drone.vy;
	float az = (drone.U1_real / drone.masa) * (cosf(drone.phi) * cosf(drone.theta)) - drone.gravedad - (drone.f_aero / drone.masa) * drone.vz;

	// Integración Numérica (Euler)
	drone.vx += ax * dt_att;
	drone.vy += ay * dt_att;
	drone.vz += az * dt_att;

	drone.x += drone.vx * dt_att;
	drone.y += drone.vy * dt_att;
	drone.z += drone.vz * dt_att;

	// Choque contra el suelo
	if (drone.z <= 0.0f) {
	  drone.z = 0.0f;
	  drone.vz = 0.0f;
	}

	drone.p += dp * dt_att;
	drone.q += dq * dt_att;
	drone.r += dr * dt_att;

	drone.phi += dphi * dt_att;
	drone.theta += dtheta * dt_att;
	drone.psi += dpsi * dt_att;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
