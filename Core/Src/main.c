/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define offSetValue		817
#define spiToRad   		0.00038352f
#define spiToDeg   		0.02197f
#define encoderRad 		0.00421845f
#define encoderDeg 		0.2416992f
#define toVolt     		0.00080566f
#define toAmp      		0.00048828f
#define twoPI		 	6.283185f
#define stepTwoPI		5.969026f
#define encoderConst 	0.000061035f
#define toRads 	   		0.383519f
#define VFstep			0.314159f // 2PI/20 - 1000Hz
#define slopeOme		0.03f
#define slopePos		0.003f


/* Filter constants-----------------------------------------------------------*/
// LPF filter feedback, cut-off frequency	[450 Hz]
#define a1_lpf 			-0.86776f
#define b1_lpf 			0.06612f
#define b2_lpf 			0.06612f

// LPF filter PLL, cut-off frequency	[50 Hz]
#define a1_lpf_pll 		-0.98441f
#define b1_lpf_pll 		0.00779f
#define b2_lpf_pll 		0.00779f

// BPF filter edge frequencies		[900 Hz, 1100 Hz]
#define a1_bpf_pll 		-1.84507f
#define a2_bpf_pll 		0.93906f
#define b1_bpf_pll 		0.03047f
#define b2_bpf_pll 		0.00000f
#define b3_bpf_pll 		-0.03047f

/* Controllers constants ------------------------------------------------------*/
#define Ki_pll 			0.00027f
#define Kp_pll 			0.07f

#define K_d				4.29398897f
#define Ki_d			0.45000000f
#define K_q				4.29398897f
#define Ki_q			0.45000000f

#define Ki_omega 		0.0616f
#define Kp_omega 		17.6f

#define Kp_fi 			10.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint16_t angle = 0;

// values in mA

// setpoint mA
float SP_id = 0.0f;
float SP_iq = 0.0f;

// measured mA
float PV_id = 0.0f;
float PV_iq = 0.0f;

// calculated mV
float ud = 0.0f;
float uq = 0.0f;

float alpha = 0.0f;
float beta = 0.0f;

// hodnoty v mV
float uA = 0.0f;
float uB = 0.0f;
float uC = 0.0f;

// pi d
float e_d = 0.0f;
float sum_d = 0.0f;

// pi q
float e_q = 0.0f;
float sum_q = 0.0f;

// p speed --------------------------------------
float sum_omega = 0.0f;
float e_omega = 0.0f;
float SP_omega = 0.0f;

// setpoint speed
float SP_speed = 0.0f;
float PV_speed = 0.0f;

// pi position --------------------------------
float K_position = 10.00f;
float Ki_position = 0.15f;
float e_position = 0.0f;
float sum_position = 0.0f;

float PV_position = 0.0f;
float PV_position_encod = 0.0f;

// setpoint position
float SP_position, SP_position_k, SP_k_position, SP_kk_position = 0.0f;



uint16_t spiRxBuffer;
uint16_t measureI[3] = {0,0,0}; 	// iA, iB, iC (ADC)
float measA, measB, measC;	    // iA, iB, iC (mA)

int16_t encoder = 0, encoderBefore = 0;
int16_t diff = 0, diffCounter = 0;

uint8_t counter = 0;

float angleRad, cosine = 1.0f, sine = 0.0f, encod;

float measurement[8000];
uint16_t pointer = 0;


// PI
float ypi = 0.0f;

float omega = 0.0f, PV_omega;
float fi, fim, fik = 0.0f;

uint32_t state = 0;

float SP_omega_k = 0.0f;;



// LPF - improved frequency - integration
float ylpf, ylpfk, ulpfk = 0.0f;

// LPF filtr feedback current
float yd,ydk,udk = 0.0f;
float yq,yqk,uqk = 0.0f;

// BPF extract usefull value from iq
float ybpf,ybpfk,ybpfkk,ubpfk,ubpfkk = 0.0f;

float VFsin = 0.0f, VFcos = 0.0f, VFcounter=0.0f;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// 1ms period
	if (htim->Instance == TIM2)
	{
		PV_speed = (float) diffCounter*toRads; // rad/s
		PV_position_encod = PV_position_encod + ((float) (diffCounter)*toRads)*0.001;
		diffCounter = 0;

//		if (SP_omega > SP_omega_k)
//		{
//			SP_omega_k = SP_omega_k + slopeOme;
//			if (SP_omega_k > SP_omega)
//			{
//				SP_omega_k = SP_omega;
//			}
//		}
//		else if (SP_omega < SP_omega_k)
//		{
//			SP_omega_k = SP_omega_k - slopeOme;
//			if (SP_omega_k < SP_omega)
//			{
//				SP_omega_k = SP_omega;
//			}
//		}

//		SP_omega_k = SP_omega;

//		 PI speed
		e_omega = SP_omega - PV_omega;
		sum_omega = Ki_omega*e_omega + sum_omega;

		if (sum_omega > 500.0f)
			sum_omega = 500.0f;
		else if (sum_omega < -500.0f)
			sum_omega = -500.0f;

		SP_iq = Kp_omega*e_omega + sum_omega - Kp_omega*0.1f*PV_omega;

//
		 //PI position
		PV_position = fim*0.090909f;

		// filtrace
//		SP_position_k = 0.99f*SP_k_position + 0.01f*SP_kk_position;
//		SP_kk_position = SP_position;
//		SP_k_position = SP_position_k;

//		if(state == 1)
//		{
//			if (SP_position > SP_position_k)
//			{
//				SP_position_k = SP_position_k + slopePos;
//				if (SP_position_k > SP_position)
//				{
//					SP_position_k = SP_position;
//				}
//			}
//			else if (SP_position < SP_position_k)
//			{
//				SP_position_k = SP_position_k - slopePos;
//				if (SP_position_k < SP_position)
//				{
//					SP_position_k = SP_position;
//				}
//			}
//		}

		e_position = SP_position - PV_position;

		SP_omega = Kp_fi*e_position;


		if(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13))
		  {
			  if (pointer > 10)
			  {
				  SP_position = 1.0f;
//				  SP_omega = 15.0f;
//				  SP_id = 0.0f;
//				  SP_iq = 200.0f;

			  }
			  measurement[pointer]        = yq;
			  measurement[(1000+pointer)] = SP_omega;
			  measurement[(2000+pointer)] = omega;
			  measurement[(3000+pointer)] = SP_iq;
			  measurement[(4000+pointer)] = PV_speed;
			  measurement[(5000+pointer)] = PV_omega;
			  measurement[(6000+pointer)] = angleRad;
			  measurement[(7000+pointer)] = fi;

			  if (pointer < 1000)
				  pointer++;

		  } else {
				  SP_position = -1.0f;
//			  SP_omega = 0.0f;
//			  SP_iq = 0.0f;
//			  sum_omega = 0.0f;
//			  omega= 0.0f;
//			  fi = 0.0f;
//			  ybpf = 0.0f;
//			  ylpf = 0.0f;
//			  pointer = 0;
		  }


	}
}

// 50us period
void HAL_ADCEx_InjectedConvCpltCallback (ADC_HandleTypeDef * hadc)
{
	if (state == 1)
	 {
		HAL_GPIO_WritePin_Fast(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		//
		  measureI[0] = hadc1.Instance->JDR1; // (HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1));
		  measureI[1] = hadc1.Instance->JDR2; // (HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2));

		  __HAL_SPI_ENABLE(&hspi3);
		  HAL_GPIO_WritePin_Fast(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

		  hspi3.Instance->DR = 0;

		  while ((hspi3.Instance->SR & SPI_FLAG_RXNE) == 0){}  	//Wait for Data Ready to Read
		  spiRxBuffer = hspi3.Instance->DR;						//Read Data Register Directly

		  HAL_GPIO_WritePin_Fast(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
		  __HAL_SPI_DISABLE(&hspi3);


		  spiRxBuffer &= 0x3FFE;
		  encoder = (0x3FFF - spiRxBuffer) - offSetValue;

		  if (encoder > 16383)
			  encoder = encoder - 16383;
		  else if (encoder < 0)
			  encoder = 16383 + encoder;

		  // modulo % 2PI - 4 us
		  encod = encoder*encoderConst*11.0f; // x / 16384 * 11 (polpares) (0.0 - 11.0)
		  angleRad = (encod - (int) encod) * twoPI;	// ((0.0 - 0.99) * 2PI

//		   2 us
//		  cosine = arm_cos_f32(angleRad);
//		  sine = arm_sin_f32(angleRad);

		  VFcos = arm_cos_f32(VFcounter)*2000.0f;
		  VFsin = arm_sin_f32(VFcounter);

		  VFcounter = VFcounter + VFstep;
		  if (VFcounter > stepTwoPI)
				  VFcounter = 0;

		  // mechanical speed of rotor
			  diff = encoder - encoderBefore;

			  if (diff < -12000)
			  {
				  diff = 16383 - encoderBefore + encoder;
			  }
			  else if (diff > 12000)
			  {
				  diff = encoder - 16383 - encoderBefore;
			  }

				  diffCounter = diffCounter + diff;
				  encoderBefore = encoder;


		  measA = ((1.5957f* (float) measureI[0])-3089.72f); 	// iA (mA)
		  measB = ((1.5957f* (float) measureI[1])-3089.72f); 	// iB (mA)
		  measC = -measA-measB;								// iC (mA) calculated

		  //Direct transformations
		  alpha = 0.333f * (2*measA - measB - measC);
		  beta  = 0.577f * (measB - measC);

		  PV_id = cosine*alpha + sine*beta;
		  PV_iq = -sine*alpha + cosine*beta;

//		  yq = PV_iq;
		  yq = -a1_lpf*yqk  + b1_lpf*PV_iq + b2_lpf*uqk;
		  yqk = yq;
		  uqk = PV_iq;

		  yd = -a1_lpf*ydk  + b1_lpf*PV_id + b2_lpf*udk;
		  ydk = yd;
		  udk = PV_id;


		  ybpf = -a1_bpf_pll*ybpfk -a2_bpf_pll*ybpfkk + b1_bpf_pll*yq + b2_bpf_pll*ubpfk + b3_bpf_pll*ubpfkk;
		  ybpfk = ybpf;
		  ybpfkk = ybpfk;
		  ubpfk = yq;
		  ubpfkk = ubpfk;

		  ybpf = ybpf * VFsin;

		  ylpf = -a1_lpf_pll*ylpfk + b1_lpf_pll*ybpf + b2_lpf_pll*ulpfk;
		  ylpfk = ylpf;
		  ulpfk = ybpf;


		  // PI controller of pll
		  ypi = Kp_pll*ylpf + omega;
		  omega = omega + Ki_pll*ylpf;

		  if (omega > 0.01f)
			  omega = 0.01f;
		  else if (omega < -0.01f)
			  omega = -0.01f;

		  fi = fi+ypi;
		  fim = fim+ypi;

		  PV_omega = omega*2000.0f;

		  if (fi > twoPI)
			  fi = 0.0f;
		  else if (fi < 0)
			  fi = twoPI;

		  cosine = arm_cos_f32(fi);
		  sine = arm_sin_f32(fi);


		  //PI current
		  e_d = SP_id - yd;
		  e_q = SP_iq - yq;

		  // anti wind-up
		  sum_d = sum_d + e_d*Ki_d;
		  if (sum_d > 10200.0f)
			  sum_d = 10200.0f;
		  else if(sum_d < -10200.0f)
			  sum_d = -10200.0f;

		  sum_q = sum_q + e_q*Ki_q;
		  if (sum_q > 10200.0f)
				sum_q = 10200.0f;
		  else if (sum_q < -10200.0f)
		  		sum_q = -10200.0f;

		  ud = (K_d*e_d + sum_d)+VFcos;
		  uq = K_q*e_q + sum_q;



	      //Inverse transformation
		  alpha = cosine*ud - sine*uq;
		  beta  = sine*ud + cosine*uq;

		  // hodnoty z mV na V
		  uA = alpha;

		  if(uA < -10200.0f)
			  uA = -10200.0f;
		  else if (uA > 10200.0f)
			  uA = 10200.0f;

		  uB = (0.866f*beta - 0.5f*alpha);

		  if(uB < -10200.0f)
				  uB = -10200.0f;
		  else if (uB > 10200.0f)
				  uB = 10200.0f;

		  uC = (-0.866f*beta - 0.5f*alpha);

		  if(uC < -10200.0f)
				  uC = -10200.0f;
		  else if (uC > 10200.0f)
				  uC = 10200.0f;

		  htim1.Instance->CCR1 = (uint32_t) (uA*0.08333f+1000.0f);
		  htim1.Instance->CCR2 = (uint32_t) (uB*0.08333f+1000.0f);
		  htim1.Instance->CCR3 = (uint32_t) (uC*0.08333f+1000.0f);

		//	  htim1.Instance->CCR1 = 1599;
		//	  htim1.Instance->CCR2 = 699;
		//	  htim1.Instance->CCR3 = 699;

		//
		  HAL_GPIO_WritePin_Fast(GPIOB, GPIO_PIN_7, GPIO_PIN_SET) ;
	 }
 }



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
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
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  if(HAL_OK != HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED))
	  Error_Handler();

  if(HAL_OK != HAL_ADCEx_InjectedStart_IT(&hadc1))
	  Error_Handler();

  if(HAL_OK != HAL_TIM_Base_Start(&htim1))
 	  Error_Handler();

  if(HAL_OK != HAL_TIM_Base_Start_IT(&htim2))
   	  Error_Handler();


  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  // initial zeroing
  if(state == 0)
  {
	  ypi = 0.0f;
	  htim1.Instance->CCR1 = 1599;
	  htim1.Instance->CCR2 = 699;
	  htim1.Instance->CCR3 = 699;

	  HAL_Delay(2000);
	  state = 1;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Disable Injected Queue
  */
  HAL_ADCEx_DisableInjectedQueue(&hadc1);
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_6;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_16;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim1.Init.Period = 2000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 899;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 50;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_FAULT_GPIO_Port, EN_FAULT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STAND_BY_GPIO_Port, STAND_BY_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin EN_FAULT_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|EN_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STAND_BY_Pin PB7 */
  GPIO_InitStruct.Pin = STAND_BY_Pin|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_WritePin_Fast(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    if(PinState != GPIO_PIN_RESET)
    	GPIOx->BSRR = GPIO_Pin;
    else
        GPIOx->BSRR = (uint32_t)GPIO_Pin << 16U;
}

__weak void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /* Add a period to guaranty minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
    wait++;
  }

  while((HAL_GetTick() - tickstart) < wait)
  {
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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
