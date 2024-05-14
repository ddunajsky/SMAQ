/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "string.h"
#include "cmsis_os.h"
#include "mongoose.h"


/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2004c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2004c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2004c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2004c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif
#define HTTP_URL "http://0.0.0.0:8888"
#define HTTPS_URL "https://0.0.0.0:8443"
#define BLINK_PERIOD_MS 1000  // LED blinking period in millis

#define SCD40_ADDR 0x62
#define SCD40_READ (SCD40_ADDR << 1)
#define SCD40_WRITE ((SCD40_ADDR << 1) | 0x01)



ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

RNG_HandleTypeDef hrng;

UART_HandleTypeDef huart3;

I2C_HandleTypeDef hi2c1;

osThreadId_t ServerHandle;
const osThreadAttr_t Server_attributes = {
  .name = "Server",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t SCDHandle;
const osThreadAttr_t SCD_attributes = {
  .name = "Sensor 1",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
//static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_RNG_Init(void);
void server(void *argument);
//void blinker(void *argument);
void sensor1(void *argument);

static double Temp;  //temperature readings from SCD-40-2
static double Hum;  // Humidity readings from SCD-40-2
static uint32_t Carb; // C02 readings from SCD-40-2
static uint32_t Pm;  // PM 2.5 readings from SNJGAC5
static double aqi = 0;
//static char *str;
int main(void){


  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

  /* Configure the system clock */
    SystemClock_Config();

  /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ETH_Init();
    MX_RNG_Init();
    MX_USART3_UART_Init();
    MX_I2C1_Init();

    osKernelInitialize();

   ServerHandle = osThreadNew(server, NULL, &Server_attributes);
   SCDHandle = osThreadNew(sensor1, NULL, &SCD_attributes);

    osKernelStart();   // start FreeRTOS kernel and OS takes over by running tasks

    while (1)
    {

    }

}

void SystemClock_Config(void)
{
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	  /** Configure the main internal regulator output voltage
	  */
	  __HAL_RCC_PWR_CLK_ENABLE();
	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	  RCC_OscInitStruct.PLL.PLLM = 8;
	  RCC_OscInitStruct.PLL.PLLN = 216;
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	  RCC_OscInitStruct.PLL.PLLQ = 9;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Activate the Over-Drive mode
	  */
	  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Initializes the CPU, AHB and APB buses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
	  {
	    Error_Handler();
	  }
	}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x600030D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

   static uint8_t MACAddr[6];  // setting MAC address for the given hardware (development board Nucleo f7)

  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;  // Setting the Ethernet mode to Reduced Media-Independent Interface
  heth.Init.TxDesc = DMATxDscrTab;  			 // Setting address for transmit and receive buffers
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 0;

  // Error handler if HAL status returns something other then an OK message
  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  // Allocating memory and setting up registers for Transmit packet configurations
  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;

}


static void MX_RNG_Init(void)
{
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{


  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
   GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
   GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   /*Configure GPIO pin : USB_VBUS_Pin */
     GPIO_InitStruct.Pin = USB_VBUS_Pin;
     GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
     GPIO_InitStruct.Pull = GPIO_NOPULL;
     HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
}


/***************************************************************
 * A function that utilizes the on-board random number
 * generator to create random buffers for Mongoose usage.
 * Certain network operations that happen behind the scenes
 * that need different initializations upon reset get directed
 * to this function. Mongoose knows to use this by defining
 * MG_ENABLE_CUSTOM_RANDOM as 1 in mongoose_custom.h
 ***************************************************************/
void mg_random(void *buf, size_t len) {  // Use on-board RNG
  extern RNG_HandleTypeDef hrng;
  for (size_t n = 0; n < len; n += sizeof(uint32_t)) {
    uint32_t r;
    HAL_RNG_GenerateRandomNumber(&hrng, &r);
    memcpy((char *) buf + n, &r, n + sizeof(r) > len ? len - n : sizeof(r));
  }
}


/*************************************************************
 * The timer_fn function is a callback that is linked to a
 * software timer that gets initialized in server TASK. The
 * timer is polled upon mg_mgr_poll() call and this
 * function called once the timer expires. Resets after
 * expiration.
 *************************************************************/
static void timer_fn(void *arg) {
  struct mg_tcpip_if *ifp = arg;  	// passes in the network structure so we can display network stats
  const char *names[] = {"down", "up", "req", "ready"};  // network stats
  MG_INFO(("Ethernet: %s, IP: %M, rx:%u, tx:%u, dr:%u, er:%u", // Function that is used to write to our LOG
           names[ifp->state], mg_print_ip4, &ifp->ip, ifp->nrecv, ifp->nsent,
           ifp->ndrop, ifp->nerr));
}


static double calcTemp(uint8_t highByte, uint8_t lowByte) {
	uint32_t word = (highByte << 8) | (lowByte);
	double result = -45 + (175 * (((double)word)/(65535)));
	return result;
}

static double calcHum(uint8_t highByte, uint8_t lowByte) {
	uint32_t word = (highByte << 8) | (lowByte);
	double result = 100 * ((double)word/65535);
	return result;
}



void sensor1(void *argument) {
	HAL_StatusTypeDef status;
	uint8_t read_buf[9];
	MG_INFO(("start"));

	uint8_t data_buf[] = {0x21, 0xb1};
	uint32_t x;

	status = HAL_I2C_Master_Transmit(&hi2c1, SCD40_ADDR << 1, data_buf, sizeof(data_buf), 500);
	for(;;){
		HAL_Delay(5000);
		status = HAL_I2C_Mem_Read(&hi2c1, SCD40_ADDR << 1, 0xec05, 2, read_buf, sizeof(read_buf), 5000);
		Carb = (read_buf[0] << 8) | (read_buf[1]);
		Temp = calcTemp(read_buf[3], read_buf[4]);
		Hum = calcHum(read_buf[6], read_buf[7]);
		MG_INFO(("status: %d", status));
	}
	status = HAL_I2C_Mem_Write(&hi2c1, SCD40_ADDR, (uint16_t) 0x3f86, 2, 0,0, 500);
	(void) argument;

}




/***********************************************
 * Event Handler for HTTP connection:
 * 	accepts the HTTP requests and feeds sensor
 * 	values back in JSON format for the client
 * 	to receive then the javascript code takes
 *  over and displays the values on our UI.
 ***********************************************/
static void fn(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {

	// polling for a an HTTP message being sent to the IP address and port we are listening on
	if (ev == MG_EV_HTTP_MSG) {
		struct mg_http_message *hm = (struct mg_http_message *) ev_data;  // structure of message received including contents
		if (mg_http_match_uri(hm, "/api/dispAQI")){						  // Function that checks the message for an API call sent by the client
			mg_http_reply(c, 200, "Content-Type: application/json\r\n",   // replies with the data that the HTTP request message is asking for
					"{%m:%f}\n", MG_ESC("aqi"), aqi);					  // printf() format as well as structures teh message into a HTTP Post
		}
		if(mg_http_match_uri(hm, "/api/AQI")){
			struct mg_str json = hm -> body;
			mg_json_get_num(json, "$.aqi", &aqi);
//			str = mg_json_get_str(json, "&.health_level");
			mg_http_reply(c, 200, NULL, NULL);
		}
		if(mg_http_match_uri(hm, "/api/sensors")){
			mg_http_reply(c, 200, "Content-Type: application/json\r\n",
					"{%m:%.2f,%m:%.2f,%m:%u,%m:%u}\n", MG_ESC("temperature"), Temp,
												   MG_ESC("humidity"), Hum,
												   MG_ESC("pm25"), Pm,
												   MG_ESC("co2"), Carb);
		}
		MG_INFO(("connection established"));
	    struct mg_http_serve_opts opts = {
	        .root_dir = "/web_root",					// Showing mongoose which directory our CSS, HTML, and javascript code are stored
	        .fs = &mg_fs_packed
	      };
	    mg_http_serve_dir(c, ev_data, &opts);			// refreshes UI page with new information provided by our HTTP post messages
	  }
	  (void) fn_data;
}


/*********************************************************************
 * Server TASK - initializes the mongoose server and event handler.
 * Also sets up the network stack for TCP integration and gives
 * low level driver for the NUCLEO F7 board so that the mongoose
 * structure uses the built in TCP/IP stack. The Server TASK
 * runs the initializations once and then stays in the mg_mgr_poll
 * function which is periodically calling a function that services
 * the HTTP messages coming from the Network.
 **********************************************************************/
void server(void *argument)
{
	Pm = 2;

	struct mg_mgr mgr;        // Initialise Mongoose event manager
	mg_mgr_init(&mgr);        // and attach it to the interface
	mg_log_set(MG_LL_DEBUG);  // Set log level - Makes it so we can print errors, info, and DEBUG messages to the LOG

	  	  	  	  	  	  	  // Initialise Mongoose network stack
	  struct mg_tcpip_driver_stm32_data driver_data = {.mdc_cr = 4};
	  struct mg_tcpip_if mif = {.mac = GENERATE_LOCALLY_ADMINISTERED_MAC(), // gives the network structure our MAC address for NUCLEO F7 board
		                        .driver = &mg_tcpip_driver_stm32,
		                        .driver_data = &driver_data};
		mg_tcpip_init(&mgr, &mif);
		mg_timer_add(&mgr, BLINK_PERIOD_MS, MG_TIMER_REPEAT, timer_fn, &mif);  // Add timer function for logging network stats
		MG_INFO(("MAC: %M. Waiting for IP...", mg_print_mac, mif.mac));
//		while (mif.state != MG_TCPIP_STATE_READY) {
//		    mg_mgr_poll(&mgr, 0);
//		}

		MG_INFO(("Initialising application..."));
		mg_http_listen(&mgr, HTTP_URL, fn, &mgr);  //Sets up the server to listen on a specific IP address and port.
//		mg_http_listen(&mgr, HTTPS_URL, fn, &mgr);
		for (;;) {
			mg_mgr_poll(&mgr, 1);               // Polling function for our server manager
		}
		mg_mgr_free(&mgr);						// shuts down server and frees up IP address on subnet
	   (void) argument;
}



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
