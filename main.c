/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"


/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdarg.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;




/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define  D_UART &huart3 
#define  C_UART &huart2 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CRC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void printmsg(char *format,...);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define  BL_DEBUG_MSG_EN
#define BF_len 0x200
/* USER CODE END 0 */
		char sampleData[]="Invalid command...\r\n";
		uint8_t bl_rx_buffer[BF_len];
		uint8_t supported_commands[]={BL_GET_VER,BL_GET_HELP,BL_GET_CID,BL_GET_RDP_STATUS,BL_GO_TO_ADDR,BL_FLASH_ERASE,BL_MEM_WRITE,BL_EN_R_W_PROTECT,BL_MEM_READ,BL_READ_SECTOR_STATUS,BL_OTP_READ,BL_DIS_R_W_PROTECT};
		uint8_t bl_versionnum;
		
		
int main(void)
{

  /* USER CODE BEGIN 1 */
	


  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_CRC_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  if(HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin)==GPIO_PIN_SET)
		{
			printmsg(" Entering to Bootloader Mode....");
			bootloader_uart_read_data();
		
		}
	else
		{
			printmsg(" Entering to User Application Mode....");
			bootloader_jump_to_user_app();
		
		}
  /* USER CODE END 3 */

}

void bootloader_uart_read_data(void)
{
	uint8_t rcvLen=0;
	while(1)
	{
	memset(bl_rx_buffer,0,200);
	HAL_UART_Receive(C_UART,bl_rx_buffer,1,HAL_MAX_DELAY);
	rcvLen=bl_rx_buffer[0];
	HAL_UART_Receive(C_UART,&bl_rx_buffer[1],rcvLen,HAL_MAX_DELAY);
	
	switch(bl_rx_buffer[1])
	{
		case BL_GET_VER:
			bl_handle_getver_cmd(bl_rx_buffer);
			break;
			
		case BL_GET_HELP:
			bl_handle_gethelp_cmd(bl_rx_buffer);
			break;
			
		case BL_GET_CID:
			bl_handle_getcid_cmd(bl_rx_buffer);
			break;
			
		case BL_GET_RDP_STATUS:
			bl_handle_getrdpstatus_cmd(bl_rx_buffer);
			break;
			
		case BL_GO_TO_ADDR:
			bl_handle_gotoaddr_cmd(bl_rx_buffer);
			break;
			
		case BL_FLASH_ERASE:
			bl_handle_flasherase_cmd(bl_rx_buffer);
			break;
			
		case BL_MEM_WRITE:
			bl_handle_memwrite_cmd(bl_rx_buffer);
			break;
		case BL_EN_R_W_PROTECT:
			bl_handle_enrwprotect_cmd(bl_rx_buffer);
			break;
		case BL_MEM_READ:
			bl_handle_memread_cmd(bl_rx_buffer);
			break;
		case BL_READ_SECTOR_STATUS:
			bl_handle_readsectorstatus_cmd(bl_rx_buffer);
			break;
		case BL_OTP_READ:
			bl_handle_otpread_cmd(bl_rx_buffer);
			break;
		case BL_DIS_R_W_PROTECT:
			bl_handle_disrwprotect_cmd(bl_rx_buffer);
			break;
				default:
					printmsg("Invalid Command\r\n");
					HAL_UART_Transmit(&huart2,(uint8_t *)sampleData,sizeof(sampleData),HAL_MAX_DELAY);
				  break;
	}
	
}
}

void bootloader_jump_to_user_app(void)
{
	void (*app_reset_handler)(void);
	uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE;
	__set_MSP(msp_value);
	uint32_t reset_handler_addr=*((volatile uint32_t *)(FLASH_SECTOR2_BASE+4));
	app_reset_handler= (void *)reset_handler_addr;
	
	app_reset_handler();
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PA4   ------> I2S3_WS
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PB10   ------> I2S2_CK
     PC7   ------> I2S3_MCK
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PC10   ------> I2S3_CK
     PC12   ------> I2S3_SD
     PB6   ------> I2C1_SCL
     PB9   ------> I2C1_SDA
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

void printmsg(char *format,...)
{
	#ifdef BL_DEBUG_MSG_EN
	char str[80];
	
	va_list args;
	va_start(args,format);
	vsprintf(str,format,args);
	HAL_UART_Transmit(D_UART,(uint8_t *)str,strlen(str),HAL_MAX_DELAY);
	va_end(args);
	
	#endif
}
/************* Boot Loader Command handle function Implementations.... **********************/

void bl_handle_getver_cmd(uint8_t * pbuffer)
{

		printmsg(" Handling GET_VER CMD\r\n");
	
	uint32_t commandlen=pbuffer[0]-3; // len+1-4
	uint32_t crc_host=*((uint32_t *)(pbuffer+commandlen));
	
	if(!bootloader_verify_crc(pbuffer,commandlen,crc_host))
	{
		printmsg(" CRC Success \r\n");
		bootloader_send_ack(pbuffer[1],1);
		bl_versionnum= get_bootloader_version();
		bootloader_uart_write_data(&bl_versionnum,1);
		
	}
	else
	{
			printmsg(" CRC Failed \r\n");
			bootloader_send_nack();
		//	HAL_UART_Transmit(&huart2,(uint8_t *)sampleData,sizeof(sampleData),HAL_MAX_DELAY);
	}
}
void bootloader_send_ack(uint8_t cmd_code,uint8_t follow_len)
{
	uint8_t ack_buf[2];
	ack_buf[0]=BL_ACK;
	ack_buf[1]=follow_len;
	HAL_UART_Transmit(&huart2,ack_buf,2,HAL_MAX_DELAY);
}

void bootloader_send_nack(void)
{
	uint8_t nack=BL_NACK;
	HAL_UART_Transmit(&huart2,&nack,2,HAL_MAX_DELAY);
}
uint8_t bootloader_verify_crc(uint8_t *pData, uint32_t len,uint32_t crc_host)
{
	uint32_t calCRCValue=0xFF;
	
	for(uint32_t i=0;i<len;i++)
	{
		uint32_t i_Data=pData[i];
		calCRCValue=HAL_CRC_Accumulate(&hcrc,&i_Data,1);
	}
	if(calCRCValue==crc_host)
	{
		return VER_CRC_SUCCESS;
	}
	else
	{
		return VER_CRC_FAIL;
	}
}
uint8_t get_bootloader_version(void)
{
	return (uint8_t)BL_VERSION;
}
	
void bootloader_uart_write_data(uint8_t *pBuf,uint8_t len)
{
	HAL_UART_Transmit(C_UART,pBuf,len,HAL_MAX_DELAY);
}
void bl_handle_gethelp_cmd(uint8_t * pbuffer)
{

	printmsg(" Handling GET_HELP CMD\r\n");
	
	uint32_t commandlen=pbuffer[0]-3; // len+1-4
	uint32_t crc_host=*((uint32_t *)(pbuffer+commandlen));
	
	if(!bootloader_verify_crc(pbuffer,commandlen,crc_host))
	{
		printmsg(" CRC Success \r\n");
		bootloader_send_ack(pbuffer[1],sizeof(supported_commands));
		
		bootloader_uart_write_data(supported_commands,sizeof(supported_commands));
		
	}
	else
	{
			printmsg(" CRC Failed \r\n");
			bootloader_send_nack();
		//	HAL_UART_Transmit(&huart2,(uint8_t *)sampleData,sizeof(sampleData),HAL_MAX_DELAY);
	}
	
}
void bl_handle_getcid_cmd(uint8_t * pbuffer)
{
	uint16_t chip_id;
	printmsg(" Handling GET_CID CMD\r\n");
	
	uint32_t commandlen=pbuffer[0]-3; // len+1-4
	uint32_t crc_host=*((uint32_t *)(pbuffer+commandlen));
	
	if(!bootloader_verify_crc(pbuffer,commandlen,crc_host))
	{
		printmsg(" CRC Success \r\n");
		bootloader_send_ack(pbuffer[1],2);
		chip_id=get_mcu_chip_id();
		bootloader_uart_write_data((uint8_t *)&chip_id,2);
		
	}
	else
	{
			printmsg(" CRC Failed \r\n");
			bootloader_send_nack();
		//	HAL_UART_Transmit(&huart2,(uint8_t *)sampleData,sizeof(sampleData),HAL_MAX_DELAY);
	}
	
}

uint16_t get_mcu_chip_id(void)
{
	uint16_t cid;
	cid=(uint16_t)(DBGMCU->IDCODE&0x0FFF);
	return cid;
}
void bl_handle_getrdpstatus_cmd(uint8_t * pbuffer)
{
		uint8_t mcu_rdp;
	printmsg(" Handling GET_CID CMD\r\n");
	
	uint32_t commandlen=pbuffer[0]-3; // len+1-4
	uint32_t crc_host=*((uint32_t *)(pbuffer+commandlen));
	
	if(!bootloader_verify_crc(pbuffer,commandlen,crc_host))
	{
		printmsg(" CRC Success \r\n");
		bootloader_send_ack(pbuffer[1],1);
		mcu_rdp=get_mcu_rdp_status();
		bootloader_uart_write_data(&mcu_rdp,1);
		
	}
	else
	{
			printmsg(" CRC Failed \r\n");
			bootloader_send_nack();
		//	HAL_UART_Transmit(&huart2,(uint8_t *)sampleData,sizeof(sampleData),HAL_MAX_DELAY);
	}
}

uint8_t get_mcu_rdp_status(void)
{
	uint8_t rdp_status;
	uint32_t * pOB= (uint32_t *)0x1FFFC000;
	rdp_status= (uint8_t)(*pOB>>8);
	return rdp_status;
}

void bl_handle_gotoaddr_cmd(uint8_t * pbuffer)
{
	uint32_t go_addr;
	uint8_t addr_VALID=ADDR_VALID;
	printmsg(" Handling GO_To_Addr CMD\r\n");
	
	uint32_t commandlen=pbuffer[0]-3; // len+1-4
	uint32_t crc_host=*((uint32_t *)(pbuffer+commandlen));
	
	if(!bootloader_verify_crc(pbuffer,commandlen,crc_host))
	{
		printmsg(" CRC Success \r\n");
		go_addr=*((uint32_t *)&pbuffer[2]);
		if(valid_address(go_addr)==ADDR_VALID)
		{
		bootloader_send_ack(pbuffer[1],1);
		go_addr +=1;
	  void (*jump_to_addr)(void) = (void *)go_addr;
			jump_to_addr();
		bootloader_uart_write_data(&addr_VALID,1);
		}
	
		
	}
	else
	{
			printmsg(" CRC Failed \r\n");
			bootloader_send_nack();
		//	HAL_UART_Transmit(&huart2,(uint8_t *)sampleData,sizeof(sampleData),HAL_MAX_DELAY);
	}
	
}
uint8_t valid_address(uint32_t addr)
{
	if(addr>=SRAM1_BASE&& addr<=SRAM1_END)
	{
		return ADDR_VALID;
	}
	else if(addr>=SRAM2_BASE && addr<=SRAM2_END)
	{
		return ADDR_VALID;
	}
	else if(addr>=FLASH_BASE && addr<=FLASH_END)
	{
		return ADDR_VALID;
	}	
	else
	{
		return ADDR_INVALID;
	}
	
}

void bl_handle_flasherase_cmd(uint8_t * pbuffer)
{
	uint8_t status_flasherase=0x00;
	
	printmsg(" Handling GO_To_Addr CMD\r\n");
	
	uint32_t commandlen=pbuffer[0]-3; // len+1-4
	uint32_t crc_host=*((uint32_t *)(pbuffer+commandlen));
	
	if(!bootloader_verify_crc(pbuffer,commandlen,crc_host))
	{
		printmsg(" CRC Success \r\n");
		bootloader_send_ack(pbuffer[1],1);
		status_flasherase=execute_flash_erase_cmd(pbuffer[2],pbuffer[3]);
		bootloader_uart_write_data(&status_flasherase,1);
	}
	else
	{
			printmsg(" CRC Failed \r\n");
			bootloader_send_nack();
		//	HAL_UART_Transmit(&huart2,(uint8_t *)sampleData,sizeof(sampleData),HAL_MAX_DELAY);
	}
		
}

uint8_t execute_flash_erase_cmd(uint8_t sector, uint8_t sectors_num)
{
	
	FLASH_EraseInitTypeDef hflasherase;
	uint8_t status_ret;
	uint32_t flash_error=0;
	
	if(sector<8 || sector==0xFF)
	{
		if(sector==0xFF)
		{
			hflasherase.TypeErase=FLASH_TYPEERASE_MASSERASE;
		}
		else
		{
			hflasherase.TypeErase=FLASH_TYPEERASE_SECTORS;
			uint8_t rem_sectors=8-sector;
			if(sectors_num>rem_sectors)
			{
				sectors_num=rem_sectors;
			}
			hflasherase.Sector=sector;
			hflasherase.NbSectors=sectors_num;
			
		}
		hflasherase.Banks=FLASH_BANK_1;
		HAL_FLASH_Unlock();
		hflasherase.VoltageRange=FLASH_VOLTAGE_RANGE_3;
		status_ret= (uint8_t)HAL_FLASHEx_Erase(&hflasherase,&flash_error);
		HAL_FLASH_Lock();
		return status_ret;
		
		
		
	}
	else
	{
	return ADDR_INVALID;
	} 
	
}

void bl_handle_memwrite_cmd(uint8_t * pbuffer)
{
printmsg(" Handling memory write command \r\n");
	
	uint32_t commandlen=pbuffer[0]-3; // len+1-4
	uint32_t crc_host=*((uint32_t *)(pbuffer+commandlen));
	uint8_t status_memwrite;
	if(!bootloader_verify_crc(pbuffer,commandlen,crc_host))
	{
		printmsg(" CRC Success \r\n");
		uint32_t memory_start= *(uint32_t *)&pbuffer[2];
		uint8_t len=pbuffer[6];
		bootloader_send_ack(pbuffer[1],1);
		status_memwrite=execute_memory_write_cmd(&pbuffer[7],memory_start,len);
		bootloader_uart_write_data(&status_memwrite,1);
	}
	else
	{
			printmsg(" CRC Failed \r\n");
			bootloader_send_nack();
		//	HAL_UART_Transmit(&huart2,(uint8_t *)sampleData,sizeof(sampleData),HAL_MAX_DELAY);
	}
	
	
}

uint8_t execute_memory_write_cmd(uint8_t * pbuffer,uint32_t memory_start,uint8_t len)
{
	uint8_t ret_status;
	if(valid_address(memory_start)==ADDR_VALID)
	{
		int i=0;
		for(i=0;i<len;i++)
		{
			HAL_FLASH_Unlock();
			ret_status=HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,memory_start+i,pbuffer[i]);
			HAL_FLASH_Lock();
		}
		
		return ret_status;
	}
	else
	{
			return ADDR_INVALID;
	}
	
	
}

void bl_handle_enrwprotect_cmd(uint8_t * pbuffer)
{

	printmsg(" Handling memory write command \r\n");
	
	uint32_t commandlen=pbuffer[0]-3; // len+1-4
	uint32_t crc_host=*((uint32_t *)(pbuffer+commandlen));
	uint8_t status_enrw;
	if(!bootloader_verify_crc(pbuffer,commandlen,crc_host))
	{
		printmsg(" CRC Success \r\n");
		uint32_t memory_start= *(uint32_t *)&pbuffer[2];
		uint8_t len=pbuffer[6];
		bootloader_send_ack(pbuffer[1],1);
		status_enrw=configure_enrw_protect(pbuffer[2],pbuffer[3],0);
		bootloader_uart_write_data(&status_enrw,1);
	}
	else
	{
			printmsg(" CRC Failed \r\n");
			bootloader_send_nack();
		//	HAL_UART_Transmit(&huart2,(uint8_t *)sampleData,sizeof(sampleData),HAL_MAX_DELAY);
	}	
	
}
uint8_t configure_enrw_protect(uint8_t sector_det, uint8_t protection, uint8_t disable)
{
	uint8_t status_ret=0x00;
	
	uint32_t *pOPTR= (uint32_t *)0x40023C14 ;
	
	if(disable)
	{
				HAL_FLASH_OB_Unlock();
		
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);
		
		*pOPTR |= (0b11111111<<16);
		
			*pOPTR |= (1<<1);
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);
		
		HAL_FLASH_OB_Lock();
		
		return 1;
		
		return 1;
	}
	if(protection==1)
			{
				//write protection
		HAL_FLASH_OB_Unlock();
		
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);
		
		*pOPTR &= ~(sector_det<<16);
		
			*pOPTR |= (1<<1);
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);
		
		HAL_FLASH_OB_Lock();
		
		return 1;
	}
	else if(protection ==2)
	{
		return 0x00;
	}
	
	
	
}
void bl_handle_memread_cmd(uint8_t * pbuffer)
{
	
}
void bl_handle_readsectorstatus_cmd(uint8_t * pbuffer)
{
	
}
void bl_handle_otpread_cmd(uint8_t * pbuffer)
{
	
}
void bl_handle_disrwprotect_cmd(uint8_t * pbuffer)
{
	
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
