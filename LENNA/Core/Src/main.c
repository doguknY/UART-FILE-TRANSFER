/* USER CODE BEGIN Header */

/*
	YILDIZ ROCKET TEAM
  LENNA FILE TRANSFER COMPUTER

	Dogukan Yalcin
*/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_fatfs.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "def.h"
#include "file_transmit.h"
#include "file_receive.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint8_t transmissionState = RECEIVING_FROM_GROUND;
uint8_t *chunk_to_send;
uint8_t receivedChar;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

void led(uint8_t state){
  if (state == 1){
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  }
  else{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  }
}
void led_notify(uint8_t count){
  for(uint8_t i = 0; i < count; i++){
    led(1);
    HAL_Delay(69);
    led(0);
    HAL_Delay(69);
  }
}




int timeout = 0;

uint32_t index = 0;

int debug_tick;

unsigned int debug_file_size;

uint8_t flag = 0;

// --------------------------------------------------

uint8_t was_the_transmission_successful = 0;
uint8_t re_v;
uint8_t re;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {


    if (huart->Instance == huart2.Instance) {
      HAL_UART_Receive_IT(&huart2, &receivedChar, 1);
    } 


    else if (huart->Instance == huart1.Instance && transmissionState == SENDING_TO_GROUND){ // Gonderici UART

      if ((uint8_t)receivedChar == (uint8_t)RECEIVE_SUCCESSFUL){
        am_i_ready_to_send_chunk = 1;
        was_the_transmission_successful = 1;
      }
      else if ((uint8_t)receivedChar == (uint8_t)RECEIVE_FAILED){
        am_i_ready_to_send_chunk = 1;
        was_the_transmission_successful = 0;
      }
      else {
        am_i_ready_to_send_chunk = 1;
        was_the_transmission_successful = 0;
      }

      HAL_UART_Receive_IT(&huart1, &receivedChar, 1);
    }


    else if (huart->Instance == huart3.Instance && transmissionState == RECEIVING_FROM_GROUND) { // Alicinin UART'i
      re_v = gimme_THE_BYTE(receivedChar);
      timeout = HAL_GetTick();
      HAL_UART_Receive_IT(&huart3, &receivedChar, 1);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{


  if (GPIO_Pin == BUTTON_Pin){

   
  }
 
}



void sendTranmissionProcessMessage(){
  

}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_SPI3_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  led_notify(5);  
  mount_sd();
  writeSD("Round and Round We Go\n\n");
  if(fr != FR_OK){    
    Error_Handler();
  }

  led(1);

  HAL_UART_Receive_IT(&huart3, &receivedChar, 1);

  re = init_file_receive();
  if (re != FR_OK) {
      Error_Handler();
  }

  uint8_t info_message[6];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    switch(transmissionState){

      case RECEIVING_FROM_GROUND:
        led(1);
        uint8_t char_to_send;
        re_v = receive_chunk();
        if (re_v == 1) {
            char_to_send = RECEIVE_SUCCESSFUL;
            sendTranmissionProcessMessage();
            HAL_UART_Transmit(&huart3, &char_to_send, 1, 100);
        } else if (re_v == 2) {
            char_to_send = RECEIVE_FAILED;
            led(0);
            while (HAL_GetTick() - timeout < 100) {
                // wait
            }
            loop_count_reset();
            HAL_UART_Transmit(&huart3, &char_to_send, 1, 100);

        }else if(re_v == 5){
          HAL_UART_Receive_IT(&huart3, &receivedChar, 1);
        }
         else if (re_v == 3){
          char_to_send = RECEIVE_SUCCESSFUL;
          sendTranmissionProcessMessage();
          HAL_UART_Transmit(&huart3, &char_to_send, 1, 100);
          transmissionState = SENDING_TO_GROUND;
          void free_received_chunk();
          led_notify(25);          

          // --------------------------------------------------
          // start sending

          HAL_UART_Receive_IT(&huart1, &receivedChar, 1);
          chunk_to_send = (uint8_t *) calloc(CHUNK_SIZE, sizeof(uint8_t));
          re_v = init_file_transmit(info_message);
          if (re_v != FR_OK) {
            //
          }
          while(am_i_ready_to_send_chunk == 0) {
            led(1);
            HAL_Delay(50);
            HAL_UART_Transmit(&huart1, (uint8_t *) info_message, 6, 1000);
            led(0);
            HAL_Delay(50);
           }



        }
        if (HAL_GetTick() - timeout > 1000 && receiving_status == 1) {
          char_to_send = RECEIVE_FAILED;
          led(0);
          loop_count_reset();
          HAL_UART_Receive_IT(&huart3, &receivedChar, 1);
          HAL_UART_Transmit(&huart3, &char_to_send, 1, 100);
        }
        break;
        
      case SENDING_TO_GROUND:
        led(0);
        re_v = create_chunk(chunk_to_send, was_the_transmission_successful);
        if (re_v == 1) {
            led(1);
            sendTranmissionProcessMessage();
            HAL_UART_Transmit(&huart1, (uint8_t *) chunk_to_send, CHUNK_SIZE, 3000);
        } else if(re_v == 2){
            sendTranmissionProcessMessage();
            led_notify(10);
            transmissionState = TRANSMISSION_COMPLETED;
        }
        break;

      case TRANSMISSION_COMPLETED:
        led(0);
        HAL_Delay(1000);
        led(1);
        HAL_Delay(1000);
        break;

      default:
        break;
    }
    



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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
}

/* USER CODE BEGIN 4 */

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
    led(0);
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
