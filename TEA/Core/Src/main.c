/* USER CODE BEGIN Header */

/*
	YILDIZ ROCKET TEAM
  TEA GROUND STATION COMPUTER
  
	Dogukan Yalcin
*/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_fatfs.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LoRa.h"
#include "stdio.h"
#include "file_transmit.h"
#include "file_doprot_pc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */




// The file sent from the computer with Python. 
/*
  ser.write(b'D')
  time.sleep(0.1)
  ser.write(open(file_name, 'rb').read())
*/





uint8_t *chunk_to_send;
uint8_t loraReceiveBuffer[100];
unsigned char olusturalacak_paket[79];
// char free_packet[250];
uint8_t *stringWithAllData[140];

Time time;
Accel accel;
Gyro gyro;
Altitude altitude;
Altitude altitudeContainer;
Velocity velocity;
Angle angle;
Gps payloadGps;
Gps containerGps;
Jei jei;

int payloadState;
int packetNo;
uint16_t teamNo;
uint16_t alt_difference;
uint8_t voltage;
int8_t spin;
int videoProcess;

SYSTEM_STATE systemState = START;




void combinePacketForInterface();
void parseSudoPacket();

void led(uint8_t state){
  if (state == 1){
    HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
  }
  else{
    HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
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

LoRa myLoRa;

void initLoRa()
{
  

	myLoRa = newLoRa();

	myLoRa.CS_port         = SPI1_NSS_GPIO_Port;
	myLoRa.CS_pin          = SPI1_NSS_Pin;
	myLoRa.reset_port      = SPI1_RST_GPIO_Port;
	myLoRa.reset_pin       = SPI1_RST_Pin;
	myLoRa.DIO0_port       = SPI1_DIO0_GPIO_Port;
	myLoRa.DIO0_pin        = SPI1_DIO0_Pin;
	myLoRa.hSPIx           = &hspi1;
	myLoRa.frequency = 433.663;			// default = 433 MHz
	myLoRa.spredingFactor = SF_9;		// default = SF_7
	myLoRa.bandWidth = BW_500KHz;		// default = BW_125KHz
	myLoRa.crcRate = CR_4_5;			// default = CR_4_5
	myLoRa.power = POWER_20db;			// default = 20db
	myLoRa.overCurrentProtection = 250; // default = 100 mA
	myLoRa.preamble = 8;				// default = 8;

	HAL_GPIO_WritePin(SPI1_RST_GPIO_Port, SPI1_RST_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
	LoRa_reset(&myLoRa);
	uint16_t LoRa_status = LoRa_init(&myLoRa);
	HAL_Delay(100);
	LoRa_startReceiving(&myLoRa);

}
// 

uint8_t loraTransmitPacket[4] = {0x00, 0x00, 0x00, 0x00};
uint32_t loraDelay;
uint32_t loraStatus = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

  if (htim == &htim2){

	  if(HAL_GetTick() - loraDelay > 1000){
      if (systemState == RECEIVING_FILE_FROM_PC) return;
      loraDelay = HAL_GetTick();

        LoRa_receive(&myLoRa, (uint8_t*) loraReceiveBuffer, sizeof(loraReceiveBuffer));
        parseSudoPacket();
        combinePacketForInterface();
        if (packetNo != 0)
          HAL_UART_Transmit(&huart1, stringWithAllData, strlen(stringWithAllData), 1000);
        LoRa_transmit(&myLoRa, loraTransmitPacket, sizeof(loraTransmitPacket), 100);

	  }
  }
}


void parseSudoCommand(uint8_t __receivedChar){
  switch (__receivedChar) {
        case BUZZER_ON_KEY:
          loraTransmitPacket[0] = BUZZER_ON;
          break;
        case BUZZER_OFF_KEY:
          loraTransmitPacket[0] = BUZZER_OFF;
          break;

        case SEPARATE_KEY:
          loraTransmitPacket[1] = SEPARATE;
          break;
        
        case REVERT_KEY:
          loraTransmitPacket[1] = REVERT;
          break;

        case MOTOR_STOP_KEY:
          loraTransmitPacket[2] = MOTOR_STOP;
          break;

        case MOTOR_START_TEST_KEY:
          loraTransmitPacket[2] = MOTOR_START_TEST;
          break;
        
        // case MOTOR_PID_TEST:
        //   loraTransmitPacket[2] = MOTOR_PID_TEST;
        //   break;

        // case MOTOR_QR_TEST:
        //   loraTransmitPacket[2] = MOTOR_QR_TEST;
        //   break;

        // case MOTOR_INPUT_TEST:
        //   loraTransmitPacket[2] = MOTOR_INPUT_TEST;
        //   break;

        default:
          // if (__receivedChar >= 0 && __receivedChar <= 1000) {
          //   loraTransmitPacket[3] = __receivedChar;
          // }
          break;
      }

}

#define RECEIVE_SUCCESSFUL 0x12
#define RECEIVE_FAILED 0x13

uint8_t was_the_transmission_successful;


uint8_t receivedChar;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    uint8_t retVal;


    if (huart->Instance == huart1.Instance) {  //PC
      retVal = gimmeTheBytePC(receivedChar);
      if (retVal == 2){
        systemState = RECEIVING_FILE_FROM_PC;
      }

      if (systemState != RECEIVING_FILE_FROM_PC){
        // get command from pc
        parseSudoCommand(receivedChar);
      }

      
      HAL_UART_Receive_IT(&huart1, &receivedChar, 1);
    } 


    else if (huart->Instance == huart2.Instance ){ // Gonderici XBEE UART 
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
      

      HAL_UART_Receive_IT(&huart2, &receivedChar, 1);
    }
 
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BUTTON_Pin){


  }
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  led_notify(5);
  mount_sd();
  writeSD("I Live For This S***\n");

  if (fr != FR_OK) {
    Error_Handler();
  }
  HAL_GPIO_WritePin(XBEE_RST_GPIO_Port,XBEE_RST_Pin, GPIO_PIN_SET );

  led(1);
  HAL_Delay(2000);

  initFileReceivePc(&huart1, 72000, NAME_OF_RECEIVED_FILE_FROM_PC, &receivedChar);
  initLoRa();
  HAL_TIM_Base_Start_IT(&htim2);
  led(0);

  systemState = LISTENING_PC_FOR_FILE;

  uint8_t retVal;
  uint8_t info_message[6];
  HAL_UART_Receive_IT(&huart2, &receivedChar, 1);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    switch (systemState)
    {
    case START:
      
      break;
    case LISTENING_PC_FOR_FILE:
      
      break;

    case RECEIVING_FILE_FROM_PC:

      retVal = saveBufferToSD();
      if (retVal == 2){
    	  systemState = FILE_SENDING_TO_SKY;
        led_notify(5);
        HAL_UART_Receive_IT(&huart2, &receivedChar, 1);
        chunk_to_send = (uint8_t *) calloc(CHUNK_SIZE, sizeof(uint8_t));
        retVal = init_file_transmit(info_message);
        if (retVal != FR_OK) {
          //
        }
        while(am_i_ready_to_send_chunk == 0) {
          led(1);
          HAL_Delay(50);
          HAL_UART_Transmit(&huart2, (uint8_t *) info_message, 6, 1000);
          led(0);
          HAL_Delay(50);
          }
      

      }
      break;

    case FILE_SENDING_TO_SKY:
      led(0);
      retVal = create_chunk(chunk_to_send, was_the_transmission_successful);
      if (retVal == 1) {
          led(1);
          HAL_UART_Transmit(&huart2, (uint8_t *) chunk_to_send, CHUNK_SIZE, 3000);
      } else if(retVal == 2){
          led_notify(10);

          // ---------------------------------------------------------------

          systemState = FILE_TRANSMISSION_COMPLETED;
          free(chunk_to_send);
      }
      HAL_UART_Receive_IT(&huart2, &receivedChar, 1);
  

      break;

    case FILE_TRANSMISSION_COMPLETED:
      led(1);
      HAL_Delay(5000);
      led(0);
      HAL_Delay(5000);
      
      break;
    
    default:

      break;
    }

//    sprintf(stringWithAllData, "aslkdalksdmkla\n");
//
//    HAL_UART_Transmit(&huart2, stringWithAllData, strlen(stringWithAllData), 4000);
//    led_notify(5);


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





void parseSudoPacket(){
  u16_to_u8 converter16;
  float_to_u8 converter32;


  converter16.u8[0]= loraReceiveBuffer[0];
  converter16.u8[1]= loraReceiveBuffer[1];
  teamNo = converter16.u16;

  converter16.u8[0]= loraReceiveBuffer[2];
  converter16.u8[1]= loraReceiveBuffer[3];
  packetNo = converter16.u16;

  converter32.u8[0]= loraReceiveBuffer[4];
  converter32.u8[1]= loraReceiveBuffer[5];
  converter32.u8[2]= loraReceiveBuffer[6];
  converter32.u8[3]= loraReceiveBuffer[7];
  payloadGps.utc_time = converter32.u32;

  converter32.u8[0]= loraReceiveBuffer[8];
  converter32.u8[1]= loraReceiveBuffer[9];
  converter32.u8[2]= loraReceiveBuffer[10];
  converter32.u8[3]= loraReceiveBuffer[11];
  altitude.pressure = converter32.u32;

  converter32.u8[0]= loraReceiveBuffer[12];
  converter32.u8[1]= loraReceiveBuffer[13];
  converter32.u8[2]= loraReceiveBuffer[14];
  converter32.u8[3]= loraReceiveBuffer[15];
  jei.altitude = converter32.u32;

  converter32.u8[0]= loraReceiveBuffer[16];
  converter32.u8[1]= loraReceiveBuffer[17];
  converter32.u8[2]= loraReceiveBuffer[18];
  converter32.u8[3]= loraReceiveBuffer[19];
  altitude.altitude = converter32.u32;

  converter32.u8[0]= loraReceiveBuffer[20];
  converter32.u8[1]= loraReceiveBuffer[21];
  converter32.u8[2]= loraReceiveBuffer[22];
  converter32.u8[3]= loraReceiveBuffer[23];
  velocity.verticalVelocity = converter32.u32;

  altitude.temperature = (float)loraReceiveBuffer[24];
  voltage = (uint8_t)loraReceiveBuffer[25];

  converter32.u8[0]= loraReceiveBuffer[26];
  converter32.u8[1]= loraReceiveBuffer[27];
  converter32.u8[2]= loraReceiveBuffer[28];
  converter32.u8[3]= loraReceiveBuffer[29];
  jei.latitude = converter32.u32;

  converter32.u8[0]= loraReceiveBuffer[30];
  converter32.u8[1]= loraReceiveBuffer[31];
  converter32.u8[2]= loraReceiveBuffer[32];
  converter32.u8[3]= loraReceiveBuffer[33];
  jei.longtitude = converter32.u32;

  converter32.u8[0]= loraReceiveBuffer[34];
  converter32.u8[1]= loraReceiveBuffer[35];
  converter32.u8[2]= loraReceiveBuffer[36];
  converter32.u8[3]= loraReceiveBuffer[37];
  jei.gpsAltitude =(float) converter32.u32;

converter32.u8[0]= loraReceiveBuffer[38];
converter32.u8[1]= loraReceiveBuffer[39];
converter32.u8[2]= loraReceiveBuffer[40];
converter32.u8[3]= loraReceiveBuffer[41];
containerGps.latitude = converter32.u32;

converter32.u8[0]= loraReceiveBuffer[42];
converter32.u8[1]= loraReceiveBuffer[43];
converter32.u8[2]= loraReceiveBuffer[44];
converter32.u8[3]= loraReceiveBuffer[45];
containerGps.longitude = converter32.u32;

converter32.u8[0]= loraReceiveBuffer[46];
converter32.u8[1]= loraReceiveBuffer[47];
converter32.u8[2]= loraReceiveBuffer[48];
converter32.u8[3]= loraReceiveBuffer[49];
containerGps.altitude = converter32.u32;

  payloadState = (uint8_t)loraReceiveBuffer[50];

  converter32.u8[0]= loraReceiveBuffer[51];
  converter32.u8[1]= loraReceiveBuffer[52];
  converter32.u8[2]= loraReceiveBuffer[53];
  converter32.u8[3]= loraReceiveBuffer[54];
  angle.roll = converter32.u32;

  converter32.u8[0]= loraReceiveBuffer[55];
  converter32.u8[1]= loraReceiveBuffer[56];
  converter32.u8[2]= loraReceiveBuffer[57];
  converter32.u8[3]= loraReceiveBuffer[58];
  angle.pitch = converter32.u32;

  converter32.u8[0]= loraReceiveBuffer[59];
  converter32.u8[1]= loraReceiveBuffer[60];
  converter32.u8[2]= loraReceiveBuffer[61];
  converter32.u8[3]= loraReceiveBuffer[62];
  angle.yaw = converter32.u32;
  

  spin = (int8_t)loraReceiveBuffer[63];
  videoProcess = (int)loraReceiveBuffer[64];

  converter32.u8[0]= loraReceiveBuffer[65];
  converter32.u8[1]= loraReceiveBuffer[66];
  converter32.u8[2]= loraReceiveBuffer[67];
  converter32.u8[3]= loraReceiveBuffer[68];
  jei.pressure = converter32.u32;

  converter32.u8[0]= loraReceiveBuffer[69];
  converter32.u8[1]= loraReceiveBuffer[70];
  converter32.u8[2]= loraReceiveBuffer[71];
  converter32.u8[3]= loraReceiveBuffer[72];
  time.current= converter32.u32;

  
}

void combinePacketForInterface(){

//  sprintf(free_packet,"SUDO,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%f,%f,%.2f,%f,%f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
//    packetNo,payloadState,altitude.pressure, altitudeContainer.pressure,altitude.altitude,altitudeContainer.altitude,velocity.verticalVelocity,altitude.temperature,accel.x,accel.y,accel.z,angle.pitch,angle.roll,angle.yaw, payloadGps.latitude, payloadGps.longitude, payloadGps.altitude,containerGps.latitude, containerGps.longitude, containerGps.altitude, maxG, maxSpeed, maxAltitude,parachuteForce, time.current);

	float a;
	a = jei.pressure;

    sprintf(stringWithAllData, "%d,%d,%f,%d,%.2f,%.2f,%f,%f,%.2f,%f,%f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f,%.2f,%d,%.2f\n",
      teamNo, packetNo, payloadGps.utc_time + 30000.0, (int)payloadState, altitude.pressure, jei.pressure, payloadGps.latitude, payloadGps.longitude, payloadGps.altitude, jei.latitude, jei.longtitude, jei.gpsAltitude, velocity.verticalVelocity, altitude.temperature , angle.roll, angle.pitch, angle.yaw, voltage, (float)videoProcess, altitude.altitude, a, spin, jei.altitude);

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
