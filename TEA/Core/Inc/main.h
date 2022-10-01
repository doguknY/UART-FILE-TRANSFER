/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
typedef struct Accel{
    float x;
    float y;
    float z;
    float prev_x;
    float prev_y;
    float prev_z;
    float offset_x;
    float offset_y;
    float offset_z;
}Accel;

typedef struct Gyro{
    float x;
    float y;
    float z;
    float prev_x;
    float prev_y;
    float prev_z;
    float offset_x;
    float offset_y;
    float offset_z;
}Gyro;

typedef struct Angle{
    float roll;
    float pitch;
    float yaw;
    float prev_roll;
    float prev_pitch; 
    float prev_yaw;
    float timeDifference;
    float prevTime;

} Angle;

typedef struct Altitude{
    float pressure;
    float basePressure;
    float temperature;
   float altitude;
   float prevAltitude;
   float prevAltitudeForVelocity;
   float maxAltitude;
   float diffToMax;
} Altitude;

typedef struct Time{
   float current;
   float prevTime;
   float liftoffTime;
   float apogeeTime;
    float timeDifference;
    float flightTime;
    float landingTime;
} Time;

typedef struct Velocity{
    
    float verticalVelocity;
    float timeDiffVertical;
    float prevTimeVertical;
    float trueVelocity;
    float timeDiffTrue;
    float prevTimeTrue;
    
} Velocity;

typedef struct Gps{
    float latitude;
    float longitude;
    float altitude;
    int sat;
    float utc_time;


    float velocity;
    float maxVelocity;
} Gps;

typedef struct Jei {
    float altitude;
    float pressure;
    float latitude;
    float longtitude;
    float gpsAltitude;
} Jei;


typedef union {
    double u64;
    uint8_t u8[8];
} double_to_u8;

typedef union {
    float u32;
    uint8_t u8[4];
} float_to_u8;

typedef union {
    uint16_t u16;
    // degeri alirken cast etmek gerekiyor
    int16_t i16;
    uint8_t u8[2];
} u16_to_u8;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_Pin GPIO_PIN_13
#define BUTTON_GPIO_Port GPIOC
#define XBEE_CTS_Pin GPIO_PIN_0
#define XBEE_CTS_GPIO_Port GPIOA
#define XBEE_RTS_Pin GPIO_PIN_1
#define XBEE_RTS_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOA
#define SPI1_RST_Pin GPIO_PIN_6
#define SPI1_RST_GPIO_Port GPIOA
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define XBEE_RST_Pin GPIO_PIN_8
#define XBEE_RST_GPIO_Port GPIOA
#define SPI1_DIO0_Pin GPIO_PIN_9
#define SPI1_DIO0_GPIO_Port GPIOA
#define SPI1_DIO1_Pin GPIO_PIN_10
#define SPI1_DIO1_GPIO_Port GPIOA
#define SPI1_NSS_Pin GPIO_PIN_15
#define SPI1_NSS_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */




typedef enum{
    START,
    LISTENING_PC_FOR_FILE,
    RECEIVING_FILE_FROM_PC,
    FILE_SENDING_TO_SKY,
    FILE_TRANSMISSION_COMPLETED
} SYSTEM_STATE;


enum sudo_commdands{
    BUZZER_ON = 0xA1, 
    BUZZER_OFF = 0xA2,
    SEPARATE = 0xB1,
    REVERT = 0xB2,
    MOTOR_STOP = 0xC1,
    MOTOR_START_TEST = 0xC2,
    MOTOR_PID_TEST = 0xC3,
    MOTOR_QR_TEST = 0xC4,
    MOTOR_INPUT_TEST = 0xC5,
};

enum pc_received_commands_key{
    BUZZER_ON_KEY = '1',
    BUZZER_OFF_KEY = '2',
    SEPARATE_KEY = '3',
    REVERT_KEY = '4',
    MOTOR_STOP_KEY = '5',
    MOTOR_START_TEST_KEY = '6',
};

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
