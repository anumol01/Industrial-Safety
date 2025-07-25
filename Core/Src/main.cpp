/* USER CODE BEGIN Header */

/**

  ******************************************************************************

  * @file           : main.c

  * @brief          : Main program body

  ******************************************************************************

  * @attention

  *

  * Copyright (c) 2025 STMicroelectronics.

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

#include "gpio.h"

extern "C" {

#include "lcd.h"

#include "FreeRTOS.h"

#include "task.h"

#include "stm32f405xx.h"

#include "semphr.h"

#include <stdio.h>

#include <stdint.h>

#include <string.h>

}

#include <cstdio>

#include <cstring>

#include <cstdint>



// GPIO Pins

#define FLAME_PIN     2   // PA2 - Flame sensor input

#define BIN1_PIN      6    // PA6 - Water Pump control IN1

#define BIN2_PIN      7    // PA7 - Water Pump control IN2

#define RED_LED_PIN   8    // PB8 - RED LED

#define YELLOW_LED_PIN   9    // PB9 - YELLOW LED

#define GREEN_LED_PIN   11    // PB11 - GREEN LED

#define BUZZER_PIN    10   // PB10 - Buzzer

#define AIN1_PIN    9  // PA9 - Motor control IN1

#define AIN2_PIN    10 // PA10 - Motor control IN1





volatile uint16_t result = 0;

volatile float gas = 0;

volatile int flameDetected = 0;

volatile float adc_value2 = 0;

volatile float voltage = 0;

volatile float t1 = 0;

volatile float temperature = 0.0;



SemaphoreHandle_t xAuthSemaphore = nullptr;

TaskHandle_t ControlTaskHandle = nullptr;





/* Private includes ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */



/* USER CODE END Includes */



/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

class SafetySystem {

public:

    const char keymap[4][4] = {

        {'1', '2', '3', 'A'},

        {'4', '5', '6', 'B'},

        {'7', '8', '9', 'C'},

        {'*', '0', '#', 'D'}

    };



    const uint8_t row_pins[4] = {0, 1, 2, 3};     // PB0–PB3

    const uint8_t col_pins_B[2] = {4, 5};         // PB4–PB5

    const uint8_t col_pins_A[2] = {3, 4};         // PA3–PA4

    const char passkey[5] = "1234";



    void GPIO_Init(void);

    char Keypad_Scan(void);

    void adc_init(void);

    uint16_t adc_read1(void);

    uint16_t adc_read2(void);

    void led_init(void);

    void led_control(uint16_t adc_value);

    void init_GPIO(void);

    void pump_on(void);

    void pump_off(void);

    void init_motor_GPIO(void);

    void motor_forward(void);

    void motor_stop(void);

    void GasTask(void);

};

SafetySystem  sys;



void SafetySystem::GPIO_Init(void)

{

    RCC->AHB1ENR |= (1 << 0) | (1 << 1) | (1 << 2); // Enable GPIOA, GPIOB, GPIOC

    // PB0–PB3: Rows as output

    for (int i = 0; i < 4; i++) {

        GPIOB->MODER &= ~(3 << (i * 2));

        GPIOB->MODER |= (1 << (i * 2));  // Output

        GPIOB->ODR |= (1 << i);          // Set high

    }



    // PB4–PB7: Columns as input with pull-up

    for (int i = 4; i < 6; i++) {

        GPIOB->MODER &= ~(3 << (i * 2));   // Input mode

        GPIOB->PUPDR &= ~(3 << (i * 2));   // Clear previous pull

        GPIOB->PUPDR |= (1 << (i * 2));    // Pull-up

    }



    // PA2–PA3: Columns as input pull-up

    for (int i = 3; i < 5; i++) {

        GPIOA->MODER &= ~(3 << (i * 2));

        GPIOA->PUPDR &= ~(3 << (i * 2));

        GPIOA->PUPDR |= (1 << (i * 2));

    }

}



// === Keypad Scan ===

char SafetySystem::Keypad_Scan(void)

{

    const uint8_t row_pins[4] = {0, 1, 2, 3};        // PB0–PB3

    const uint8_t col_pins_B[2] = {4, 5};            // PB4–PB5

    const uint8_t col_pins_A[2] = {3, 4};            // PA3-PA4

    for (int row = 0; row < 4; row++)

    {

        // Set all rows high first

        for (int r = 0; r < 4; r++)

            GPIOB->ODR |= (1 << row_pins[r]);

        // Pull current row low

        GPIOB->ODR &= ~(1 << row_pins[row]);

        vTaskDelay(pdMS_TO_TICKS(2)); // Debounce

        // Check PB4, PB5 (Columns)

        for (int col = 0; col < 2; col++)

        {

            if ((GPIOB->IDR & (1 << col_pins_B[col])) == 0)

            {

                vTaskDelay(pdMS_TO_TICKS(20));

                while ((GPIOB->IDR & (1 << col_pins_B[col])) == 0);

                return keymap[row][col];

            }

        }

        // Check PA3, PA4 (Columns)

        for (int col = 0; col < 2; col++)

        {

            if ((GPIOA->IDR & (1 << col_pins_A[col])) == 0)

            {

                vTaskDelay(pdMS_TO_TICKS(20));

                while ((GPIOA->IDR & (1 << col_pins_A[col])) == 0);

                return keymap[row][col + 2];  // Offset by 2

            }

        }

    }

    return 0; // No key press detected

}





// ADC Initialization

void SafetySystem::adc_init(void)

{

// Enable ADC1 clock and GPIOC/PA

RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOAEN;

RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;



// Configure PC2 (channel 12) and PA5 (channel 5) as analog

GPIOC->MODER |= (3 << (2 * 2));  // PC2

GPIOA->MODER |= (3 << (5 * 2));  // PA5



ADC1->CR1 |= ADC_CR1_SCAN;       // Enable scan mode

ADC1->SQR1 &= ~ADC_SQR1_L;       // Clear length

ADC1->SQR1 |= (1 << 20);         // 2 conversions (L[3:0] = 1 → 2 conversions)



ADC1->SQR3 &= ~0x3FFFFFFF;       // Clear SQR3

ADC1->SQR3 |= (5 << 0);          // Rank 1: Channel 5

ADC1->SQR3 |= (12 << 5);         // Rank 2: Channel 12



ADC1->CR2 |= ADC_CR2_CONT;       // Continuous mode

ADC1->CR2 |= ADC_CR2_ADON;       // Enable ADC



}



// ADC Read

// Read first channel (PA5, Channel 5)

uint16_t SafetySystem::adc_read1(void) {

    ADC1->SQR3 &= ~0x3FFFFFFF;

    ADC1->SQR3 |= (5 << 0);       // Only one channel

    ADC1->SQR1 &= ~ADC_SQR1_L;    // Only 1 conversion

    ADC1->CR2 |= ADC_CR2_SWSTART;

    while (!(ADC1->SR & ADC_SR_EOC));

    return ADC1->DR;

}



// Read second channel (PC2, Channel 12)

uint16_t SafetySystem::adc_read2(void) {

    ADC1->SQR3 &= ~0x3FFFFFFF;

    ADC1->SQR3 |= (12 << 0);

    ADC1->SQR1 &= ~ADC_SQR1_L;

    ADC1->CR2 |= ADC_CR2_SWSTART;

    while (!(ADC1->SR & ADC_SR_EOC));

    return ADC1->DR;

}



// LED Initialization for PC3 to PC7

void SafetySystem::led_init(void)

{

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;



    for (int i = 3; i <= 7; i++)

        GPIOC->MODER |= (1 << (i * 2));         // Output mode

}



// LED Control based on ADC Value

void SafetySystem::led_control(uint16_t adc_value)

{

GPIOC->ODR &= ~0xF8;  // Clear PC3–PC7

int index = (adc_value * 6) / 4100;



    switch (index)

    {

        case 5: GPIOC->ODR |= (1 << 7);

        case 4: GPIOC->ODR |= (1 << 6);

        case 3: GPIOC->ODR |= (1 << 5);

        case 2: GPIOC->ODR |= (1 << 4);

        case 1: GPIOC->ODR |= (1 << 3);

        default: break;

    }



}



void SafetySystem::init_GPIO(void) {

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;



    // PA6 and PA7 as output for pump

    GPIOA->MODER |= (1 << (BIN1_PIN * 2)) | (1 << (BIN2_PIN * 2));

    GPIOA->MODER &= ~((1 << (BIN1_PIN * 2 + 1)) | (1 << (BIN2_PIN * 2 + 1)));



    // PA2 as input for flame sensor

    GPIOA->MODER &= ~(3 << (FLAME_PIN * 2)); // input mode

    GPIOA->PUPDR &= ~(3 << (FLAME_PIN * 2)); // Clear pull-up/down

    GPIOA->PUPDR |=  (2 << (FLAME_PIN * 2)); // Pull-down



    // PB8, PB9 , PB10 as output

    GPIOB->MODER |= (1 << (RED_LED_PIN * 2))|(1 << (YELLOW_LED_PIN * 2)) |(1 << (GREEN_LED_PIN * 2)) | (1 << (BUZZER_PIN * 2));

    GPIOB->MODER &= ~((1 << (RED_LED_PIN * 2 + 1))|(1 << (YELLOW_LED_PIN * 2 + 1)) |(1 << (GREEN_LED_PIN * 2 + 1)) | (1 << (BUZZER_PIN * 2 + 1)));

}



// Water pump control

void SafetySystem::pump_on(void) {

    GPIOA->ODR |=  (1 << BIN1_PIN);

    GPIOA->ODR &= ~(1 << BIN2_PIN);

}



void SafetySystem::pump_off(void) {

    GPIOA->ODR &= ~(1 << BIN1_PIN);

    GPIOA->ODR &= ~(1 << BIN2_PIN);

}

// === GPIO Init for Motor ===

void SafetySystem::init_motor_GPIO(void) {

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER |= (1 << (AIN1_PIN * 2)) | (1 << (AIN2_PIN * 2));

    GPIOA->OTYPER &= ~((1 << AIN1_PIN) | (1 << AIN2_PIN));

    GPIOA->OSPEEDR |= (3 << (AIN1_PIN * 2)) | (3 << (AIN2_PIN * 2));

}



// === Motor Control ===

void SafetySystem::motor_forward(void) {

    GPIOA->ODR |= (1 << AIN1_PIN);

    GPIOA->ODR &= ~(1 << AIN2_PIN);

}



void SafetySystem::motor_stop(void) {

    GPIOA->ODR &= ~(1 << AIN1_PIN);

    GPIOA->ODR &= ~(1 << AIN2_PIN);

}



// FreeRTOS Task for Gas Monitoring

void SafetySystem::GasTask(void)

{

        char str[20];

        led_control(result);

       // sprintf(str,(char*) "Gas: %lu%%", gas);

        sprintf(str, "Gas: %lu%%", static_cast<unsigned long>(gas));

        lprint(0x80,(char*) "Monitoring Gas");  // Line 1

        lprint(0xC0, str);               // Line 2

}



// === FreeRTOS Task for Keypad & LCD ===

void Task_Keypad(void *params)

{

    char passkey[5] = "1234";

    char entered[5];

    int key_index = 0;

    lprint(0x80, (char*)"Welcome");

    vTaskDelay(pdMS_TO_TICKS(1000));



    while (1)

    {

        key_index = 0;

        memset(entered, 0, sizeof(entered));

        while (1)

        {

            char key = sys.Keypad_Scan();

            if (key != 0)

            {

                if (key == '*')

                {

                    key_index = 0;

                    memset(entered, 0, sizeof(entered));

                    lprint(0x80,(char*) "Input Passkey      ");

                    lprint(0xC0,(char*) "                   ");

                }



                else if (key == '#' && key_index > 0)

                {

                    entered[key_index] = '\0';

                    if (strcmp(entered, passkey) == 0)

                    {

                        LcdFxn(0,0x01);

                        lprint(0x80,(char*) "Please Come In   ");

                        GPIOB->ODR |= (1 << 12);  // Green ON

                        GPIOB->ODR &= ~(1 << 8); // Red OFF

                        vTaskDelay(pdMS_TO_TICKS(3000));

                        xSemaphoreGive(xAuthSemaphore);

                        break;

                    }



                    else

                    {

                    	LcdFxn(0,0x01);

                        lprint(0x80,(char*) "Wrong Passkey!!!    ");

                        GPIOB->ODR |= (1 << 8);  // Red ON

                        GPIOB->ODR &= ~(1 << 12); // Green OFF

                        vTaskDelay(pdMS_TO_TICKS(2000));

                        lprint(0x80, (char*)"Try Again!!!        ");

                        vTaskDelay(pdMS_TO_TICKS(1000));

                        lprint(0x80,(char*) "Input Passkey       ");

                        lprint(0xC0,(char*) "                    ");

                        key_index = 0;

                        memset(entered, 0, sizeof(entered));

                    }

                }

                else if (key_index < 4)

                {

                    entered[key_index++] = key;

                    char buf[17] = "Entered: ";

                    for (int i = 0; i < key_index; i++)

                                buf[9 + i] = '*';

                   // strncpy(&buf[9], entered, key_index);

                   // buf[9 + key_index] = '\0';

                    lprint(0xC0, buf);

                }

            }

            vTaskDelay(pdMS_TO_TICKS(50));

        }

    }

}



// FreeRTOS Task for Control

void ControlTask(void *argument)

{

xSemaphoreTake(xAuthSemaphore, portMAX_DELAY);

LcdFxn(0,0x01);

GPIOB->ODR &=~  (1 << 8);

GPIOB->ODR &=~  (1 << 9);

GPIOB->ODR &=~  (1 << 11);

    while (1)

    {

        result = sys.adc_read1();

        gas = result / 41;

        flameDetected = (GPIOA->IDR & (1 << 2)) ? 1 : 0; // PA2 input

        adc_value2 = sys.adc_read2();

        voltage = (adc_value2 / 4095.0) * 3.3;

        t1 = -50.0 * voltage;

        temperature = t1 + 107.5;

        if(gas>20 && (flameDetected == 1) && temperature > 27.0)//111

        {

        	    sys.GasTask();

                GPIOB->ODR |=  (1 << 10);// Buzzer ON

                GPIOB->ODR &=~  (1 << 8);

                GPIOB->ODR |=  (1 << 8); // TOGGLE RED

                GPIOB->ODR &=~  (1 << 9);

                GPIOB->ODR |=  (1 << 9); // TOGGLE YELLOW

                GPIOB->ODR &=~  (1 << 11); // GREEN OFF

                sys.pump_on();

                sys.motor_forward();

        }

        else if(gas>20 && (flameDetected == 1) && temperature < 27.0) // 110

        {

        	    sys.GasTask();

           	    GPIOB->ODR |=  (1 << 10);// Buzzer ON

           		GPIOB->ODR &=~  (1 << 8);

           		GPIOB->ODR |=  (1 << 8);// TOGGLE RED

           		GPIOB->ODR &=~  (1 << 9);// YELLOW OFF

           		GPIOB->ODR &=~  (1 << 11); // GREEN OFF

           		sys.pump_on();

           		sys.motor_stop();

        }

        else if(gas>20 && !(flameDetected == 1) && temperature > 27.0) //101

        {

        	    sys.GasTask();

                GPIOB->ODR |=  (1 << 10);// Buzzer ON

                GPIOB->ODR &=~  (1 << 8);// RED  OFF

                GPIOB->ODR &=~  (1 << 9);

                GPIOB->ODR |=  (1 << 9);// TOGGLE YELLOW

                GPIOB->ODR &=~  (1 << 11); // GREEN OFF

                sys.pump_off();

                sys.motor_forward();

        }

        else if(gas>20 && !(flameDetected == 1) && temperature < 27.0) //100

        {

        	    sys.GasTask();

        	    GPIOB->ODR &=~  (1 << 10);// Buzzer OFF

        	    GPIOB->ODR &=~  (1 << 8);// RED  OFF

        	    GPIOB->ODR &=~  (1 << 9);// YELLOW OFF

        	    GPIOB->ODR &=~  (1 << 11); // GREEN OFF

        	    sys.pump_off();

        	    sys.motor_stop();

        }

        else if(gas<20 && (flameDetected == 1) && temperature > 27.0)//011

        {

                GPIOB->ODR |=  (1 << 10);// Buzzer ON

                GPIOB->ODR &=~  (1 << 8);

                GPIOB->ODR |=  (1 << 8);// TOGGLE RED

                GPIOB->ODR &=~  (1 << 9);

                GPIOB->ODR |=  (1 << 9);// TOGGLE YELLOW

                GPIOB->ODR &=~  (1 << 11); // GREEN OFF

                sys.pump_on();

                sys.motor_forward();

                lprint(0x80,(char*) "FIRE ALERT!!!             ");

                lprint(0xC0,(char*) "SMOKE ALERT!!!            ");

        }

        else if(gas<20 && (flameDetected == 1) && temperature < 27.0) //010

        {

    	        GPIOB->ODR |=  (1 << 10);// Buzzer ON

    		    GPIOB->ODR &=~  (1 << 8);

    		    GPIOB->ODR |=  (1 << 8);// TOGGLE RED

    		    GPIOB->ODR &=~  (1 << 9);// YELLOW OFF

    		    GPIOB->ODR &=~  (1 << 11); // GREEN OFF

    		    sys.pump_on();

    		    sys.motor_stop();

          	    lprint(0x80,(char*) "FIRE ALERT!!!              ");

          	    lprint(0xC0,(char*) "                           ");

        }

        else if(gas<20 && !(flameDetected == 1) && temperature > 27.0) //001

        {

                GPIOB->ODR |=  (1 << 10);// Buzzer ON

                GPIOB->ODR &=~  (1 << 8);// RED  OFF

                GPIOB->ODR &=~  (1 << 9);

                GPIOB->ODR |=  (1 << 9);// TOGGLE YELLOW

                GPIOB->ODR &=~  (1 << 11); // GREEN OFF

                sys.pump_off();

                sys.motor_forward();

                lprint(0x80,(char*) "SMOKE ALERT!!!             ");

                lprint(0xC0, (char*)"                           ");

        }

        else                                                          //000

        {

        	    GPIOB->ODR &=~  (1 << 10);// Buzzer OFF

        	    GPIOB->ODR &=~  (1 << 8);// RED  OFF

        	    GPIOB->ODR &=~  (1 << 9);// YELLOW OFF

        	    sys.pump_off();

        	    sys.motor_stop();

        	    GPIOB->ODR |=  (1 << 11);// GREEN LED ON

        	    lprint(0x80,(char*) "SAFE                       ");

        	    lprint(0xC0,(char*) "                           ");

        }



        vTaskDelay(pdMS_TO_TICKS(100));

    }

}



/* USER CODE END PTD */



/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */

/* USER CODE END PD */



/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */



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

  /* USER CODE BEGIN 2 */

     sys.GPIO_Init();

     sys.adc_init();

     sys.led_init();

     sys.init_GPIO();

     sys.init_motor_GPIO();

      LcdInit();



      xAuthSemaphore= xSemaphoreCreateBinary();

      xTaskCreate(Task_Keypad, "Keypad", 256, NULL, 2, NULL);

      xTaskCreate(ControlTask, "ControlTask", 256, NULL, 1, &ControlTaskHandle);

      vTaskStartScheduler();

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

  RCC_OscInitStruct.PLL.PLLM = 16;

  RCC_OscInitStruct.PLL.PLLN = 192;

  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;

  RCC_OscInitStruct.PLL.PLLQ = 4;

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

  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;

  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;



  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)

  {

    Error_Handler();

  }

}



/* USER CODE BEGIN 4 */



/* USER CODE END 4 */



/**

  * @brief  Period elapsed callback in non blocking mode

  * @note   This function is called  when TIM3 interrupt took place, inside

  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment

  * a global variable "uwTick" used as application time base.

  * @param  htim : TIM handle

  * @retval None

  */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)

{

  /* USER CODE BEGIN Callback 0 */



  /* USER CODE END Callback 0 */

  if (htim->Instance == TIM3) {

    HAL_IncTick();

  }

  /* USER CODE BEGIN Callback 1 */



  /* USER CODE END Callback 1 */

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

