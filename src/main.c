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
#include "stm32f429xx.h"
#include <stdint.h>
/* USER CODE BEGIN Includes */
#include "lib_gpio.h"
#include "lib_i2c.h"
#include "mpu6050.h"
#include "lib_uart.h"
//#include <SysprogsProfiler.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern I2C_TypeDef *i2c;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//I2C_TypeDef *i2c;


void process_new_sensor_values(float gyro_x, float gyro_y, float gyro_z, float accel_x, float accel_y, float accel_z, float magn_x, float magn_y, float magn_z) {

	// sensor fusion with Madgwick's Filter
	// MadgwickAHRSupdate(gyro_z, gyro_y, -gyro_x, accel_z, accel_y, -accel_x, magn_z, magn_y, -magn_x);
	uart_send_csv_floats(3,
		accel_x,accel_y, accel_z);
	return;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	  SystemCoreClockUpdate();
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */

  /* USER CODE BEGIN SysInit */
//	InitializeSamplingProfiler();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
 // MX_GPIO_Init();
 // MX_I2C2_Init();
 // MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
	// i2c device addresses
	//#define MPU6050_ADDRESS  0b1101000
	//#define HMC5883L_ADDRESS 0b0011110
	// configure i2c
	//i2c = I2C2;
	//i2c_setup(i2c, FAST_MODE_400KHZ, PF1, PF0);
    //uint8_t rx_buffer[20];

	//readBytes(MPU6050_ADDRESS, hi2c2, 0x3B, &rx_buffer[0], 14);

	// configure the MPU6050 (gyro/accelerometer)
	//i2c_write_register(i2c, MPU6050_ADDRESS, 0x6B, 0x00);                      // exit sleep
	//i2c_write_register(i2c, MPU6050_ADDRESS, 0x19, 109);                       // sample rate = 8kHz / 110 = 72.7Hz
	//i2c_write_register(i2c, MPU6050_ADDRESS, 0x1B, 0x18);                      // gyro full scale = +/- 2000dps
	//i2c_write_register(i2c, MPU6050_ADDRESS, 0x1C, 0x08);                      // accelerometer full scale = +/- 4g
	//i2c_write_register(i2c, MPU6050_ADDRESS, 0x38, 0x01);                      // enable INTA interrupt

	// configure the HMC5883L (magnetometer)
//	i2c_write_register(i2c, MPU6050_ADDRESS, 0x6A, 0x00);                      // disable i2c master mode
//	i2c_write_register(i2c, MPU6050_ADDRESS, 0x37, 0x02);                      // enable i2c master bypass mode
//	i2c_write_register(i2c, HMC5883L_ADDRESS, 0x00, 0x18);                      // sample rate = 75Hz
//	i2c_write_register(i2c, HMC5883L_ADDRESS, 0x01, 0x60);                      // full scale = +/- 2.5 Gauss
//	i2c_write_register(i2c, HMC5883L_ADDRESS, 0x02, 0x00);                      // continuous measurement mode
//	i2c_write_register(i2c, MPU6050_ADDRESS, 0x37, 0x00);                      // disable i2c master bypass mode
//	i2c_write_register(i2c, MPU6050_ADDRESS, 0x6A, 0x20);                      // enable i2c master mode

	// configure the MPU6050 to automatically read the magnetometer
//	i2c_write_register(i2c, MPU6050_ADDRESS, 0x25, HMC5883L_ADDRESS | 0x80);   // slave 0 i2c address, read mode
//	i2c_write_register(i2c, MPU6050_ADDRESS, 0x26, 0x03);                      // slave 0 register = 0x03 (x axis)
//	i2c_write_register(i2c, MPU6050_ADDRESS, 0x27, 6 | 0x80);                  // slave 0 transfer size = 6, enabled
//	i2c_write_register(i2c, MPU6050_ADDRESS, 0x67, 1);                         // enable slave 0 delay
	gpio_setup(PB7, OUTPUT, PUSH_PULL, FIFTY_MHZ, NO_PULL, AF0);
	mpu6050_hmc5883l_setup(PF1, PF0, PF2, &process_new_sensor_values);
	uart_setup(PD8, 115200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  gpio_high(PB7);
	  for (int i=0;i<10000000;i++);
	//  i2c_read_registers(i2c, MPU6050_ADDRESS, 20, 0x3B, rx_buffer);
	  gpio_low(PB7);
	  for (int i=0;i<10000000;i++);

	//  test_function();
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/




/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
