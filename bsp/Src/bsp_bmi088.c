#include "bsp_bmi088.h"
#include "main.h"
#include "spi.h"
#define BMI088_USE_SPI 
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_Accel_GPIO_Port,CS1_Accel_Pin,GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_Accel_GPIO_Port,CS1_Accel_Pin,GPIO_PIN_SET);
}
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port,CS1_Gyro_Pin,GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port,CS1_Gyro_Pin,GPIO_PIN_SET);
}
uint8_t BMI088_Read_Write_Byte(uint8_t Txdata)
{
    uint8_t Rxdata = 0;

		HAL_SPI_TransmitReceive(&hspi1,&Txdata,&Rxdata,1,1000);
	
    return Rxdata;
}



