#include "ADgpio.h"
#include "ADspi.h"
#include "ADtim.h"
#include "AD7606.h"

//SPI_HandleTypeDef hspi_AD7190;

void AD7606Reset(void)
{
    /*! ___|-----|________  >= 50ns */
    AD7606Rst_Low();
    AD7606Rst_High();
    for(int i = 20; i > 0; i--){
        __NOP();//1000/168 ns = 5.85ns
    }
    AD7606Rst_Low();
}

void AD7606_start(void)
{  
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1); 
}

void AD7606Stop(void)
{
    HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
    AD7606Cs_High();
    
}
void AD7606_ReadData(int16_t * DB_data) 
{ 
	HAL_SPI_Receive(&hspi1, (uint8_t *)DB_data, 8, 1000);
} 

uint8_t SPI1_ReadWriteByte(uint8_t TxData)//发送一个字节，并从寄存器返回一个字节
{
	uint8_t Rxdata;
	HAL_SPI_TransmitReceive(&hspi1, &TxData, &Rxdata, 1, 1000);       
 	return Rxdata;          		    
}

/*uint8_t SPI1_ReadWriteByte(void)//发送一个字节，并从寄存器返回一个字节
{
	uint8_t Rxdata;
	HAL_SPI_Receive(&hspi_AD7190, &Rxdata, 1, 1000);       
 	return Rxdata;          		    
}*/



