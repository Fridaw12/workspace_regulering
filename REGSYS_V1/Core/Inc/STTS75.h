#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_i2c.h"

#ifndef STTS75_H_
#define STTS75_H_

//kommenter ut hvilken du ikke skal ha
// A0,A1 og A2 pinnene bestemmer adressen til temperatursensoren:


//viss alle pinnene er koblet til jord:
#define STTS75_I2C_AD_reg ( 0x48U << 1) // A0 ,A1 A2 = GND
#define STTS75_I2C_AD_driv ( 0x49U << 1)
//viss alle pinnene er koblet til VDD:
//#define STTS75_I2C_ADDR (0x4FU << 1) // A0 ,A1 A2 = VDD

#define STTS75_WRITE (STTS75_I2C_AD_reg & 0xFEU)
#define STTS75_WRITE_2 0b10010000


#define STTS75_COMMAND_POINTER_WRITE 0x01U

#define STTS75_TEMP_REG 0x00U
#define STTS75_CONFIG_REG 0x1
#define STTS75_HYSTERESIS_REG 0x02U
#define STTS75_OVERTEMPERATURE_SHUTDOWN_REG 0x03U
#define	STTS75_9BIT 0x00U		//0.5
#define STTS75_10BIT (0x01U  << 5)	//0.25
#define STTS75_11BIT (0x02U << 5)		//0.125
#define STTS75_12BIT (0x03U << 5)//0.0625


typedef struct
{
    I2C_HandleTypeDef *hi2c;
    int16_t temp_raw;
    float temp_degc_reg;
    float temp_degc_driv;
    uint16_t temp_16bit_reg;
    uint16_t temp_16bit_driv;
    uint8_t bitsize;
    uint8_t config_register;
    int16_t shutdown_temp_degree_raw;
    float shutdown_temp_degree_float;
    uint8_t error;

} STTS75;

// bitmønster definert som error-melding: lettere lesbart
// brukt i feilsøking
typedef enum
{
    STTS75_OK             = 0x00U,
    STTS75_HAL_ERROR      = 0x01U,
    STTS75_HAL_BUSY       = 0x02U,
    STTS75_HAL_TIMEOUT    = 0x03U,
} STTS75_StatusTypeDef;


typedef enum{
	shutdown_80			    = 0x5000,		// default , decimal 5000
	shutdown_90			    = 0x5A00,
	shutdown_95			    = 0x5F00,
	shutdown_100			= 0x6400,
	shutdown_110			= 0x6E00,
	shutdown_120			= 0x7800,
	shutdown_124			= 0x7C00,
	shutdown_125			= 0x7D00,
} SHUTDOWN_TEMP;


void STTS75_Init(STTS75 *sensor, I2C_HandleTypeDef *hi2c, uint32_t resolution);
STTS75_StatusTypeDef STTS75_Read_Regsiter(STTS75 *sensor, uint8_t reg, uint8_t *rx_buf);
STTS75_StatusTypeDef STTS75_Write_Register(STTS75 *sensor, uint8_t reg);
STTS75_StatusTypeDef STTS75_Read_Temp(STTS75 *sensor);
STTS75_StatusTypeDef STTS75_Write_Bitsize(STTS75 *sensor, uint32_t resolution);
STTS75_StatusTypeDef STTS75_Set_Overheat_Shutdown(STTS75 *sensor, SHUTDOWN_TEMP shutdown_temp);

#endif /* ifndef STTS75_H_ */
