
#include "STTS75.h"
#include "stm32g4xx_hal_i2c.h"
#include "stm32g4xx_hal.h"

void STTS75_Init(STTS75 *sensor, I2C_HandleTypeDef *hi2c, uint32_t resolution)
{
    STTS75_StatusTypeDef retval = STTS75_OK;
    sensor->hi2c = hi2c;

    if (resolution != STTS75_9BIT)
    {
        STTS75_Write_Bitsize(sensor, resolution);
    }

}
STTS75_StatusTypeDef STTS75_Read_Regsiter(STTS75 *sensor, uint8_t reg, uint8_t *rx_buf)
{
    return (STTS75_StatusTypeDef) HAL_I2C_Mem_Read(sensor->hi2c, STTS75_I2C_AD_reg, reg, 1, rx_buf, 1, 1000);
    //
}

STTS75_StatusTypeDef STTS75_Read_Temp(STTS75 *sensor)
{
    STTS75_StatusTypeDef retval;
    sensor->error = 0x00;

    //reg-kort
    uint8_t rx_buf[2];
    retval = ( STTS75_StatusTypeDef)HAL_I2C_Mem_Read(sensor->hi2c, STTS75_I2C_AD_reg, STTS75_TEMP_REG, 1, rx_buf, 2, 1000);

    if (retval != STTS75_OK) {
    	sensor->error = 0xC8U;
    	return retval;
    }
    sensor->temp_raw = rx_buf[0] << 8 | rx_buf[1];
    sensor->temp_degc_reg = (float) (sensor->temp_raw / 256.0f)*100;

    //denne verdien logges på til CAN og COM
    sensor->temp_16bit_reg =(uint16_t) ((sensor->temp_degc_reg));

    //driv-kort

    // ( Dersom får error: statusen til STTS75_StatusTypeDef er 1 )
    // retval -> returvalue

    retval = ( STTS75_StatusTypeDef)HAL_I2C_Mem_Read(sensor->hi2c, STTS75_I2C_AD_driv, STTS75_TEMP_REG, 1, rx_buf, 2, 1000);

    if (retval != STTS75_OK) {
    	sensor->error = 0xC8U;
    	return retval;
    }
    sensor->temp_raw = rx_buf[0] << 8 | rx_buf[1];
    sensor->temp_degc_driv = (float) (sensor->temp_raw / 256.0f)*100;

    //denne verdien logges på til CAN og COM
    sensor->temp_16bit_driv =(uint16_t) ((sensor->temp_degc_driv));

    return retval;
}


STTS75_StatusTypeDef STTS75_Write_Bitsize(STTS75 *sensor, uint32_t resolution)
{
	STTS75_StatusTypeDef retval = STTS75_OK;

	uint8_t bits_in_config_reg;
	uint8_t look_in_conf_reg;

	// 1.
	//read to confirm bit size change
	retval = STTS75_Read_Regsiter(sensor, STTS75_CONFIG_REG, &look_in_conf_reg);
	if (retval != STTS75_OK) return retval;
	sensor->config_register = look_in_conf_reg;


	//HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout)
	bits_in_config_reg &= ~(0x03 << 5); //settter de to config register bits til 0 ved '&' med ~0x03<<5 = 110011111
	bits_in_config_reg |= resolution; // gjør en eller operasjon for å fylle de to plassene med ønsket bit


	//2.
	retval = (STTS75_StatusTypeDef) HAL_I2C_Mem_Write(sensor->hi2c, 
            STTS75_WRITE,STTS75_CONFIG_REG, 1, 
            &bits_in_config_reg, 1, 1000);

	if (retval != STTS75_OK) return retval;


	//3.
	//read to confirm bit size change
	retval = STTS75_Read_Regsiter(sensor, STTS75_CONFIG_REG, &look_in_conf_reg);
	if (retval != STTS75_OK) return retval;

	//sensor->bit size = choose_bit;
	sensor->config_register = look_in_conf_reg;
	sensor->bitsize = resolution;
	return retval;

}

