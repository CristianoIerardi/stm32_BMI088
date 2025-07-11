/***********************************************************************************
  * @file    mcp3564.c
  * @brief   C Library for configuring the MCP3561/2/4 1/2/4 channel
  *          24 Bit sigma-delta ADC on STM32
  * @version 0.1
  * @date    2021-11-29
  * @license Apache 2.0
  * @author  Simon Burkhardt
  *
  *          FHNW University of Applied Sciences and Arts Northwestern Switzerland
  *          https://www.fhnw.ch/ise/
  *          https://github.com/fhnw-ise-qcrypt/mcp3564
  *
  *          GAP Quantum Technologies University of Geneva
  *          https://www.unige.ch/gap/qic/qtech/
  *
  * @see     https://www.microchip.com/en-us/product/MCP3561
************************************************************************************
*/

#include "main.h"
#include "mcp3564.h"

/* --- DMA storage variables ----------------*/
#define MCP3561_DMA_RX_SIZE 6
uint8_t mcp356x_rx_buf[MCP3561_DMA_RX_SIZE];
const uint8_t mcp356x_tx_buf[MCP3561_DMA_RX_SIZE] = {
    MCP3561_SREAD_DATA_COMMAND,  // Comando di lettura
    0x00, 0x00, 0x00, 0x00, 0x00 // Padding
};
/* ----------------------------------------- */

void _MCP3561_write(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t size){
	// manually operate the !CS signal, because the STM32 hardware NSS signal is (sadly) useless
	HAL_GPIO_WritePin(MCP3561_CHIP_SELECT_GPIO_Port, MCP3561_CHIP_SELECT_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, pData, size, MCP3561_HAL_TIMEOUT);
	HAL_GPIO_WritePin(MCP3561_CHIP_SELECT_GPIO_Port, MCP3561_CHIP_SELECT_GPIO_Pin, GPIO_PIN_SET);
}

uint8_t _MCP3561_sread(SPI_HandleTypeDef *hspi, uint8_t *cmd){
	uint8_t reg8[2];
	// manually operate the !CS signal, because the STM32 hardware NSS signal is (sadly) useless
	HAL_GPIO_WritePin(MCP3561_CHIP_SELECT_GPIO_Port, MCP3561_CHIP_SELECT_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspi, cmd, reg8, 2, MCP3561_HAL_TIMEOUT);
	HAL_GPIO_WritePin(MCP3561_CHIP_SELECT_GPIO_Port, MCP3561_CHIP_SELECT_GPIO_Pin, GPIO_PIN_SET);
	return reg8[1];
}

/**
 * @brief  Initializes the MCP356x chip according to user config
 * @note   must be edited by the user
 */
void MCP3561_Init(SPI_HandleTypeDef *hspi){
	uint8_t cmd[4] = {0,0,0,0};

	// be careful with the bitwise or operator "|"
	cmd[0]  = MCP3561_CONFIG0_WRITE;
	cmd[1]  = MCP3561_CONFIG0_CLK_SEL_EXT;   // clock selection
	cmd[1] |= MCP3561_CONFIG0_ADC_MODE_CONV; // standby or converting
	cmd[1] |= MCP3561_CONFIG0_CS_SEL_NONE;   // input current
	cmd[1] |= (1 << 7);                      // Enable extern VREF (VREF_SEL = 1)	/* added by user*/
	_MCP3561_write(hspi, cmd, 2);

	cmd[0]  = MCP3561_CONFIG1_WRITE;
	cmd[1]  = MCP3561_CONFIG1_OSR_256;       // over sampling rate
	cmd[1] |= MCP3561_CONFIG1_AMCLK_DIV8;    // sampling clock prescaler
	_MCP3561_write(hspi, cmd, 2);

	cmd[0]  = MCP3561_CONFIG2_WRITE;
	cmd[1]  = MCP3561_CONFIG2_BOOST_x1;   // Boost
	cmd[1] |= MCP3561_CONFIG2_GAIN_x64;    // Gain
	cmd[1] |= MCP3561_CONFIG2_AZ_MUX_OFF; // offset cancellation algorithm
	cmd[1] += 3; // last two bytes must always be '11'
	_MCP3561_write(hspi, cmd, 2);

	cmd[0]  = MCP3561_CONFIG3_WRITE;
	cmd[1]  = MCP3561_CONFIG3_CONV_MODE_CONTINUOUS; // conversion mode
	cmd[1] |= MCP3561_CONFIG3_DATA_FORMAT_32BIT_CHID_SGN; 	//
	cmd[1] |= MCP3561_CONFIG3_CRCCOM_OFF;            // CRC
	cmd[1] |= MCP3561_CONFIG3_GAINCAL_OFF;          // gain calibration
	cmd[1] |= MCP3561_CONFIG3_OFFCAL_OFF;           // offset calibration
	_MCP3561_write(hspi, cmd, 2);


	cmd[0]  = MCP3561_IRQ_WRITE;
	cmd[1]  = MCP3561_IRQ_MODE_IRQ_HIGH;  // IRQ default pin state
	cmd[1] |= MCP3561_IRQ_FASTCMD_ON;     // fast commands
	cmd[1] |= MCP3561_IRQ_STP_ON;         // start of conversion IRQ
	_MCP3561_write(hspi, cmd, 2);

	cmd[0]  = MCP3561_MUX_WRITE;
	//cmd[1]  = (MCP3561_MUX_CH0 << 4) | MCP3561_MUX_CH1;   // [7..4] VIN+ / [3..0] VIN-
	//cmd[1]  = (MCP3561_MUX_CH_AVDD << _MCP3561_MUX_VIN_P_POS) | (MCP3561_MUX_CH_AGND << _MCP3561_MUX_VIN_N_POS);
	_MCP3561_write(hspi, cmd, 2);

	// configure SCAN mode to automatically cycle through channels
	// only available for MCP3562 and MCP356^4, and only for certain input combinations
	// @see Datasheet Table 5-14 on p. 54

	cmd[0] = MCP3561_SCAN_WRITE;
	cmd[1] = MCP3561_SCAN_DLY_NONE;
	cmd[2] = 0x0F;  // MBS
	cmd[3] = 0x00;	// LSB
	_MCP3561_write(hspi, cmd, 4);

}

/**
 * @brief prints the configuration registers content
 */
void MCP3561_PrintRegisters(SPI_HandleTypeDef *hspi){
	uint8_t reg8 = 0;
	uint8_t cmd [5] = {0,0,0,0,0};

	cmd[0] = MCP3561_CONFIG0_SREAD;
	reg8 = _MCP3561_sread(hspi, cmd);
	printf("CONF0: %02x\n", reg8);

	cmd[0] = MCP3561_CONFIG1_SREAD;
	reg8 = _MCP3561_sread(hspi, cmd);
	printf("CONF1: %02x\n", reg8);

	cmd[0] = MCP3561_CONFIG2_SREAD;
	reg8 = _MCP3561_sread(hspi, cmd);
	printf("CONF2: %02x\n", reg8);

	cmd[0] = MCP3561_CONFIG3_SREAD;
	reg8 = _MCP3561_sread(hspi, cmd);
	printf("CONF3: %02x\n", reg8);

	cmd[0] = MCP3561_IRQ_SREAD;
	reg8 = _MCP3561_sread(hspi, cmd);
	printf("IRQ  : %02x\n", reg8);

	cmd[0] = MCP3561_MUX_SREAD;
	reg8 = _MCP3561_sread(hspi, cmd);
	printf("MUX  : %02x\n", reg8);

	/*ADDED BY USER . . .*/
    cmd[0] = MCP3561_SCAN_SREAD; // Assicurati di avere MCP3561_SCAN_SREAD definito nel tuo .h
    reg8 = _MCP3561_sread(hspi, cmd);
    printf("SCAN : %02x\n", reg8);

    cmd[0] = MCP3561_SCAN_SREAD; // Questa leggerà SCAN_H (0x09)
    reg8 = _MCP3561_sread(hspi, cmd);
    printf("SCAN_H : %02x\n", reg8);

    // Per leggere SCAN_M (0x09 + 1 = 0x0A)
    cmd[0] = ( (MCP3561_SCAN_ADDR + 1) << _MCP3561_COMMAND_ADDR_POS) | MCP3561_SREAD_DATA_COMMAND;
    reg8 = _MCP3561_sread(hspi, cmd);
    printf("SCAN_M : %02x\n", reg8);

    // Per leggere SCAN_L (0x09 + 2 = 0x0B)
    cmd[0] = ( (MCP3561_SCAN_ADDR + 2) << _MCP3561_COMMAND_ADDR_POS) | MCP3561_SREAD_DATA_COMMAND;
    reg8 = _MCP3561_sread(hspi, cmd);
    printf("SCAN_L : %02x\n", reg8);

	/* @todo all the remaining registers */
}

/**
 * @brief resets the configuration to the default values
 * @todo  test this function
 */
void MCP3561_Reset(SPI_HandleTypeDef *hspi){
	uint8_t cmd;
	cmd = DEVICE_RESET_COMMAND;
	HAL_SPI_Transmit(hspi, &cmd, 1, 10);
}



/**
 * @brief read 24 Bit left justified ADC register
 * @todo  how to read from other data formats?
 */
uint8_t MCP3561_ReadADCData(SPI_HandleTypeDef *hspi, float *adc_volt)
{
	uint8_t val[6] = {0,0,0,0,0,0};
	uint8_t cmd[6] = {0,0,0,0,0,0};
	cmd[0] = MCP3561_SREAD_DATA_COMMAND;

	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0);
	HAL_SPI_TransmitReceive(hspi, &cmd[0], &val[0], 6, 1);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1);

	//printf("%02X %02X %02X %02X %02X %02X\t\t", val[0], val[1], val[2], val[3], val[4], val[5]);

	// Extract 24 bit raw data values
	uint8_t channel_id = 0;
	uint32_t current_adc_raw_val = 0xFFFF;
	if(val[0] != 0x13)			// Delay with the transmission
	{
		if((val[2] & 0x80) != 0)
		{
			current_adc_raw_val = 	((uint32_t)val[3] << 16) |
									((uint32_t)val[4] << 8)  |
									((uint32_t)val[5]);
			// Find corresponding channel
			channel_id = (val[2] & 0x30) >> 4; // Extract the ID
		}
		else
		{
			return 0;
		}
	}
	else if(val[0] == 0x13)		// if it's not 0x13 it is trush data
	{
		if((val[1] & 0x80) != 0)
		{
			current_adc_raw_val = 	((uint32_t)val[2] << 16) |
									((uint32_t)val[3] << 8)  |
									((uint32_t)val[4]);
			// Find corresponding channel
			channel_id = (val[1] & 0x30) >> 4; // Extract the ID
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}

	// Convert from row data to float
	adc_volt[channel_id] = convertAdcToVoltage(current_adc_raw_val);

	// If it is the channel 1 it means that we did an entire conversion of every differential channel
	if(channel_id == 0)
		return 1;
	else
		return 0;

	//printf("ch: %d\t V: %.6f V\n", channel_id, adc_volt[channel_id]);

}

void MCP3561_StartReadADCData_DMA(SPI_HandleTypeDef *hspi)
{
    // CS pin low
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    // Start transmission
    HAL_SPI_TransmitReceive_DMA(hspi, mcp356x_tx_buf, mcp356x_rx_buf, MCP3561_DMA_RX_SIZE);
}

/**
 * @brief read 24 Bit left justified ADC register
 * @todo  how to read from other data formats?
 */
uint8_t MCP3561_ReadADCData_DMA(SPI_HandleTypeDef *hspi, float *adc_volt)
{
	uint8_t *val = mcp356x_rx_buf;

	//printf("%02X %02X %02X %02X %02X %02X\t\t", val[0], val[1], val[2], val[3], val[4], val[5]);	// Debug received data

	// Extract 24 bit raw data values
	uint8_t channel_id = 0;
	uint32_t current_adc_raw_val = 0xFFFF;
	if(val[0] != 0x13)			// Delay with the transmission
	{
		if((val[2] & 0x80) != 0)
		{
			current_adc_raw_val = 	((uint32_t)val[3] << 16) |
									((uint32_t)val[4] << 8)  |
									((uint32_t)val[5]);
			// Find corresponding channel
			channel_id = (val[2] & 0x30) >> 4; // Extract the ID
		}
		else
		{
			return 0;
		}
	}
	else if(val[0] == 0x13)		// if it's not 0x13 it is trush data
	{
		if((val[1] & 0x80) != 0)
		{
			current_adc_raw_val = 	((uint32_t)val[2] << 16) |
									((uint32_t)val[3] << 8)  |
									((uint32_t)val[4]);
			// Find corresponding channel
			channel_id = (val[1] & 0x30) >> 4; // Extract the ID
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}

	// Convert from row data to float
	adc_volt[channel_id] = convertAdcToVoltage(current_adc_raw_val);

	// If it is the channel 1 it means that we did an entire conversion of every differential channel
	if(channel_id == 0)
		return 1;
	else
		return 0;

	//printf("ch: %d\t V: %.6f V\n", channel_id, adc_volt[channel_id]);

}



























/*
 * @bug this does not work because it will skip the transaction and write to the chip select pin
 */
uint32_t MCP3561_ReadADCData_IT(SPI_HandleTypeDef *hspi){
	uint8_t val[5] = {0,0,0,0,0};
	uint8_t cmd[5] = {0,0,0,0,0};
	cmd[0] = MCP3561_SREAD_DATA_COMMAND;

	// manually operate the !CS signal, because the STM32 hardware NSS signal is (sadly) useless
	HAL_GPIO_WritePin(MCP3561_CHIP_SELECT_GPIO_Port, MCP3561_CHIP_SELECT_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspi, cmd, val, 5, MCP3561_HAL_TIMEOUT);
	HAL_GPIO_WritePin(MCP3561_CHIP_SELECT_GPIO_Port, MCP3561_CHIP_SELECT_GPIO_Pin, GPIO_PIN_SET);

	//uint32_t value = (val[1] << 16) | (val[2] << 8) | val[3];
	uint32_t value = ((uint32_t)val[0] << 24) |
	                 ((uint32_t)val[1] << 16) |
	                 ((uint32_t)val[2] << 8)  |
	                 ((uint32_t)val[3]);
	return value;
}



// Function to convert a row value into float voltage
double convertAdcToVoltage(uint32_t raw_adc_value) {
    int32_t signed_adc_value;

    // Convert the unsigned 24-bit value (from uint32_t) to a signed 24-bit value (in int32_t)
    // If bit 23 is set (MSB of a 24-bit number), it is negative.
    if ((raw_adc_value & 0x00800000) != 0) { 						// If bit 23 is 1, it is a negative number
        signed_adc_value = (int32_t)(raw_adc_value | 0xFF000000); 	// Extend the sign for the upper bits (only for display in int32_t)
    } else {
        signed_adc_value = (int32_t)raw_adc_value;
    }

    double voltage = ((double)signed_adc_value * VREF_VOLTAGE_V) / (ADC_MAX_COUNT * ADC_GAIN);
    return voltage;
}

