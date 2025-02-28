#include "tas5827.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief begin with the I2C address and the I2C handle
 *
 * @param address 7-bits I2C address
 * @param DUMMY_I2C_HANDLE TODO: PLEASE HANDLE THIS
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::begin(uint8_t address, uint8_t DUMMY_I2C_HANDLE)
{
	this->address    = address;
	this->i2cHandler = DUMMY_I2C_HANDLE;
	return true;
}

/* ------------------------------------------------------------ */
/* Setters                                                      */
/* ------------------------------------------------------------ */

/**
 * @brief set the loop bandwidth for the class D amplifier
 *
 * @param loopBW only can be 80, 100, 120 or 175 kHz
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setLoopBW(Loop_BW_t loopBW)
{
	// PHASE_CTL here is always set to 0 = out of phase, since it is not used
	// only one TAS5827 is used
	return writeRegister(REG_ANA_CTRL, static_cast<uint8_t>(loopBW));
}

/**
 * @brief set the analog gain
 *
 * @param gain 0 to 31 (0 dB to -15.5 dB)
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setAnalogGain(uint8_t gain)
{
	gain = gain & 0x1F;
	return writeRegister(REG_AGAIN, gain);
}

/**
 * @brief set the GPIOs to be input/output
 *
 * @param gpio_ctrl_0 GPIO 0 input/output
 * @param gpio_ctrl_1 GPIO 1 input/output
 * @param gpio_ctrl_2 GPIO 2 input/output
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setGPIOMode(GPIO_Mode_t gpio_ctrl_0, GPIO_Mode_t gpio_ctrl_1, GPIO_Mode_t gpio_ctrl_2)
{
	uint8_t gpio_ctrl = 0;

	gpio_ctrl |= static_cast<uint8_t>(gpio_ctrl_2) & 0x01 << 2;
	gpio_ctrl |= static_cast<uint8_t>(gpio_ctrl_1) & 0x01 << 1;
	gpio_ctrl |= static_cast<uint8_t>(gpio_ctrl_0) & 0x01;

	return writeRegister(REG_GPIO_CTRL, gpio_ctrl);
}

/**
 * @brief set the GPIOs to be OFF, WARNZ, FAULTZ, PVDD_DROP, SDOUT or RAMP_CLK
 *
 * @param gpio_mode_0 GPIO 0 mode
 * @param gpio_mode_1 GPIO 1 mode
 * @param gpio_mode_2 GPIO 2 mode
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setGPIOSel(GPIO_Sel_t gpio_mode_0, GPIO_Sel_t gpio_mode_1, GPIO_Sel_t gpio_mode_2)
{
	if (!writeRegister(REG_GPIO0_SEL, static_cast<uint8_t>(gpio_mode_0) & 0x0F))
		return false;
	if (!writeRegister(REG_GPIO1_SEL, static_cast<uint8_t>(gpio_mode_1) & 0x0F))
		return false;
	if (!writeRegister(REG_GPIO2_SEL, static_cast<uint8_t>(gpio_mode_2) & 0x0F))
		return false;
	return true;
}

/* ------------------------------------------------------------ */
/* Getters                                                      */
/* ------------------------------------------------------------ */

/**
 * @brief get the loop bandwidth
 *
 * @param p_loopBW pointer to return the loop bandwidth in enum Loop_BW_t
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getLoopBW(Loop_BW_t* p_loopBW)
{
	uint8_t reg;

	if (readRegister(REG_ANA_CTRL, &reg)) {
		*p_loopBW = static_cast<Loop_BW_t>(reg & 0x60);
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the analog gain
 *
 * @param p_gain pointer to return the analog p_ in dB
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getAnalogGain(float* p_gain)
{
	uint8_t reg;

	if (readRegister(REG_AGAIN, &reg)) {
		*p_gain = static_cast<float>(reg & 0x1F) * -0.5f;
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the PVDD voltage
 *
 * @param p_pvdd pointer to return the PVDD voltage
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getPVDD(float* p_pvdd)
{
	uint8_t reg;

	if (readRegister(REG_ADC_RPT, &reg)) {
		// LSB is 0.12 V
		*p_pvdd = static_cast<float>(reg) * 0.12f;
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the GPIOs mode
 *
 * @param p_gpio_mode_0 pointer to GPIO 0 mode to return
 * @param p_gpio_mode_1 pointer to GPIO 1 mode to return
 * @param p_gpio_mode_2 pointer to GPIO 2 mode to return
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getGPIOMode(GPIO_Mode_t* p_gpio_mode_0, GPIO_Mode_t* p_gpio_mode_1, GPIO_Mode_t* p_gpio_mode_2)
{
	uint8_t reg;

	if (readRegister(REG_GPIO_CTRL, &reg)) {
		*p_gpio_mode_0 = static_cast<GPIO_Mode_t>(reg & 0x01);
		*p_gpio_mode_1 = static_cast<GPIO_Mode_t>((reg & 0x02) >> 1);
		*p_gpio_mode_2 = static_cast<GPIO_Mode_t>((reg & 0x04) >> 2);
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the GPIOs function
 *
 * @param p_gpio_sel_0 pointer to GPIO 0 function to return
 * @param p_gpio_sel_1 pointer to GPIO 1 function to return
 * @param p_gpio_sel_2 pointer to GPIO 2 function to return
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getGPIOSel(GPIO_Sel_t* p_gpio_sel_0, GPIO_Sel_t* p_gpio_sel_1, GPIO_Sel_t* p_gpio_sel_2)
{
	uint8_t reg;

	if (readRegister(REG_GPIO0_SEL, &reg)) {
		*p_gpio_sel_0 = static_cast<GPIO_Sel_t>(reg & 0x0F);
	}
	else {
		return false;
	}

	if (readRegister(REG_GPIO1_SEL, &reg)) {
		*p_gpio_sel_1 = static_cast<GPIO_Sel_t>(reg & 0x0F);
	}
	else {
		return false;
	}

	if (readRegister(REG_GPIO2_SEL, &reg)) {
		*p_gpio_sel_2 = static_cast<GPIO_Sel_t>(reg & 0x0F);
	}
	else {
		return false;
	}

	return true;
}

/* ------------------------------------------------------------ */
/* Private Helpers                                              */
/* ------------------------------------------------------------ */

/**
 * @brief write to register
 *
 * @param reg register address
 * @param value value to write
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::writeRegister(uint8_t reg, uint8_t value)
{
	return true;
}

/**
 * @brief read from register
 *
 * @param reg register address
 * @param p_value pointer to return value read
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::readRegister(uint8_t reg, uint8_t* p_value)
{
	return true;
}