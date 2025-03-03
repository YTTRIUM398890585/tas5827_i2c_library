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
 * @param gpioMode0 GPIO 0 input/output
 * @param gpioMode1 GPIO 1 input/output
 * @param gpioMode2 GPIO 2 input/output
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setGPIOMode(GPIO_Mode_t gpioMode0, GPIO_Mode_t gpioMode1, GPIO_Mode_t gpioMode2)
{
	uint8_t gpio_ctrl = 0;

	gpio_ctrl |= static_cast<uint8_t>(gpioMode2) & 0x01 << 2;
	gpio_ctrl |= static_cast<uint8_t>(gpioMode1) & 0x01 << 1;
	gpio_ctrl |= static_cast<uint8_t>(gpioMode0) & 0x01;

	return writeRegister(REG_GPIO_CTRL, gpio_ctrl);
}

/**
 * @brief set the GPIOs to be OFF, WARNZ, FAULTZ, PVDD_DROP, SDOUT or RAMP_CLK
 *
 * @param gpioSel0 GPIO 0 mode
 * @param gpioSel1 GPIO 1 mode
 * @param gpioSel2 GPIO 2 mode
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setGPIOSel(GPIO_Sel_t gpioSel0, GPIO_Sel_t gpioSel1, GPIO_Sel_t gpioSel2)
{
	if (!writeRegister(REG_GPIO0_SEL, static_cast<uint8_t>(gpioSel0) & 0x0F))
		return false;
	if (!writeRegister(REG_GPIO1_SEL, static_cast<uint8_t>(gpioSel1) & 0x0F))
		return false;
	if (!writeRegister(REG_GPIO2_SEL, static_cast<uint8_t>(gpioSel2) & 0x0F))
		return false;
	return true;
}

/**
 * @brief set the GPIOs inversion setting
 *
 * @param gpioInv0 GPIO 0 inversion, true for inverted
 * @param gpioInv1 GPIO 1 inversion, true for inverted
 * @param gpioInv2 GPIO 2 inversion, true for inverted
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setMiscCtrl2(bool gpioInv0, bool gpioInv1, bool gpioInv2)
{
	uint8_t miscCtrl2 = 0;

	miscCtrl2 |= gpioInv2 ? 1 << 2 : 0;
	miscCtrl2 |= gpioInv1 ? 1 << 1 : 0;
	miscCtrl2 |= gpioInv0 ? 1 << 0 : 0;

	return writeRegister(REG_MISC_CTRL2, miscCtrl2);
}

/**
 * @brief disable the spread spectrum
 *
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setDisableSpreadSpectrum(void)
{
	return writeRegister(REG_RAMP_SS_CTRL0, 0);
}

/**
 * @brief enable the random spread spectrum
 *
 * @param randRange range of the random spread spectrum
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setRandomSpreadSpectrum(SS_Rand_Range_t randRange)
{
	uint8_t spreadSpectrumCtrl0 = 0;
	spreadSpectrumCtrl0 |= 1 << 1;

	uint8_t spreadSpectrumCtrl1 = 0;
	spreadSpectrumCtrl1 |= (static_cast<uint8_t>(randRange) & 0x07) << 4;

	if (!writeRegister(REG_RAMP_SS_CTRL0, spreadSpectrumCtrl0))
		return false;
	if (!writeRegister(REG_RAMP_SS_CTRL1, spreadSpectrumCtrl1))
		return false;
	return true;
}

/**
 * @brief enable the triangular spread spectrum
 *
 * @param triRange range of the triangular spread spectrum
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setTriangularSpreadSpectrum(SS_Tri_Range_t triRange)
{
	uint8_t spreadSpectrumCtrl0 = 0;
	spreadSpectrumCtrl0 |= 1 << 0;

	uint8_t spreadSpectrumCtrl1 = 0;
	spreadSpectrumCtrl1 |= (static_cast<uint8_t>(triRange) & 0x0F);

	if (!writeRegister(REG_RAMP_SS_CTRL0, spreadSpectrumCtrl0))
		return false;
	if (!writeRegister(REG_RAMP_SS_CTRL1, spreadSpectrumCtrl1))
		return false;
	return true;
}

/**
 * @brief set the pin control 1
 *
 * @param pinCtrl1 value to write
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setPinCtrl1(uint8_t pinCtrl1)
{
	return writeRegister(REG_PIN_CONTROL1, pinCtrl1);
}

/**
 * @brief set the pin control 2
 *
 * @param pinCtrl2 value to write
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setPinCtrl2(uint8_t pinCtrl2)
{
	return writeRegister(REG_PIN_CONTROL2, pinCtrl2);
}

/**
 * @brief set the miscellaneous control 3
 *
 * @param miscCtrl3 value to write
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setMiscCtrl3(uint8_t miscCtrl3)
{
	miscCtrl3 = miscCtrl3 & 0x90;
	return writeRegister(REG_MISC_CONTROL3, miscCtrl3);
}

/**
 * @brief set the CBC control
 *
 * @param levelSel Cycle-By-Cycle current limiting level, percentage of the Over-Current Threshold:
 * @param cbcEn CBC enable
 * @param cbcWarnEn CBC warning enable
 * @param cbcFaultEn CBC fault enable
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setCBCCtrl(CBC_Sel_t levelSel, bool cbcEn, bool cbcWarnEn, bool cbcFaultEn)
{
	uint8_t cbcCtrl = 0;

	cbcCtrl |= (static_cast<uint8_t>(levelSel) & 0x03) << 3;
	cbcCtrl |= cbcEn ? 1 << 2 : 0;
	cbcCtrl |= cbcWarnEn ? 1 << 1 : 0;
	cbcCtrl |= cbcFaultEn ? 1 << 0 : 0;

	return writeRegister(REG_CBC_CONTROL, cbcCtrl);
}

/**
 * @brief clear the analog fault
 *
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setFaultClear(void)
{
	return writeRegister(REG_FAULT_CLEAR, 0x80);
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
 * @param p_gpioMode0 pointer to GPIO 0 mode to return
 * @param p_gpioMode1 pointer to GPIO 1 mode to return
 * @param p_gpioMode2 pointer to GPIO 2 mode to return
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getGPIOMode(GPIO_Mode_t* p_gpioMode0, GPIO_Mode_t* p_gpioMode1, GPIO_Mode_t* p_gpioMode2)
{
	uint8_t reg;

	if (readRegister(REG_GPIO_CTRL, &reg)) {
		*p_gpioMode0 = static_cast<GPIO_Mode_t>(reg & 0x01);
		*p_gpioMode1 = static_cast<GPIO_Mode_t>((reg & 0x02) >> 1);
		*p_gpioMode2 = static_cast<GPIO_Mode_t>((reg & 0x04) >> 2);
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the GPIOs function
 *
 * @param p_gpioSel0 pointer to GPIO 0 function to return
 * @param p_gpioSel1 pointer to GPIO 1 function to return
 * @param p_gpioSel2 pointer to GPIO 2 function to return
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getGPIOSel(GPIO_Sel_t* p_gpioSel0, GPIO_Sel_t* p_gpioSel1, GPIO_Sel_t* p_gpioSel2)
{
	uint8_t reg;

	if (readRegister(REG_GPIO0_SEL, &reg)) {
		*p_gpioSel0 = static_cast<GPIO_Sel_t>(reg & 0x0F);
	}
	else {
		return false;
	}

	if (readRegister(REG_GPIO1_SEL, &reg)) {
		*p_gpioSel1 = static_cast<GPIO_Sel_t>(reg & 0x0F);
	}
	else {
		return false;
	}

	if (readRegister(REG_GPIO2_SEL, &reg)) {
		*p_gpioSel2 = static_cast<GPIO_Sel_t>(reg & 0x0F);
	}
	else {
		return false;
	}

	return true;
}

/**
 * @brief get the GPIOs inversion setting
 *
 * @param p_gpioInv0 pointer to GPIO 0 inversion to return, true for inverted
 * @param p_gpioInv1 pointer to GPIO 1 inversion to return, true for inverted
 * @param p_gpioInv2 pointer to GPIO 2 inversion to return, true for inverted
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getMiscCtrl2(bool* p_gpioInv0, bool* p_gpioInv1, bool* p_gpioInv2)
{
	uint8_t reg;

	if (readRegister(REG_MISC_CTRL2, &reg)) {
		*p_gpioInv0 = static_cast<bool>(reg & 0x01);
		*p_gpioInv1 = static_cast<bool>(reg & 0x02);
		*p_gpioInv2 = static_cast<bool>(reg & 0x04);
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the power state
 *
 * @param p_powState pointer to return the power state
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getPowState(Power_State_t* p_powState)
{
	uint8_t reg;

	if (readRegister(REG_POWER_STATE, &reg)) {
		*p_powState = static_cast<Power_State_t>(reg & 0x03);
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the auto mute state
 *
 * @param p_ch1Mute pointer to return the channel 1 mute state
 * @param p_ch2Mute pointer to return the channel 2 mute state
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getAutoMuteState(bool* p_ch1Mute, bool* p_ch2Mute)
{
	uint8_t reg;

	if (readRegister(REG_AUTOMUTE_STATE, &reg)) {
		*p_ch1Mute = static_cast<bool>(reg & 0x01);
		*p_ch2Mute = static_cast<bool>(reg & 0x02);
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the spread spectrum control
 *
 * @param p_triangularEn pointer to return the triangular spread spectrum enable
 * @param p_randomEn pointer to return the random spread spectrum enable
 * @param p_randRange pointer to return the random spread spectrum range
 * @param p_triRange pointer to return the triangular spread spectrum range
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getSpreadSpectrumCtrl(
	bool* p_triangularEn, bool* p_randomEn, SS_Rand_Range_t* p_randRange, SS_Tri_Range_t* p_triRange
)
{
	uint8_t reg;

	if (readRegister(REG_RAMP_SS_CTRL0, &reg)) {
		*p_triangularEn = static_cast<bool>(reg & 0x01);
		*p_randomEn     = static_cast<bool>(reg & 0x02);
	}
	else {
		return false;
	}

	if (readRegister(REG_RAMP_SS_CTRL1, &reg)) {
		*p_randRange = static_cast<SS_Rand_Range_t>((reg & 0x70) >> 4);
		*p_triRange  = static_cast<SS_Tri_Range_t>(reg & 0x0F);
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the channel fault
 *
 * @param p_chanFault pointer to return the channel fault
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getChanFault(uint8_t* p_chanFault)
{
	uint8_t reg;

	if (readRegister(REG_CHAN_FAULT, &reg)) {
		*p_chanFault = reg & 0x0F;
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the global fault 1
 *
 * @param p_globalFault1 pointer to return the global fault 1
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getGlobalFault1(uint8_t* p_globalFault1)
{
	uint8_t reg;

	if (readRegister(REG_GLOBAL_FAULT1, &reg)) {
		*p_globalFault1 = reg & 0x67;
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the global fault 2
 *
 * @param p_globalFault2 pointer to return the global fault 2
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getGlobalFault2(uint8_t* p_globalFault2)
{
	uint8_t reg;

	if (readRegister(REG_GLOBAL_FAULT2, &reg)) {
		*p_globalFault2 = reg & 0x07;
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the warning
 *
 * @param p_warning pointer to return the warning
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getWarning(uint8_t* p_warning)
{
	uint8_t reg;

	if (readRegister(REG_WARNING, &reg)) {
		*p_warning = reg & 0x07;
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the pin control 1
 *
 * @param p_pinCtrl1 pointer to return the pin control 1
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getPinCtrl1(uint8_t* p_pinCtrl1)
{
	uint8_t reg;

	if (readRegister(REG_PIN_CONTROL1, &reg)) {
		*p_pinCtrl1 = reg;
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the pin control 2
 *
 * @param p_pinCtrl2 pointer to return the pin control 2
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getPinCtrl2(uint8_t* p_pinCtrl2)
{
	uint8_t reg;

	if (readRegister(REG_PIN_CONTROL2, &reg)) {
		*p_pinCtrl2 = reg;
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the miscellaneous control 3
 *
 * @param p_miscCtrl3 pointer to return the miscellaneous control 3
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getMiscCtrl3(uint8_t* p_miscCtrl3)
{
	uint8_t reg;

	if (readRegister(REG_MISC_CONTROL3, &reg)) {
		*p_miscCtrl3 = reg & 0x90;
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the CBC control
 *
 * @param p_levelSel pointer to return the Cycle-By-Cycle current limiting level
 * @param p_cbcEn pointer to return the CBC enable
 * @param p_cbcWarnEn pointer to return the CBC warning enable
 * @param p_cbcFaultEn pointer to return the CBC fault enable
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getCbcCtrl(CBC_Sel_t* p_levelSel, bool* p_cbcEn, bool* p_cbcWarnEn, bool* p_cbcFaultEn)
{
	uint8_t cbcCtrl = 0;

	if (readRegister(REG_CBC_CONTROL, &cbcCtrl)) {
		*p_levelSel   = static_cast<CBC_Sel_t>((cbcCtrl & 0x18) >> 3);
		*p_cbcEn      = static_cast<bool>(cbcCtrl & 0x04);
		*p_cbcWarnEn  = static_cast<bool>(cbcCtrl & 0x02);
		*p_cbcFaultEn = static_cast<bool>(cbcCtrl & 0x01);
		return true;
	}
	else {
		return false;
	}
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