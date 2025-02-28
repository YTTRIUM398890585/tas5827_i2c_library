#pragma once

#include <stdbool.h>
#include <stdint.h>

class TAS5827
{
public:
	/* Enums */
	enum class Loop_BW_t : uint8_t
	{
		LOOP_BW_80KHZ  = 0b00100000,
		LOOP_BW_100kHZ = 0b00000000,
		LOOP_BW_120kHZ = 0b01000000,
		LOOP_BW_175kHZ = 0b01100000,
	};

	enum class GPIO_Mode_t : uint8_t
	{
		INPUT  = 0b0,
		OUTPUT = 0b1,
	};

	enum class GPIO_Sel_t : uint8_t
	{
		OFF       = 0b0000,
		WARNZ     = 0b1000,
		FAULTZ    = 0b1011,
		PVDD_DROP = 0b1100,
		SDOUT     = 0b1101,
		RAMP_CLK  = 0b1110,
	};
	TAS5827();
	~TAS5827();

	bool begin(uint8_t address, uint8_t DUMMY_I2C_HANDLE);

	/* Setters */
	bool setLoopBW(Loop_BW_t loopBW);
	bool setAnalogGain(uint8_t gain);
	bool setGPIOMode(GPIO_Mode_t gpio_mode_0, GPIO_Mode_t gpio_mode_1, GPIO_Mode_t gpio_mode_2);
	bool setGPIOSel(GPIO_Sel_t gpio_sel_0, GPIO_Sel_t gpio_sel_1, GPIO_Sel_t gpio_sel_2);
	/* Getters */
	bool getLoopBW(Loop_BW_t* p_loopBW);
	bool getAnalogGain(float* p_gain);
	bool getPVDD(float* p_pvdd);
	bool getGPIOMode(GPIO_Mode_t* p_gpio_mode_0, GPIO_Mode_t* p_gpio_mode_1, GPIO_Mode_t* p_gpio_mode_2);
	bool getGPIOSel(GPIO_Sel_t* p_gpio_sel_0, GPIO_Sel_t* p_gpio_sel_1, GPIO_Sel_t* p_gpio_sel_2);
private:
	uint8_t address;
	uint8_t i2cHandler;

	/* Registers */
const uint8_t REG_RESET_CTRL        = 0x01;
const uint8_t REG_DEVICE_CTRL1      = 0x02;
const uint8_t REG_DEVICE_CTRL2      = 0x03;
const uint8_t REG_PVDD_UV_CONTROL   = 0x04;
const uint8_t REG_I2C_PAGE_AUTO_INC = 0x0F;
const uint8_t REG_SIG_CH_CTRL       = 0x28;
const uint8_t REG_CLOCK_DET_CTRL    = 0x29;
const uint8_t REG_SDOUT_SEL         = 0x30;
const uint8_t REG_I2S_CTRL          = 0x31;
const uint8_t REG_SAP_CTRL1         = 0x33;
const uint8_t REG_SAP_CTRL2         = 0x34;
const uint8_t REG_SAP_CTRL3         = 0x35;
const uint8_t REG_FS_MON            = 0x37;
const uint8_t REG_BCLK_MON          = 0x38;
const uint8_t REG_CLKDET_STATUS     = 0x39;
const uint8_t REG_DSP_PGM_MODE      = 0x40;
const uint8_t REG_DSP_CTRL          = 0x46;
const uint8_t REG_DIG_VOL_LEFT      = 0x4C;
const uint8_t REG_DIG_VOL_RIGHT     = 0x4D;
const uint8_t REG_DIG_VOL_CTRL2     = 0x4E;
const uint8_t REG_DIG_VOL_CTRL3     = 0x4F;
const uint8_t REG_AUTO_MUTE_CTRL    = 0x50;
const uint8_t REG_AUTO_MUTE_TIME    = 0x51;
const uint8_t REG_ANA_CTRL          = 0x53;
const uint8_t REG_AGAIN             = 0x54;
const uint8_t REG_ADC_RPT           = 0x5E;
const uint8_t REG_GPIO_CTRL         = 0x60;
const uint8_t REG_GPIO0_SEL         = 0x61;
const uint8_t REG_GPIO1_SEL         = 0x62;
const uint8_t REG_GPIO2_SEL         = 0x63;
const uint8_t REG_GPIO_INPUT_SEL    = 0x64;
const uint8_t REG_MISC_CTRL1        = 0x65;
const uint8_t REG_MISC_CTRL2        = 0x66;
const uint8_t REG_DIE_ID            = 0x67;
const uint8_t REG_POWER_STATE       = 0x68;
const uint8_t REG_AUTOMUTE_STATE    = 0x69;
const uint8_t REG_RAMP_PHASE_CTRL   = 0x6A;
const uint8_t REG_RAMP_SS_CTRL0     = 0x6B;
const uint8_t REG_RAMP_SS_CTRL1     = 0x6C;
const uint8_t REG_CHAN_FAULT        = 0x70;
const uint8_t REG_GLOBAL_FAULT1     = 0x71;
const uint8_t REG_GLOBAL_FAULT2     = 0x72;
const uint8_t REG_WARNING           = 0x73;
const uint8_t REG_PIN_CONTROL1      = 0x74;
const uint8_t REG_PIN_CONTROL2      = 0x75;
const uint8_t REG_MISC_CONTROL3     = 0x76;
const uint8_t REG_CBC_CONTROL       = 0x77;
const uint8_t REG_FAULT_CLEAR       = 0x78;

	bool writeRegister(uint8_t reg, uint8_t value);
	bool readRegister(uint8_t reg, uint8_t* p_value);
};