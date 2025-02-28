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

	enum class Power_State_t : uint8_t
	{
		DEEP_SLEEP = 0b00,
		SLEEP      = 0b01,
		HI_Z       = 0b10,
		PLAY       = 0b11,
	};

	enum class CBC_Sel_t : uint8_t
	{
		CBC_80_PERCENT = 0b00,
		CBC_60_PERCENT = 0b10,
		CBC_40_PERCENT = 0b01,
	};

	/* Fault Masks */
	// CHAN_FAULT
	const uint8_t MASK_CHAN_FAULT_CH1DC = 1 << 3;
	const uint8_t MASK_CHAN_FAULT_CH2DC = 1 << 2;
	const uint8_t MASK_CHAN_FAULT_CH1OC = 1 << 1;
	const uint8_t MASK_CHAN_FAULT_CH2OC = 1 << 0;

	// GLOBAL_FAULT1
	const uint8_t MASK_GLOBAL_FAULT1_BQ     = 1 << 6;
	const uint8_t MASK_GLOBAL_FAULT1_EEPROM = 1 << 5;
	const uint8_t MASK_GLOBAL_FAULT1_CLK    = 1 << 2;
	const uint8_t MASK_GLOBAL_FAULT1_PVDDOV = 1 << 1;
	const uint8_t MASK_GLOBAL_FAULT1_PVDDUV = 1 << 0;

	// GLOBAL_FAULT2
	const uint8_t MASK_GLOBAL_FAULT2_CH2CBC = 1 << 2;
	const uint8_t MASK_GLOBAL_FAULT2_CH1CBC = 1 << 1;
	const uint8_t MASK_GLOBAL_FAULT2_OTSD   = 1 << 0;

	// WARNING
	const uint8_t MASK_WARNING_CH1CBCW = 1 << 5;
	const uint8_t MASK_WARNING_CH2CBCW = 1 << 4;
	const uint8_t MASK_WARNING_OTW4    = 1 << 3;
	const uint8_t MASK_WARNING_OTW3    = 1 << 2;
	const uint8_t MASK_WARNING_OTW2    = 1 << 1;
	const uint8_t MASK_WARNING_OTW1    = 1 << 0;

	// PIN_CONTROL1
	const uint8_t MASK_PIN_CTRL1_OTSD     = 1 << 7;
	const uint8_t MASK_PIN_CTRL1_DVDDUV   = 1 << 6;
	const uint8_t MASK_PIN_CTRL1_DVDDOV   = 1 << 5;
	const uint8_t MASK_PIN_CTRL1_CLKERROR = 1 << 4;
	const uint8_t MASK_PIN_CTRL1_PVDDUV   = 1 << 3;
	const uint8_t MASK_PIN_CTRL1_PVDDOV   = 1 << 2;
	const uint8_t MASK_PIN_CTRL1_DC       = 1 << 1;
	const uint8_t MASK_PIN_CTRL1_OC       = 1 << 0;

	// PIN_CONTROL2
	const uint8_t EN_PIN_CTRL2_CBCFAULTLATCH = 1 << 7;
	const uint8_t EN_PIN_CTRL2_CBCWARNLATCH  = 1 << 6;
	const uint8_t EN_PIN_CTRL2_CLKFAULTLATCH = 1 << 5;
	const uint8_t EN_PIN_CTRL2_OTSDLATCH     = 1 << 4;
	const uint8_t EN_PIN_CTRL2_OTWLATCH      = 1 << 3;
	const uint8_t MASK_PIN_CTRL2_OTW         = 1 << 2;
	const uint8_t MASK_PIN_CTRL2_CBCWARN     = 1 << 1;
	const uint8_t MASK_PIN_CTRL2_CBCFAULT    = 1 << 0;

	// MISC_CONTROL3
	const uint8_t EN_CLKDET_LATCH = 1 << 7;
	const uint8_t EN_OTSD_AUTOREC = 1 << 4;

	TAS5827();
	~TAS5827();

	bool begin(uint8_t address, uint8_t DUMMY_I2C_HANDLE);

	/* Setters */
	bool setLoopBW(Loop_BW_t loopBW);
	bool setAnalogGain(uint8_t gain);
	bool setGPIOMode(GPIO_Mode_t gpio_mode_0, GPIO_Mode_t gpio_mode_1, GPIO_Mode_t gpio_mode_2);
	bool setGPIOSel(GPIO_Sel_t gpio_sel_0, GPIO_Sel_t gpio_sel_1, GPIO_Sel_t gpio_sel_2);
	bool setMiscCtrl2(bool gpio_inv_0, bool gpio_inv_1, bool gpio_inv_2);
	bool setPinCtrl1(uint8_t pinCtrl1);
	bool setPinCtrl2(uint8_t pinCtrl2);
	bool setMiscCtrl3(uint8_t miscCtrl3);
	bool setCBCCtrl(CBC_Sel_t level_sel, bool cbc_en, bool cbc_warn_en, bool cbc_fault_en);
	bool setFaultClear(void);

	/* Getters */
	bool getLoopBW(Loop_BW_t* p_loopBW);
	bool getAnalogGain(float* p_gain);
	bool getPVDD(float* p_pvdd);
	bool getGPIOMode(GPIO_Mode_t* p_gpio_mode_0, GPIO_Mode_t* p_gpio_mode_1, GPIO_Mode_t* p_gpio_mode_2);
	bool getGPIOSel(GPIO_Sel_t* p_gpio_sel_0, GPIO_Sel_t* p_gpio_sel_1, GPIO_Sel_t* p_gpio_sel_2);
	bool getMiscCtrl2(bool* p_gpio_inv_0, bool* p_gpio_inv_1, bool* p_gpio_inv_2);
	bool getPowState(Power_State_t* p_powState);
	bool getChanFault(uint8_t* p_chanFault);
	bool getGlobalFault1(uint8_t* p_globalFault1);
	bool getGlobalFault2(uint8_t* p_globalFault2);
	bool getWarning(uint8_t* p_warning);
	bool getPinCtrl1(uint8_t* p_pinCtrl1);
	bool getPinCtrl2(uint8_t* p_pinCtrl2);
	bool getMiscCtrl3(uint8_t* p_miscCtrl3);
	bool getCbcCtrl(CBC_Sel_t* p_level_sel, bool* p_cbc_en, bool* p_cbc_warn_en, bool* p_cbc_fault_en);

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