/// @brief VL53L0X Driver
/// @note Driver based heavily off of a driver developed here: https://github.com/artfulbytes/vl6180x_vl53l0x_msp430
/// @author George Kouretas

#include "vl53l0x_driver.h"
#include <stdio.h>

// Private macros
#define I2C_READ_8BIT(status, ptr_intf, reg, buf) \
do { \
	status = vl53l0x_i2c_read(ptr_intf, reg, &buf, sizeof(buf)); \
	if (status != HAL_OK) \
	{ \
		printf("i2c err: %d, status\n"); \
		return status; \
	} \
} while (0)

#define I2C_WRITE_8BIT(status, ptr_intf, reg, buf) \
do { \
	status = vl53l0x_i2c_write(ptr_intf, reg); \
	if (status != HAL_OK) \
	{ \
		printf("i2c reg write err: %d, status\n"); \
		return status; \
	} \
	status = vl53l0x_i2c_write(ptr_intf, buf); \
	if (status != HAL_OK) \
	{ \
		printf("i2c data write err: %d, status\n"); \
		return status; \
	} \
} while (0)

// Register definitions
#define REG_IDENTIFICATION_MODEL_ID (0xC0)
#define REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV (0x89)
#define REG_MSRC_CONFIG_CONTROL (0x60)
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT (0x44)
#define REG_SYSTEM_SEQUENCE_CONFIG (0x01)
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET (0x4F)
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD (0x4E)
#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT (0xB6)
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO (0x0A)
#define REG_GPIO_HV_MUX_ACTIVE_HIGH (0x84)
#define REG_SYSTEM_INTERRUPT_CLEAR (0x0B)
#define REG_RESULT_INTERRUPT_STATUS (0x13)
#define REG_SYSRANGE_START (0x00)
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 (0xB0)
#define REG_RESULT_RANGE_STATUS (0x14)
#define REG_SLAVE_DEVICE_ADDRESS (0x8A)

#define RANGE_SEQUENCE_STEP_TCC (0x10) /* Target CentreCheck */
#define RANGE_SEQUENCE_STEP_MSRC (0x04) /* Minimum Signal Rate Check */
#define RANGE_SEQUENCE_STEP_DSS (0x28) /* Dynamic SPAD selection */
#define RANGE_SEQUENCE_STEP_PRE_RANGE (0x40)
#define RANGE_SEQUENCE_STEP_FINAL_RANGE (0x80)

#define VL53L0X_EXPECTED_DEVICE_ID (0xEE)

// Private type-definitions
struct VL53l0X_Interface {
	I2C_HandleTypeDef *i2c_handle;
	uint16_t i2c_addr;

	struct {
		GPIO_TypeDef* port;
		uint16_t pin;
	} gpio_handle;

	uint8_t stop_variable;
};

typedef struct {
	uint8_t reg;
	uint8_t data;
} I2CPacket_t;

static VL53l0X_Interface_t vl53l0x_interface;

static HAL_StatusTypeDef vl53l0x_i2c_write(VL53l0X_Interface_t *interface, uint8_t data)
{
	return HAL_I2C_Master_Transmit(interface->i2c_handle, interface->i2c_addr, &data, 1, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef vl53l0x_i2c_read(VL53l0X_Interface_t *interface, uint8_t reg_addr, uint8_t *data, uint32_t len)
{
	HAL_StatusTypeDef status = vl53l0x_i2c_write(interface, reg_addr);
	if (status == HAL_OK)
	{
		return HAL_I2C_Master_Receive(interface->i2c_handle, interface->i2c_addr, data, len, HAL_MAX_DELAY);
	}
	else
	{
		printf("Failed to write register (status: %d)\n", status);
		return status;
	}
}


/* Based on data_init at https://github.com/artfulbytes/vl6180x_vl53l0x_msp430/blob/main/drivers/vl53l0x.c */
static HAL_StatusTypeDef vl53l0x_boot_config(VL53l0X_Interface_t *interface)
{
	static const I2CPacket_t i2c_standard_mode_setup[] = {
		{0x88, 0x00},
		{0x80, 0x01},
		{0xFF, 0x01},
		{0x00, 0x00}
	};

	HAL_StatusTypeDef i2c_status;

	/* Set 2v8 mode */
	uint8_t vhv_config_scl_sda = 0;
	I2C_READ_8BIT(i2c_status, interface, REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, vhv_config_scl_sda);
	vhv_config_scl_sda |= 0x01;
	I2C_WRITE_8BIT(i2c_status, interface, REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, vhv_config_scl_sda);

	for (size_t i = 0; i < sizeof(i2c_standard_mode_setup) / sizeof(i2c_standard_mode_setup[0]); ++i)
	{
		/* i2c standard mode */
		I2C_WRITE_8BIT(i2c_status, interface, i2c_standard_mode_setup[i].reg, i2c_standard_mode_setup[i].data);
	}

	/* Get stop variable */
	I2C_READ_8BIT(i2c_status, interface, 0x91, interface->stop_variable);

	/* Mysterious setup stuff */
	I2C_WRITE_8BIT(i2c_status, interface, 0x00, 0x01);
	I2C_WRITE_8BIT(i2c_status, interface, 0xFF, 0x00);
	I2C_WRITE_8BIT(i2c_status, interface, 0x80, 0x00);

    return HAL_OK;
}

static HAL_StatusTypeDef vl5310x_tuning_settings(VL53l0X_Interface_t *interface)
{
	static const I2CPacket_t i2c_tuning_settings[] = {
		{0x00, 0x00},
		{0xFF, 0x00},
		{0x09, 0x00},
		{0x10, 0x00},
		{0x11, 0x00},
		{0x24, 0x01},
		{0x25, 0xFF},
		{0x75, 0x00},
		{0xFF, 0x01},
		{0x4E, 0x2C},
		{0x48, 0x00},
		{0x30, 0x20},
		{0xFF, 0x00},
		{0x30, 0x09},
		{0x54, 0x00},
		{0x31, 0x04},
		{0x32, 0x03},
		{0x40, 0x83},
		{0x46, 0x25},
		{0x60, 0x00},
		{0x27, 0x00},
		{0x50, 0x06},
		{0x51, 0x00},
		{0x52, 0x96},
		{0x56, 0x08},
		{0x57, 0x30},
		{0x61, 0x00},
		{0x62, 0x00},
		{0x64, 0x00},
		{0x65, 0x00},
		{0x66, 0xA0},
		{0xFF, 0x01},
		{0x22, 0x32},
		{0x47, 0x14},
		{0x49, 0xFF},
		{0x4A, 0x00},
		{0xFF, 0x00},
		{0x7A, 0x0A},
		{0x7B, 0x00},
		{0x78, 0x21},
		{0xFF, 0x01},
		{0x23, 0x34},
		{0x42, 0x00},
		{0x44, 0xFF},
		{0x45, 0x26},
		{0x46, 0x05},
		{0x40, 0x40},
		{0x0E, 0x06},
		{0x20, 0x1A},
		{0x43, 0x40},
		{0xFF, 0x00},
		{0x34, 0x03},
		{0x35, 0x44},
		{0xFF, 0x01},
		{0x31, 0x04},
		{0x4B, 0x09},
		{0x4C, 0x05},
		{0x4D, 0x04},
		{0xFF, 0x00},
		{0x44, 0x00},
		{0x45, 0x20},
		{0x47, 0x08},
		{0x48, 0x28},
		{0x67, 0x00},
		{0x70, 0x04},
		{0x71, 0x01},
		{0x72, 0xFE},
		{0x76, 0x00},
		{0x77, 0x00},
		{0xFF, 0x01},
		{0x0D, 0x01},
		{0xFF, 0x00},
		{0x80, 0x01},
		{0x01, 0xF8},
		{0xFF, 0x01},
		{0x8E, 0x01},
		{0x00, 0x01},
		{0xFF, 0x00},
		{0x80, 0x00}
	};

	HAL_StatusTypeDef i2c_status;

	for (size_t i = 0; i < sizeof(i2c_tuning_settings) / sizeof(i2c_tuning_settings[0]); ++i)
	{
		/* i2c standard mode */
		I2C_WRITE_8BIT(i2c_status, interface, i2c_tuning_settings[i].reg, i2c_tuning_settings[i].data);
	}

	return HAL_OK;
}

static bool vl510x_setup(VL53l0X_Interface_t *interface)
{
	HAL_StatusTypeDef i2c_status;
	i2c_status = vl53l0x_boot_config(interface);

	if (i2c_status != HAL_OK)
	{
		printf("boot config err: %d\n", i2c_status);
		return false;
	}

	i2c_status = vl5310x_tuning_settings(interface);
	if (i2c_status != HAL_OK)
	{
		printf("tuning config err: %d\n", i2c_status);
		return false;
	}

	return true;
}

VL53l0X_Interface_t *vl53l0x_init(I2C_HandleTypeDef *i2c_handle, uint16_t i2c_addr, GPIO_TypeDef *gpio_port, uint16_t gpio_pin)
{
	// Setup interface
	vl53l0x_interface.i2c_handle = i2c_handle;
	vl53l0x_interface.i2c_addr = i2c_addr;
	vl53l0x_interface.gpio_handle.port = gpio_port;
	vl53l0x_interface.gpio_handle.pin = gpio_pin;

	// Set shutdown pin high to turn on IC
	HAL_GPIO_WritePin(vl53l0x_interface.gpio_handle.port, vl53l0x_interface.gpio_handle.pin, GPIO_PIN_SET);

	// 1ms delay
	HAL_Delay(100);

	// Verify ID is valid
	uint8_t id = 0;
	if (vl53l0x_i2c_read(&vl53l0x_interface, 0xC0, &id, 1) != HAL_OK)
	{
		printf("Failed to get ID\n");
		return NULL;
	}

	if (id != VL53L0X_EXPECTED_DEVICE_ID)
	{
		printf("ID (0x%x) did not match expected ID (0x%x)\n", id, VL53L0X_EXPECTED_DEVICE_ID);
		return NULL;
	}

	if (!vl510x_setup(&vl53l0x_interface))
	{
		printf("Failed to setup IC\n");
		return NULL;
	}

	printf("Success!\n");

	return &vl53l0x_interface;
}
