/// @brief VL53L0X Driver
/// @note Driver based heavily off of a driver developed here: https://github.com/artfulbytes/vl6180x_vl53l0x_msp430
/// @author George Kouretas

#include "vl53l0x_driver.h"
#include <stdio.h>

// Private macros
#define ARR_LEN(x) (sizeof((x)) / sizeof((x)[0]))

#define I2C_READ(status, ptr_intf, reg, buf) \
do { \
	status = vl53l0x_i2c_read(ptr_intf, reg, (uint8_t *)&(buf), sizeof((buf))); \
	if (status != HAL_OK) \
	{ \
		printf("i2c err [code: %d] (%s) (%s):%d\n", status, __FILE__, __func__, __line__); \
		return status; \
	} \
} while (0)

#define I2C_WRITE(status, ptr_intf, data) \
do { \
	status = HAL_I2C_Master_Transmit(ptr_intf->i2c_handle, ptr_intf->i2c_addr, (uint8_t *)&(data), sizeof((data)), HAL_MAX_DELAY); \
	if (status != HAL_OK) \
	{ \
		printf("i2c reg write err [code: %d] (%s) (%s):%d\n", status, __FILE__, __func__, __line__); \
		return status; \
	} \
} while (0)

#define I2C_WRITE_8BIT(status, ptr_intf, reg, data) \
do { \
	status = vl53l0x_i2c_write_packet(ptr_intf, reg, data); \
	if (status != HAL_OK) \
	{ \
		printf("i2c reg write err [code: %d] (%s) (%s):%d\n", status, __FILE__, __func__, __line__); \
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
static volatile bool isr_triggered = false;

static HAL_StatusTypeDef vl53l0x_i2c_write_packet(VL53l0X_Interface_t *interface, uint8_t reg, uint8_t data)
{
	uint8_t payload[2] = {reg, data};
	return HAL_I2C_Master_Transmit(interface->i2c_handle, interface->i2c_addr, payload, 2, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef vl53l0x_i2c_write_reg(VL53l0X_Interface_t *interface, uint8_t data)
{
	return HAL_I2C_Master_Transmit(interface->i2c_handle, interface->i2c_addr, &data, 1, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef vl53l0x_i2c_read(VL53l0X_Interface_t *interface, uint8_t reg_addr, uint8_t *data, uint32_t len)
{
	HAL_StatusTypeDef status = vl53l0x_i2c_write_reg(interface, reg_addr);
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

static HAL_StatusTypeDef vl5310x_wait_for_isr(VL53l0X_Interface_t *interface)
{
	while (!isr_triggered);
	isr_triggered = false;

	HAL_StatusTypeDef i2c_status;
	uint8_t interrupt_status;
    do {
    	I2C_READ(i2c_status, interface, REG_RESULT_INTERRUPT_STATUS, interrupt_status);
    	break;
    } while ((interrupt_status & 0x07) == 0);

	return HAL_OK;
}

static HAL_StatusTypeDef vl5310x_clear_isr(VL53l0X_Interface_t *interface)
{
	HAL_StatusTypeDef i2c_status;
	I2C_WRITE_8BIT(i2c_status, interface, REG_SYSTEM_INTERRUPT_CLEAR, 0x1);

	return HAL_OK;
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
	I2C_READ(i2c_status, interface, REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, vhv_config_scl_sda);
	vhv_config_scl_sda |= 0x01;
	I2C_WRITE_8BIT(i2c_status, interface, REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, vhv_config_scl_sda);

	for (size_t i = 0; i < ARR_LEN(i2c_standard_mode_setup); ++i)
	{
		/* i2c standard mode */
		I2C_WRITE_8BIT(i2c_status, interface, i2c_standard_mode_setup[i].reg, i2c_standard_mode_setup[i].data);
	}

	/* Get stop variable */
	I2C_READ(i2c_status, interface, 0x91, interface->stop_variable);

	/* Mysterious setup stuff */
	I2C_WRITE_8BIT(i2c_status, interface, 0x00, 0x01);
	I2C_WRITE_8BIT(i2c_status, interface, 0xFF, 0x00);
	I2C_WRITE_8BIT(i2c_status, interface, 0x80, 0x00);

//	uint8_t config_control_resp;
//	I2C_READ(i2c_status, interface, REG_MSRC_CONFIG_CONTROL, config_control_resp);
//	I2C_WRITE_8BIT(i2c_status, interface, REG_MSRC_CONFIG_CONTROL, config_control_resp | 0x12);


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

	for (size_t i = 0; i < ARR_LEN(i2c_tuning_settings); ++i)
	{
		/* i2c standard mode */
		I2C_WRITE_8BIT(i2c_status, interface, i2c_tuning_settings[i].reg, i2c_tuning_settings[i].data);
	}

	return HAL_OK;
}

static HAL_StatusTypeDef vl5310x_config_interrupt(VL53l0X_Interface_t *interface)
{
	HAL_StatusTypeDef i2c_status;

	I2C_WRITE_8BIT(i2c_status, interface, REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);

	uint8_t gpio_hv_mux_active_high = 0;
	I2C_READ(i2c_status, interface, REG_GPIO_HV_MUX_ACTIVE_HIGH, gpio_hv_mux_active_high);
	gpio_hv_mux_active_high &= ~0x10;

	I2C_WRITE_8BIT(i2c_status, interface, REG_GPIO_HV_MUX_ACTIVE_HIGH, gpio_hv_mux_active_high);
	I2C_WRITE_8BIT(i2c_status, interface, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

	return HAL_OK;
}

typedef enum
{
    CALIBRATION_TYPE_VHV,
    CALIBRATION_TYPE_PHASE
} calibration_type_t;

static HAL_StatusTypeDef perform_single_ref_calibration(VL53l0X_Interface_t *interface, calibration_type_t calib_type)
{
	printf("Entering %d calibration\n", calib_type);
    uint8_t sysrange_start = 0;
    uint8_t sequence_config = 0;
    switch (calib_type)
    {
    case CALIBRATION_TYPE_VHV:
        sequence_config = 0x01;
        sysrange_start = 0x01 | 0x40;
        break;
    case CALIBRATION_TYPE_PHASE:
        sequence_config = 0x02;
        sysrange_start = 0x01 | 0x00;
        break;
    }

    HAL_StatusTypeDef i2c_status;

	I2C_WRITE_8BIT(i2c_status, interface, REG_SYSTEM_SEQUENCE_CONFIG, sequence_config);
	I2C_WRITE_8BIT(i2c_status, interface, REG_SYSRANGE_START, sysrange_start);

    /* Wait for interrupt */
	vl5310x_wait_for_isr(interface);

    if (vl5310x_clear_isr(interface) != HAL_OK)
    {
    	printf("Failed to clear ISR\n");
    }

    I2C_WRITE_8BIT(i2c_status, interface, REG_SYSRANGE_START, 0x00);

    return HAL_OK;
}

static HAL_StatusTypeDef read_strobe(VL53l0X_Interface_t *interface)
{
	HAL_StatusTypeDef i2c_status;
    uint8_t strobe = 0;

    I2C_WRITE_8BIT(i2c_status, interface, 0x83, 0x00);

    do {
        I2C_READ(i2c_status, interface, 0x83, strobe);
    } while (strobe == 0);

    I2C_WRITE_8BIT(i2c_status, interface, 0x83, 0x01);

    return HAL_OK;
}

static HAL_StatusTypeDef get_spad_info_from_nvm(VL53l0X_Interface_t *interface, uint8_t *spad_count, uint8_t *spad_type, uint8_t good_spad_map[6])
{
    uint8_t spad_info_8b = 0;
    uint32_t spad_info_32b = 0;

    static const I2CPacket_t i2c_spad_settings_p1[] = {
		{0x80, 0x01},
		{0xFF, 0x01},
		{0x00, 0x00},
		{0xFF, 0x06},
    };

    static const I2CPacket_t i2c_spad_settings_p2[] = {
		{0xFF, 0x07},
		{0x81, 0x01},
		{0x80, 0x01},
	};

    static const I2CPacket_t i2c_spad_settings_p3[] = {
		{0x81, 0x00},
		{0xFF, 0x06},
	};

    static const I2CPacket_t i2c_spad_settings_p4[] = {
		{0xFF, 0x01},
		{0x00, 0x01},
		{0xFF, 0x00},
		{0x80, 0x00},
	};

    HAL_StatusTypeDef i2c_status;

    /* Setup to read from NVM */
    for (size_t i = 0; i < ARR_LEN(i2c_spad_settings_p1); ++i)
    {
    	I2C_WRITE_8BIT(i2c_status, interface, i2c_spad_settings_p1[i].reg, i2c_spad_settings_p1[i].data);
    }

    I2C_READ(i2c_status, interface, 0x83, spad_info_8b);
    I2C_WRITE_8BIT(i2c_status, interface, 0x83, spad_info_8b | 0x04);

    for (size_t i = 0; i < ARR_LEN(i2c_spad_settings_p2); ++i)
    {
    	I2C_WRITE_8BIT(i2c_status, interface, i2c_spad_settings_p2[i].reg, i2c_spad_settings_p2[i].data);
    }

    /* Get the SPAD count and type */
    I2C_WRITE_8BIT(i2c_status, interface, 0x94, 0x6b);

    if (read_strobe(interface) != HAL_OK) {
        return HAL_ERROR;
    }

    I2C_READ(i2c_status, interface, 0x90, spad_info_32b);

    *spad_count = (spad_info_32b >> 8) & 0x7f;
    *spad_type = (spad_info_32b >> 15) & 0x01;

    /* Restore after reading from NVM */
    for (size_t i = 0; i < ARR_LEN(i2c_spad_settings_p3); ++i)
	{
    	I2C_WRITE_8BIT(i2c_status, interface, i2c_spad_settings_p3[i].reg, i2c_spad_settings_p3[i].data);
	}

    I2C_READ(i2c_status, interface, 0x83, spad_info_8b);
    I2C_WRITE_8BIT(i2c_status, interface, 0x83, spad_info_8b & 0xfb);

    for (size_t i = 0; i < ARR_LEN(i2c_spad_settings_p4); ++i)
	{
    	I2C_WRITE_8BIT(i2c_status, interface, i2c_spad_settings_p4[i].reg, i2c_spad_settings_p4[i].data);
	}

    return vl53l0x_i2c_read(interface, REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, good_spad_map, 6);
}

static HAL_StatusTypeDef set_spads_from_nvm(VL53l0X_Interface_t *interface)
{
/* There are two types of SPAD: aperture and non-aperture. My understanding
 * is that aperture ones let it less light (they have a smaller opening), similar
 * to how you can change the aperture on a digital camera. Only 1/4 th of the
 * SPADs are of type non-aperture. */
#define SPAD_TYPE_APERTURE (0x01)
/* The total SPAD array is 16x16, but we can only activate a quadrant spanning 44 SPADs at
 * a time. In the ST api code they have (for some reason) selected 0xB4 (180) as a starting
 * point (lies in the middle and spans non-aperture (3rd) quadrant and aperture (4th) quadrant). */
#define SPAD_START_SELECT (0xB4)
/* The total SPAD map is 16x16, but we should only activate an area of 44 SPADs at a time. */
#define SPAD_MAX_COUNT (44)
/* The 44 SPADs are represented as 6 bytes where each bit represents a single SPAD.
 * 6x8 = 48, so the last four bits are unused. */
#define SPAD_MAP_ROW_COUNT (6)
#define SPAD_ROW_SIZE (8)
/* Since we start at 0xB4 (180), there are four quadrants (three aperture, one aperture),
 * and each quadrant contains 256 / 4 = 64 SPADs, and the third quadrant is non-aperture, the
 * offset to the aperture quadrant is (256 - 64 - 180) = 12 */
#define SPAD_APERTURE_START_INDEX (12)

    uint8_t spad_map[SPAD_MAP_ROW_COUNT] = { 0 };
    uint8_t good_spad_map[SPAD_MAP_ROW_COUNT] = { 0 };
    uint8_t spads_enabled_count = 0;
    uint8_t spads_to_enable_count = 0;
    uint8_t spad_type = 0;
    volatile uint32_t total_val = 0;

    HAL_StatusTypeDef i2c_status;

    static const I2CPacket_t i2c_spad_settings_p1[] = {
		{0xFF, 0x01},
		{REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00},
		{REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C},
		{0xFF, 0x00},
		{REG_GLOBAL_CONFIG_REF_EN_START_SELECT, SPAD_START_SELECT},
	};

    if (get_spad_info_from_nvm(interface, &spads_to_enable_count, &spad_type, good_spad_map) != HAL_OK) {
        return HAL_ERROR;
    }

    for (int i = 0; i < 6; i++) {
        total_val += good_spad_map[i];
    }

    for (size_t i = 0; i < ARR_LEN(i2c_spad_settings_p1); ++i)
	{
		I2C_WRITE_8BIT(i2c_status, interface, i2c_spad_settings_p1[i].reg, i2c_spad_settings_p1[i].data);
	}

    uint8_t offset = (spad_type == SPAD_TYPE_APERTURE) ? SPAD_APERTURE_START_INDEX : 0;

    /* Create a new SPAD array by selecting a subset of the SPADs suggested by the good SPAD map.
     * The subset should only have the number of type enabled as suggested by the reading from
     * the NVM (spads_to_enable_count and spad_type). */
    for (int row = 0; row < SPAD_MAP_ROW_COUNT; row++) {
        for (int column = 0; column < SPAD_ROW_SIZE; column++) {
            int index = (row * SPAD_ROW_SIZE) + column;
            if (index >= SPAD_MAX_COUNT) {
                return false;
            }
            if (spads_enabled_count == spads_to_enable_count) {
                /* We are done */
                break;
            }
            if (index < offset) {
                continue;
            }
            if ((good_spad_map[row] >> column) & 0x1) {
                spad_map[row] |= (1 << column);
                spads_enabled_count++;
            }
        }
        if (spads_enabled_count == spads_to_enable_count) {
            /* To avoid looping unnecessarily when we are already done. */
            break;
        }
    }

    if (spads_enabled_count != spads_to_enable_count) {
        return false;
    }

    /* Write the new SPAD configuration */
    uint8_t reg = REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0;
    I2C_WRITE(i2c_status, interface, reg);
    I2C_WRITE(i2c_status, interface, spad_map);

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

	i2c_status = set_spads_from_nvm(interface);
	if (i2c_status != HAL_OK)
	{
		printf("spads config err: %d\n", i2c_status);
		return false;
	}

	i2c_status = vl5310x_tuning_settings(interface);
	if (i2c_status != HAL_OK)
	{
		printf("tuning config err: %d\n", i2c_status);
		return false;
	}

	i2c_status = vl5310x_config_interrupt(interface);
	if (i2c_status != HAL_OK)
	{
		printf("tuning config err: %d\n", i2c_status);
		return false;
	}

	I2C_WRITE_8BIT(i2c_status, interface, REG_SYSTEM_SEQUENCE_CONFIG, RANGE_SEQUENCE_STEP_DSS +
            RANGE_SEQUENCE_STEP_PRE_RANGE +
            RANGE_SEQUENCE_STEP_FINAL_RANGE);

	i2c_status = perform_single_ref_calibration(interface, CALIBRATION_TYPE_VHV);
	if (i2c_status != HAL_OK)
	{
		printf("calibration vhv err: %d\n", i2c_status);
		return false;
	}

	i2c_status = perform_single_ref_calibration(interface, CALIBRATION_TYPE_PHASE);
	if (i2c_status != HAL_OK)
	{
		printf("calibration phase err: %d\n", i2c_status);
		return false;
	}

	I2C_WRITE_8BIT(i2c_status, interface, REG_SYSTEM_SEQUENCE_CONFIG, RANGE_SEQUENCE_STEP_DSS +
	            RANGE_SEQUENCE_STEP_PRE_RANGE +
	            RANGE_SEQUENCE_STEP_FINAL_RANGE);


	return true;
}

VL53l0X_Interface_t *vl53l0x_init(I2C_HandleTypeDef *i2c_handle, uint16_t i2c_addr, GPIO_TypeDef *gpio_port, uint16_t gpio_pin)
{
	// Setup interface
	vl53l0x_interface.i2c_handle = i2c_handle;
	vl53l0x_interface.i2c_addr = i2c_addr;
	vl53l0x_interface.gpio_handle.port = gpio_port;
	vl53l0x_interface.gpio_handle.pin = gpio_pin;

	// Set shuttle pin low to reset IC
	HAL_GPIO_WritePin(vl53l0x_interface.gpio_handle.port, vl53l0x_interface.gpio_handle.pin, GPIO_PIN_RESET);

	// 3s delay
	HAL_Delay(3000);

	// Set shuttle pin high to activate IC
	HAL_GPIO_WritePin(vl53l0x_interface.gpio_handle.port, vl53l0x_interface.gpio_handle.pin, GPIO_PIN_SET);

	// 10ms delay
	HAL_Delay(10);

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

void vl53l0x_set_isr_flag()
{
	isr_triggered = true;
}

HAL_StatusTypeDef vl53l0x_prepare_sample(VL53l0X_Interface_t *interface)
{
	const I2CPacket_t setup_cmd[] = {
		{0x80, 0x01},
		{0xFF, 0x01},
		{0x91, interface->stop_variable},
		{0x00, 0x01},
		{0xFF, 0x00},
		{0x80, 0x00}
	};

	HAL_StatusTypeDef i2c_status;

	for (size_t i = 0; i < ARR_LEN(setup_cmd); ++i)
	{
		I2C_WRITE_8BIT(i2c_status, interface, setup_cmd[i].reg, setup_cmd[i].data);
	}

	I2C_WRITE_8BIT(i2c_status, interface, REG_SYSRANGE_START, 0x01);

	uint8_t sysrange_start = 0;
	do
	{
		I2C_READ(i2c_status, interface, REG_SYSRANGE_START, sysrange_start);
	} while (sysrange_start & 0x01);

	return HAL_OK;
}

HAL_StatusTypeDef vl53l0x_read_range_single(VL53l0X_Interface_t *interface, uint16_t *range, bool prepare_sample)
{
	HAL_StatusTypeDef i2c_status;

	if (prepare_sample)
	{
		if (vl53l0x_prepare_sample(interface) != HAL_OK)
		{
			return HAL_ERROR;
		}
	}

    vl5310x_wait_for_isr(interface);

    I2C_READ(i2c_status, interface, REG_RESULT_RANGE_STATUS + REG_SYSTEM_INTERRUPT_CONFIG_GPIO, *range);

    // Flip bytes
    *range = ((*range << 8) & 0xFF00) | ((*range >> 8) & 0x00FF);

    if (vl5310x_clear_isr(interface) != HAL_OK)
    {
    	printf("Failed to clear ISR\n");
    }

    return HAL_OK;
}
