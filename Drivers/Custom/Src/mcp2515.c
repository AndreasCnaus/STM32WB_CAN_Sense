#include <mcp2515.h>
#include <logger.h>

const char *mode_strings[] = {
    "Normal Operation mode",  // 0x00
    "Sleep mode",             // 0x01
    "Loopback mode",          // 0x02
    "Listen-Only mode",       // 0x03
    "Configuration mode"      // 0x04
};

/**
 * @brief  Writes a value to a specific register of the MCP2515 via SPI.
 *
 * @param  hspi  Pointer to the SPI handle structure used for communication.
 * @param  reg   Register address to write to.
 * @param  value Value to be written to the specified register.
 *
 * @retval HAL_StatusTypeDef Status of the SPI transmission (HAL_OK on success).
 */
HAL_StatusTypeDef mcp2515_write_reg(SPI_HandleTypeDef *hspi, u8 reg, u8 value)
{
    u8 tx_buf[3];          		// Buffer to hold transmit data
    tx_buf[0] = MCP2515_WRITE; 	// MCP2515 Write command
    tx_buf[1] = reg;           	// Register address to write
    tx_buf[2] = value;         	// Value to write into the register

    // Pull NSS (CS) Low to begin communication
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    // Perform SPI transmission
    HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, tx_buf, 3, HAL_MAX_DELAY);

    // Pull NSS (CS) High to end communication
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    return status;
}

/**
 * @brief  Reads a value from a specific register of the MCP2515 via SPI.
 *
 * @param  hspi  Pointer to the SPI handle structure used for communication.
 * @param  reg   Register address to read from.
 * @param  value Pointer to a variable where the read register value will be stored.
 *
 * @retval HAL_StatusTypeDef Status of the SPI transaction (HAL_OK on success).
 */
HAL_StatusTypeDef mcp2515_read_reg(SPI_HandleTypeDef *hspi, u8 reg, u8 *value)
{
    HAL_StatusTypeDef status = HAL_OK;

    u8 tx_buf[3] = {0}; // Transmit buffer to send the MCP2515 Read command and register address
    u8 rx_buf[3] = {0}; // Receive buffer to capture the response from the MCP2515

    // Prepare the transmit buffer
    tx_buf[0] = MCP2515_READ; // MCP2515 Read command
    tx_buf[1] = reg;          // Register address to read from

    // Pull NSS (CS) Low to begin communication
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    // Perform SPI transaction: Send tx_buf and receive data simultaneously into rx_buf
    status = HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, 3, HAL_MAX_DELAY);

    // Pull NSS (CS) High to end communication
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    // If the transaction is successful, extract the register value from the response
    if (status == HAL_OK) {
        *value = rx_buf[2];  // The second received byte contains the register value
    }

    return status;
}

/**
 * @brief  Modifies specific bits in a register of the MCP2515 via SPI.
 *
 * @param  hspi  Pointer to the SPI handle structure used for communication.
 * @param  reg   Register address to modify.
 * @param  mask  Bitmask specifying which bits to modify.
 * @param  value Value to write to the specified bits (applied using the mask).
 *
 * @retval HAL_StatusTypeDef Status of the SPI transaction (HAL_OK on success).
 */
HAL_StatusTypeDef mcp2515_write_bit(SPI_HandleTypeDef *hspi, u8 reg, u8 mask, u8 value)
{
    HAL_StatusTypeDef status = HAL_OK;
    u8 tx_buf[4];                      	// Transmit buffer to hold the command and data

    // Prepare the transmit buffer
    tx_buf[0] = MCP2515_BIT_MODIFY;  // MCP2515 Bit Modify command
    tx_buf[1] = reg;                 // Register address to modify
    tx_buf[2] = mask;                // Mask specifying which bits to modify
    tx_buf[3] = value;               // Value to set for the specified bits

    // Pull NSS (CS) Low to begin communication
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    // Perform SPI transmission: Send the command and data to the MCP2515
    status = HAL_SPI_Transmit(hspi, tx_buf, sizeof(tx_buf), HAL_MAX_DELAY);

    // Pull NSS (CS) High to end communication
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    return status;
}

/**
 * @brief  Sets the desired operation mode for the MCP2515.
 *
 * @param  hspi  Pointer to the SPI handle structure.
 * @param  mode  Desired operation mode (REQOP[2:0]).
 *
 * @retval HAL_StatusTypeDef Status of the operation (HAL_OK on success).
 */
HAL_StatusTypeDef mcp2515_set_opmode(SPI_HandleTypeDef *hspi, u8 mode)
{
    HAL_StatusTypeDef status = HAL_OK;
    u8 des_opmode = (mode << MCP2515_OPMOD_O);  // Align mode to REQOP[2:0]

    // Read the CANCTRL register
    u8 canctl = 0x00;
    status = mcp2515_read_reg(hspi, MCP2515_CANCTRL, &canctl);
    if (status != HAL_OK) {
        LOG_ERROR("Failed to read CANCTRL register with status %d", status);
        return status;
    }

    LOG_INFO("CANCTL Register value: %u", canctl);

    // Clear REQOP bits and set the desired operation mode
    canctl = (des_opmode & MCP2515_OPMOD_M);
    status = mcp2515_write_reg(hspi, MCP2515_CANCTRL, canctl);
    if (status != HAL_OK) {
        LOG_ERROR("Failed to write CANCTRL register with status %d", status);
        return status;
    }

    // Wait for mode transition (10ms)
    HAL_Delay(10);

    // Read the CANSTAT register to verify the current mode
    u8 canstat = 0x00;
    status = mcp2515_read_reg(hspi, MCP2515_CANSTAT, &canstat);  // Correct register
    if (status != HAL_OK) {
        LOG_ERROR("Failed to read CANSTAT register with status %d", status);
        return status;
    }

    // Verify the current operation mode matches the desired operation mode
    if ((canstat & MCP2515_OPMOD_M) != des_opmode) {
        LOG_ERROR("Operation mode transition failed (requested: 0x%02X, current: 0x%02X)",
                  des_opmode >> MCP2515_OPMOD_O,
                  (canstat & MCP2515_OPMOD_M) >> MCP2515_OPMOD_O);
        return HAL_ERROR;
    }

    return HAL_OK; // Return success status
}

/**
 * @brief  Retrieves the current operating mode of the MCP2515.
 *
 * @param  hspi  Pointer to the SPI handle structure used for SPI communication.
 * @param  mode  Pointer to a variable where the current operating mode will be stored (OPMOD[2:0]).
 *
 * @retval HAL_StatusTypeDef Status of the operation (HAL_OK on success, HAL_ERROR on failure).
 */
HAL_StatusTypeDef mcp2515_get_opmode(SPI_HandleTypeDef *hspi, u8 *mode)
{
    HAL_StatusTypeDef status = HAL_OK;
    u8 canstat = 0x00;                 // Value of the CANSTAT register

    // Read the CANSTAT register to retrieve the current operating mode
    status = mcp2515_read_reg(hspi, MCP2515_CANSTAT, &canstat);
    if (status != HAL_OK) {
        LOG_ERROR("Failed to read CANSTAT register with status %d", status);
        return status;
    }

    // Extract the OPMOD[2:0] bits and store them in the mode variable
    *mode = ((canstat & MCP2515_OPMOD_M) >> MCP2515_OPMOD_O);

    return HAL_OK; // Return success status
}

/**
 * @brief  Resets the MCP2515 hardware and verifies it transitions to Configuration Mode.
 *
 * @param  hspi  Pointer to the SPI handle structure used for SPI communication.
 *
 * @retval HAL_StatusTypeDef Status of the operation (HAL_OK on success).
 */
HAL_StatusTypeDef mcp2515_reset_hw(SPI_HandleTypeDef *hspi)
{
    HAL_StatusTypeDef status = HAL_OK;

    // Send the reset command to the MCP2515
    u8 reset_cmd = MCP2515_RESET;  // The MCP2515 Reset command
    status = HAL_SPI_Transmit(hspi, &reset_cmd, sizeof(reset_cmd), HAL_MAX_DELAY);
    if (status != HAL_OK) {
        LOG_ERROR("Failed to send reset command to MCP2515 (status: %d)", status);
        return status;
    }

    // Wait for the MCP2515 to reset and stabilize (10ms)
    HAL_Delay(10);

    // Read the current operation mode from the MCP2515
    u8 curr_mode = 0x00;  // Variable to hold the current operation mode
    status = mcp2515_get_opmode(hspi, &curr_mode);  // Read the operation mode
    if (status != HAL_OK) {
        LOG_ERROR("Failed to read current operation mode after reset (status: %d)", status);
        return status;
    }

    // Verify the MCP2515 transitioned to Configuration Mode after reset
    if (curr_mode != MCP2515_CONFIG_MODE) {
        LOG_ERROR("MCP2515 did not transition to Configuration Mode after reset (current mode: 0x%02X)", curr_mode);
        return HAL_ERROR;
    }

    // LOG_INFO("MCP2515 successfully reset and is in Configuration Mode.");
    return HAL_OK;  // Return success status
}


size_t get_opmode_strings_len(void) {
    return sizeof(mode_strings) / sizeof(mode_strings[0]);
}

const char* get_opmode_string(u8 mode) {
    if (mode >= 0 && mode <= 4) {
        return mode_strings[mode];
    } else {
        return "Invalid operation mode";
    }
}


