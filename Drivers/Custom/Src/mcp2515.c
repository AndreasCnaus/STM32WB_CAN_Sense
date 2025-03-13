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
    u8 des_opmode = ((mode << MCP2515_OPMOD_O) & MCP2515_OPMOD_M);  // Align mode to REQOP[2:0]

    // Read the CANCTRL register
    u8 canctl = 0x00;
    status = mcp2515_read_reg(hspi, MCP2515_CANCTRL, &canctl);
    if (status != HAL_OK) {
        LOG_ERROR("Failed to read CANCTRL register (status %d)", status);
        return status;
    }

    // Clear REQOP bits and set the desired operation mode
    canctl = ((canctl & ~MCP2515_OPMOD_M) | des_opmode);
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
        LOG_ERROR("Failed to read CANSTAT register (status %d)", status);
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
 * @retval HAL_StatusTypeDef Status of the operation (HAL_OK on success).
 */
HAL_StatusTypeDef mcp2515_get_opmode(SPI_HandleTypeDef *hspi, u8 *mode)
{
    HAL_StatusTypeDef status = HAL_OK;
    u8 canstat = 0x00;                 // Value of the CANSTAT register

    // Read the CANSTAT register to retrieve the current operating mode
    status = mcp2515_read_reg(hspi, MCP2515_CANSTAT, &canstat);
    if (status != HAL_OK) {
        LOG_ERROR("Failed to read CANSTAT register (status %d)", status);
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


/**
 * @brief  Configures the MCP2515 hardware, including baud rate, sample point, and enabling interrupts.
 *
 * @param  hspi  Pointer to the SPI handle structure used for SPI communication.
 *
 * @retval HAL_StatusTypeDef Status of the operation (HAL_OK on success).
 */
HAL_StatusTypeDef mcp2515_config_hw(SPI_HandleTypeDef *hspi)
{
    HAL_StatusTypeDef status = HAL_OK;

    // Register configuration values
    u8 cnf1 = 0;     // Baud rate configuration register 1 value
    u8 cnf2 = 0;     // Baud rate configuration register 2 value
    u8 cnf3 = 0;     // Baud rate configuration register 3 value
    u8 caninte = 0;  // Interrupt enable register value

    // Bit-timing and baud rate parameters
    u8 sjw = 1;          // Synchronization jump width
    u8 bpr = 2;          // Baud rate prescaler to divide the oscillator frequency
    u8 btlmode = 1;      // Length of PS2 determined by PHSEG2 bits of CNF3
    u8 sam = 1;          // Bus line sampled three times at the sample point
    u8 ps1 = 7;          // Phase segment 1 time in time quanta (TQ)
    u8 ps2 = 6;          // Phase segment 2 time in TQ
    u8 tprop_seg = 2;    // Propagation segment time in TQ
    u8 tsync = 1;        // Synchronization segment time in TQ
    u32 osc_freq = 8000000;  // Oscillator frequency, 8 MHz
    u8 nofTq = 0;            // Number of time quanta (will be calculated)
    u32 baudrate = 0;        // Baud rate (will be calculated)

    // Set CAN-Control register to 0x00 (reset state)
    status = mcp2515_write_reg(hspi, MCP2515_CANCTRL, 0x00);
    if (status != HAL_OK) {
        LOG_ERROR("Failed to initialize CAN-Control register (status: %d)", status);
        return status;
    }

    // Configure bit-timing and baud rate settings (CNF1, CNF2, CNF3)
    cnf1 |= (sjw << MCP2515_SJW_O) | bpr;  // Set synchronization jump width and prescaler
    cnf2 |= (btlmode << MCP2515_BTLMODE_O) | (sam << MCP2515_SAM_O) |
            (ps1 << MCP2515_PHSEG1_O) | (tprop_seg << MCP2515_PRSEG_O);  // Configure sample point and PS1
    cnf3 |= ps2;  // Set phase segment 2 timing

    // Write configuration to MCP2515 registers
    status = mcp2515_write_reg(hspi, MCP2515_CNF1, cnf1);
    if (status != HAL_OK) {
        LOG_ERROR("Failed to write CNF1 register (status: %d)", status);
        return HAL_ERROR;
    }
    status = mcp2515_write_reg(hspi, MCP2515_CNF2, cnf2);
    if (status != HAL_OK) {
        LOG_ERROR("Failed to write CNF2 register (status: %d)", status);
        return HAL_ERROR;
    }
    status = mcp2515_write_reg(hspi, MCP2515_CNF3, cnf3);
    if (status != HAL_OK) {
        LOG_ERROR("Failed to write CNF3 register (status: %d)", status);
        return HAL_ERROR;
    }

    // Calculate and log baud rate (for debugging purposes)
    nofTq = tsync + tprop_seg + ps1 + ps2;
    baudrate = osc_freq / (2 * bpr * nofTq);
    LOG_INFO("Calculated baud rate: %d bps", baudrate);

    // Enable interrupts via CANINTE register
    caninte = 0xFF;  // Enable all interrupts
    status = mcp2515_write_reg(hspi, MCP2515_CANINTE, caninte);
    if (status != HAL_OK) {
        LOG_ERROR("Failed to set interrupt enable register (status: %d)", status);
        return status;
    }

    LOG_INFO("The MCP2515 device has been configured successfully.");
    return HAL_OK;	// Return success status
}

/**
 * @brief  Sets the TX buffer 2 identifier (standard 11-bit CAN ID) on the MCP2515.
 *
 * @param  hspi  Pointer to the SPI handle structure used for SPI communication.
 * @param  id    The 11-bit CAN identifier to set for TX buffer 2.
 *
 * @retval HAL_StatusTypeDef Status of the operation (HAL_OK on success).
 */
HAL_StatusTypeDef mcp2515_set_tx2_sid(SPI_HandleTypeDef *hspi, u16 id)
{
    HAL_StatusTypeDef status = HAL_OK;

    // Validate the 11-bit CAN ID
    if (id > 0x7FF) {
        LOG_ERROR("Invalid CAN ID: 0x%X. Must be 11 bits", id);
        return HAL_ERROR;
    }

    // Standard Identifier (SID) consists of 11 bits: split into 8 higher and 3 lower bits
    u8 sidl, sidh;

    // Split the ID: lower 3 bits to SIDL (properly shifted), upper 8 bits to SIDH
    sidl = (u8)((id & 0x07) << MCP2515_TXB_SIDL_O);  // Mask and shift lower 3 bits
    sidh = (u8)(id >> 3);                            // Shift higher 8 bits

    // Write SIDL register (lower 3 bits)
    status = mcp2515_write_reg(hspi, MCP2515_TXB2SIDL, sidl);
    if (status != HAL_OK) {
        LOG_ERROR("Failed to write SIDL register for TXB2 (ID: 0x%X, status: %d)", id, status);
        return status;
    }

    // Write SIDH register (upper 8 bits)
    status = mcp2515_write_reg(hspi, MCP2515_TXB2SIDH, sidh);
    if (status != HAL_OK) {
        LOG_ERROR("Failed to write SIDH register for TXB2 (ID: 0x%X, status: %d)", id, status);
        return status;
    }

    return HAL_OK;	// Return success status
}

/**
 * @brief  Sets the TX buffer 2 data payload on the MCP2515.
 *
 * @param  hspi  Pointer to the SPI handle structure used for SPI communication.
 * @param  data  Pointer to the data array to transmit.
 * @param  len   Length of the data array (max: MCP2515_MAXDL).
 *
 * @retval HAL_StatusTypeDef Status of the operation (HAL_OK on success).
 */
HAL_StatusTypeDef mcp2515_set_tx2_data(SPI_HandleTypeDef *hspi, const u8 *data, u8 len)
{
    HAL_StatusTypeDef status = HAL_OK;

    // Limit the data length to the maximum allowed value
    u8 dlc = (len < (u8)MCP2515_MAXDL) ? len : (u8)MCP2515_MAXDL;
    dlc = (dlc & MCP2515_TXB_DLC_M);  // Mask DLC to ensure proper format

    // Set the data length code (DLC) register
    status = mcp2515_write_reg(hspi, MCP2515_TXB2DLC, dlc);
    if (status != HAL_OK) {
        LOG_ERROR("Failed to set the data length code (len: %d, status: %d)", len, status);
        return status;
    }

    // Write the data bytes to the transmit buffer
    for (int i = 0; i < dlc; i++) {
        status = mcp2515_write_reg(hspi, MCP2515_TXB2D0 + i, data[i]);
        if (status != HAL_OK) {
            LOG_ERROR("Failed to write data byte(%d) to the transmit buffer 2 (status: %d, len: %d)", i, status, len);
            return status;
        }
    }

    return HAL_OK;	// Return success status
}

/**
 * @brief  Transmit a low-priority CAN message using transmit buffer 2.
 *
 * @param  hspi  Pointer to the SPI handle structure used for SPI communication.
 * @param  id    CAN identifier (11-bit standard ID).
 * @param  data  Pointer to the data array to transmit.
 * @param  len   Length of the data array (max: MCP2515_MAXDL).
 *
 * @retval HAL_StatusTypeDef Status of the operation (HAL_OK on success, or other HAL error codes).
 */
HAL_StatusTypeDef mcp2515_write_can_frame(SPI_HandleTypeDef *hspi, u16 id, const u8 *data, u8 len)
{
    HAL_StatusTypeDef status = HAL_OK;
    u8 txb2ctrl = 0;

    // Validate input parameters
    if (hspi == NULL || data == NULL) {
        LOG_ERROR("Invalid SPI handle or data pointer provided\n");
        return HAL_ERROR;
    }

    // Configure the TXB2CTRL register: Set TXREQ and low priority
    txb2ctrl |= MCP2515_TXB_TXREQ;  // Set transmit request
    txb2ctrl |= MCP2515_TXP_L_PRIO; // Set message priority to low

    // Set the message standard ID (SID)
    status = mcp2515_set_tx2_sid(hspi, id);
    if (status != HAL_OK) {
        LOG_ERROR("Failed to set Message SID (status: %d)\n", status);
        return status;
    }

    // Set the message data
    status = mcp2515_set_tx2_data(hspi, data, len);
    if (status != HAL_OK) {
        LOG_ERROR("Failed to set Message data (status: %d)\n", status);
        return status;
    }

    // Initiate message transmission
    status = mcp2515_write_reg(hspi, MCP2515_TXB2CTRL, txb2ctrl);
    if (status != HAL_OK) {
        LOG_ERROR("Failed to initiate message transmission on TX buffer 2 (status: %d)\n", status);
        return status;
    }

    return HAL_OK; // Return success status
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


