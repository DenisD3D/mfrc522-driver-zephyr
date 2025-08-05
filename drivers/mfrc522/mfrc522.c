/*
 * Copyright (c) 2025 Your Name
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_mfrc522

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include "mfrc522.h"

LOG_MODULE_REGISTER(mfrc522, CONFIG_MFRC522_LOG_LEVEL);

/* Driver data and config structures */
struct mfrc522_config {
    struct spi_dt_spec bus;
    struct gpio_dt_spec reset_gpio;
};

struct mfrc522_data {
    uint8_t version;
};

void mfrc522_hw_reset(const struct device *dev) {
    const struct mfrc522_config *config = dev->config;

    gpio_pin_set_dt(&config->reset_gpio, 0);
    k_sleep(K_USEC(2)); /* Hold reset for 2ms */
    gpio_pin_set_dt(&config->reset_gpio, 1);
    k_sleep(K_MSEC(50)); /* Wait for device to stabilize */
}


int mfrc522_read_reg(const struct device *dev, const enum PCD_Register reg, uint8_t *data) {
    return mfrc522_read_reg_multi(dev, reg, data, 1);
}

int mfrc522_read_reg_multi(const struct device *dev, const enum PCD_Register reg, uint8_t *data, const size_t len) {
    const struct mfrc522_config *config = dev->config;

    if (len + 1 > 64) {
        return -ENOMEM; // Buffer too small
    }

    uint8_t tx_buf[64] = {0};
    uint8_t rx_buf[64] = {0};

    tx_buf[0] = ((reg << 1) & 0x7E) | 0x80; /* Address format: 1XXXXXX0 for read */
    for (size_t i = 1; i <= len; i++) {
        tx_buf[i] = 0x00; // Dummy bytes to clock out data
    }

    const struct spi_buf tx_bufs = {.buf = tx_buf, .len = len + 1};
    const struct spi_buf rx_bufs = {.buf = rx_buf, .len = len + 1};
    const struct spi_buf_set tx = {.buffers = &tx_bufs, .count = 1};
    const struct spi_buf_set rx = {.buffers = &rx_bufs, .count = 1};

    int ret = spi_transceive_dt(&config->bus, &tx, &rx);
    if (ret < 0) {
        return ret;
    }

    /* The actual data is in rx_buf[1..len] */
    memcpy(data, &rx_buf[1], len);

    return 0;
}

int mfrc522_write_reg(const struct device *dev, const enum PCD_Register reg, const uint8_t data) {
    uint8_t tmp = data;
    return mfrc522_write_reg_multi(dev, reg, &tmp, 1);
}

int mfrc522_write_reg_multi(const struct device *dev, const enum PCD_Register reg, const uint8_t *data, const size_t len) {
    const struct mfrc522_config *config = dev->config;
    uint8_t tx_buf[64];

    if (len + 1 > sizeof(tx_buf)) {
        return -ENOMEM;  // Buffer too small
    }

    tx_buf[0] = (reg << 1) & 0x7E; /* Address format: 0XXXXXX0 for write */
    memcpy(&tx_buf[1], data, len);

    const struct spi_buf tx_bufs = {.buf = tx_buf, .len = len + 1};
    const struct spi_buf_set tx = {.buffers = &tx_bufs, .count = 1};

    return spi_write_dt(&config->bus, &tx);
}

/* Driver initialization function */
int mfrc522_init(const struct device *dev) {
    const struct mfrc522_config *config = dev->config;
    int ret;

    /* Check if SPI bus is ready */
    if (!spi_is_ready_dt(&config->bus)) {
        LOG_ERR("SPI bus %s not ready", config->bus.bus->name);
        return -ENODEV;
    }

    /* Check if reset GPIO is ready */
    if (!gpio_is_ready_dt(&config->reset_gpio)) {
        LOG_ERR("Reset GPIO device not ready");
        return -ENODEV;
    }

    /* Configure reset pin as output */
    ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure reset GPIO: %d", ret);
        return ret;
    }

    /* Perform hardware reset */
    mfrc522_hw_reset(dev);

    /* Reset baud rates */
    mfrc522_write_reg(dev, TxModeReg, 0x00);
    mfrc522_write_reg(dev, RxModeReg, 0x00);

    /* Reset ModWidthReg */
    mfrc522_write_reg(dev, ModWidthReg, 0x26);

    /* Set timeout
    f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
    TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg. */
    mfrc522_write_reg(dev, TModeReg, 0x80);
    mfrc522_write_reg(dev, TPrescalerReg, 0xA9);
    mfrc522_write_reg(dev, TReloadRegH, 0x03);
    mfrc522_write_reg(dev, TReloadRegL, 0xE8);

    mfrc522_write_reg(dev, TxASKReg, 0x40);
    mfrc522_write_reg(dev, ModeReg, 0x3D);


    mfrc522_pcd_antenna_on(dev);

    LOG_DBG("MFRC522 initialized");
    return 0;
}

int mfrc522_get_version(const struct device *dev, uint8_t *version) {
    int ret;

    ret = mfrc522_read_reg(dev, VersionReg, version);
    if (ret < 0) {
        LOG_ERR("Failed to read version register: %d", ret);
        return ret;
    }

    return 0;
}

int mfrc522_pcd_antenna_on(const struct device *dev) {
    int ret;
    uint8_t value;

    ret = mfrc522_read_reg(dev, TxControlReg, &value);
    if (ret < 0) {
        LOG_ERR("Failed to turn on antenna: %d", ret);
        return ret;
    }

    if ((value & 0x03) != 0x03) {
        ret = mfrc522_write_reg(dev, TxControlReg, value | 0x03);
        if (ret < 0) {
            LOG_ERR("Failed to turn on antenna: %d", ret);
            return ret;
        }
    }

    return 0;
}

bool mfrc522_picc_is_new_card_present(const struct device *dev) {
    uint8_t buffer_atqa[2];
    uint8_t buffer_size = sizeof(buffer_atqa);

    // Reset baud rates
    mfrc522_write_reg(dev, TxModeReg, 0x00);
    mfrc522_write_reg(dev, RxModeReg, 0x00);
    // Reset ModWidthReg
    mfrc522_write_reg(dev, ModWidthReg, 0x26);

    const enum StatusCode result = mfrc522_picc_request_a(dev, buffer_atqa, &buffer_size);
    return (result == STATUS_OK || result == STATUS_COLLISION);
}

enum StatusCode mfrc522_picc_request_a(const struct device *dev, uint8_t *buffer_atqa, uint8_t *buffer_size) {
    return mfrc522_picc_reqa_or_wupa(dev, PICC_CMD_REQA, buffer_atqa, buffer_size);
}

enum StatusCode mfrc522_picc_wakeup_a(const struct device *dev, uint8_t *buffer_atqa, uint8_t *buffer_size) {
    return mfrc522_picc_reqa_or_wupa(dev, PICC_CMD_WUPA, buffer_atqa, buffer_size);
}

enum StatusCode mfrc522_picc_reqa_or_wupa(const struct device *dev, uint8_t command,
                                                 uint8_t *buffer_atqa, uint8_t *buffer_size) {
    uint8_t valid_bits;
    enum StatusCode status;

    // The ATQA response is 2 bytes long.
    if (buffer_atqa == NULL || *buffer_size < 2) {
        return STATUS_NO_ROOM;
    }

    // ValuesAfterColl=1 => Bits received after collision are cleared.
    mfrc522_pcd_clear_register_bitmask(dev, CollReg, 0x80);

    // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte.
    // TxLastBits = BitFramingReg[2..0]
    valid_bits = 7;

    status = mfrc522_pcd_transceive_data(dev, &command, 1, buffer_atqa, buffer_size, &valid_bits, 0, false);
    if (status != STATUS_OK) {
        return status;
    }

    // ATQA must be exactly 16 bits.
    if (*buffer_size != 2 || valid_bits != 0) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

int mfrc522_pcd_clear_register_bitmask(const struct device *dev, enum PCD_Register reg, uint8_t mask) {
    uint8_t tmp;
    int ret = mfrc522_read_reg(dev, reg, &tmp);
    if (ret < 0) {
        return ret;
    }

    // Clear the mask bits
    return mfrc522_write_reg(dev, reg, tmp & (~mask));
}

enum StatusCode mfrc522_pcd_transceive_data(const struct device *dev,
                                                   const uint8_t *send_data, uint8_t send_len,
                                                   uint8_t *back_data, uint8_t *back_len,
                                                   uint8_t *valid_bits, uint8_t rx_align, bool check_crc) {
    uint8_t wait_irq = 0x30; // RxIRq and IdleIRq
    return mfrc522_pcd_communicate_with_picc(dev, PCD_Transceive, wait_irq,
                                             send_data, send_len,
                                             back_data, back_len,
                                             valid_bits, rx_align, check_crc);
}

enum StatusCode mfrc522_pcd_communicate_with_picc(const struct device *dev,
                                                        uint8_t command, uint8_t wait_irq,
                                                        const uint8_t *send_data, uint8_t send_len,
                                                        uint8_t *back_data, uint8_t *back_len,
                                                        uint8_t *valid_bits, uint8_t rx_align,
                                                        bool check_crc) {
    // Prepare values for BitFramingReg
    uint8_t tx_last_bits = valid_bits ? *valid_bits : 0;
    uint8_t bit_framing = (rx_align << 4) + tx_last_bits;  // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

    mfrc522_write_reg(dev, CommandReg, PCD_Idle);         // Stop any active command
    mfrc522_write_reg(dev, ComIrqReg, 0x7F);              // Clear all seven interrupt request bits
    mfrc522_write_reg(dev, FIFOLevelReg, 0x80);           // FlushBuffer = 1, FIFO initialization

    // Write sendData to the FIFO
    mfrc522_write_reg_multi(dev, FIFODataReg, send_data, send_len);

    mfrc522_write_reg(dev, BitFramingReg, bit_framing);   // Bit adjustments
    mfrc522_write_reg(dev, CommandReg, command);          // Execute the command

    if (command == PCD_Transceive) {
        mfrc522_pcd_set_register_bitmask(dev, BitFramingReg, 0x80);  // StartSend=1, transmission of data starts
    }

    // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer
    // automatically starts when the PCD stops transmitting.
    //
    // Wait here for the command to complete. The bits specified in the
    // `wait_irq` parameter define what bits constitute a completed command.
    // When they are set in the ComIrqReg register, then the command is
    // considered complete. If the command is not indicated as complete in
    // ~36ms, then consider the command as timed out.
    bool completed = false;
    int64_t start_time = k_uptime_get();
    int64_t deadline = start_time + 36;  // 36ms timeout

    do {
        uint8_t irq;
        mfrc522_read_reg(dev, ComIrqReg, &irq);  // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq

        if (irq & wait_irq) {  // One of the interrupts that signal success has been set
            completed = true;
            break;
        }

        if (irq & 0x01) {  // Timer interrupt - nothing received in 25ms
            return STATUS_TIMEOUT;
        }

        k_yield();  // Equivalent to Arduino's yield()
    } while (k_uptime_get() < deadline);

    // 36ms and nothing happened. Communication with the MFRC522 might be down.
    if (!completed) {
        return STATUS_TIMEOUT;
    }

    // Stop now if any errors except collisions were detected.
    uint8_t error_reg;
    mfrc522_read_reg(dev, ErrorReg, &error_reg);  // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
    if (error_reg & 0x13) {  // BufferOvfl ParityErr ProtocolErr
        return STATUS_ERROR;
    }

    uint8_t _valid_bits = 0;

    // If the caller wants data back, get it from the MFRC522.
    if (back_data && back_len) {
        uint8_t fifo_level;
        mfrc522_read_reg(dev, FIFOLevelReg, &fifo_level);  // Number of bytes in the FIFO

        if (fifo_level > *back_len) {
            return STATUS_NO_ROOM;
        }

        *back_len = fifo_level;  // Number of bytes returned

        for (uint8_t i = 0; i < fifo_level; i++) {
            uint8_t value;
            mfrc522_read_reg(dev, FIFODataReg, &value);

            // Apply bit shifting if needed
            if (i == 0 && rx_align > 0) {
                // Create bit mask for bit positions rxAlign..7
                uint8_t mask = (0xFF << rx_align) & 0xFF;
                // Get bits rxAlign..7
                value = (value & mask);
                // We need to store the first byte from the first read
                // this will be needed later
                if (i < *back_len) {
                    back_data[i] = value;
                }
            } else {
                // Normal case, no bit shifting needed
                if (i < *back_len) {
                    back_data[i] = value;
                }
            }
        }

        // RxLastBits[2:0] indicates the number of valid bits in the last received byte.
        // If this value is 000b, the whole byte is valid.
        uint8_t control_reg;
        mfrc522_read_reg(dev, ControlReg, &control_reg);
        _valid_bits = control_reg & 0x07;

        if (valid_bits) {
            *valid_bits = _valid_bits;
        }
    }

    // Tell about collisions
    if (error_reg & 0x08) {  // CollErr
        return STATUS_COLLISION;
    }

    // Perform CRC_A validation if requested.
    if (back_data && back_len && check_crc) {
        // In this case a MIFARE Classic NAK is not OK.
        if (*back_len == 1 && _valid_bits == 4) {
            return STATUS_MIFARE_NACK;
        }

        // We need at least the CRC_A value and all 8 bits of the last byte must be received.
        if (*back_len < 2 || _valid_bits != 0) {
            return STATUS_CRC_WRONG;
        }

        // Verify CRC_A - do our own calculation and store the control in control_buffer.
        // Note: In a real implementation, you would need to implement the PCD_CalculateCRC function
        uint8_t control_buffer[2];
        enum StatusCode status = mfrc522_pcd_calculate_crc(dev, &back_data[0], *back_len - 2, &control_buffer[0]);
        if (status != STATUS_OK) {
            return status;
        }

        if ((back_data[*back_len - 2] != control_buffer[0]) || (back_data[*back_len - 1] != control_buffer[1])) {
            return STATUS_CRC_WRONG;
        }
    }

    return STATUS_OK;
}


int mfrc522_pcd_set_register_bitmask(const struct device *dev, enum PCD_Register reg, uint8_t mask) {
    uint8_t tmp;
    int ret = mfrc522_read_reg(dev, reg, &tmp);
    if (ret < 0) {
        return ret;
    }

    ret = mfrc522_write_reg(dev, reg, tmp | mask); // set bit mask
    return ret;
}

enum StatusCode mfrc522_picc_select(const struct device *dev, struct Uid *uid, uint8_t valid_bits) {
    bool uid_complete = false;
    bool select_done = false;
    bool use_cascade_tag = false;
    uint8_t cascade_level = 1;
    enum StatusCode result;
    uint8_t count;
    uint8_t check_bit;
    uint8_t index;
    uint8_t uid_index;                  // The first index in uid->uid_byte[] that is used in the current Cascade Level
    int8_t current_level_known_bits;    // The number of known UID bits in the current Cascade Level
    uint8_t buffer[9];                  // The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
    uint8_t buffer_used;                // The number of bytes used in the buffer, i.e., the number of bytes to transfer to the FIFO
    uint8_t rx_align;                   // Used in BitFramingReg. Defines the bit position for the first bit received
    uint8_t tx_last_bits;               // Used in BitFramingReg. The number of valid bits in the last transmitted byte
    uint8_t *response_buffer;
    uint8_t response_length;

    // Description of buffer structure:
    //    Byte 0: SEL             Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
    //    Byte 1: NVB             Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits.
    //    Byte 2: UID-data or CT  See explanation below. CT means Cascade Tag.
    //    Byte 3: UID-data
    //    Byte 4: UID-data
    //    Byte 5: UID-data
    //    Byte 6: BCC             Block Check Character - XOR of bytes 2-5
    //    Byte 7: CRC_A
    //    Byte 8: CRC_A

    // Sanity checks
    if (valid_bits > 80) {
        return STATUS_INVALID;
    }

    // Prepare MFRC522
    mfrc522_pcd_clear_register_bitmask(dev, CollReg, 0x80);  // ValuesAfterColl=1 => Bits received after collision are cleared

    // Repeat Cascade Level loop until we have a complete UID
    while (!uid_complete) {
        // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2
        switch (cascade_level) {
            case 1:
                buffer[0] = PICC_CMD_SEL_CL1;
                uid_index = 0;
                use_cascade_tag = valid_bits && uid->size > 4;  // When we know that the UID has more than 4 bytes
                break;

            case 2:
                buffer[0] = PICC_CMD_SEL_CL2;
                uid_index = 3;
                use_cascade_tag = valid_bits && uid->size > 7;  // When we know that the UID has more than 7 bytes
                break;

            case 3:
                buffer[0] = PICC_CMD_SEL_CL3;
                uid_index = 6;
                use_cascade_tag = false;                       // Never used in CL3
                break;

            default:
                return STATUS_INTERNAL_ERROR;
        }

        // How many UID bits are known in this Cascade Level?
        current_level_known_bits = valid_bits - (8 * uid_index);
        if (current_level_known_bits < 0) {
            current_level_known_bits = 0;
        }

        // Copy the known bits from uid->uid_byte[] to buffer[]
        index = 2;  // destination index in buffer[]
        if (use_cascade_tag) {
            buffer[index++] = PICC_CMD_CT;
        }

        uint8_t bytes_to_copy = current_level_known_bits / 8 + (current_level_known_bits % 8 ? 1 : 0);  // The number of bytes needed to represent the known bits for this level
        if (bytes_to_copy) {
            uint8_t max_bytes = use_cascade_tag ? 3 : 4;  // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
            if (bytes_to_copy > max_bytes) {
                bytes_to_copy = max_bytes;
            }
            for (count = 0; count < bytes_to_copy; count++) {
                buffer[index++] = uid->uid_byte[uid_index + count];
            }
        }

        // Now that the data has been copied we need to include the 8 bits in CT in current_level_known_bits
        if (use_cascade_tag) {
            current_level_known_bits += 8;
        }

        // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations
        select_done = false;
        while (!select_done) {
            // Find out how many bits and bytes to send and receive
            if (current_level_known_bits >= 32) {  // All UID bits in this Cascade Level are known. This is a SELECT.
                buffer[1] = 0x70;  // NVB - Number of Valid Bits: Seven whole bytes

                // Calculate BCC - Block Check Character
                buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];

                // Calculate CRC_A
                result = mfrc522_pcd_calculate_crc(dev, buffer, 7, &buffer[7]);
                if (result != STATUS_OK) {
                    return result;
                }

                tx_last_bits = 0;  // 0 => All 8 bits are valid
                buffer_used = 9;

                // Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
                response_buffer = &buffer[6];
                response_length = 3;
            }
            else {  // This is an ANTICOLLISION
                tx_last_bits = current_level_known_bits % 8;
                count = current_level_known_bits / 8;     // Number of whole bytes in the UID part
                index = 2 + count;                        // Number of whole bytes: SEL + NVB + UIDs
                buffer[1] = (index << 4) + tx_last_bits;  // NVB - Number of Valid Bits
                buffer_used = index + (tx_last_bits ? 1 : 0);

                // Store response in the unused part of buffer
                response_buffer = &buffer[index];
                response_length = sizeof(buffer) - index;
            }

            // Set bit adjustments
            rx_align = tx_last_bits;  // Having a separate variable is overkill. But it makes the next line easier to read
            mfrc522_write_reg(dev, BitFramingReg, (rx_align << 4) + tx_last_bits);  // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

            // Transmit the buffer and receive the response
            result = mfrc522_pcd_transceive_data(dev, buffer, buffer_used, response_buffer, &response_length, &tx_last_bits, rx_align, false);

            if (result == STATUS_COLLISION) {  // More than one PICC in the field => collision
                uint8_t value_of_coll_reg;
                mfrc522_read_reg(dev, CollReg, &value_of_coll_reg);  // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]

                if (value_of_coll_reg & 0x20) {  // CollPosNotValid
                    return STATUS_COLLISION;  // Without a valid collision position we cannot continue
                }

                uint8_t collision_pos = value_of_coll_reg & 0x1F;  // Values 0-31, 0 means bit 32
                if (collision_pos == 0) {
                    collision_pos = 32;
                }

                if (collision_pos <= current_level_known_bits) {  // No progress - should not happen
                    return STATUS_INTERNAL_ERROR;
                }

                // Choose the PICC with the bit set
                current_level_known_bits = collision_pos;
                count = current_level_known_bits % 8;  // The bit to modify
                check_bit = (current_level_known_bits - 1) % 8;
                index = 1 + (current_level_known_bits / 8) + (count ? 1 : 0);  // First byte is index 0
                buffer[index] |= (1 << check_bit);
            }
            else if (result != STATUS_OK) {
                return result;
            }
            else {  // STATUS_OK
                if (current_level_known_bits >= 32) {  // This was a SELECT
                    select_done = true;  // No more anticollision
                    // We continue below outside the while
                }
                else {  // This was an ANTICOLLISION
                    // We now have all 32 bits of the UID in this Cascade Level
                    current_level_known_bits = 32;
                    // Run loop again to do the SELECT
                }
            }
        }  // End of while (!select_done)

        // We do not check the CBB - it was constructed by us above

        // Copy the found UID bytes from buffer[] to uid->uid_byte[]
        index = (buffer[2] == PICC_CMD_CT) ? 3 : 2;  // source index in buffer[]
        bytes_to_copy = (buffer[2] == PICC_CMD_CT) ? 3 : 4;

        for (count = 0; count < bytes_to_copy; count++) {
            uid->uid_byte[uid_index + count] = buffer[index++];
        }

        // Check response SAK (Select Acknowledge)
        if (response_length != 3 || tx_last_bits != 0) {  // SAK must be exactly 24 bits (1 byte + CRC_A)
            return STATUS_ERROR;
        }

        // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore
        result = mfrc522_pcd_calculate_crc(dev, response_buffer, 1, &buffer[2]);
        if (result != STATUS_OK) {
            return result;
        }

        if ((buffer[2] != response_buffer[1]) || (buffer[3] != response_buffer[2])) {
            return STATUS_CRC_WRONG;
        }

        if (response_buffer[0] & 0x04) {  // Cascade bit set - UID not complete yet
            cascade_level++;
        }
        else {
            uid_complete = true;
            uid->sak = response_buffer[0];
        }
    }  // End of while (!uid_complete)

    // Set correct uid->size
    uid->size = 3 * cascade_level + 1;

    return STATUS_OK;
}

enum StatusCode mfrc522_pcd_calculate_crc(const struct device *dev,
                                               const uint8_t *data, uint8_t length,
                                               uint8_t *result) {
    mfrc522_write_reg(dev, CommandReg, PCD_Idle);        // Stop any active command
    mfrc522_write_reg(dev, DivIrqReg, 0x04);             // Clear the CRCIRq interrupt request bit
    mfrc522_write_reg(dev, FIFOLevelReg, 0x80);          // FlushBuffer = 1, FIFO initialization

    // Write data to the FIFO
    mfrc522_write_reg_multi(dev, FIFODataReg, data, length);

    mfrc522_write_reg(dev, CommandReg, PCD_CalcCRC);     // Start the calculation

    // Wait for the CRC calculation to complete. Check for the register to
    // indicate that the CRC calculation is complete in a loop. If the
    // calculation is not indicated as complete in ~89ms, then time out
    // the operation.
    int64_t start_time = k_uptime_get();
    int64_t deadline = start_time + 89;  // 89ms timeout

    do {
        // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
        uint8_t n;
        mfrc522_read_reg(dev, DivIrqReg, &n);
        if (n & 0x04) {  // CRCIRq bit set - calculation done
            mfrc522_write_reg(dev, CommandReg, PCD_Idle);  // Stop calculating CRC for new content in the FIFO

            // Transfer the result from the registers to the result buffer
            mfrc522_read_reg(dev, CRCResultRegL, &result[0]);
            mfrc522_read_reg(dev, CRCResultRegH, &result[1]);
            return STATUS_OK;
        }
        k_yield();  // Equivalent to Arduino's yield()
    } while (k_uptime_get() < deadline);

    // 89ms passed and nothing happened. Communication with the MFRC522 might be down.
    return STATUS_TIMEOUT;
}


bool mfrc522_picc_read_card_serial(const struct device *dev, struct Uid *uid) {
    enum StatusCode result = mfrc522_picc_select(dev, uid, 0);
    return (result == STATUS_OK);
}

enum StatusCode mfrc522_picc_halt_a(const struct device *dev) {
    enum StatusCode result;
    uint8_t buffer[4];

    // Build command buffer
    buffer[0] = PICC_CMD_HLTA;
    buffer[1] = 0;

    // Calculate CRC_A
    result = mfrc522_pcd_calculate_crc(dev, buffer, 2, &buffer[2]);
    if (result != STATUS_OK) {
        return result;
    }

    // Send the command.
    // The standard says:
    //    If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
    //    HLTA command, this response shall be interpreted as 'not acknowledge'.
    // We interpret that way: Only STATUS_TIMEOUT is a success.
    uint8_t back_len = 0; // We don't expect any response
    result = mfrc522_pcd_transceive_data(dev, buffer, sizeof(buffer), NULL, &back_len, NULL, 0, false);

    if (result == STATUS_TIMEOUT) {
        return STATUS_OK;
    }
    if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
        return STATUS_ERROR;
    }

    return result;
}

void mfrc522_pcd_antenna_off(const struct device *dev) {
    mfrc522_pcd_clear_register_bitmask(dev, TxControlReg, 0x03);
}

int mfrc522_pcd_get_antenna_gain(const struct device *dev, uint8_t *gain) {
    uint8_t value;
    int ret = mfrc522_read_reg(dev, RFCfgReg, &value);
    if (ret < 0) {
        return ret;
    }

    *gain = value & (0x07 << 4);
    return 0;
}

int mfrc522_pcd_set_antenna_gain(const struct device *dev, uint8_t mask) {
    uint8_t current_gain;
    int ret = mfrc522_pcd_get_antenna_gain(dev, &current_gain);
    if (ret < 0) {
        return ret;
    }

    if (current_gain != mask) {  // only bother if there is a change
        // clear needed to allow 000 pattern
        ret = mfrc522_pcd_clear_register_bitmask(dev, RFCfgReg, (0x07 << 4));
        if (ret < 0) {
            return ret;
        }

        // only set RxGain[2:0] bits
        ret = mfrc522_pcd_set_register_bitmask(dev, RFCfgReg, mask & (0x07 << 4));
        if (ret < 0) {
            return ret;
        }
    }

    return 0;
}

int mfrc522_pcd_reset(const struct device *dev) {
    int ret = mfrc522_write_reg(dev, CommandReg, PCD_SoftReset);  // Issue the SoftReset command
    if (ret < 0) {
        return ret;
    }

    // The datasheet does not mention how long the SoftReset command takes to complete.
    // But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg)
    // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
    uint8_t count = 0;
    do {
        // Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
        k_sleep(K_MSEC(50));

        uint8_t command_reg;
        ret = mfrc522_read_reg(dev, CommandReg, &command_reg);
        if (ret < 0) {
            return ret;
        }

        if (!(command_reg & (1 << 4))) {  // PowerDown bit cleared
            break;
        }
    } while ((++count) < 3);

    if (count >= 3) {
        return -ETIMEDOUT;  // Device didn't come out of power-down mode
    }

    return 0;
}

int mfrc522_pcd_soft_power_down(const struct device *dev) {
    uint8_t val;
    int ret = mfrc522_read_reg(dev, CommandReg, &val);  // Read state of the command register
    if (ret < 0) {
        return ret;
    }

    val |= (1 << 4);  // set PowerDown bit (bit 4) to 1
    return mfrc522_write_reg(dev, CommandReg, val);  // write new value to the command register
}

int mfrc522_pcd_soft_power_up(const struct device *dev) {
    uint8_t val;
    int ret = mfrc522_read_reg(dev, CommandReg, &val);  // Read state of the command register
    if (ret < 0) {
        return ret;
    }

    val &= ~(1 << 4);  // set PowerDown bit (bit 4) to 0
    ret = mfrc522_write_reg(dev, CommandReg, val);  // write new value to the command register
    if (ret < 0) {
        return ret;
    }

    // wait until PowerDown bit is cleared (this indicates end of wake up procedure)
    int64_t timeout = k_uptime_get() + 500;  // create timer for timeout (just in case)

    while (k_uptime_get() <= timeout) {  // set timeout to 500 ms
        ret = mfrc522_read_reg(dev, CommandReg, &val);  // Read state of the command register
        if (ret < 0) {
            return ret;
        }

        if (!(val & (1 << 4))) {  // if powerdown bit is 0
            break;  // wake up procedure is finished
        }

        k_yield();
    }

    if (val & (1 << 4)) {  // PowerDown bit still set after timeout
        return -ETIMEDOUT;
    }

    return 0;
}

enum StatusCode mfrc522_pcd_authenticate(const struct device *dev, uint8_t command, uint8_t block_addr,
                                          const uint8_t *key, const struct Uid *uid) {
    uint8_t wait_irq = 0x10;  // IdleIRq

    // Build command buffer
    uint8_t send_data[12];
    send_data[0] = command;
    send_data[1] = block_addr;

    for (uint8_t i = 0; i < MF_KEY_SIZE; i++) {  // 6 key bytes
        send_data[2 + i] = key[i];
    }

    // Use the last uid bytes as specified in http://cache.nxp.com/documents/application_note/AN10927.pdf
    // section 3.2.5 "MIFARE Classic Authentication".
    // The only missed case is the MF1Sxxxx shortcut activation,
    // but it requires cascade tag (CT) byte, that is not part of uid.
    for (uint8_t i = 0; i < 4; i++) {  // The last 4 bytes of the UID
        send_data[8 + i] = uid->uid_byte[i + uid->size - 4];
    }

    // Start the authentication
    return mfrc522_pcd_communicate_with_picc(dev, PCD_MFAuthent, wait_irq, send_data, sizeof(send_data),
                                              NULL, NULL, NULL, 0, false);
}

void mfrc522_pcd_stop_crypto1(const struct device *dev) {
    // Clear MFCrypto1On bit
    mfrc522_pcd_clear_register_bitmask(dev, Status2Reg, 0x08);  // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved MFCrypto1On ModemState[2:0]
}

enum StatusCode mfrc522_mifare_read(const struct device *dev, uint8_t block_addr, uint8_t *buffer, uint8_t *buffer_size) {
    enum StatusCode result;

    // Sanity check
    if (buffer == NULL || *buffer_size < 18) {
        return STATUS_NO_ROOM;
    }

    // Build command buffer
    buffer[0] = PICC_CMD_MF_READ;
    buffer[1] = block_addr;
    // Calculate CRC_A
    result = mfrc522_pcd_calculate_crc(dev, buffer, 2, &buffer[2]);
    if (result != STATUS_OK) {
        return result;
    }

    // Transmit the buffer and receive the response, validate CRC_A.
    return mfrc522_pcd_transceive_data(dev, buffer, 4, buffer, buffer_size, NULL, 0, true);
}

enum StatusCode mfrc522_pcd_mifare_transceive(const struct device *dev, const uint8_t *send_data, uint8_t send_len,
                                               bool accept_timeout) {
    enum StatusCode result;
    uint8_t cmd_buffer[18];  // We need room for 16 bytes data and 2 bytes CRC_A.

    // Sanity check
    if (send_data == NULL || send_len > 16) {
        return STATUS_INVALID;
    }

    // Copy send_data[] to cmd_buffer[] and add CRC_A
    memcpy(cmd_buffer, send_data, send_len);
    result = mfrc522_pcd_calculate_crc(dev, cmd_buffer, send_len, &cmd_buffer[send_len]);
    if (result != STATUS_OK) {
        return result;
    }
    send_len += 2;

    // Transceive the data, store the reply in cmd_buffer[]
    uint8_t wait_irq = 0x30;  // RxIRq and IdleIRq
    uint8_t cmd_buffer_size = sizeof(cmd_buffer);
    uint8_t valid_bits = 0;
    result = mfrc522_pcd_communicate_with_picc(dev, PCD_Transceive, wait_irq, cmd_buffer, send_len,
                                               cmd_buffer, &cmd_buffer_size, &valid_bits, 0, false);
    if (accept_timeout && result == STATUS_TIMEOUT) {
        return STATUS_OK;
    }
    if (result != STATUS_OK) {
        return result;
    }

    // The PICC must reply with a 4 bit ACK
    if (cmd_buffer_size != 1 || valid_bits != 4) {
        return STATUS_ERROR;
    }
    if (cmd_buffer[0] != MF_ACK) {
        return STATUS_MIFARE_NACK;
    }

    return STATUS_OK;
}

enum StatusCode mfrc522_mifare_write(const struct device *dev, uint8_t block_addr, const uint8_t *buffer, uint8_t buffer_size) {
    enum StatusCode result;

    // Sanity check
    if (buffer == NULL || buffer_size < 16) {
        return STATUS_INVALID;
    }

    // Mifare Classic protocol requires two communications to perform a write.
    // Step 1: Tell the PICC we want to write to block block_addr.
    uint8_t cmd_buffer[2];
    cmd_buffer[0] = PICC_CMD_MF_WRITE;
    cmd_buffer[1] = block_addr;
    result = mfrc522_pcd_mifare_transceive(dev, cmd_buffer, 2, false);  // Adds CRC_A and checks that the response is MF_ACK.
    if (result != STATUS_OK) {
        return result;
    }

    // Step 2: Transfer the data
    result = mfrc522_pcd_mifare_transceive(dev, buffer, 16, false);  // Adds CRC_A and checks that the response is MF_ACK.
    if (result != STATUS_OK) {
        return result;
    }

    return STATUS_OK;
}

/* Define driver API (can be expanded as needed) */
#define MFRC522_DEVICE_INIT(inst)						\
	static struct mfrc522_data mfrc522_data_##inst;			\
										\
	static const struct mfrc522_config mfrc522_config_##inst = {		\
		.bus = SPI_DT_SPEC_INST_GET(					\
			inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB |		\
			SPI_OP_MODE_MASTER, 0),				\
		.reset_gpio = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),		\
	};									\
										\
	DEVICE_DT_INST_DEFINE(inst,						\
			    mfrc522_init,					\
			    NULL,						\
			    &mfrc522_data_##inst,				\
			    &mfrc522_config_##inst,				\
			    POST_KERNEL,					\
			    CONFIG_MFRC522_INIT_PRIORITY,			\
			    NULL);

DT_INST_FOREACH_STATUS_OKAY(MFRC522_DEVICE_INIT)
