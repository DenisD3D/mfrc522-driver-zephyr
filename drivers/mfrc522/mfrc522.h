#ifndef DRIVERS_RFID_MFRC522_H_
#define DRIVERS_RFID_MFRC522_H_

#include <zephyr/types.h>
#include <zephyr/device.h>

#include "mfrc522_reg.h"

/**
 * @brief Read a register from MFRC522
 *
 * @param dev Pointer to the device structure
 * @param reg Register address to read
 * @param data Pointer to store the read value
 * @return 0 if successful, negative errno code on failure
 */
int mfrc522_read_reg(const struct device *dev, enum PCD_Register reg, uint8_t *data);

/**
 * @brief Read a register from MFRC522
 *
 * @param dev Pointer to the device structure
 * @param reg Register address to read
 * @param data Pointer to store the read value
 * @param len Length of data to read
 * @return 0 if successful, negative errno code on failure
 */
int mfrc522_read_reg_multi(const struct device *dev, const enum PCD_Register reg, uint8_t *data, const size_t len);

/**
 * @brief Write a register to MFRC522
 *
 * @param dev Pointer to the device structure
 * @param reg Register address to write
 * @param data Value to write
 * @return 0 if successful, negative errno code on failure
 */
int mfrc522_write_reg(const struct device *dev, const enum PCD_Register reg, const uint8_t data);

/**
 * @brief Write a register to MFRC522
 *
 * @param dev Pointer to the device structure
 * @param reg Register address to write
 * @param data Value to write
 * @param len Length of data
 * @return 0 if successful, negative errno code on failure
 */
int mfrc522_write_reg_multi(const struct device *dev, const enum PCD_Register reg, const uint8_t *data, const size_t len);

/**
 * @brief Reset the MFRC522 hardware
 *
 * @param dev Pointer to the device structure
 */
void mfrc522_hw_reset(const struct device *dev);

/**
 * @brief Get the MFRC522 version.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param version Pointer to store the version value.
 * @return 0 if successful, negative errno code on failure.
 */
int mfrc522_get_version(const struct device *dev, uint8_t *version);


/**
 * @brief Turns the antenna on by enabling pins TX1 and TX2. After a reset these pins are disabled.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @return 0 if successful, negative errno code on failure.
 */
int mfrc522_pcd_antenna_on(const struct device *dev);

/**
 * @brief Check if a new card is present
 *
 * @param dev Pointer to the device structure
 * @return true if a new card is present, false otherwise
 */
bool mfrc522_picc_is_new_card_present(const struct device *dev);

/**
 * @brief Request card presence (ISO/IEC 14443, Type A)
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY
 * and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time it may often return
 * STATUS_TIMEOUT - probably due to bad antenna design.
 *
 * @param dev Pointer to the device structure
 * @param buffer_atqa Buffer to store the ATQA (Answer To Request) data
 * @param buffer_size Size of the buffer (in: size of buffer, out: received bytes)
 * @return StatusCode indicating success or type of error
 */
enum StatusCode mfrc522_picc_request_a(const struct device *dev, uint8_t *buffer_atqa, uint8_t *buffer_size);

enum StatusCode mfrc522_picc_wakeup_a(const struct device *dev, uint8_t *buffer_atqa, uint8_t *buffer_size);

/**
 * @brief Internal helper function for REQA and WUPA commands
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time it may often return
 * STATUS_TIMEOUT - probably due to bad antenna design.
 *
 * @param dev Pointer to the device structure
 * @param command The command to use (PICC_CMD_REQA or PICC_CMD_WUPA)
 * @param buffer_atqa Buffer to store the ATQA (Answer To Request) data
 * @param buffer_size Size of the buffer (in: size of buffer, out: received bytes)
 * @return StatusCode indicating success or type of error
 */
enum StatusCode mfrc522_picc_reqa_or_wupa(const struct device *dev, uint8_t command, uint8_t *buffer_atqa,
                                                 uint8_t *buffer_size);

/**
 * @brief Clear a bitmask in a register
 *
 * @param dev Pointer to the device structure
 * @param reg Register address
 * @param mask Bitmask to clear
 * @return 0 if successful, negative errno code on failure
 */
int mfrc522_pcd_clear_register_bitmask(const struct device *dev, enum PCD_Register reg, uint8_t mask);

/**
 * @brief Transceive data to/from a PICC
 *
 * @param dev Pointer to the device structure
 * @param send_data Data to send
 * @param send_len Length of data to send
 * @param back_data Buffer to store received data
 * @param back_len in: max length of buffer, out: actual bytes received
 * @param valid_bits in/out: Valid bits in last byte of send_data/back_data
 * @param rx_align Number of bits shift in the first byte received
 * @param check_crc True = verify CRC
 * @return StatusCode indicating success or type of error
 */
enum StatusCode mfrc522_pcd_transceive_data(const struct device *dev, const uint8_t *send_data, uint8_t send_len,
                                                   uint8_t *back_data, uint8_t *back_len, uint8_t *valid_bits,
                                                   uint8_t rx_align, bool check_crc);

/**
 * @brief Communicate with a PICC
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if back_data and back_len are specified.
 *
 * @param dev Pointer to the device structure
 * @param command The command to execute. One of the PCD_Command enums.
 * @param wait_irq The bits in the ComIrqReg register that signals successful completion of the command.
 * @param send_data Pointer to the data to transfer to the FIFO.
 * @param send_len Number of bytes to transfer to the FIFO.
 * @param back_data NULL or pointer to buffer if data should be read back after executing the command.
 * @param back_len In: Max number of bytes to write to back_data. Out: The number of bytes returned.
 * @param valid_bits In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
 * @param rx_align In: Defines the bit position in back_data[0] for the first bit received.
 * @param check_crc In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
 * @return StatusCode indicating success or type of error
 */
enum StatusCode mfrc522_pcd_communicate_with_picc(const struct device *dev, uint8_t command, uint8_t wait_irq,
                                                         const uint8_t *send_data, uint8_t send_len, uint8_t *back_data,
                                                         uint8_t *back_len, uint8_t *valid_bits, uint8_t rx_align,
                                                         bool check_crc);
/**
 * @brief Set a bitmask in a register
 *
 * @param dev Pointer to the device structure
 * @param reg Register address
 * @param mask Bitmask to set
 */
void mfrc522_pcd_set_register_bitmask(const struct device *dev, enum PCD_Register reg, uint8_t mask);

/**
 * @brief Transmits SELECT/ANTICOLLISION commands to select a single PICC
 *
 * Before calling this function the PICCs must be placed in the READY(*) state by calling
 * mfrc522_picc_request_a() or mfrc522_picc_wakeup_a().
 * On success:
 *   - The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT.
 *   - The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 *
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 *   UID size    Number of UID bytes    Cascade levels    Example of PICC
 *   ========    ===================    ==============    ===============
 *   single              4                    1           MIFARE Classic
 *   double              7                    2           MIFARE Ultralight
 *   triple             10                    3           Not currently in use?
 *
 * @param dev Pointer to the device structure
 * @param uid Pointer to Uid struct. Normally output, but can also be used to supply a known UID
 * @param valid_bits The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size
 * @return StatusCode indicating success or type of error
 */
enum StatusCode mfrc522_picc_select(const struct device *dev, struct Uid *uid, uint8_t valid_bits);

/**
 * @brief Calculate the CRC_A for data
 *
 * @param dev Pointer to the device structure
 * @param data Pointer to the data to transfer to the FIFO for CRC calculation
 * @param length The number of bytes to transfer
 * @param result Pointer to result buffer. Result is written to result[0..1], low byte first
 * @return StatusCode indicating success or type of error
 */
enum StatusCode mfrc522_pcd_calculate_crc(const struct device *dev,
                                               const uint8_t *data, uint8_t length,
                                               uint8_t *result);

/**
 * @brief Simple wrapper around mfrc522_picc_select
 *
 * Returns true if a UID could be read.
 * Remember to call mfrc522_picc_is_new_card_present(), mfrc522_picc_request_a() or
 * mfrc522_picc_wakeup_a() first.
 * The read UID is available in the uid struct passed by reference.
 *
 * @param dev Pointer to the device structure
 * @param uid Pointer to Uid struct where the read UID will be stored
 * @return true if successful, false otherwise
 */
bool mfrc522_picc_read_card_serial(const struct device *dev, struct Uid *uid);

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @param dev Pointer to the device structure
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode mfrc522_picc_halt_a(const struct device *dev);

#endif /* DRIVERS_RFID_MFRC522_H_ */
