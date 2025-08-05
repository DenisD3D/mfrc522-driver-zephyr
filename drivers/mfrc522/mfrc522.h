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
 * @return 0 if successful, negative errno code on failure
 */
int mfrc522_pcd_set_register_bitmask(const struct device *dev, enum PCD_Register reg, uint8_t mask);

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

/**
 * @brief Turns the antenna off by disabling pins TX1 and TX2.
 *
 * @param dev Pointer to the device structure for the driver instance.
 */
void mfrc522_pcd_antenna_off(const struct device *dev);

/**
 * @brief Get the current MFRC522 Receiver Gain (RxGain[2:0]) value.
 *
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Return value scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param gain Pointer to store the RxGain value, scrubbed to the 3 bits used.
 * @return 0 if successful, negative errno code on failure.
 */
int mfrc522_pcd_get_antenna_gain(const struct device *dev, uint8_t *gain);

/**
 * @brief Set the MFRC522 Receiver Gain (RxGain) to value specified by given mask.
 *
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Given mask is scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param mask The gain mask to set, scrubbed to the 3 bits used.
 * @return 0 if successful, negative errno code on failure.
 */
int mfrc522_pcd_set_antenna_gain(const struct device *dev, uint8_t mask);

/**
 * @brief Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @return 0 if successful, negative errno code on failure.
 */
int mfrc522_pcd_reset(const struct device *dev);

/**
 * @brief Enter soft power down mode by setting PowerDown bit in CommandReg.
 *
 * NOTE: Only soft power down mode is available through software.
 * Calling any other function that uses CommandReg will disable soft power down mode.
 * For more details about power control, refer to the datasheet - page 33 (8.6).
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @return 0 if successful, negative errno code on failure.
 */
int mfrc522_pcd_soft_power_down(const struct device *dev);

/**
 * @brief Exit soft power down mode by clearing PowerDown bit in CommandReg.
 *
 * Waits until PowerDown bit is cleared, indicating the end of wake up procedure.
 * For more details about power control, refer to the datasheet - page 33 (8.6).
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @return 0 if successful, negative errno code on failure.
 */
int mfrc522_pcd_soft_power_up(const struct device *dev);

/**
 * @brief Executes the MFRC522 MFAuthent command.
 *
 * This command manages MIFARE authentication to enable a secure communication to any MIFARE Mini, MIFARE 1K and MIFARE 4K card.
 * The authentication is described in the MFRC522 datasheet section 10.3.1.9 and http://www.nxp.com/documents/data_sheet/MF1S503x.pdf section 10.1.
 * For use with MIFARE Classic PICCs.
 * The PICC must be selected - ie in state ACTIVE(*) - before calling this function.
 * Remember to call mfrc522_pcd_stop_crypto1() after communicating with the authenticated PICC - otherwise no new communications can start.
 *
 * All keys are set to FFFFFFFFFFFFh at chip delivery.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param command PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B
 * @param block_addr The block number
 * @param key Pointer to the Crypto1 key to use (6 bytes)
 * @param uid Pointer to Uid struct. The first 4 bytes of the UID is used.
 * @return STATUS_OK on success, STATUS_??? otherwise. Probably STATUS_TIMEOUT if you supply the wrong key.
 */
enum StatusCode mfrc522_pcd_authenticate(const struct device *dev, uint8_t command, uint8_t block_addr,
                                          const uint8_t *key, const struct Uid *uid);

/**
 * @brief Used to exit the PCD from its authenticated state.
 *
 * Remember to call this function after communicating with an authenticated PICC - otherwise no new communications can start.
 *
 * @param dev Pointer to the device structure for the driver instance.
 */
void mfrc522_pcd_stop_crypto1(const struct device *dev);

/**
 * @brief Reads 16 bytes (+ 2 bytes CRC_A) from the active PICC.
 *
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 *
 * For MIFARE Ultralight only addresses 00h to 0Fh are decoded.
 * The MF0ICU1 returns a NAK for higher addresses.
 * The MF0ICU1 responds to the READ command by sending 16 bytes starting from the page address defined by the command argument.
 * For example; if blockAddr is 03h then pages 03h, 04h, 05h, 06h are returned.
 * A roll-back is implemented: If blockAddr is 0Eh, then the contents of pages 0Eh, 0Fh, 00h and 01h are returned.
 *
 * The buffer must be at least 18 bytes because a CRC_A is also returned.
 * Checks the CRC_A before returning STATUS_OK.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param block_addr MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
 * @param buffer The buffer to store the data in
 * @param buffer_size Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK.
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode mfrc522_mifare_read(const struct device *dev, uint8_t block_addr, uint8_t *buffer, uint8_t *buffer_size);

/**
 * @brief Wrapper for MIFARE protocol communication.
 *
 * Adds CRC_A, executes the Transceive command and checks that the response is MF_ACK or a timeout.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param send_data Pointer to the data to transfer to the FIFO. Do NOT include the CRC_A.
 * @param send_len Number of bytes in send_data.
 * @param accept_timeout True => A timeout is also success
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode mfrc522_pcd_mifare_transceive(const struct device *dev, const uint8_t *send_data, uint8_t send_len,
                                               bool accept_timeout);

/**
 * @brief Writes 16 bytes to the active PICC.
 *
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 *
 * For MIFARE Ultralight the operation is called "COMPATIBILITY WRITE".
 * Even though 16 bytes are transferred to the Ultralight PICC, only the least significant 4 bytes (bytes 0 to 3)
 * are written to the specified address. It is recommended to set the remaining bytes 04h to 0Fh to all logic 0.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param block_addr MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The page (2-15) to write to.
 * @param buffer The 16 bytes to write to the PICC
 * @param buffer_size Buffer size, must be at least 16 bytes. Exactly 16 bytes are written.
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode mfrc522_mifare_write(const struct device *dev, uint8_t block_addr, const uint8_t *buffer, uint8_t buffer_size);


#endif /* DRIVERS_RFID_MFRC522_H_ */
