#ifndef MFRC522_REG_H_
#define MFRC522_REG_H_

#include <stdint.h>

// Define the size of the MFRC522 FIFO
#define FIFO_SIZE 64
#define UNUSED_PIN UINT8_MAX

// Enumerate PCD registers
enum PCD_Register {
    CommandReg				= 0x01,
    ComIEnReg				= 0x02,
    DivIEnReg				= 0x03,
    ComIrqReg				= 0x04,
    DivIrqReg				= 0x05,
    ErrorReg				= 0x06,
    Status1Reg				= 0x07,
    Status2Reg				= 0x08,
    FIFODataReg				= 0x09,
    FIFOLevelReg			= 0x0A,
    WaterLevelReg			= 0x0B,
    ControlReg				= 0x0C,
    BitFramingReg			= 0x0D,
    CollReg					= 0x0E,
    ModeReg					= 0x11,
    TxModeReg				= 0x12,
    RxModeReg				= 0x13,
    TxControlReg			= 0x14,
    TxASKReg				= 0x15,
    TxSelReg				= 0x16,
    RxSelReg				= 0x17,
    RxThresholdReg			= 0x18,
    DemodReg				= 0x19,
    MfTxReg					= 0x1C,
    MfRxReg					= 0x1D,
    SerialSpeedReg			= 0x1F,
    CRCResultRegH			= 0x21,
    CRCResultRegL			= 0x22,
    ModWidthReg				= 0x24,
    RFCfgReg				= 0x26,
    GsNReg					= 0x27,
    CWGsPReg				= 0x28,
    ModGsPReg				= 0x29,
    TModeReg				= 0x2A,
    TPrescalerReg			= 0x2B,
    TReloadRegH				= 0x2C,
    TReloadRegL				= 0x2D,
    TCounterValueRegH		= 0x2E,
    TCounterValueRegL		= 0x2F,
    TestSel1Reg				= 0x31,
    TestSel2Reg				= 0x32,
    TestPinEnReg			= 0x33,
    TestPinValueReg			= 0x34,
    TestBusReg				= 0x35,
    AutoTestReg				= 0x36,
    VersionReg				= 0x37,
    AnalogTestReg			= 0x38,
    TestDAC1Reg				= 0x39,
    TestDAC2Reg				= 0x3A,
    TestADCReg				= 0x3B
};

// Enumerate PCD commands
enum PCD_Command {
    PCD_Idle				= 0x00,
    PCD_Mem					= 0x01,
    PCD_GenerateRandomID	= 0x02,
    PCD_CalcCRC				= 0x03,
    PCD_Transmit			= 0x04,
    PCD_NoCmdChange			= 0x07,
    PCD_Receive				= 0x08,
    PCD_Transceive 			= 0x0C,
    PCD_MFAuthent 			= 0x0E,
    PCD_SoftReset			= 0x0F
};

// Enumerate RxGain values
enum PCD_RxGain {
    RxGain_18dB				= 0x00 << 4,
    RxGain_23dB				= 0x01 << 4,
    RxGain_18dB_2			= 0x02 << 4,
    RxGain_23dB_2			= 0x03 << 4,
    RxGain_33dB				= 0x04 << 4,
    RxGain_38dB				= 0x05 << 4,
    RxGain_43dB				= 0x06 << 4,
    RxGain_48dB				= 0x07 << 4,
    RxGain_min				= 0x00 << 4,
    RxGain_avg				= 0x04 << 4,
    RxGain_max				= 0x07 << 4
};

// Enumerate PICC commands
enum PICC_Command {
    PICC_CMD_REQA			= 0x26,
    PICC_CMD_WUPA			= 0x52,
    PICC_CMD_CT				= 0x88,
    PICC_CMD_SEL_CL1		= 0x93,
    PICC_CMD_SEL_CL2		= 0x95,
    PICC_CMD_SEL_CL3		= 0x97,
    PICC_CMD_HLTA			= 0x50,
    PICC_CMD_RATS           = 0xE0,
    PICC_CMD_MF_AUTH_KEY_A	= 0x60,
    PICC_CMD_MF_AUTH_KEY_B	= 0x61,
    PICC_CMD_MF_READ		= 0x30,
    PICC_CMD_MF_WRITE		= 0xA0,
    PICC_CMD_MF_DECREMENT	= 0xC0,
    PICC_CMD_MF_INCREMENT	= 0xC1,
    PICC_CMD_MF_RESTORE		= 0xC2,
    PICC_CMD_MF_TRANSFER	= 0xB0,
    PICC_CMD_UL_WRITE		= 0xA2
};

// MIFARE constants
enum MIFARE_Misc {
    MF_ACK					= 0xA,
    MF_KEY_SIZE				= 6
};

// PICC types
enum PICC_Type {
    PICC_TYPE_UNKNOWN,
    PICC_TYPE_ISO_14443_4,
    PICC_TYPE_ISO_18092,
    PICC_TYPE_MIFARE_MINI,
    PICC_TYPE_MIFARE_1K,
    PICC_TYPE_MIFARE_4K,
    PICC_TYPE_MIFARE_UL,
    PICC_TYPE_MIFARE_PLUS,
    PICC_TYPE_MIFARE_DESFIRE,
    PICC_TYPE_TNP3XXX,
    PICC_TYPE_NOT_COMPLETE	= 0xff
};

// Status codes
enum StatusCode {
    STATUS_OK,
    STATUS_ERROR,
    STATUS_COLLISION,
    STATUS_TIMEOUT,
    STATUS_NO_ROOM,
    STATUS_INTERNAL_ERROR,
    STATUS_INVALID,
    STATUS_CRC_WRONG,
    STATUS_MIFARE_NACK		= 0xff
};

struct Uid {
    uint8_t size;           // Number of bytes in the UID
    uint8_t uid_byte[10];   // The UID (10 bytes max)
    uint8_t sak;            // The SAK (Select acknowledge) byte
};


#endif // MFRC522_REG_H_