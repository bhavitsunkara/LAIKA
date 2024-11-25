#ifndef LORA_H
#define LORA_H

#ifdef __cplusplus
extern C {
#endif

#include "main.h"

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// IRQ Flags
#define MODE_TX_DONE							0x08
#define MODE_RX_DONE							0x40
#define MODE_CRC_ERROR						0x20
#define MODE_RX_TIMEOUT           0x80

#define RF_MID_BAND_THRESHOLD    	525E6
#define RSSI_OFFSET_HF_PORT      	157
#define RSSI_OFFSET_LF_PORT      	164

// PA Config 
#define PA_BOOST									0x80

#define MAX_PKT_LENGTH           255

// Common frequencies (in Hz)
#define FREQ_433_MHZ               433000000
#define FREQ_868_MHZ               868000000
#define FREQ_915_MHZ               915000000

// Pin definitions for SPI1
#define LORA_NSS_PIN               0x00000004  // PA4
#define LORA_SCK_PIN               0x00000005  // PA5
#define LORA_MISO_PIN              0x00000006  // PA6
#define LORA_MOSI_PIN              0x00000007  // PA7
#define LORA_RESET_PIN             0x00000003  // PA3

// Function prototypes
// public
void spi_init(void);

void lora_reset(void);

void lora_write_reg(uint8_t reg, uint8_t value);
uint8_t lora_read_reg(uint8_t reg);

uint8_t begin(uint32_t frequency);
void end();

uint8_t beginPacket(uint8_t implicitHeader); 	// implcitHeader = 0
uint8_t endPacket();							

uint8_t parsePacket(uint8_t size);
uint16_t packetRssi();
float packetSnt();
long packetFrequencyError();

int rssi();

// from print in cpp
uint8_t write(const uint8_t *buffer, uint8_t size);
 
// from stream in cpp
int available();
int read();
int peek();

void idle();
void sleep();

void setTxPower(uint8_t level);
void setFrequency(uint32_t frequency);
void setSpreadingFactor(uint8_t sf);
long getSignalBandwidth();
void setSignalBandwidth(uint32_t sbw);
void setCodingRate4(uint8_t denominator);
void setPreambleLength(long length);
void setSyncWord(int sw);
void enableCrc();
void disableCrc();
void enableInvertIQ();
void disableInvertIQ();

void setOCP(uint8_t mA); 		// Over current protection
void setGain(uint8_t gain);

uint8_t random();

//private
void explicitHeaderMode();
void implicitHeaderMode();

//void handleDio0Rise();
bool isTransmitting();

uint8_t getSpreadingFactor();
long getSignalBandwidth();

void setLdoFlag();

#ifdef __cplusplus
}
#endif

#endif /* LORA_H */