#include "lora.h"

volatile uint32_t _frequency = 0;
volatile int _packetIndex = 0;
volatile int _implicitHeaderMode = 0;

void spi_init(void) {
    // Enable GPIOA and SPI1 clocks
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Configure GPIO pins for SPI1 (PA5: SCK, PA6: MISO, PA7: MOSI)
    GPIOA->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7);  // Clear PA5-PA7
    GPIOA->MODER |= GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1; // Set to Alternate Function
    GPIOA->AFR[0] &= ~((0xF << GPIO_AFRL_AFSEL5_Pos) | (0xF << GPIO_AFRL_AFSEL6_Pos) | (0xF << GPIO_AFRL_AFSEL7_Pos));
    GPIOA->AFR[0] |= (0x0 << GPIO_AFRL_AFSEL5_Pos) | (0x0 << GPIO_AFRL_AFSEL6_Pos) | (0x0 << GPIO_AFRL_AFSEL7_Pos); // AF0 for SPI1
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEED5 | GPIO_OSPEEDER_OSPEED6 | GPIO_OSPEEDER_OSPEED7; // High speed
    GPIOA->OTYPER &= ~(LORA_MISO_PIN | LORA_MOSI_PIN | LORA_SCK_PIN); // Push-pull
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7); // No pull-up, no pull-down

    // Optional: Configure NSS (PA4) as manual GPIO output if used
    GPIOA->MODER &= ~GPIO_MODER_MODE4;
    GPIOA->MODER |= GPIO_MODER_MODE4_0;  // Output mode
    GPIOA->OTYPER &= ~LORA_NSS_PIN;   // Push-pull

    // Optional: Configure RESET (PA3) as manual GPIO output if used
    GPIOA->MODER &= ~GPIO_MODER_MODE3;
    GPIOA->MODER |= GPIO_MODER_MODE3_0;  // Output mode
    GPIOA->OTYPER &= ~LORA_RESET_PIN;   // Push-pull

    // Reset SPI1 to ensure proper initialization
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;

    // Configure SPI1
    SPI1->CR1 = 0;
    SPI1->CR1 |= SPI_CR1_BR_2;           // fPCLK/32 baud rate
    SPI1->CR1 |= SPI_CR1_MSTR;           // Master mode
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; // Software slave management
    SPI1->CR1 |= SPI_CR1_SPE;            // Enable SPI
}

// Reset RFM95 module
void lora_reset(void) {
    GPIOA->BSRR = (1 << LORA_RESET_PIN);  // Set high
    for(volatile int i = 0; i < 10000; i++);  // Delay
    GPIOA->BRR = (1 << LORA_RESET_PIN);   // Set low
    for(volatile int i = 0; i < 10000; i++);  // Delay
    GPIOA->BSRR = (1 << LORA_RESET_PIN);  // Set high
    for(volatile int i = 0; i < 10000; i++);  // Delay
}

// Write to RFM95 register
void lora_write_reg(uint8_t reg, uint8_t value) {
    GPIOA->BRR = (1 << LORA_NSS_PIN);  // NSS low
    
    while(!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = reg | 0x80;  // Send address with write bit
    while(!(SPI1->SR & SPI_SR_RXNE));
    (void)SPI1->DR;
    
    while(!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = value;
    while(!(SPI1->SR & SPI_SR_RXNE));
    (void)SPI1->DR;
    GPIOA->BSRR = (1 << LORA_NSS_PIN);  // NSS high
}

// Read from RFM95 register
uint8_t lora_read_reg(uint8_t reg) {
    GPIOA->BRR = (1 << LORA_NSS_PIN);  // NSS low
    
    while(!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = reg & 0x7F;  // Send address with read bit
    while(!(SPI1->SR & SPI_SR_RXNE));
    (void)SPI1->DR;
    
    while(!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = 0x00;  // Dummy byte
    while(!(SPI1->SR & SPI_SR_RXNE));
    uint8_t value = SPI1->DR;
	
    GPIOA->BSRR = (1 << LORA_NSS_PIN);  // NSS high
    return value;
}

uint8_t begin(uint32_t frequency){
	
	spi_init();
	
	_frequency = frequency;
	
	lora_reset();
	
	//	check version
	uint8_t version = lora_read_reg(REG_VERSION);
	if (version != 0x12){
		return 0;
	}
	
	sleep();
	
	setFrequency(frequency);
	
	// Set base address
	lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0);
	lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0);
	
	// Set LNA Boost
	lora_write_reg(REG_LNA, lora_read_reg(REG_LNA) | 0x03);
	
	// set auto AGC
	lora_write_reg(REG_MODEM_CONFIG_3, 0x04);
	
	// set output power to 17 dBm
	setTxPower(17);
	
	idle();
	
	return 1;
}

void end(){
	sleep();
}

uint8_t beginPacket(uint8_t implicitHeader){

	if (isTransmitting()){
		return 0;
	}
	
	// put in standby mode
	idle();
	
	if (implicitHeader){
		implicitHeaderMode();
	} else {
		explicitHeaderMode();
	}
	
	// reset FIFO address and payload length
	lora_write_reg(REG_FIFO_ADDR_PTR, 0);
	lora_write_reg(REG_PAYLOAD_LENGTH, 0);
		
	return 1;
}

uint8_t endPacket(){
	lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
		
	while ((lora_read_reg(REG_IRQ_FLAGS) & MODE_TX_DONE) == 0);
	lora_write_reg(REG_IRQ_FLAGS, MODE_TX_DONE);
	
	return 1;
}

bool isTransmitting() {

	if ((lora_read_reg(REG_OP_MODE) & MODE_TX) == MODE_TX){
		return true;
	}
	
	if(lora_read_reg(REG_IRQ_FLAGS) & MODE_TX_DONE) {
			lora_write_reg(REG_IRQ_FLAGS, MODE_TX_DONE);
	}
	
	return false;
}

uint8_t parsePacket(uint8_t size) {

	int packetLength = 0;
	uint8_t irqFlags = lora_read_reg(REG_IRQ_FLAGS);
	
	if (size > 0){
		implicitHeaderMode();
		lora_write_reg(REG_PAYLOAD_LENGTH, size & 0xff);	
	} else {
		explicitHeaderMode();
	}
	
	lora_write_reg(REG_IRQ_FLAGS, irqFlags);
	
	if ((irqFlags & MODE_RX_DONE) && (irqFlags & MODE_CRC_ERROR) == 0){
		// received a packet
		_packetIndex = 0;
		
		// read packet length
		if(_implicitHeaderMode) {
			packetLength = lora_read_reg(REG_PAYLOAD_LENGTH);
		} else {
			packetLength = lora_read_reg(REG_RX_NB_BYTES);
		}
		
		// set FIFO address  to current RX address
		lora_write_reg(REG_FIFO_ADDR_PTR, lora_read_reg(REG_FIFO_RX_CURRENT_ADDR));
		idle();
	} else if (lora_read_reg(REG_OP_MODE) !=  (MODE_LONG_RANGE_MODE|MODE_RX_SINGLE)) {
		// reset FIFO address
		lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);
		
		// put in single RX mode
		lora_write_reg(REG_OP_MODE, (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE));
	}
	
	return packetLength;
}

uint16_t packetRssi()
{
  return (lora_read_reg(REG_PKT_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

float packetSnr()
{
  return ((int8_t)lora_read_reg(REG_PKT_SNR_VALUE)) * 0.25;
}

long packetFrequencyError()
{
  int32_t freqError = 0;
  freqError = (int32_t)(lora_read_reg(REG_FREQ_ERROR_MSB) & 0b111);
  freqError <<= 8L;
  freqError += (int32_t)(lora_read_reg(REG_FREQ_ERROR_MID));
  freqError <<= 8L;
  freqError += (int32_t)(lora_read_reg(REG_FREQ_ERROR_LSB));

  if (lora_read_reg(REG_FREQ_ERROR_MSB) & 0b1000) { // Sign bit is on
     freqError -= 524288; // B1000'0000'0000'0000'0000
  }

  const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
  const float fError = (((float)(freqError) * (1L << 24)) / fXtal) * (getSignalBandwidth() / 500000.0f); // p. 37

  return (long)(fError);
}

int rssi() {
	return (lora_read_reg(REG_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

uint8_t write(const uint8_t *buffer, uint8_t size){

	uint8_t currentLength = lora_read_reg(REG_PAYLOAD_LENGTH);
	
	// check size
	if((currentLength + size) > MAX_PKT_LENGTH){
		size = MAX_PKT_LENGTH - currentLength;
	}
	
	// write data
	for(int i = 0; i < size; i++){
		lora_write_reg(REG_FIFO, buffer[i]);
	}

	// update length
	lora_write_reg(REG_PAYLOAD_LENGTH, currentLength + size);
	
	return size;
}

int available(){
	return (lora_read_reg(REG_RX_NB_BYTES) - _packetIndex);
}

int read() {

	if (!available()){
		return -1;
	}
	
	_packetIndex++;
	
	return lora_read_reg(REG_FIFO);
}

int peek() {

	if (!available())
		return -1;
	
	int currentAddr = lora_read_reg(REG_FIFO_ADDR_PTR);
	
	uint8_t b = lora_read_reg(REG_FIFO);
	
	lora_write_reg(REG_FIFO_ADDR_PTR, currentAddr);
	
	return b;
}

void sleep(){
	lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void idle(){
	lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void setTxPower(uint8_t level) {
	// PA BOOST
  if (level > 17) {
    if (level > 20) {
      level = 20;
    }

  // subtract 3 from level, so 18 - 20 maps to 15 - 17
  level -= 3;

  // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
  lora_write_reg(REG_PA_DAC, 0x87);
  setOCP(140);
  } else {
    if (level < 2) {
      level = 2;
    }
    
		//Default value PA_HF/LF or +17dBm
    lora_write_reg(REG_PA_DAC, 0x84);
    setOCP(100);
  }

	lora_write_reg(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

void setFrequency(uint32_t frequency)
{
  _frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
  lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
  lora_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

uint8_t getSpreadingFactor()
{
  return lora_read_reg(REG_MODEM_CONFIG_2) >> 4;
}

void setSpreadingFactor(uint8_t sf)
{
  if (sf < 6) {
    sf = 6;
  } else if (sf > 12) {
    sf = 12;
  }

  if (sf == 6) {
    lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc5);
    lora_write_reg(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc3);
    lora_write_reg(REG_DETECTION_THRESHOLD, 0x0a);
  }

  lora_write_reg(REG_MODEM_CONFIG_2, (lora_read_reg(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
  setLdoFlag();
}

long getSignalBandwidth()
{
  uint8_t bw = (lora_read_reg(REG_MODEM_CONFIG_1) >> 4);

  switch (bw) {
    case 0: return 7.8E3;
    case 1: return 10.4E3;
    case 2: return 15.6E3;
    case 3: return 20.8E3;
    case 4: return 31.25E3;
    case 5: return 41.7E3;
    case 6: return 62.5E3;
    case 7: return 125E3;
    case 8: return 250E3;
    case 9: return 500E3;
  }

  return -1;
}

void setSignalBandwidth(uint32_t sbw)
{
  int bw;

  if (sbw <= 7.8E3) {
    bw = 0;
  } else if (sbw <= 10.4E3) {
    bw = 1;
  } else if (sbw <= 15.6E3) {
    bw = 2;
  } else if (sbw <= 20.8E3) {
    bw = 3;
  } else if (sbw <= 31.25E3) {
    bw = 4;
  } else if (sbw <= 41.7E3) {
    bw = 5;
  } else if (sbw <= 62.5E3) {
    bw = 6;
  } else if (sbw <= 125E3) {
    bw = 7;
  } else if (sbw <= 250E3) {
    bw = 8;
  } else /*if (sbw <= 250E3)*/ {
    bw = 9;
  }

  lora_write_reg(REG_MODEM_CONFIG_1, (lora_read_reg(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
  setLdoFlag();
}

void setLdoFlag()
{
  // Section 4.1.1.5
  long symbolDuration = 1000 / ( getSignalBandwidth() / (1L << getSpreadingFactor()) ) ;

  // Section 4.1.1.6
  bool ldoOn = symbolDuration > 16;

  uint8_t config3 = lora_read_reg(REG_MODEM_CONFIG_3);
	
	if (ldoOn)
		config3  |= 0x08;
  
	lora_write_reg(REG_MODEM_CONFIG_3, config3);
}

void setCodingRate4(uint8_t denominator)
{
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }

  int cr = denominator - 4;

  lora_write_reg(REG_MODEM_CONFIG_1, (lora_read_reg(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void setPreambleLength(long length)
{
  lora_write_reg(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  lora_write_reg(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void setSyncWord(int sw)
{
  lora_write_reg(REG_SYNC_WORD, sw);
}

void enableCrc()
{
  lora_write_reg(REG_MODEM_CONFIG_2, lora_read_reg(REG_MODEM_CONFIG_2) | 0x04);
}

void disableCrc()
{
  lora_write_reg(REG_MODEM_CONFIG_2, lora_read_reg(REG_MODEM_CONFIG_2) & 0xfb);
}

void enableInvertIQ()
{
  lora_write_reg(REG_INVERTIQ,  0x66);
  lora_write_reg(REG_INVERTIQ2, 0x19);
}

void disableInvertIQ()
{
  lora_write_reg(REG_INVERTIQ,  0x27);
  lora_write_reg(REG_INVERTIQ2, 0x1d);
}

void setOCP(uint8_t mA)
{
  uint8_t ocpTrim = 27;

  if (mA <= 120) {
    ocpTrim = (mA - 45) / 5;
  } else if (mA <=240) {
    ocpTrim = (mA + 30) / 10;
  }

  lora_write_reg(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void setGain(uint8_t gain)
{
  // check allowed range
  if (gain > 6) {
    gain = 6;
  }
  
  // set to standby
  idle();
  
  // set gain
  if (gain == 0) {
    // if gain = 0, enable AGC
    lora_write_reg(REG_MODEM_CONFIG_3, 0x04);
  } else {
    // disable AGC
    lora_write_reg(REG_MODEM_CONFIG_3, 0x00);
	
    // clear Gain and set LNA boost
    lora_write_reg(REG_LNA, 0x03);
	
    // set gain
    lora_write_reg(REG_LNA, lora_read_reg(REG_LNA) | (gain << 5));
  }
}

uint8_t random()
{
  return lora_read_reg(REG_RSSI_WIDEBAND);
}

void explicitHeaderMode()
{
  _implicitHeaderMode = 0;

  lora_write_reg(REG_MODEM_CONFIG_1, lora_read_reg(REG_MODEM_CONFIG_1) & 0xfe);
}

void implicitHeaderMode()
{
  _implicitHeaderMode = 1;

  lora_write_reg(REG_MODEM_CONFIG_1, lora_read_reg(REG_MODEM_CONFIG_1) | 0x01);
}