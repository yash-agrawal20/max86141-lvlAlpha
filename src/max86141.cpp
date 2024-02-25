#include "MAX86141.h"

//Write to a register function
void MAX86141::write_reg(uint8_t address, uint8_t data_in) {

    //Buffer with data to transfer.
    m_tx_buf[0] = address;  //Target Register
    m_tx_buf[1] = WRITE_EN; //Set Write mode
    m_tx_buf[2] = data_in;  //Byte to Write

    if(debug == true) {
        /* Serial.print("W_TX ");
        Serial.print(m_tx_buf[0], HEX);
        Serial.print("|");
        Serial.print(m_tx_buf[1], HEX);
        Serial.print("|");
        Serial.println(m_tx_buf[2], HEX);*/
    }

    //Buffer for incoming data.
    m_rx_buf[0] = 0;
    m_rx_buf[1] = 0;
    m_rx_buf[2] = 0;

    digitalWrite(SS, HIGH);
    spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
    digitalWrite(SS, LOW);

    m_rx_buf[0] = spi->transfer(address);
    m_rx_buf[1] = spi->transfer(WRITE_EN);
    m_rx_buf[2] = spi->transfer(data_in);

    digitalWrite(SS, HIGH);
    spi->endTransaction();
    digitalWrite(SS, LOW);

    if(debug == true) {
        /*Serial.print("W_RX ");
        Serial.print(m_rx_buf[0], HEX);
        Serial.print("|");
        Serial.print(m_rx_buf[1], HEX);
        Serial.print("|");
        Serial.println(m_rx_buf[2], HEX);*/
    }
}


/*read register function*/
uint8_t MAX86141::read_reg(uint8_t address) {

    //Buffer with data to transfer.
    m_tx_buf[0] = address;  //Target Address
    m_tx_buf[1] = READ_EN;  //Set Read mode
    m_tx_buf[2] = 0x00;     

    //Buffer for incoming data.
    m_rx_buf[0] = 0;
    m_rx_buf[1] = 0;
    m_rx_buf[2] = 0;

    if(debug == true) {
    //     Serial.print("R_TX ");
    //     Serial.print(m_tx_buf[0], HEX);
    //     Serial.print("|");
    //     Serial.print(m_tx_buf[1], HEX);
    //     Serial.print("|");
    //     Serial.println(m_tx_buf[2], HEX);
    }

    digitalWrite(SS, HIGH);
    spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
    digitalWrite(SS, LOW);
    spi->transfer(m_tx_buf,3);
    digitalWrite(SS, HIGH);
    spi->endTransaction();
    digitalWrite(SS, LOW);

    m_rx_buf[0] = m_tx_buf[0];
    m_rx_buf[1] = m_tx_buf[1];
    m_rx_buf[2] = m_tx_buf[2];

    if(debug == true){
        /* Serial.print("R_RX ");
        Serial.print(m_rx_buf[0], HEX);
        Serial.print("|");
        Serial.print(m_rx_buf[1], HEX);
        Serial.print("|");
        Serial.println(m_rx_buf[2], HEX);*/
    }

    return m_rx_buf[2];
}


//Using the pseudo-code available on MAX86141 datasheet for initialisation (Page 21)
void MAX86141::init(int newSpiClk=1000000)
{
    setSpiClk(newSpiClk);
    uint8_t temp;

	// Soft-Reset
	write_reg(REG_MODE_CONFIG, 0x01);
	delay(1);
	// Clear interrupts
	read_reg(REG_INT_STAT_1);
	read_reg(REG_INT_STAT_2);
    //Shutdown
    write_reg(REG_MODE_CONFIG, 0x02);
    //Pulse Width = 123.8ms
    write_reg(REG_PPG_CONFIG_1, 0x03);
    //ADC Range - 16micro ampere
    write_reg(REG_FIFO_CONFIG_1, 0x28);
    //Sample Averaging - 1, and Sample Rate - 25sps
    write_reg(REG_PPG_CONFIG_2, 0x00)
    //LED Settling Time
    write_reg(REG_PPG_CONFIG_3, 0xC0)
    //PD1 Biasing
    write_reg(REG_PD_BIAS, 0x11)

	// LED 1,2 Range = 124mA
	write_reg(REG_LED_RANGE_1, 0b00001111);
    // LED 1 current - 15.36mA
	write_reg(REG_LED1_PA, 0x02); 
    // LED 2 current - 15.36mA
	write_reg(REG_LED2_PA, 0x02);
    //Low Power Mode
    write_reg(REG_MODE_CONFIG, 0x04)

    //FIFO Configurations
    write_reg(REG_FIFO_CONFIG_1, 0x01);
    write_reg(REG_FIFO_CONFIG_2, 0x02);
    write_reg(REG_LED_SEQ_1, 0x11);
    write_reg(REG_LED_SEQ_2, 0x00);
    write_reg(REG_LED_SEQ_3, 0x00);

    //Shutdown
    write_reg(REG_MODE_CONFIG, 0x00);
}


//Read FIFO
void MAX86141::read_fifo(uint8_t data_buffer[], int count)
{
    data_buffer[0] = REG_FIFO_DATA;
    data_buffer[1] = READ_EN;
    digitalWrite(SS, HIGH);
    spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
    digitalWrite(SS, LOW);
    spi->transfer(data_buffer, 2);
    spi->transfer(data_buffer, count * 3);
    digitalWrite(SS, HIGH);
    spi->endTransaction();
    digitalWrite(SS, LOW);

    if(debug == true){
        // Serial.println("Data buffer");
        // Serial.println(data_buffer[0]);
    }
}


void MAX86141::fifo_intr()
{
    uint8_t count;
    count = read_reg(REG_FIFO_DATA_COUNT); 

    //indicates full FIFO
    if (count == 0x80) 
    { 
        device_data_read();
    }
 }

//Using the pseudo code available in the datasheet (Page 21)
void MAX86141::clearInt() {
  uint16_t intr = 0x00;
  intr = read_reg(REG_INT_STAT_1) << 8;
  intr |= read_reg(REG_INT_STAT_2);
}


void MAX86141::setSS(int pin){
    SS = pin;
}

void MAX86141::setSPI(SPIClass * newspi){
    spi = newspi;
}

void MAX86141::setSpiClk(int newSpiClk) {
    spiClk = newSpiClk;
}

void MAX86141::setDebug(bool setdebug) {
    debug = setdebug;
}


//Using the pseudo-code available in MAX86141 datasheet (Page 22)
void MAX86141::device_data_read()
{
    int sample_count;
    uint8_t reg_val;
    sample_count =  read_reg(REG_FIFO_DATA_COUNT);
    //Serial.println("sample_count :"+String(read_reg(REG_FIFO_DATA_COUNT)));

    //Dynamic Allocation
    uint8_t *dataBuf = (uint8_t*)malloc(sample_count*3*sizeof(uint8_t));
    memset(dataBuf, 0, sample_count*3*sizeof(uint8_t));   // Flush buffer

    read_fifo(dataBuf,sample_count);

    //Single photodiode chnnels and LED channels
    //Reading of 1 sample for Serial Test
    tagSeq1A_PD1 = (dataBuf[0] >> 3) & 0x1f;
    ledSeq1A_PD1 = ((dataBuf[0] << 16) | (dataBuf[1] << 8) | (dataBuf[2])) & 0x7ffff;

    tagSeq1B_PD1 = (dataBuf[3] >> 3) & 0x1f;
    ledSeq1B_PD1 = ((dataBuf[3] << 16) | (dataBuf[4] << 8) | (dataBuf[5])) & 0x7ffff;

    tagSeq2A_PD1 = (dataBuf[6] >> 3) & 0x1f;
    ledSeq2A_PD1 = ((dataBuf[6] << 16) | (dataBuf[7] << 8) | (dataBuf[8])) & 0x7ffff;

    tagSeq2B_PD1 = (dataBuf[9] >> 3) & 0x1f;
    ledSeq2B_PD1 = ((dataBuf[9] << 16) | (dataBuf[10] << 8) | (dataBuf[11])) & 0x7ffff;

    tagSeq3A_PD1 = (dataBuf[12] >> 3) & 0x1f;
    ledSeq3A_PD1 = ((dataBuf[12] << 16) | (dataBuf[13] << 8) | (dataBuf[14])) & 0x7ffff;

    tagSeq3B_PD1 = (dataBuf[15] >> 3) & 0x1f;
    ledSeq3B_PD1 = ((dataBuf[15] << 16) | (dataBuf[16] << 8) | (dataBuf[17])) & 0x7ffff;

    free(dataBuf);
    free(tagSeq1A_PD1);
    free(ledSeq1A_PD1);
    free(tagSeq1B_PD1);
    free(ledSeq1B_PD1);
    free(tagSeq2A_PD1);
    free(ledSeq2A_PD1);
    free(tagSeq2B_PD1);
    free(ledSeq2B_PD1);
    free(tagSeq3A_PD1);
    free(ledSeq3A_PD1);
    free(tagSeq3B_PD1);
    free(ledSeq3B_PD1);

    clearInt();
}