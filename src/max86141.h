#ifndef MAX86141_H
#define MAX86141_H

#include "Arduino.h"
#include <stdint.h>
#include <SPI.h>

//Register Mapping (Datasheet Page: 55)
//Status
#define REG_INT_STAT_1         (0x00)      //Interrupt Status 1
#define REG_INT_STAT_2         (0x01)      //Interrupt Status 2
#define REG_INT_EN_1           (0x02)      //Interrupt Enable 1
#define REG_INT_EN_2           (0x03)      //Interrupt Enable 2

//FIFO
#define REG_FIFO_WR_PTR        (0x04)      //FIFO Buffer Write Pointer
#define REG_FIFO_RD_PTR        (0x05)      //FIFO Buffer Read Pointer
#define REG_OVF_COUNTER        (0x06)      //Over Flow Counter
#define REG_FIFO_DATA_COUNT    (0x07)      //FIFO Data Counter
#define REG_FIFO_DATA          (0x08)      //FIFO Data Register
#define REG_FIFO_CONFIG_1      (0x09)      //FIFO Configuration 1
#define REG_FIFO_CONFIG_2      (0x0A)      //FIFO Configuration 2

//System Control
#define REG_MODE_CONFIG        (0x0D)      //System Control

//PPG Configuration
#define REG_PPG_SYNC_CTRL      (0x10)      //PPG Sync Control
#define REG_PPG_CONFIG_1       (0x11)      //PPG Configuration Settings Group 1
#define REG_PPG_CONFIG_2       (0x12)      //PPG Configuration Settings Group 2
#define REG_PPG_CONFIG_3       (0x13)      //PPG Configuration Settings Group 3
#define REG_PROX_INTR_THRESH   (0x14)      //Prox Interrupt Threshold
#define REG_PD_BIAS            (0x15)      //Photo Diode Bias

//PPG Picket Fence Detect and Replace
#define REG_PICKET_FENCE       (0x16)      //Picket Fence Settings

//LED Sequence Control
#define REG_LED_SEQ_1          (0x20)      //LED Sequence 1
#define REG_LED_SEQ_2          (0x21)      //LED Sequence 2
#define REG_LED_SEQ_3          (0x22)      //LED Sequence 3

//LED Pulse Amplitude
#define REG_LED1_PA            (0x23)      //LED 1 Pulse Amplitude
#define REG_LED2_PA            (0x24)      //LED 2 Pulse Amplitude
#define REG_LED3_PA            (0x25)      //LED 3 Pulse Amplitude
#define REG_LED4_PA            (0x26)      //LED 4 Pulse Amplitude
#define REG_LED5_PA            (0x27)      //LED 5 Pulse Amplitude
#define REG_LED6_PA            (0x28)      //LED 6 Pulse Amplitude
#define REG_LED_PILOT_PA       (0x29)      //LED Pilot Pulse Amplitude
#define REG_LED_RANGE_1        (0x2A)      //LED Amplitude Range 1
#define REG_LED_RANGE_2        (0x2B)      //LED Amplitude Range 2
 
//PPG1_HI_RES_DAC (DAC settings for first LED)
#define REG_S1_HI_RES_DAC1     (0x2C)
#define REG_S2_HI_RES_DAC1     (0x2D)
#define REG_S3_HI_RES_DAC1     (0x2E)
#define REG_S4_HI_RES_DAC1     (0x2F)
#define REG_S5_HI_RES_DAC1     (0x30)
#define REG_S6_HI_RES_DAC1     (0x31)

//PPG2_HI_RES_DAC (DAC settings for second LED)
#define REG_S1_HI_RES_DAC2     (0x32)
#define REG_S2_HI_RES_DAC2     (0x33)
#define REG_S3_HI_RES_DAC2     (0x34)
#define REG_S4_HI_RES_DAC2     (0x35)
#define REG_S5_HI_RES_DAC2     (0x36)
#define REG_S6_HI_RES_DAC2     (0x37)

//DIE Temperature
#define REG_TEMP_CONFIG        (0x40)
#define REG_TEMP_INTR          (0x41)
#define REG_TEMP_FRAC          (0x42)

//SHA256
#define REG_SHA_CMD            (0xF0)
#define REG_SHA_CONFIG         (0xF1)

//Memory
#define REG_MEM_CTRL           (0xF2)
#define REG_MEM_IDX            (0xF3)
#define REG_MEM_DATA           (0xF4)

//Part ID
#define REG_PART_ID            (0xFF)

#define WRITE_EN               (0x00)
#define READ_EN                (0xFF)

class MAX86141{

  public:

    SPIClass * spi = NULL;
    int SS;
    //8MHz clock on MAX86141 Max, only 200KHz necessary.
    int spiClk = 1000000; 
    bool debug = false;
    
    int led1A[128];
    int led1B[128];
    int led2A[128];
    int led2B[128];
    
    uint8_t       m_tx_buf[3];                       /**< TX buffer. */
    uint8_t       m_rx_buf[3];                       /**< RX buffer. */
    const uint8_t m_length = sizeof(m_tx_buf);       /**< Transfer length. */

    //Functions
    void init(int setSpiClk);
    void write_reg(uint8_t address, uint8_t data_in);
    uint8_t read_reg(uint8_t address);
    void fifo_intr();
    void read_fifo(uint8_t data_buffer[], uint8_t count);
    void device_data_read(void);
    void setSS(int pin);
    void setSPI(SPIClass * newspi);
    void setSpiClk(int newSpiClk);
    void setDebug(bool setdebug);
    void clearInt();
};


#endif