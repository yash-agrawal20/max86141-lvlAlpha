#include "max86141.h"
#include "algorithms_by_RF.h"

static int spiClk = 2000000; // 4 MHz Maximum
unsigned long currentMicros;
unsigned long LEDMicros;
unsigned long LEDFrequency = 7500; //Time (in Microseconds) between each change in LED state (Red, IR);

#define MAX_RR_INTERVALS 50 //Maximum number of RR intervals to store

uint32_t rr_intervals[MAX_RR_INTERVALS]; //Array to store RR intervals
int rr_count = 0; //Counter to keep track of the number of RR intervals stored


// Pin Definitions.
#define MISO_PIN              19
#define MOSI_PIN              23
#define SCK_PIN               18
#define SS_PIN                5

#define VSPI_MISO             MISO
#define VSPI_MOSI             MOSI
#define VSPI_SCLK             SCK
#define VSPI_SS               SS_PIN

#define INT_PIN               17

#define RED                   12
#define IR                    14

//uninitalised pointers to SPI objects
MAX86141 pulseOx1;

void setup() {

    Serial.begin(115200);

    // Configure IO.
    pinMode(SS_PIN, OUTPUT);
    pinMode(INT_PIN, INPUT_PULLUP);

    pinMode(RED, OUTPUT);
    pinMode(IR, OUTPUT);
    digitalWrite(RED, LOW);
    digitalWrite(IR, LOW);

    digitalWrite(SS_PIN, LOW);

    //initialise SPI
    pulseOx1.spi = new SPIClass(VSPI);
    pulseOx1.SS = SS_PIN;
    Serial.println("Init Device");
    pulseOx1.spi->begin();
    delay(100);

    pulseOx1.setDebug(false);
    pulseOx1.init(spiClk);
    currentMicros = micros();//esp_timer_get_time();
    LEDMicros = currentMicros;

}

void calculateHRV() {

    //Calculating SDNN (standard deviation of RR intervals)
    float sum_squared_diff = 0.0;
    float mean_rr = 0.0;

    //Calculating mean RR interval
    for (int i = 0; i < MAX_RR_INTERVALS; i++) {

        mean_rr += rr_intervals[i];
    }

    mean_rr /= MAX_RR_INTERVALS;

    //Calculating sum of squared differences from mean
    for (int i = 0; i < MAX_RR_INTERVALS; i++) {

        float diff = rr_intervals[i] - mean_rr;
        sum_squared_diff += diff * diff;
    }

    float sdnn = sqrt(sum_squared_diff / MAX_RR_INTERVALS);

    Serial.print("SDNN: ");
    Serial.println(sdnn);
}


float detectRespirationRate(int *signal, int length) {
    
    //Performing peak detection on the signal
    float threshold = 0.5; //To be set based on the signal
    int peak_count = 0;
    bool is_peak = false;

    for (int i = 1; i < length - 1; ++i) {

        if (signal[i] > signal[i - 1] && signal[i] > signal[i + 1]) {
            // Peak detected
            if (!is_peak && signal[i] > threshold) {
                peak_count++;
                is_peak = true;
            }
        } 
        else {
            is_peak = false;
        }
    }

    //Calculating respiration rate based on the peak count and sampling rate
    float sampling_rate = 25; //Hz
    float respiration_rate = (peak_count * 60.0) / (length / sampling_rate);

    return respiration_rate;
}

// the loop function runs over and over again until power down or reset
void loop() {
  
    currentMicros = micros();

    if(pulseOx1.read_reg(REG_FIFO_DATA_COUNT) >= 6){
    
        pulseOx1.device_data_read();
        int led1A[128] = pulseOx1.led1A;
        int led2A[128] = pulseOx1.led2A;
        int led1B[128] = pulseOx1.led1B;
        int led2B[128] = pulseOx1.led2B;

        int RED_LED_buffer[128];
        int IR_LED_buffer[128];

        for (int i = 0; i < 128; ++i) {
            RED_LED_buffer[i] = (led1A[i] + led2A[i]) / 2;
        }

        for (int i = 0; i < 128; ++i) {
            IR_LED_buffer[i] = (led1B[i] + led2B[i]) / 2;
        }

        float spo2;
        int32_t heart_rate;
        int8_t spo2_valid;
        int8_t hr_valid;
        float ratio;
        float correl;

        rf_heart_rate_and_oxygen_saturation(IR_LED_buffer, 32, RED_LED_buffer, &spo2, &spo2_valid, &heart_rate, &hr_valid, &ratio, &correl);

        if (spo2_valid == 1) {
            Serial.print("SpO2: ");
            Serial.println(spo2);
        } 
        
        else {
            Serial.println("Invalid SpO2 value");
        }

        if (hr_valid == 1) {
            Serial.print("Heart Rate: ");
            Serial.println(heart_rate);

            // Calculating RR interval from heart rate and converting heart rate to milliseconds
            uint32_t rr_interval = 60000 / heart_rate;

            if (rr_count < MAX_RR_INTERVALS) {
                rr_intervals[rr_count++] = rr_interval;
            }
        } 
        
        else {
            Serial.println("Invalid Heart Rate");
        }

        Serial.println();

        // Calculate HRV
        if (rr_count >= MAX_RR_INTERVALS) {
            calculateHRV();
        }

        // Reset rr_count
        if (rr_count >= MAX_RR_INTERVALS) {
            rr_count = 0;
        }

        //Calculating Respiration Rate using the IR_LED_buffer
        float respiration_rate = detectRespirationRate(IR_LED_buffer, 128);
        Serial.print("Respiration Rate: ");
        Serial.println(respiration_rate);
        
        LEDMicros = currentMicros;
        delayMicroseconds(2500);
        //pulseOx1.device_data_read();
    }
}