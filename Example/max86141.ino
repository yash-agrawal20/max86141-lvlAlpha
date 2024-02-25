#include "max86141.h"

static int spiClk = 2000000; // 8 MHz Maximum
unsigned long currentMicros;
unsigned long LEDMicros;
unsigned long LEDFrequency = 7500; //Time (in Microseconds) between each change in LED state (Red, IR, Ambient);

bool RED_ON = false;
bool IR_ON = false;
bool AMBIENT = true;

float RED_AVG, IR_AVG, AMBIENT_AVG;


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

#define GPIO1_PIN             16
#define GPIO2_PIN             4

#define RED                   12
#define IR                    14

//uninitalised pointers to SPI objects
MAX86141 pulseOx1;
//MAX86141 pulseOx2;

void setup() {

    Serial.begin(115200);

    // Configure IO.
    pinMode(SS_PIN, OUTPUT);
    pinMode(INT_PIN, INPUT_PULLUP);
    //pinMode(GPIO1_PIN, OUTPUT);
    //pinMode(GPIO2_PIN, INPUT_PULLUP);

    pinMode(RED, OUTPUT);
    pinMode(IR, OUTPUT);
    digitalWrite(RED, LOW);
    digitalWrite(IR, LOW);

    digitalWrite(SS_PIN, LOW);
    //digitalWrite(GPIO1_PIN, HIGH);

    //initialise SPI
    pulseOx1.spi = new SPIClass(VSPI);
    pulseOx1.SS = 5;
    Serial.println("Init Device");
    pulseOx1.spi->begin();
    delay(100);
    pulseOx1.setDebug(false);
    pulseOx1.init(spiClk);
    currentMicros = micros();//esp_timer_get_time();
    LEDMicros = currentMicros;

}

void switch_LED(){

    //pulseOx1.device_data_read();
    if(IR_ON){

        digitalWrite(RED,LOW);
        digitalWrite(IR,LOW);
        //Serial.println("AMBIENT");
        RED_ON = false;
        IR_ON = false;
        AMBIENT = true;
        return;
    }

    if(RED_ON){

        digitalWrite(RED,LOW);
        digitalWrite(IR,HIGH);
        //Serial.println("IR");
        RED_ON = false;
        AMBIENT = false;
        IR_ON = true;
        return;
    }

    if(AMBIENT){

        digitalWrite(IR,LOW);
        digitalWrite(RED,HIGH);
        //Serial.println("RED");
        AMBIENT = false;
        IR_ON = false;
        RED_ON = true;
        return;
    }
}

// the loop function runs over and over again until power down or reset
void loop() {
  
    currentMicros = micros();

    if(pulseOx1.read_reg(REG_FIFO_DATA_COUNT) >= 6){
    
        pulseOx1.device_data_read();
        int led1A = pulseOx1.ledSeq1A_PD1[0];
        int led2A = pulseOx1.ledSeq2A_PD1[0];
        int led1B = pulseOx1.ledSeq1B_PD1[0];
        int led2B = pulseOx1.ledSeq2B_PD1[0];

        AMBIENT_AVG = (pulseOx1.ledSeq3A_PD1[0] + pulseOx1.ledSeq3B_PD1[0])*0.5;

        RED_AVG = (led1A + led1B)*0.5;
        // Serial.print("Red: ");
        // Serial.print(RED_AVG);

        IR_AVG = (led2A + led2B)*0.5;
        // Serial.print("\t");
        // Serial.print("IR: ");
        // Serial.print(IR_AVG);

        // Serial.print("\t");
        // Serial.print("Ambient: ");
        // Serial.println(AMBIENT_AVG); 

        float ratio = RED_AVG/IR_AVG;
        Serial.print("Ratio: ");
        Serial.println(ratio*4);
        
        //switch_LED();
        LEDMicros = currentMicros;
        //delayMicroseconds(2500);
        //pulseOx1.device_data_read();
    }
}