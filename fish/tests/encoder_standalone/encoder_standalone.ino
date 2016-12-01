// encoder (optional)
//
// EMS22A30
// C28-MS6
// 1309M MEX
//
//    ENCODER                ARDUINO
//   
//    PIN1(red)   INPUT
//    PIN2        CLOCK        5
//    PIN3        GROUND
//    PIN4        OUTPUT       6
//    PIN5        VCC
//    PIN6        CS           7

#include <SPI.h>


// EMS PINS
#define PIN_EMS_CLK                               5
#define PIN_EMS_DATA                              6
#define PIN_EMS_CS                                7



int read_encoder() {
    byte i;
    digitalWrite(PIN_EMS_CS, HIGH);
    digitalWrite(PIN_EMS_CS, LOW);
    int encoder_angle = 0;

    for (i = 0; i < 10; i++) {
        digitalWrite(PIN_EMS_CLK, LOW);
        digitalWrite(PIN_EMS_CLK, HIGH);
        if (digitalRead(PIN_EMS_DATA)) {
            encoder_angle |= 1 << (9 - i);
        }
    }

    for (i = 0; i < 7; i++) {
        digitalWrite(PIN_EMS_CLK, LOW);
        digitalWrite(PIN_EMS_CLK, HIGH);
    }
    return encoder_angle;
}


void setup() {
    Serial.begin(115200);
}


void loop() {
    int encoder_angle = read_encoder();
    Serial.println(encoder_angle);

    pinMode(PIN_EMS_CLK, OUTPUT);
    pinMode(PIN_EMS_CS, OUTPUT);
    pinMode(PIN_EMS_DATA, INPUT);

    digitalWrite(PIN_EMS_CLK, HIGH);
    digitalWrite(PIN_EMS_CS, LOW);
}

