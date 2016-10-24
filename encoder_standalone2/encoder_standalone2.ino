// http://www.mouser.com/ds/2/54/EMS22A-50229.pdf
// The encoder is designed for Daisy chain arrangement
// Need to send activate signal to read (50ns or more high signal to CS pin)
// Pin 1 = Digital Input (DI) => Left empty otherwise connect to last encoder
// Pin 2 = Clock (CLK) => PIN_CLK
// Pin 3 = GND 
// Pin 4 = Digital Output (DO) => PIN_DATA
// Pin 5 = VCC = 3.3V (for EMS22A-30, otherwise 5V for EMS22A-50)
// Pin 6 = CS => PIN_CS

const int PIN_CLOCK = 8; // pin 2
const int PIN_DATA = 9;  // pin 4
const int PIN_CS = 10;    //   pin 6

void setup() {
  Serial.begin(9600);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
  pinMode(PIN_DATA, INPUT);

  digitalWrite(PIN_CLOCK, HIGH);
  digitalWrite(PIN_CS, LOW);
}


void loop() {
    digitalWrite(PIN_CS, HIGH);
    digitalWrite(PIN_CS, LOW);
    int pos = 0;

    for (int i = 0; i < 17; i++) {
        digitalWrite(PIN_CLOCK, LOW);
        digitalWrite(PIN_CLOCK, HIGH);

        if ((i < 10) && digitalRead(PIN_DATA)) {
            pos |= 1 << (9 - i);
        }
    }
    
    Serial.println(pos);
}
