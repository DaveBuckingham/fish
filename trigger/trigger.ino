const int buttonPin = 7;     // the number of the pushbutton pin

int buttonState = 0;         // variable for reading the pushbutton status

void setup() {
    pinMode(buttonPin, INPUT);
    Serial.begin(115200);
}

void loop() {
    buttonState = digitalRead(buttonPin);
    Serial.println(buttonState);
    delay(50);
}
