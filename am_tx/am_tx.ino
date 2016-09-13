


// Has been tested on Arduino UNO.
// Will likely work on other models with same (16mhz) clock speed (i.e. not Mega).
// "imu" = MPU-9250 by InvenSense.
// "mag" = AK8963 embedded in the imu.
// We can communicate with the imu over spi, but the imu communicates with mag over i2c.



// Wiring arduino to imus:
//
//    ARDUINO     PIN      IMU
//    
//     CS1        9        NCS
//     CS2        10       NCS
//     MOSI       11       SDA
//     MISO       12       AD0
//     SCK        13       SCL
//     TRIGGER    7
//
// MOSI, MISO, and SCK pins are specified by the arduino spi library.
// Chip selects CS1 and CS2 are specified by in code (IMU_SELECT[]).




#include <SPI.h>



/////////////////////////////////////////
//            CONSTANTS                //
/////////////////////////////////////////

// #define USE_ENCODER

// #define USE_TRIGGER
#define SPI_CLOCK                                 1000000        // 1MHz clock specified for imus
#define SAMPLE_FREQ_HZ                            200            // attempted samples per second
#define NUM_IMUS                                  2              // how many imus, 1 or 2.
const uint8_t IMU_SELECT[]                      = {9, 10};       // chip select pins for imus (len == NUM_IMUS)
#define TRIGGER_PIN                               7

#ifdef USE_ENCODER
#define RESPONSE_LEN                              42             // how many bytes tx per sample
#else
#define RESPONSE_LEN                              40
#endif

#define SERIAL_BUFF_LENGTH                        80             // buffer for serial com: RESPONSE_LEN + room for byte stuffing

// IMU REGISTERS
#define REG_WHO_AM_I                              0x75           // 117
#define REG_CONFIG                                0x1A
#define GYRO_CONFIG                               0X1B           // 27
#define ACCEL_CONFIG_1                            0x1C           // 28
#define ACCEL_CONFIG_2                            0x1D           // 29
#define REG_I2C_MST_CTRL                          0x24
#define REG_I2C_SLV0_ADDR                         0x25
#define REG_I2C_SLV0_REG                          0x26
#define REG_I2C_SLV0_CTRL                         0x27
#define REG_EXT_SENS_DATA_00                      0x49
#define REG_I2C_SLV0_DO                           0x63
#define REG_USER_CTRL                             0x6A
#define REG_PWR_MGMT_1                            0x6B
#define REG_PWR_MGMT_2                            0x6C
#define REG_ACCEL_FIRST                           0x3B
#define REG_GYRO_FIRST                            0x43
#define REG_TEMP_OUT_H                            0x41
#define REG_TEMP_OUT_L                            0x42

// MAGNETOMETER I2C ADDRESS
#define I2C_ADDRESS_MAG                           0x0C

// MAG REGISTERS
#define MAG_HXL                                   0x03           // beginning of mag 6 bytes of mag data
#define MAG_CNTL1                                 0x0A
#define MAG_CNTL2                                 0x0B
#define MAG_ASAX                                  0x10

// FLAGS FOR COMMUNICATION WITH IMU AND MAG
#define ENABLE_SLAVE_FLAG                         0x80           // use when specifying data length for i2c
#define READ_FLAG                                 0x80           // for spi com
 
// FOR TXRX PACKETS OVER SERIAL
#define COM_FLAG                                  0x7E
#define COM_ESCAPE                                0X7D
#define COM_XOR                                   0X20

// EMS PINS
#define PIN_EMS_CLK                               5
#define PIN_EMS_DATA                              6
#define PIN_EMS_CS                                7


byte serial_buffer[SERIAL_BUFF_LENGTH];        // for framing and byte stuffing for tx
unsigned long next_sample_id;                  // counter for sample ids
int trigger_val;



void tx_packet(byte *in_buffer, unsigned int num_bytes) {
    byte val;
    byte i;
    byte j = 0;
    serial_buffer[j++] = COM_FLAG;
    for (i = 0; i < num_bytes; i++) {
        if (j >= SERIAL_BUFF_LENGTH - 3) {     // data byte, possible escape, and end flag
            return;
        }
        val = in_buffer[i];
        if ((val == COM_FLAG) || (val == COM_ESCAPE)) {
            serial_buffer[j++] = COM_ESCAPE;
            serial_buffer[j++] = val ^ COM_XOR;
        }
        else {
            serial_buffer[j++] = val;
        }
    }
    serial_buffer[j++] = COM_FLAG;
    Serial.write(serial_buffer, j);
}



unsigned int write_register(byte chip, uint8_t WriteAddr, uint8_t WriteData ) {
    unsigned int temp_val;

    digitalWrite(chip, LOW);
    SPI.transfer(WriteAddr);
    temp_val=SPI.transfer(WriteData);
    digitalWrite(chip, HIGH);

    return temp_val;
}

byte read_register(byte chip, byte address) {
    digitalWrite(chip, LOW);
    SPI.transfer(address | READ_FLAG);
    byte data = SPI.transfer(0x00);
    delay(1);
    digitalWrite(chip, HIGH);
    return data;
}


void read_multiple_registers(byte chip, uint8_t address, uint8_t *buff, unsigned int num_bytes ) {
    unsigned int  i = 0;

    digitalWrite(chip, LOW);
    SPI.transfer(address | READ_FLAG);
    for(i = 0; i < num_bytes; i++)
        buff[i] = SPI.transfer(0x00);
    digitalWrite(chip, HIGH);
}

#ifdef USE_ENCODER
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
#endif


// IMPLEMENTATION FROM "MPU-6500 ACCELEROMETER AND GYROSCOPE SELF-TEST IMPLEMENTATION.
// ACCORDING TO INVENSENSE TECH SUPPORT THIS PROCEDURE APPLIES TO MPU-9250.
// ROTATION SHOULD VARY LESS THAN 5 DEGREES DURING SELF-TEST.
// GYRO SELF-TEST WILL FAIL IF IT ROTATES MORE THAN +-2.5 DEGREES.
// FOR ACCEL, CHANGES IN LINEAR VELOCITY SHOULD BE < 0.2m/s.
// CHANGES IN TILT ANGLE SHOULD BE < 6 deg.
void self_test(byte chip) {
    int i;

	SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));

    ///// STEP 1 /////
    write_register(chip, REG_CONFIG, 0x02);
    write_register(chip, ACCEL_CONFIG_2, 0x02);

    uint8_t gyro_old_fs = read_register(chip, GYRO_CONFIG) | 0x18;
    write_register(chip, GYRO_CONFIG, 0x00);

    uint8_t accel_old_fs = read_register(chip, ACCEL_CONFIG_1) | 0x18;
    write_register(chip, ACCEL_CONFIG_1, 0x00);

    ///// STEP 2 /////
    uint8_t temp_buffer[6];
    long gyro_x;
    long gyro_y;
    long gyro_z;
    long accel_x;
    long accel_y;
    long accel_z;

    for (i=0; i<200; i++) {

        read_multiple_registers(chip, REG_ACCEL_FIRST, temp_buffer, 6);
        gyro_x += (temp_buffer[0] << 8) | temp_buffer[1];
        gyro_y += (temp_buffer[2] << 8) | temp_buffer[3];
        gyro_z += (temp_buffer[4] << 8) | temp_buffer[5];

        read_multiple_registers(chip, REG_GYRO_FIRST, temp_buffer, 6);
        accel_x += (temp_buffer[0] << 8) | temp_buffer[1];
        accel_y += (temp_buffer[2] << 8) | temp_buffer[3];
        accel_z += (temp_buffer[4] << 8) | temp_buffer[5];

        delay(1);
    }

    gyro_x /= 200;
    gyro_y /= 200;
    gyro_z /= 200;
    accel_x /= 200;
    accel_y /= 200;
    accel_z /= 200;

    ///// STEP 3 /////


    // gyro_st_response = gyro_st_enabled - gyro_st_disabled;
    // gyro_change_from_factory = (gyro_st_response - gyro_factory_trim) / gyro_factory_trim;
    // gyro_passed = (gyro_change_from_factory > gyro_lower_limit) && (gyro_change_from_factory < gyro_upper_limit);

    // accel_st_response = accel_st_enabled - accel_st_disabled;
    // accel_change_from_factory = (accel_st_response - accel_factory_trim) / accel_factory_trim;
    // accel_passed = (accel_change_from_factory > accel_lower_limit) && (accel_change_from_factory < accel_upper_limit);

    SPI.endTransaction;
    SPI.end();

}





void initialize(){
    next_sample_id = 0;

	SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));

    pinMode(TRIGGER_PIN, INPUT);

    uint8_t i;
    for (i=0; i < NUM_IMUS; i++) {

        pinMode(IMU_SELECT[i], OUTPUT);
        digitalWrite(IMU_SELECT[i], HIGH);
        delay(200);                                    

        write_register(IMU_SELECT[i], REG_PWR_MGMT_1, 0x81);   // reset mpu and set clock source
        delay(1);

        write_register(IMU_SELECT[i], REG_CONFIG, 0x01);             // DLPF: GYRO BANDWIDTH = 184HZ, TEMP BANDWIDTH = 188HZ
        write_register(IMU_SELECT[i], REG_USER_CTRL, 0x20);          // RESERVED??
        write_register(IMU_SELECT[i], REG_I2C_MST_CTRL, 0x0D);       // SET I2C MASTER CLOCK SPEED TO 400 KHZ

        // SOFT RESET MAGNETOMETER
        write_register(IMU_SELECT[i], REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG);
        write_register(IMU_SELECT[i], REG_I2C_SLV0_REG, MAG_CNTL2);
        write_register(IMU_SELECT[i], REG_I2C_SLV0_DO, 0x01);
        write_register(IMU_SELECT[i], REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);

        // SET MAGNETOMETER TO CONTINUOUS MEASUREMENT MODE 2, 100HZ
        write_register(IMU_SELECT[i], REG_I2C_SLV0_REG, MAG_CNTL1);
        write_register(IMU_SELECT[i], REG_I2C_SLV0_DO, 0x16);                                          // 100hz, 16-bit
        write_register(IMU_SELECT[i], REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);

    }
}


void tx_asa(){
    int i;
    uint8_t response[6];
    for (i=0; i < 6; i++) {
        response[i] = 0;
    }
    float data;


    for (i=0; i < NUM_IMUS; i++) {
        write_register(IMU_SELECT[i], REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);
        write_register(IMU_SELECT[i], REG_I2C_SLV0_REG, MAG_ASAX); 
        write_register(IMU_SELECT[i], REG_I2C_SLV0_CTRL, 3 | ENABLE_SLAVE_FLAG); 
        read_multiple_registers(IMU_SELECT[i], REG_EXT_SENS_DATA_00, response + (3*i), 3);
    }

    tx_packet(response, 6);
}




void read_sample(){
    uint8_t response[RESPONSE_LEN + 1];     // extra byte for reading STATUS2 from mag
    uint8_t i;
    uint8_t j;



    for (j=0; j < RESPONSE_LEN; j++) {
        response[j] = 0;
    }

    j = 0;
    
    // SAMPLE ID
    response[j++] = next_sample_id >> 24;
    response[j++] = next_sample_id >> 16;
    response[j++] = next_sample_id >> 8;
    response[j++] = next_sample_id;
    next_sample_id++;


#ifdef USE_ENCODER
    int encoder_angle = read_encoder();
    response[j++] = encoder_angle >> 8;
    response[j++] = encoder_angle;
#endif


    for (i=0; i < NUM_IMUS; i++) {

        // READ TEMPERATURE
        // read_multiple_registers(IMU_SELECT[i], REG_TEMP_OUT_H, response + j, 2);

        // READ ACCEL
        read_multiple_registers(IMU_SELECT[i], REG_ACCEL_FIRST, response + j, 6);
        j += 6;

        // READ GYRO
        read_multiple_registers(IMU_SELECT[i], REG_GYRO_FIRST, response + j, 6);
        j += 6;

        // READ MAG
        // WE READ 7 BYTES SO READING STATUS2 TRIGGERS DATA RESET
        write_register(IMU_SELECT[i], REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);      // specify mag i2c address
        write_register(IMU_SELECT[i], REG_I2C_SLV0_REG, MAG_HXL);                           // specify desired mag register
        write_register(IMU_SELECT[i], REG_I2C_SLV0_CTRL, 7 | ENABLE_SLAVE_FLAG);            // set num bytes to read 
        read_multiple_registers(IMU_SELECT[i], REG_EXT_SENS_DATA_00, response + j, 7);

        j += 6;

    }
    tx_packet(response, RESPONSE_LEN);

#ifdef USE_TRIGGER
    trigger_val = digitalRead(TRIGGER_PIN);
    tx_packet((byte*)&trigger_val, 1);
    if (!trigger_val) {
        stop_recording();
    }
#endif
}





// SHOULD ALSO CHECK IMU WHOAMIS
void imu_whoami() {
    uint8_t response[2];
    response[0] = NUM_IMUS > 0 ? read_register(IMU_SELECT[0], REG_WHO_AM_I) : 0;
    response[1] = NUM_IMUS > 1 ? read_register(IMU_SELECT[1], REG_WHO_AM_I) : 0;
    tx_packet(response, 2);
}

void start_recording() {
    noInterrupts();                                      // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    OCR1A = int((16000000 / 256) / SAMPLE_FREQ_HZ);      // compare match register = clock speed / prescaler / sample freq
    TCCR1B |= (1 << WGM12);                              // CTC mode
    TCCR1B |= (1 << CS12);                               // 256 prescaler 
    TIMSK1 |= (1 << OCIE1A);                             // enable timer compare interrupt
    interrupts();                                        // enable all interrupts
}

void stop_recording() {
    TIMSK1 &= ~(1 << OCIE1A);                            // disable timer compare interrupt
    // SET MAGS TO POWER-DOWN MODE
    byte i;
    for (i=0; i < NUM_IMUS; i++) {
        write_register(IMU_SELECT[i], REG_I2C_SLV0_REG, MAG_CNTL1);
        write_register(IMU_SELECT[i], REG_I2C_SLV0_DO, 0x10);
        write_register(IMU_SELECT[i], REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);
    }
    SPI.endTransaction();
    SPI.end();
    Serial.flush();
}

void setup() {
	Serial.begin(115200);
}


ISR(TIMER1_COMPA_vect) {
    read_sample();
}


void loop() {
    if (Serial.available() > 0) {
        byte i;
        switch (Serial.read()) {
            case 'i':
                initialize();
                break;
            case 'w':
                imu_whoami();
                break;
            case 'm':
                tx_asa();
                break;
            case 'r':
#ifdef USE_TRIGGER
                trigger_val = digitalRead(TRIGGER_PIN);
                if (trigger_val) {
                    start_recording();
                }
#else
                start_recording();
#endif
                break;
            case 's':
                stop_recording();
                break;
            case 't':
                for (i=0; i < NUM_IMUS; i++) {
                    self_test(i);
                }
                break;
            default:
                break;
        }
    }
}
