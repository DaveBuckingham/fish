#include <SPI.h>

#define SPI_CLOCK               1000000        // 1MHz clock

// const uint8_t IMU_SELECT[]    = {9, 10};
// const uint8_t NUM_IMUS        = 2;

//const uint8_t IMU_SELECT[]    = {9};
const uint8_t IMU_SELECT[]    = {4};
const uint8_t NUM_IMUS        = 1;

// IMU REGISTERS
#define REG_WHO_AM_I            0x75           // 117
#define REG_CONFIG              0x1A
#define GYRO_CONFIG            0X1B           // 27
#define ACCEL_CONFIG_1          0x1C           // 28
#define ACCEL_CONFIG_2          0x1D           // 29
#define REG_I2C_MST_CTRL        0x24
#define REG_I2C_SLV0_ADDR       0x25
#define REG_I2C_SLV0_REG        0x26
#define REG_I2C_SLV0_CTRL       0x27
#define REG_EXT_SENS_DATA_00    0x49
#define REG_I2C_SLV0_DO         0x63
#define REG_USER_CTRL           0x6A
#define REG_PWR_MGMT_1          0x6B
#define REG_PWR_MGMT_2          0x6C
#define REG_ACCEL_FIRST         0x3B
#define REG_GYRO_FIRST          0x43

#define REG_TEMP_OUT_H          0x41
#define REG_TEMP_OUT_L          0x42

#define I2C_ADDRESS_MAG         0x0C

// MAG REGISTERS
#define MAG_HXL                 0x03
#define MAG_CNTL1               0x0A
#define MAG_CNTL2               0x0B
#define MAG_ASAX                0x10

#define ENABLE_SLAVE_FLAG       0x80           // use when specifying data length to SLV0_CTRL
#define READ_FLAG               0x80
 
#define MAG_SENSITIVITY_SCALE   0.15

#define SAMPLE_FREQ_HZ          100           // 250 seems ok. starts to break around 300.


// COMMUNICATION
#define COM_FLAG                0x7E
#define COM_ESCAPE              0X7D
#define COM_XOR                 0X20


// EMS PINS
#define PIN_EMS_CLK             5
#define PIN_EMS_DATA            6
#define PIN_EMS_CS              7



#define I2C_DELAY               1200           // us between i2c txrx


#define SERIAL_BUFF_LENGTH      80             // leave room for byte stuffing
byte serial_buffer[SERIAL_BUFF_LENGTH];        // for framing and byte stuffing for tx
 

int encoder_angle;
unsigned long next_sample_id;            // count sample ids
int whoami;


uint8_t data_ready;


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

// READ FROM EMS
inline void read_encoder() {
    byte i;
    digitalWrite(PIN_EMS_CS, HIGH);
    digitalWrite(PIN_EMS_CS, LOW);
    encoder_angle = 0;

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
}


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
        write_register(IMU_SELECT[i], REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);                       delayMicroseconds(I2C_DELAY);

        // SET MAGNETOMETER TO CONTINUOUS MEASUREMENT MODE 2, 100HZ
        write_register(IMU_SELECT[i], REG_I2C_SLV0_REG, MAG_CNTL1);
        write_register(IMU_SELECT[i], REG_I2C_SLV0_DO, 0x16);                                          // 100hz, 16-bit
        write_register(IMU_SELECT[i], REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);                       delayMicroseconds(I2C_DELAY);

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

        // read_multiple_mag_registers(IMU_SELECT[i], MAG_ASAX, response

        // write_register(IMU_SELECT[i], REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);
        // write_register(IMU_SELECT[i], REG_I2C_SLV0_REG, MAG_ASAX);
        // write_register(IMU_SELECT[i], REG_I2C_SLV0_CTRL, 0x03 | ENABLE_SLAVE_FLAG);
        // read_multiple_registers_slow(IMU_SELECT[i], REG_EXT_SENS_DATA_00,response + (3*i),3);
    }

    tx_packet(response, 6);
}



byte read_mag_register(byte chip, byte mag_register) {
    write_register(chip, REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);
    write_register(chip, REG_I2C_SLV0_REG, mag_register);
    delayMicroseconds(I2C_DELAY);
    write_register(chip, REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);
    delayMicroseconds(I2C_DELAY);
    return read_register(chip, REG_EXT_SENS_DATA_00);
}

void read_multiple_mag_registers(byte chip, byte mag_register, byte *buff, unsigned int num_bytes ) {
    if (num_bytes < 2) {
        return;
    }

    write_register(chip, REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);
    write_register(chip, REG_I2C_SLV0_REG, mag_register); 
    delayMicroseconds(I2C_DELAY);
    write_register(chip, REG_I2C_SLV0_CTRL, num_bytes | ENABLE_SLAVE_FLAG); 
    delayMicroseconds(I2C_DELAY);
    // delayMicroseconds(I2C_DELAY);
    // read_multiple_registers_slow(chip, REG_EXT_SENS_DATA_00, buff, num_bytes);

    // CANT USE read_multiple_registers() BECAUSE NEED DELAY BETWEEN TXS
    unsigned int  i = 0;
    digitalWrite(chip, LOW);
    SPI.transfer(REG_EXT_SENS_DATA_00 | READ_FLAG);
    for(i = 0; i < num_bytes; i++)
        buff[i] = SPI.transfer(0x00);
        delayMicroseconds(I2C_DELAY);
    digitalWrite(chip, HIGH);

}






void read_sample(){
    uint8_t response_len = 52;
    uint8_t response[response_len];
    uint8_t i;
    uint8_t j;

    for (j=0; j < response_len; j++) {
        response[j] = 0;
    }


    j = 0;
    
    // SAMPLE ID
    response[j++] = next_sample_id >> 24;
    response[j++] = next_sample_id >> 16;
    response[j++] = next_sample_id >> 8;
    response[j++] = next_sample_id;
    next_sample_id++;

    // ENCODER VALUE
    read_encoder();
    response[j++] = encoder_angle >> 8;
    response[j++] = encoder_angle;


    for (i=0; i < NUM_IMUS; i++) {

        // TEMP, ACCEL, GYRO
        read_multiple_registers(IMU_SELECT[i], REG_TEMP_OUT_H, response + j, 2);
        j += 2;
        read_multiple_registers(IMU_SELECT[i], REG_ACCEL_FIRST, response + j, 6);
        j += 6;
        read_multiple_registers(IMU_SELECT[i], REG_GYRO_FIRST, response + j, 6);
        j += 6;


        // MAG DEVICE ID
        response[j++] = read_mag_register(IMU_SELECT[i], 0x00);

        // DATA READY?
        response[j++] = read_mag_register(IMU_SELECT[i], 0x02);

        // MAG DATA
        read_multiple_mag_registers(IMU_SELECT[i], 0x03, response + j, 7);


        j += 7;


    }

    tx_packet(response, response_len);

}





void imu_whoami() {
    uint8_t response[2];


//    #ifdef CS_IMU_0
//        response[0] = read_register(CS_IMU_0, REG_WHO_AM_I);  delay(20);
//    #else
//        response[0] = 0;
//    #endif
//
//    #ifdef CS_IMU_1
//        response[1] = read_register(CS_IMU_1, REG_WHO_AM_I);  delay(20);
//    #else
//        response[1] = 0;
//    #endif

    tx_packet(response, 2);
}

void setup() {
	Serial.begin(115200);
	//Serial.begin(9600);
}


//ISR(TIMER2_OVF_vect) {
//    read_sample();
//    TCNT2 = 130;           //Reset Timer to 130 out of 255
//    TIFR2 = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
//};


ISR(TIMER1_COMPA_vect) {
    read_sample();
}

//ISR(TIMER1_OVF_vect) {
//    TCNT1 = 34286;            // preload timer
//    read_sample();
//}


void loop() {
    if (Serial.available() > 0) {
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

                noInterrupts();           // disable all interrupts
                TCCR1A = 0;
                TCCR1B = 0;
                TCNT1  = 0;
                OCR1A = int((16000000 / 256) / SAMPLE_FREQ_HZ);          // compare match register = clock speed / prescaler / sample freq
                TCCR1B |= (1 << WGM12);   // CTC mode
                TCCR1B |= (1 << CS12);    // 256 prescaler 
                TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
                interrupts();             // enable all interrupts


                break;
            case 's':
                SPI.endTransaction();
                SPI.end();

                // SET MAGS TO POWER-DOWN MODE
                uint8_t i;
                for (i=0; i < NUM_IMUS; i++) {
                    write_register(IMU_SELECT[i], REG_I2C_SLV0_REG, MAG_CNTL1);                                       delay(I2C_DELAY);
                    write_register(IMU_SELECT[i], REG_I2C_SLV0_DO, 0x10);                                             delay(I2C_DELAY);
                    write_register(IMU_SELECT[i], REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);                       delay(I2C_DELAY);
                }

                break;
            case 't':
                // imu self tests
                break;
            default:
                break;
        }
    }
}
