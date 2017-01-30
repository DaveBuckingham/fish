


// Has been tested on Arduino UNO.
// Will likely work on other models with same (16mhz) clock speed (i.e. not Mega).
// "imu" = MPU-9250 by InvenSense.
// "mag" = AK8963 embedded in the imu.
// We can communicate with the imu over spi, but the imu communicates with mag over i2c.



// Wiring arduino to imus:
//
//    ARDUINO     PIN      IMU        COLOR
//    
//     CS1        8        NCS        WHITE/CLEAR
//     CS2        9        NCS        WHITE/CLEAR
//     CS3        10       NCS        WHITE/CLEAR
//     MOSI       11       SDA/MOSI   GREEN
//     MISO       12       AD0/MISO   BLUE
//     SCK        13       SCL        YELLOW
//     TRIGGER    4
//
// MOSI, MISO, and SCK pins are specified by the arduino spi library.
// Chip selects CS1 and CS2 are specified by in code (IMU_SELECT[]).

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



/////////////////////////////////////////
//            CONSTANTS                //
/////////////////////////////////////////

#define DEL                          1 

#define USE_ENCODER

#define USE_TRIGGER
#define SPI_CLOCK                                 1000000        // 1MHz clock specified for imus
#define SAMPLE_FREQ_HZ                            200            // attempted samples per second
#define MAX_CHIP_SELECTS                          3              // how many imus, 1 or 2.
const uint8_t IMU_SELECT_OPTIONS[]                 = {8, 9, 10};    // len = MAX_CHIP_SELECTS
#define TRIGGER_PIN                               4

#define SERIAL_BUFF_LENGTH                        200             // buffer for serial com: response_len + room for byte stuffing

// IMU REGISTERS
#define SELF_TEST_X_GYRO                          0x00
#define SELF_TEST_Y_GYRO                          0x01
#define SELF_TEST_Z_GYRO                          0x02
#define SELF_TEST_X_ACCEL                         0x0D
#define SELF_TEST_Y_ACCEL                         0x0E
#define SELF_TEST_Z_ACCEL                         0x0F

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
#define REG_SAMPLE_RATE_DIVIDER                   0x19

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
#define COM_FLAG_START                            0x7E
#define COM_FLAG_END                              0x7F
#define COM_FLAG_ESCAPE                           0X7D
#define COM_FLAG_XOR                              0X20

// TO SPECIFY TYPE OF A (POSSIBLY EMPTY) PACKET SENT FROM ARDUINO TO PC
#define COM_PACKET_SAMPLE                         0x60
#define COM_PACKET_ASA                            0x61
#define COM_PACKET_TRIGGER                        0x63
#define COM_PACKET_STRING                         0x64
#define COM_PACKET_TEST                           0x65
#define COM_PACKET_HELLO                          0x66
#define COM_PACKET_NUMIMUS                        0x67

// SINGLE BYTE COMMANDS TO SEND FROM PC TO ARDUINO
#define COM_SIGNAL_INIT                           0x50
#define COM_SIGNAL_ASA                            0x52
#define COM_SIGNAL_RUN                            0x53
#define COM_SIGNAL_STOP                           0x54
#define COM_SIGNAL_TEST                           0x55
#define COM_SIGNAL_HELLO                          0x56

#define IMU_WHOAMI_VAL                            0x71



// EMS PINS
#define PIN_EMS_CLK                               5
#define PIN_EMS_DATA                              6
#define PIN_EMS_CS                                7



byte serial_buffer[SERIAL_BUFF_LENGTH];        // for framing and byte stuffing for tx
unsigned long next_sample_id;                  // counter for sample ids
byte num_imus;
byte imu_select[MAX_CHIP_SELECTS];
byte response_len;
byte recording;



void tx_packet(byte *in_buffer, unsigned int num_bytes, byte message_type) {
    byte val;
    byte i;
    byte j = 0;
    if ((message_type == COM_FLAG_START) || (message_type == COM_FLAG_END) || (message_type == COM_FLAG_ESCAPE)) {
        return;
    }
    serial_buffer[j++] = COM_FLAG_START;
    serial_buffer[j++] = message_type;
    for (i = 0; i < num_bytes; i++) {
        if (j >= SERIAL_BUFF_LENGTH - 3) {     // data byte, possible escape, and end flag
            return;
        }
        val = in_buffer[i];
        if ((val == COM_FLAG_START) || (val == COM_FLAG_ESCAPE) || (val == COM_FLAG_END)) {
            serial_buffer[j++] = COM_FLAG_ESCAPE;
            serial_buffer[j++] = val ^ COM_FLAG_XOR;
        }
        else {
            serial_buffer[j++] = val;
        }
    }
    serial_buffer[j++] = COM_FLAG_END;
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




// Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
// IMPLEMENTATION FROM "MPU-6500 ACCELEROMETER AND GYROSCOPE SELF-TEST IMPLEMENTATION.
// ACCORDING TO INVENSENSE TECH SUPPORT THIS PROCEDURE APPLIES TO MPU-9250.
// ROTATION SHOULD VARY LESS THAN 5 DEGREES DURING SELF-TEST.
// GYRO SELF-TEST WILL FAIL IF IT ROTATES MORE THAN +-2.5 DEGREES.
// FOR ACCEL, CHANGES IN LINEAR VELOCITY SHOULD BE < 0.2m/s.
// CHANGES IN TILT ANGLE SHOULD BE < 6 deg.
//
// MUST BE CALLED FROM test() FUNCTION SO SPI IS SET UP

#define NUM_REPS         200

byte self_test(byte chip) {
    // uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
    // uint8_t selfTest[6];
    // int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
    // float factoryTrim[6];

    uint8_t FS = 0;

    int i;
    uint8_t temp_buffer[6];

    int32_t GX_OS = 0;
    int32_t GY_OS = 0;
    int32_t GZ_OS = 0;
    int32_t AX_OS = 0;
    int32_t AY_OS = 0;
    int32_t AZ_OS = 0;

    int32_t GX_ST_OS = 0;
    int32_t GY_ST_OS = 0;
    int32_t GZ_ST_OS = 0;
    int32_t AX_ST_OS = 0;
    int32_t AY_ST_OS = 0;
    int32_t AZ_ST_OS = 0;


    ///// STEP 3.0.1 /////

    write_register(chip, REG_SAMPLE_RATE_DIVIDER, 0x00);           // Set gyro sample rate to 1 kHz

    write_register(chip, REG_CONFIG, 0x02);
    write_register(chip, ACCEL_CONFIG_2, 0x02);

    uint8_t gyro_old_fs = read_register(chip, GYRO_CONFIG) | 0x18;
    write_register(chip, GYRO_CONFIG, 0x00);
    // write_register(chip, GYRO_CONFIG, 1<<FS);

    uint8_t accel_old_fs = read_register(chip, ACCEL_CONFIG_1) | 0x18;
    write_register(chip, ACCEL_CONFIG_1, 0x00);
    // write_register(chip, ACCEL_CONFIG_1, 1<<FS);


    ///// STEP 3.0.2 /////
    for (i=0; i<NUM_REPS; i++) {

        read_multiple_registers(chip, REG_ACCEL_FIRST, temp_buffer, 6);
        // GX_OS += ((temp_buffer[0] << 8) | temp_buffer[1]);
        // GY_OS += ((temp_buffer[2] << 8) | temp_buffer[3]);
        // GZ_OS += ((temp_buffer[4] << 8) | temp_buffer[5]);

        GX_OS +=   (int16_t)(((int16_t)temp_buffer[0] << 8) | temp_buffer[1]);
        GY_OS +=   (int16_t)(((int16_t)temp_buffer[2] << 8) | temp_buffer[3]);
        GZ_OS +=   (int16_t)(((int16_t)temp_buffer[4] << 8) | temp_buffer[5]);

        read_multiple_registers(chip, REG_GYRO_FIRST, temp_buffer, 6);
        // AX_OS += ((temp_buffer[0] << 8) | temp_buffer[1]);
        // AY_OS += ((temp_buffer[2] << 8) | temp_buffer[3]);
        // AZ_OS += ((temp_buffer[4] << 8) | temp_buffer[5]);

        AX_OS +=   (int16_t)(((int16_t)temp_buffer[0] << 8) | temp_buffer[1]);
        AY_OS +=   (int16_t)(((int16_t)temp_buffer[2] << 8) | temp_buffer[3]);
        AZ_OS +=   (int16_t)(((int16_t)temp_buffer[4] << 8) | temp_buffer[5]);

        delay(1);
    }

    GX_OS /= NUM_REPS;
    GY_OS /= NUM_REPS;
    GZ_OS /= NUM_REPS;
    AX_OS /= NUM_REPS;
    AY_OS /= NUM_REPS;
    AZ_OS /= NUM_REPS;

    Serial.println("---START---");
    Serial.println(GX_OS);
    Serial.println(GY_OS);
    Serial.println(GZ_OS);
    Serial.println(AX_OS);
    Serial.println(AY_OS);
    Serial.println(AZ_OS);


    ///// STEP 3.0.3 /////

    // SET SELF-TEST FLAGS
    write_register(chip, GYRO_CONFIG, 0xE0);
    write_register(chip, ACCEL_CONFIG_1, 0xE0);


    ///// STEP 3.0.4 /////
    // delay(20);
    delay(25);


    ///// STEP 3.0.5 /////
    for (i=0; i<NUM_REPS; i++) {

        read_multiple_registers(chip, REG_ACCEL_FIRST, temp_buffer, 6);
        GX_ST_OS += (temp_buffer[0] << 8) | temp_buffer[1];
        GY_ST_OS += (temp_buffer[2] << 8) | temp_buffer[3];
        GZ_ST_OS += (temp_buffer[4] << 8) | temp_buffer[5];

        read_multiple_registers(chip, REG_GYRO_FIRST, temp_buffer, 6);
        AX_ST_OS += (temp_buffer[0] << 8) | temp_buffer[1];
        AY_ST_OS += (temp_buffer[2] << 8) | temp_buffer[3];
        AZ_ST_OS += (temp_buffer[4] << 8) | temp_buffer[5];

        delay(1);
    }

    GX_ST_OS /= NUM_REPS;
    GY_ST_OS /= NUM_REPS;
    GZ_ST_OS /= NUM_REPS;
    AX_ST_OS /= NUM_REPS;
    AY_ST_OS /= NUM_REPS;
    AZ_ST_OS /= NUM_REPS;

    Serial.println("---OS----");
    Serial.println(GX_ST_OS);
    Serial.println(GY_ST_OS);
    Serial.println(GZ_ST_OS);
    Serial.println(AX_ST_OS);
    Serial.println(AY_ST_OS);
    Serial.println(AZ_ST_OS);


    ///// STEP 3.0.6 /////
    int32_t GXST = GX_ST_OS - GX_OS;
    int32_t GYST = GY_ST_OS - GY_OS;
    int32_t GZST = GZ_ST_OS - GZ_OS;
    int32_t AXST = AX_ST_OS - AX_OS;
    int32_t AYST = AY_ST_OS - AY_OS;
    int32_t AZST = AZ_ST_OS - AZ_OS;

    Serial.println("---DIFF----");
    Serial.println(GXST);
    Serial.println(GYST);
    Serial.println(GZST);
    Serial.println(AXST);
    Serial.println(AYST);
    Serial.println(AZST);


    ///// STEP 3.1.1 /////
    write_register(chip, GYRO_CONFIG, 0x00);
    write_register(chip, ACCEL_CONFIG_1, 0x00);

    ///// STEP 3.1.2 /////
    delay(25);                                    
    // delay(20);                                    


    ///// STEP 3.1.3 /////
    write_register(chip, GYRO_CONFIG, gyro_old_fs);
    write_register(chip, ACCEL_CONFIG_1, accel_old_fs);


    ///// STEP 3.2.1 /////
    byte self_test_gyro_x =  read_register(chip, SELF_TEST_X_GYRO);
    byte self_test_gyro_y =  read_register(chip, SELF_TEST_Y_GYRO);
    byte self_test_gyro_z =  read_register(chip, SELF_TEST_Z_GYRO);
    byte self_test_accel_x = read_register(chip, SELF_TEST_X_ACCEL);
    byte self_test_accel_y = read_register(chip, SELF_TEST_Y_ACCEL);
    byte self_test_accel_z = read_register(chip, SELF_TEST_Z_ACCEL);

    Serial.println("---selftest----");
    Serial.println(self_test_gyro_x);
    Serial.println(self_test_gyro_y);
    Serial.println(self_test_gyro_z);
    Serial.println(self_test_accel_x);
    Serial.println(self_test_accel_y);
    Serial.println(self_test_accel_z);

    ///// STEP 3.2.2 /////
    // float GXST_OTP =  (float)(2620/1<<FS)*(pow( 1.01 , ((float)self_test_gyro_x  - 1.0) ));
    // float GYST_OTP =  (float)(2620/1<<FS)*(pow( 1.01 , ((float)self_test_gyro_y  - 1.0) ));
    // float GZST_OTP =  (float)(2620/1<<FS)*(pow( 1.01 , ((float)self_test_gyro_z  - 1.0) ));
    // float AXST_OTP =  (float)(2620/1<<FS)*(pow( 1.01 , ((float)self_test_accel_x - 1.0) ));
    // float AYST_OTP =  (float)(2620/1<<FS)*(pow( 1.01 , ((float)self_test_accel_y - 1.0) ));
    // float AZST_OTP =  (float)(2620/1<<FS)*(pow( 1.01 , ((float)self_test_accel_z - 1.0) ));

    float GXST_OTP =  (2620)*(pow( 1.01 , ((float)self_test_gyro_x  - 1.0) ));
    float GYST_OTP =  (2620)*(pow( 1.01 , ((float)self_test_gyro_y  - 1.0) ));
    float GZST_OTP =  (2620)*(pow( 1.01 , ((float)self_test_gyro_z  - 1.0) ));
    float AXST_OTP =  (2620)*(pow( 1.01 , ((float)self_test_accel_x - 1.0) ));
    float AYST_OTP =  (2620)*(pow( 1.01 , ((float)self_test_accel_y - 1.0) ));
    float AZST_OTP =  (2620)*(pow( 1.01 , ((float)self_test_accel_z - 1.0) ));

    Serial.println("---OTP----");
    Serial.println(GXST_OTP);
    Serial.println(GYST_OTP);
    Serial.println(GZST_OTP);
    Serial.println(AXST_OTP);
    Serial.println(AYST_OTP);
    Serial.println(AZST_OTP);



    // for (int i = 0; i < 3; i++) {
    //     destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.;
    //     destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.;
    // }

    ///// STEP 3.2.3 (a and b) /////
    byte result_gyro_x =  (GXST_OTP != 0.0) ? ((GXST / GXST_OTP) > 5) : (abs(GXST) >= 60);
    byte result_gyro_y =  (GYST_OTP != 0.0) ? ((GYST / GYST_OTP) > 5) : (abs(GYST) >= 60);
    byte result_gyro_z =  (GZST_OTP != 0.0) ? ((GZST / GZST_OTP) > 5) : (abs(GZST) >= 60);
    byte result_accel_x = (AXST_OTP != 0.0) ? ((AXST / AXST_OTP) > 5) && ((AXST / AXST_OTP) < 1.5) : (abs(AXST) >= 60);
    byte result_accel_y = (AYST_OTP != 0.0) ? ((AYST / AYST_OTP) > 5) && ((AYST / AYST_OTP) < 1.5) : (abs(AYST) >= 60);
    byte result_accel_z = (AZST_OTP != 0.0) ? ((AZST / AZST_OTP) > 5) && ((AYST / AYST_OTP) < 1.5) : (abs(AZST) >= 60);

    // GXST = GXST & 0x0F;
    // GYST = GYST & 0x0F;
    // GZST = GZST & 0x0F;
    // AXST = AXST & 0x0F;
    // AYST = AYST & 0x0F;
    // AZST = AZST & 0x0F;
    // GXST_OTP = GXST_OTP & 0x0F;
    // GYST_OTP = GYST_OTP & 0x0F;
    // GZST_OTP = GZST_OTP & 0x0F;
    // AXST_OTP = AXST_OTP & 0x0F;
    // AYST_OTP = AYST_OTP & 0x0F;
    // AZST_OTP = AZST_OTP & 0x0F;



    Serial.println("---result----");
    Serial.println(GXST / GXST_OTP);
    Serial.println(GYST / GYST_OTP);
    Serial.println(GZST / GZST_OTP);
    Serial.println(AXST / AXST_OTP);
    Serial.println(AYST / AYST_OTP);
    Serial.println(AZST / AZST_OTP);


    Serial.println("---END---");

    // STEP 3.2.3.c SAYS TO CHECK OFFSET VALUES, BUT I DON'T KNOW HOW TO GET THESE

    return (result_gyro_x  && result_gyro_y  && result_gyro_z && result_accel_x && result_accel_y && result_accel_z);
   

    // gyro_st_response = gyro_st_enabled - gyro_st_disabled;
    // gyro_change_from_factory = (gyro_st_response - gyro_factory_trim) / gyro_factory_trim;
    // gyro_passed = (gyro_change_from_factory > gyro_lower_limit) && (gyro_change_from_factory < gyro_upper_limit);

    // accel_st_response = accel_st_enabled - accel_st_disabled;
    // accel_change_from_factory = (accel_st_response - accel_factory_trim) / accel_factory_trim;
    // accel_passed = (accel_change_from_factory > accel_lower_limit) && (accel_change_from_factory < accel_upper_limit);

}



void begin_imu_com() {

    SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));

    uint8_t i;

//     for (i=0; i < num_imus; i++) {
//         pinMode(imu_select[i], OUTPUT);
//         digitalWrite(imu_select[i], HIGH);
//     }

    num_imus = 0;

    for (i=0; i < MAX_CHIP_SELECTS; i++) {
        pinMode(IMU_SELECT_OPTIONS[i], OUTPUT);
        digitalWrite(IMU_SELECT_OPTIONS[i], HIGH);
        delay(400); // NEED THIS???
        if (read_register(IMU_SELECT_OPTIONS[i], REG_WHO_AM_I) == IMU_WHOAMI_VAL) {
           imu_select[num_imus] = i;
           num_imus++;
        }
    }
}

void end_imu_com() {
    SPI.endTransaction();
    SPI.end();
    Serial.flush();
}



void initialize(){
    next_sample_id = 0;

    pinMode(TRIGGER_PIN, INPUT);

    begin_imu_com();
    tx_packet(&num_imus, 1, COM_PACKET_NUMIMUS);

    response_len = 4 + (12 * num_imus) + 1;
    #ifdef USE_ENCODER
        response_len += 2;
    #endif


    uint8_t i;


    for (i=0; i < num_imus; i++) {

        write_register(imu_select[i], REG_PWR_MGMT_1, 0x81);   // reset mpu and set clock source
        delay(1);

        write_register(imu_select[i], REG_CONFIG, 0x01);             // DLPF: GYRO BANDWIDTH = 184HZ, TEMP BANDWIDTH = 188HZ
        write_register(imu_select[i], REG_USER_CTRL, 0x20);          // RESERVED??
        write_register(imu_select[i], REG_I2C_MST_CTRL, 0x0D);       // SET I2C MASTER CLOCK SPEED TO 400 KHZ

        // SOFT RESET MAGNETOMETER
        write_register(imu_select[i], REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG);
        write_register(imu_select[i], REG_I2C_SLV0_REG, MAG_CNTL2);
        write_register(imu_select[i], REG_I2C_SLV0_DO, 0x01);
        write_register(imu_select[i], REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);

        // SET MAGNETOMETER TO CONTINUOUS MEASUREMENT MODE 2, 100HZ
        write_register(imu_select[i], REG_I2C_SLV0_REG, MAG_CNTL1);
        write_register(imu_select[i], REG_I2C_SLV0_DO, 0x16);                                          // 100hz, 16-bit
        write_register(imu_select[i], REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);

    }


#ifdef USE_ENCODER
    pinMode(PIN_EMS_CLK, OUTPUT);
    pinMode(PIN_EMS_CS, OUTPUT);
    pinMode(PIN_EMS_DATA, INPUT);

    digitalWrite(PIN_EMS_CLK, HIGH);
    digitalWrite(PIN_EMS_CS, LOW);
#endif


}


void tx_asa(){
    int i;
    uint8_t response[3];

    for (i=0; i < num_imus; i++) {
        write_register(imu_select[i], REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);
        write_register(imu_select[i], REG_I2C_SLV0_REG, MAG_ASAX);
        write_register(imu_select[i], REG_I2C_SLV0_CTRL, 3 | ENABLE_SLAVE_FLAG);
        delay(DEL);
        read_multiple_registers(imu_select[i], REG_EXT_SENS_DATA_00, response, 3);
        delay(DEL);
        tx_packet(response, 3, COM_PACKET_ASA);
    }

}



void read_sample(){
    uint8_t response[response_len];
    uint8_t i;
    uint8_t j;


    for (j=0; j < response_len; j++) {
        response[j] = 0;
    }

    j = 0;

    // COULD SPEED THING UP BY REMOVING SAMPLE IDS...
    
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


    for (i=0; i < num_imus; i++) {

        // READ TEMPERATURE
        // read_multiple_registers(imu_select[i], REG_TEMP_OUT_H, response + j, 2);

        // READ ACCEL
        read_multiple_registers(imu_select[i], REG_ACCEL_FIRST, response + j, 6);
        j += 6;

        // READ GYRO
        read_multiple_registers(imu_select[i], REG_GYRO_FIRST, response + j, 6);
        j += 6;

        // READ MAG
        // WE READ 7 BYTES SO READING STATUS2 TRIGGERS DATA RESET
        // N.B., MAG REGISTERS ARE LAID OUT IN LITTLE-ENDIAN ORDER, UNLIKE ACCEL AND GYRO
        write_register(imu_select[i], REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);   // specify mag i2c address
        write_register(imu_select[i], REG_I2C_SLV0_REG, MAG_HXL);                           // specify desired mag register
        write_register(imu_select[i], REG_I2C_SLV0_CTRL, 7 | ENABLE_SLAVE_FLAG);           // set num bytes to read 
        read_multiple_registers(imu_select[i], REG_EXT_SENS_DATA_00, response + j, 7);
        j += 6;

    }

    // will overwrite the extra byte from mag STATUS2
    response[j++] = (digitalRead(TRIGGER_PIN) == HIGH) ? 0x01 : 0x00;

    tx_packet(response, response_len, COM_PACKET_SAMPLE);
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
    recording = 1;
}

void stop_recording() {
    TIMSK1 &= ~(1 << OCIE1A);                            // disable timer compare interrupt
    // SET MAGS TO POWER-DOWN MODE
    byte i;
    for (i=0; i < num_imus; i++) {
        write_register(imu_select[i], REG_I2C_SLV0_REG, MAG_CNTL1);
        write_register(imu_select[i], REG_I2C_SLV0_DO, 0x10);
        write_register(imu_select[i], REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);
    }
    end_imu_com();
    recording = 0;
}

void setup() {
    Serial.begin(115200);
}


ISR(TIMER1_COMPA_vect) {
    read_sample();
}


void run_test() {

// INITIALIZE FIRST???

    uint8_t response[num_imus * 3];
    uint8_t i;
    uint8_t j = 0;

//    begin_imu_com();
//
//    for (i=0; i < num_imus; i++) {
//
//        // COMMUNICATE WITH THE IMUS
//        response[j++] = (read_register(imu_select[i], REG_WHO_AM_I) == IMU_WHOAMI_VAL) ? 1 : 0;
//
//
//        // RUN IMU SELF TESTS
//        response[j++] = (self_test(imu_select[i])) ? 1 : 0;
//
//        // MAG SELF TESTS NOT IMPLEMENTED YET
//        response[j++] = 0;
//    }
//
//    end_imu_com();

    tx_packet(response, j, COM_PACKET_TEST);
}


void loop() {
    byte val;
    if (Serial.available() > 0) {
        switch (Serial.read()) {
            case COM_SIGNAL_HELLO:
                if (recording) {
                    stop_recording;
                }
                tx_packet(0, 0, COM_PACKET_HELLO);
            case COM_SIGNAL_INIT:
                if (recording) {
                    stop_recording;
                }
                initialize();
                break;
            case COM_SIGNAL_ASA:
                tx_asa();
                break;
            case COM_SIGNAL_RUN:
                start_recording();
                break;
            case COM_SIGNAL_STOP:
                stop_recording();
                break;
            case COM_SIGNAL_TEST:
                run_test();
                break;
            default:
                break;
        }
    }
}

