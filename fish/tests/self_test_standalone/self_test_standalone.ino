


// Wiring arduino to imus:
//
//    ARDUINO     PIN      IMU        COLOR
//    
//     CS1        9        NCS        WHITE/CLEAR
//     CS2        10       NCS        WHITE/CLEAR
//     MOSI       11       SDA/MOSI   GREEN
//     MISO       12       AD0/MISO   BLUE
//     SCK        13       SCL        YELLOW
//     TRIGGER    7
//
// MOSI, MISO, and SCK pins are specified by the arduino spi library.
// Chip selects CS1 and CS2 are specified by in code (IMU_SELECT[]).


#include <SPI.h>



/////////////////////////////////////////
//            CONSTANTS                //
/////////////////////////////////////////

// #define USE_ENCODER

#define USE_TRIGGER
#define SPI_CLOCK                                 1000000        // 1MHz clock specified for imus
#define SAMPLE_FREQ_HZ                            200            // attempted samples per second
#define NUM_IMUS                                  1              // how many imus, 1 or 2.
const uint8_t IMU_SELECT[]                      = {9};       // chip select pins for imus (len == NUM_IMUS)
#define TRIGGER_PIN                               7

#define RESPONSE_LEN                              41

// IMU REGISTERS
#define SELF_TEST_X_GYRO                          0x00
#define SELF_TEST_Y_GYRO                          0x01
#define SELF_TEST_Z_GYRO                          0x02
#define SELF_TEST_X_ACCEL                         0x0D
#define SELF_TEST_Y_ACCEL                         0x0E
#define SELF_TEST_Z_ACCEL                         0x0F

#define REG_WHO_AM_I                              0x75           // 117
#define REG_CONFIG                                0x1A           // 26
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


// FLAGS FOR COMMUNICATION WITH IMU AND MAG
#define ENABLE_SLAVE_FLAG                         0x80           // use when specifying data length for i2c
#define READ_FLAG                                 0x80           // for spi com
 
#define IMU_WHOAMI_VAL                            0x71





// EMS PINS
#define PIN_EMS_CLK                               5
#define PIN_EMS_DATA                              6
#define PIN_EMS_CS                                7


#define NUM_REPS                                  200



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




// MUST BE CALLED FROM test() FUNCTION SO SPI IS SET UP

byte self_test(byte chip) {

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

    write_register(chip, REG_CONFIG, 0x02);       // set gyro dlpf code to 2

    write_register(chip, ACCEL_CONFIG_2, 0x02);   // set accel dlpf code to 2

    uint8_t gyro_old_fs = read_register(chip, GYRO_CONFIG) & 0x18;     // store existing full scale range
    write_register(chip, GYRO_CONFIG, 0x00);                           // set full scale range to 250dps
    // write_register(chip, GYRO_CONFIG, 1<<FS);

    uint8_t accel_old_fs = read_register(chip, ACCEL_CONFIG_1) & 0x18;  // store existing full scale range code
    write_register(chip, ACCEL_CONFIG_1, 0x00);                         // select full scale range 20 +-2g
    // write_register(chip, ACCEL_CONFIG_1, 1<<FS);


    ///// STEP 3.0.2 /////

    write_register(chip, REG_SAMPLE_RATE_DIVIDER, 0x00);           // Set gyro sample rate to 1 kHz
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

    // THESE ARE THE "LSB OF GX_OS, GY_OS..." (page 5)
    GX_OS /= NUM_REPS;
    GY_OS /= NUM_REPS;
    GZ_OS /= NUM_REPS;
    AX_OS /= NUM_REPS;
    AY_OS /= NUM_REPS;
    AZ_OS /= NUM_REPS;

    // GX_OS &= 0xff;
    // GY_OS &= 0xff;
    // GZ_OS &= 0xff;
    // AX_OS &= 0xff;
    // AY_OS &= 0xff;
    // AZ_OS &= 0xff;

    Serial.println("---START---");
    Serial.println(GX_OS);
    Serial.println(GY_OS);
    Serial.println(GZ_OS);
    Serial.println(AX_OS);
    Serial.println(AY_OS);
    Serial.println(AZ_OS);


    ///// STEP 3.0.3 /////


    // SET SELF-TEST FLAGS
    // write_register(chip, GYRO_CONFIG, B111);
    // write_register(chip, ACCEL_CONFIG_1, B111);

    // ENABLE SELF TEST  (0xE0 == 0b11100000)
    write_register(chip, GYRO_CONFIG, 0xE0);
    write_register(chip, ACCEL_CONFIG_1, 0xE0);


    ///// STEP 3.0.4 /////
    delay(20);              // "wait 20ms for oscillations to stabilize"


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

        delay(1);  // documentation says to read at a 1kHz rate, but other implemenations have no delay
    }

    // "THE AVERAGE VALUES WILL BE THE LSB OF GX_ST_OS, GY_ST_OS..." (page 5)
    GX_ST_OS /= NUM_REPS;
    GY_ST_OS /= NUM_REPS;
    GZ_ST_OS /= NUM_REPS;
    AX_ST_OS /= NUM_REPS;
    AY_ST_OS /= NUM_REPS;
    AZ_ST_OS /= NUM_REPS;

    // GX_ST_OS &= 0xff;
    // GY_ST_OS &= 0xff;
    // GZ_ST_OS &= 0xff;
    // AX_ST_OS &= 0xff;
    // AY_ST_OS &= 0xff;
    // AZ_ST_OS &= 0xff;

    Serial.println("---ST_OS---");
    Serial.println(GX_ST_OS);
    Serial.println(GY_ST_OS);
    Serial.println(GZ_ST_OS);
    Serial.println(AX_ST_OS);
    Serial.println(AY_ST_OS);
    Serial.println(AZ_ST_OS);


    ///// STEP 3.0.6 /////
    // GET THE "SELF-TEST VALUES" OR THE "SELF-TEST RESPONSE" (page 5)
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
    delay(20);                                    


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

    double GXST_OTP =  (double)(2620)*(pow( 1.01 , ((double)self_test_gyro_x  - 1.0) ));
    double GYST_OTP =  (double)(2620)*(pow( 1.01 , ((double)self_test_gyro_y  - 1.0) ));
    double GZST_OTP =  (double)(2620)*(pow( 1.01 , ((double)self_test_gyro_z  - 1.0) ));
    double AXST_OTP =  (double)(2620)*(pow( 1.01 , ((double)self_test_accel_x - 1.0) ));
    double AYST_OTP =  (double)(2620)*(pow( 1.01 , ((double)self_test_accel_y - 1.0) ));
    double AZST_OTP =  (double)(2620)*(pow( 1.01 , ((double)self_test_accel_z - 1.0) ));

    // uint8_t GXST_OTP =  2620 * (pow( 1.01 , ((double)self_test_gyro_x  - 1.0) ));
    // uint8_t GYST_OTP =  2620 * (pow( 1.01 , ((double)self_test_gyro_y  - 1.0) ));
    // uint8_t GZST_OTP =  2620 * (pow( 1.01 , ((double)self_test_gyro_z  - 1.0) ));
    // uint8_t AXST_OTP =  2620 * (pow( 1.01 , ((double)self_test_accel_x - 1.0) ));
    // uint8_t AYST_OTP =  2620 * (pow( 1.01 , ((double)self_test_accel_y - 1.0) ));
    // uint8_t AZST_OTP =  2620 * (pow( 1.01 , ((double)self_test_accel_z - 1.0) ));


    // double GXST_OTP =  (double)(2620/1<<gyro_old_fs)*(pow( 1.01 , ((double)self_test_gyro_x  - 1.0) ));
    // double GYST_OTP =  (double)(2620/1<<gyro_old_fs)*(pow( 1.01 , ((double)self_test_gyro_y  - 1.0) ));
    // double GZST_OTP =  (double)(2620/1<<gyro_old_fs)*(pow( 1.01 , ((double)self_test_gyro_z  - 1.0) ));
    // double AXST_OTP =  (double)(2620/1<<accel_old_fs)*(pow( 1.01 , ((double)self_test_accel_x - 1.0) ));
    // double AYST_OTP =  (double)(2620/1<<accel_old_fs)*(pow( 1.01 , ((double)self_test_accel_y - 1.0) ));
    // double AZST_OTP =  (double)(2620/1<<accel_old_fs)*(pow( 1.01 , ((double)self_test_accel_z - 1.0) ));



    Serial.println("---OTP----");
    Serial.println(GXST_OTP);
    Serial.println(GYST_OTP);
    Serial.println(GZST_OTP);
    Serial.println(AXST_OTP);
    Serial.println(AYST_OTP);
    Serial.println(AZST_OTP);



    ///// STEP 3.2.3 (a and b) /////
    byte result_gyro_x =  (GXST_OTP != 0.0) ? ((GXST / GXST_OTP) > 0.5) : (abs(GXST) >= 60);
    byte result_gyro_y =  (GYST_OTP != 0.0) ? ((GYST / GYST_OTP) > 0.5) : (abs(GYST) >= 60);
    byte result_gyro_z =  (GZST_OTP != 0.0) ? ((GZST / GZST_OTP) > 0.5) : (abs(GZST) >= 60);
    byte result_accel_x = (AXST_OTP != 0.0) ? ((AXST / AXST_OTP) > 0.5) && ((AXST / AXST_OTP) < 1.5) : (abs(AXST) >= 60);
    byte result_accel_y = (AYST_OTP != 0.0) ? ((AYST / AYST_OTP) > 0.5) && ((AYST / AYST_OTP) < 1.5) : (abs(AYST) >= 60);
    byte result_accel_z = (AZST_OTP != 0.0) ? ((AZST / AZST_OTP) > 0.5) && ((AYST / AYST_OTP) < 1.5) : (abs(AZST) >= 60);


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
    for (i=0; i < NUM_IMUS; i++) {
        pinMode(IMU_SELECT[i], OUTPUT);
        digitalWrite(IMU_SELECT[i], HIGH);
    }
    delay(400);                                    
}

void end_imu_com() {
    SPI.endTransaction();
    SPI.end();
    Serial.flush();
}




void initialize(){

    pinMode(TRIGGER_PIN, INPUT);

    begin_imu_com();

    uint8_t i;
    for (i=0; i < NUM_IMUS; i++) {


        write_register(IMU_SELECT[i], REG_PWR_MGMT_1, 0x81);   // reset mpu and set clock source
        delay(1);

        write_register(IMU_SELECT[i], REG_CONFIG, 0x01);             // DLPF: GYRO BANDWIDTH = 184HZ, TEMP BANDWIDTH = 188HZ
        write_register(IMU_SELECT[i], REG_USER_CTRL, 0x20);          // RESERVED??
        write_register(IMU_SELECT[i], REG_I2C_MST_CTRL, 0x0D);       // SET I2C MASTER CLOCK SPEED TO 400 KHZ

    }
}


void setup() {
    Serial.begin(115200);

    begin_imu_com();
    delay(1000);
    self_test(9);
    end_imu_com();
}

void loop() {
}



