/////////////////////////////////////////
//                                     //
//             CODE BY                 //
//          David Buckingham           //
//     david.buckingham@tufts.edu      //
//                                     //
/////////////////////////////////////////


// IMU
// http://store.invensense.com/datasheets/invensense/MPU9250REV1.0.pdf
// https://strawberry-linux.com/pub/RM-MPU-9250A-00.pdf
// SPI mode = 3
// transactions are 16 bits
// first byte is register, second is data
// first bit is high for read, low for write


// EMS
// http://www.mouser.com/ds/2/54/EMS22A-50229.pdf
// The encoder is designed for Daisy chain arrangement.
// Set CS high to read.
// Pin 1 = Digital Input => Not used
// Pin 2 = CLK => PIN_EMS_CLK
// Pin 3 = GND 
// Pin 4 = Digital Output => PIN_EMS_DATA
// Pin 5 = VCC (3.3V)
// Pin 6 = CS => PIN_EMS_CS


// ARDUINO UNO PORTS
// 5  <= EMS CLK
// 6  <= EMS DATA
// 7  <= EMS CS
// 9  <= IMU CS0
// 10 <= IMU CS1
// 11 <= IMU MOSI
// 12 <= IMU MISO
// 13 <= IMU CLK


#include "SPI.h"
#include "TimerOne.h"


// EMS PINS
#define PIN_EMS_CLK          5
#define PIN_EMS_DATA         6
#define PIN_EMS_CS           7


// IMU PINS
#define PIN_IMU_CS0          9
#define PIN_IMU_CS1          10


// IMU REGISTERS
#define REG_USER_CTRL            106
#define REG_WHO_AM_I             117

#define REG_SELF_TEST_X_GYRO     0          // self test output generated during manufacturing tests
#define REG_SELF_TEST_Y_GYRO     1
#define REG_SELF_TEST_Z_GYRO     2
#define REG_SELF_TEST_X_ACCEL    13
#define REG_SELF_TEST_Y_ACCEL    14
#define REG_SELF_TEST_Z_ACCEL    15

#define REG_GYRO_CONFIG          27
#define REG_ACCEL_CONFIG         28

#define REG_ACCEL_FIRST          59
#define REG_ACCEL_XOUT_H         59
#define REG_ACCEL_XOUT_L         60
#define REG_ACCEL_YOUT_H         61
#define REG_ACCEL_YOUT_L         62
#define REG_ACCEL_ZOUT_H         63
#define REG_ACCEL_ZOUT_L         64

#define REG_GYRO_FIRST           67
#define REG_GYRO_XOUT_H          67
#define REG_GYRO_XOUT_L          68
#define REG_GYRO_YOUT_H          69
#define REG_GYRO_YOUT_L          70
#define REG_GYRO_ZOUT_H          71
#define REG_GYRO_ZOUT_L          72


// MAGNETOMETER REGISTERS
#define REG_MAG_WHO_AM_I         0x00         // should return 0x48
#define REG_MAG_INFO             0x01
#define REG_MAG_ST1              0x02         // data ready status bit 0
#define REG_MAG_XOUT_L           0x03         // data
#define REG_MAG_XOUT_H           0x04
#define REG_MAG_YOUT_L           0x05
#define REG_MAG_YOUT_H           0x06
#define REG_MAG_ZOUT_L           0x07
#define REG_MAG_ZOUT_H           0x08
#define REG_MAG_ST2              0x09         // Data overflow bit 3 and data read error status bit 2
#define REG_MAG_CNTL             0x0A         // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define REG_MAG_ASTC             0x0C         // Self test control
#define REG_MAG_I2CDIS           0x0F         // I2C disable
#define REG_MAG_ASAX             0x10         // Fuse ROM x-axis sensitivity adjustment value
#define REG_MAG_ASAY             0x11         // Fuse ROM y-axis sensitivity adjustment value
#define REG_MAG_ASAZ             0x12         // Fuse ROM z-axis sensitivity adjustment value

#define REG_I2C_SLV0_ADDR        0x25         // 0x0C ????????
#define REG_I2C_SLV0_REG         0x26
#define REG_I2C_SLV0_CTRL        0x27
#define REG_EXT_SENS_DATA_00     0x49

#define MAG_I2C_ADDRESS          0x0C




// IMU BYTES
// register 27 - gyroscope configuration
#define XGYRO_Cten           1 << 7   // gyro self test
#define YGYRO_Cten           1 << 6
#define ZGYRO_Cten           1 << 5
#define GYRO_FS_SEL          0        // gyro scale = +250dps

// register 28 - accelerometer configuration
#define ax_st_en             1 << 7   // accel self test
#define ay_st_en             1 << 6
#define az_st_en             1 << 5
#define ACCEL_FS_SEL         0        // accel scale = +-2g

// register 106 USER_CTRL
#define I2C_IF_DIS      0x08         // bit to disable I2C on imu


// OTHER CONSTANTS
#define SERIAL_FLAG                    0X7E
#define SAMPLE_FREQ_HZ                 200          // 250 seems ok. starts to break around 300.
#define BAUD                           115200       // serial com
#define SPI_SPEED_HZ                   1000000      // ms
#define READ_FLAG                      0x80         // first byte of imu tx
#define START_UP_TIME                  50           // typ:11ms max:100ms

#define SERIAL_BUFF_LENGTH             45       // id = 4 bytes
                                                // encoder = 2 bytes
                                                // gyros = 2 bytes * 3 dims * 2 chips = 12 bytes
                                                // accels = 2 bytes * 3 dims * 2 chips = 12 bytes
                                                // mags = 12
                                                // flags = 3 bytes



// VARIABLES
unsigned long next_sample_id;            // count sample ids
byte serial_buffer[SERIAL_BUFF_LENGTH];  // hold data for serial tx
byte buffer_i;                           // index for perparing data to tx
byte i;                                  // general purpose index
int encoder_angle;


// READ FROM EMS
inline void read_encoder() {
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


// WRITE TO IMU
void write_imu_register(byte chip, byte address, byte data) {
    digitalWrite(chip, LOW);
    SPI.transfer(address);
    SPI.transfer(data);
    digitalWrite(chip, HIGH);
}

// READ FROM IMU
inline byte read_imu_register(byte chip, byte address) {
    digitalWrite(chip, LOW);
    SPI.transfer(address | READ_FLAG);
    byte data = SPI.transfer(0x00);
    digitalWrite(chip, HIGH);
    return data;
}

inline void read_multiple_registers(byte chip, byte address, byte *buffer, unsigned int num_bytes) {
    i = 0;
    digitalWrite(chip, LOW);
    SPI.transfer(address | READ_FLAG);
    for(i = 0; i < num_bytes; i++) {
        buffer[i] = SPI.transfer(0x00);
    }
    digitalWrite(chip, HIGH);
}




byte imu_test() {

}


// RX AND TX ALL DATA
void read_sample() {

    buffer_i = 0;
    
    // SIGNAL A NEW SAMPLE
    serial_buffer[buffer_i++] = SERIAL_FLAG;
    serial_buffer[buffer_i++] = 42;             // packet size

    // SAMPLE ID
    serial_buffer[buffer_i++] = next_sample_id >> 24;
    serial_buffer[buffer_i++] = next_sample_id >> 16;
    serial_buffer[buffer_i++] = next_sample_id >> 8;
    serial_buffer[buffer_i++] = next_sample_id;
    next_sample_id++;

    // ENCODER VALUE
    read_encoder();
    serial_buffer[buffer_i++] = encoder_angle >> 8;
    serial_buffer[buffer_i++] = encoder_angle;

    // IMU 0 ACCELEROMETER VALUES
    read_multiple_registers(PIN_IMU_CS0, REG_ACCEL_FIRST, serial_buffer + buffer_i, 6);
    buffer_i += 6;

    // IMU 0 GYROSCOPE VALUES
    read_multiple_registers(PIN_IMU_CS0, REG_GYRO_FIRST, serial_buffer + buffer_i, 6);
    buffer_i += 6;

    // IMU 0 MAG VALUES
    write_imu_register(PIN_IMU_CS0, REG_I2C_SLV0_REG,  REG_MAG_XOUT_L);     //I2C slave 0 register address to read data.  MOVE TO SETUP ???
    write_imu_register(PIN_IMU_CS0, REG_I2C_SLV0_CTRL, 0x07 | READ_FLAG);
    read_multiple_registers(PIN_IMU_CS0, REG_GYRO_FIRST, serial_buffer + buffer_i, 7);
    buffer_i += 6;

    // IMU 1 ACCELEROMETER VALUES
    read_multiple_registers(PIN_IMU_CS1, REG_ACCEL_FIRST, serial_buffer + buffer_i, 6);
    buffer_i += 6;

    // IMU 1 GYROSCOPE VALUES
    read_multiple_registers(PIN_IMU_CS1, REG_GYRO_FIRST, serial_buffer + buffer_i, 6);
    buffer_i += 6;

    // IMU 1 MAG VALUES
    write_imu_register(PIN_IMU_CS1, REG_I2C_SLV0_REG,  REG_MAG_XOUT_L);     //I2C slave 0 register address to read data.  MOVE TO SETUP ???
    write_imu_register(PIN_IMU_CS1, REG_I2C_SLV0_CTRL, 0x07 | READ_FLAG);
    read_multiple_registers(PIN_IMU_CS1, REG_GYRO_FIRST, serial_buffer + buffer_i, 7);
    buffer_i += 6;

    serial_buffer[buffer_i++] = SERIAL_FLAG;

    // SERIAL TX
    Serial.write(serial_buffer, buffer_i);
}


// READ AND TX MAGNETOMETER SENSITIVITY ADJUSTMENT
void tx_asa() {
    write_imu_register(PIN_IMU_CS0, REG_I2C_SLV0_ADDR, MAG_I2C_ADDRESS | READ_FLAG); //SET THE I2C SLAVE ADDRES OF AK8963 AND SET FOR READ.
    write_imu_register(PIN_IMU_CS0, REG_I2C_SLV0_REG,  REG_MAG_ASAX);                 //I2C SLAVE 0 REGISTER ADDRESS FROM WHERE TO READ ASA
    write_imu_register(PIN_IMU_CS0, REG_I2C_SLV0_CTRL, 0x03 | READ_FLAG);            //READ 3 BYTES

    write_imu_register(PIN_IMU_CS1, REG_I2C_SLV0_ADDR, MAG_I2C_ADDRESS | READ_FLAG);
    write_imu_register(PIN_IMU_CS1, REG_I2C_SLV0_REG,  REG_MAG_ASAX);
    write_imu_register(PIN_IMU_CS1, REG_I2C_SLV0_CTRL, 0x03 | READ_FLAG);

    buffer_i = 0;
    serial_buffer[buffer_i++] = SERIAL_FLAG;
    serial_buffer[buffer_i++] = 6;

    read_multiple_registers(PIN_IMU_CS0, REG_EXT_SENS_DATA_00, serial_buffer + buffer_i, 3);
    buffer_i += 3;

    read_multiple_registers(PIN_IMU_CS1, REG_EXT_SENS_DATA_00, serial_buffer + buffer_i, 3);
    buffer_i += 3;

    serial_buffer[buffer_i++] = SERIAL_FLAG;
    Serial.write(serial_buffer, buffer_i);
}


// INITIALIZE
void setup() {

    next_sample_id = 0;
    Serial.begin(BAUD, SERIAL_8N1);  // 8 bits words, no parity, 1 stop bit

    // ENCODER
    pinMode(PIN_EMS_CS, OUTPUT);
    pinMode(PIN_EMS_CLK, OUTPUT);
    pinMode(PIN_EMS_DATA, INPUT);
    digitalWrite(PIN_EMS_CLK, HIGH);
    digitalWrite(PIN_EMS_CS, LOW);

    // IMU
    pinMode(PIN_IMU_CS0, OUTPUT);
    pinMode(PIN_IMU_CS1, OUTPUT);
    SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_SPEED_HZ, MSBFIRST, SPI_MODE3));
    delay(START_UP_TIME);
    write_imu_register(PIN_IMU_CS0, REG_USER_CTRL,    I2C_IF_DIS);            // PREVENT SWITCHING TO I2C
    write_imu_register(PIN_IMU_CS1, REG_USER_CTRL,    I2C_IF_DIS);
    write_imu_register(PIN_IMU_CS0, REG_GYRO_CONFIG,  GYRO_FS_SEL);           // SET GYRO RANGE
    write_imu_register(PIN_IMU_CS1, REG_GYRO_CONFIG,  GYRO_FS_SEL);
    write_imu_register(PIN_IMU_CS0, REG_ACCEL_CONFIG, ACCEL_FS_SEL);          // SET ACCEL RANGE
    write_imu_register(PIN_IMU_CS1, REG_ACCEL_CONFIG, ACCEL_FS_SEL);

    //imu_test();
    tx_asa();

    // TIMER
    Timer1.initialize(1000000 / SAMPLE_FREQ_HZ);  // arg in microseconds
    Timer1.attachInterrupt(read_sample);
}



// NOTHING HERE. MAIN PROCEDURE IS IN read_sample();
void loop() {
}
