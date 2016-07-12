/////////////////////////////////////////
//                                     //
//             CODE BY                 //
//          David Buckingham           //
//     david.buckingham@tufts.edu      //
//               and                   //
//          Vishesh Vikas              //
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
#include "MPU9250.h"


// EMS PINS
#define PIN_EMS_CLK          5
#define PIN_EMS_DATA         6
#define PIN_EMS_CS           7


// IMU PINS
#define PIN_IMU_CS0          9
#define PIN_IMU_CS1          10


#define SPI_CLOCK 8000000  // 8MHz clock works.

#define COM_FLAG          0x7e

#define SERIAL_BUFF_LENGTH  33       // id = 4 bytes
                                     // encoder = 2 bytes
                                     // gyros = 2 bytes * 3 dims * 2 chips = 12 bytes
                                     // accels = 2 bytes * 3 dims * 2 chips = 12 bytes
                                     // flags = 3 bytes





MPU9250 mpu0(SPI_CLOCK, PIN_IMU_CS0);
MPU9250 mpu1(SPI_CLOCK, PIN_IMU_CS1);


// VARIABLES
byte i;                               // for loops
byte val;                             // for reading data
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



// RX AND TX ALL DATA
void read_sample() {

    buffer_i = 0;
    
    // SIGNAL A NEW SAMPLE
    serial_buffer[buffer_i++] = COM_FLAG;
    serial_buffer[buffer_i++] = COM_FLAG;
    serial_buffer[buffer_i++] = COM_FLAG;


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

	mpu.read_all();

	Serial.print(mpu.gyro_data[0]); 
	Serial.print(mpu.gyro_data[1]); 
	Serial.print(mpu.gyro_data[2]); 
	Serial.print(mpu.accel_data[0]);
	Serial.print(mpu.accel_data[1]);
	Serial.print(mpu.accel_data[2]);
	Serial.print(mpu.mag_data[0]);  
	Serial.print(mpu.mag_data[1]);  
	Serial.print(mpu.mag_data[2]);  
	Serial.println(mpu.temperature);

    // IMU 0 ACCELEROMETER VALUES
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS0, REG_ACCEL_XOUT_H);
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS0, REG_ACCEL_XOUT_L);
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS0, REG_ACCEL_YOUT_H);
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS0, REG_ACCEL_YOUT_L);
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS0, REG_ACCEL_ZOUT_H);
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS0, REG_ACCEL_ZOUT_L);

    // IMU 0 GYROSCOPE VALUES
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS0, REG_GYRO_XOUT_H);
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS0, REG_GYRO_XOUT_L);
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS0, REG_GYRO_YOUT_H);
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS0, REG_GYRO_YOUT_L);
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS0, REG_GYRO_ZOUT_H);
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS0, REG_GYRO_ZOUT_L);

    // IMU 1 ACCELEROMETER VALUES
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS1, REG_ACCEL_XOUT_H);
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS1, REG_ACCEL_XOUT_L);
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS1, REG_ACCEL_YOUT_H);
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS1, REG_ACCEL_YOUT_L);
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS1, REG_ACCEL_ZOUT_H);
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS1, REG_ACCEL_ZOUT_L);

    // IMU 1 GYROSCOPE VALUES
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS1, REG_GYRO_XOUT_H);
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS1, REG_GYRO_XOUT_L);
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS1, REG_GYRO_YOUT_H);
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS1, REG_GYRO_YOUT_L);
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS1, REG_GYRO_ZOUT_H);
    serial_buffer[buffer_i++] = read_imu_register(PIN_IMU_CS1, REG_GYRO_ZOUT_L);


    // SERIAL TX
    Serial.write(serial_buffer, SERIAL_BUFF_LENGTH);
}



void setup() {
    // ENCODER
    pinMode(PIN_EMS_CS, OUTPUT);
    pinMode(PIN_EMS_CLK, OUTPUT);
    pinMode(PIN_EMS_DATA, INPUT);
    digitalWrite(PIN_EMS_CLK, HIGH);
    digitalWrite(PIN_EMS_CS, LOW);

    // IMU
    pinMode(PIN_IMU_CS0, OUTPUT);
    pinMode(PIN_IMU_CS1, OUTPUT);

    // SERIAL
    Serial.begin(BAUD, SERIAL_8N1);  // 8 bits words, no parity, 1 stop bit

    SPI.begin();
	mpu.init(true, true);

    tx(mpu.whoami() == 0x71);
	tx(mpu.AK8963_whoami() == 0x48);

    next_sample_id = 0;

    Timer1.initialize(1000000 / SAMPLE_FREQ_HZ);  // arg in microseconds
    Timer1.attachInterrupt(read_sample);
}


void tx(byte val) {
    buffer_i = 0;
    serial_buffer[buffer_i++] = COM_FLAG;
    serial_buffer[buffer_i++] = val;
    serial_buffer[buffer_i++] = COM_FLAG;
    Serial.write(serial_buffer, 3);
}



// NOTHING HERE. MAIN PROCEDURE IS IN read_sample();
void loop() {
}

