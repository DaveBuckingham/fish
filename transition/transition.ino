#include <SPI.h>
#include "TimerOne.h"

#define SPI_CLOCK               1000000        // 1MHz clock

// const uint8_t IMU_SELECT[]    = {9, 10};
// const uint8_t NUM_IMUS        = 2;

//const uint8_t IMU_SELECT[]    = {9};
const uint8_t IMU_SELECT[]    = {10};
const uint8_t NUM_IMUS        = 1;

// IMU REGISTERS
#define REG_WHO_AM_I            0x75           // 117
#define REG_CONFIG              0x1A
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

#define SAMPLE_FREQ_HZ          10            // 250 seems ok. starts to break around 300.


// COMMUNICATION
#define COM_FLAG                0x7E
#define COM_ESCAPE              0X7D
#define COM_XOR                 0X20


// EMS PINS
#define PIN_EMS_CLK             5
#define PIN_EMS_DATA            6
#define PIN_EMS_CS              7


#define MAG_MEMORY_LIMIT        3


#define I2C_DELAY               10           // ms between i2c txrx


#define SERIAL_BUFF_LENGTH      80             // leave room for byte stuffing
byte serial_buffer[SERIAL_BUFF_LENGTH];        // for framing and byte stuffing for tx
 

int encoder_angle;
unsigned long next_sample_id;            // count sample ids
int whoami;

uint8_t mag_memory_count_0;
uint8_t mag_memory_count_1;

uint8_t mag_memory_0[6];
uint8_t mag_memory_1[6];

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

    //SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    digitalWrite(chip, LOW);
    SPI.transfer(WriteAddr);
    temp_val=SPI.transfer(WriteData);
    digitalWrite(chip, HIGH);

    return temp_val;
    //SPI.endTransaction();
}

byte read_register(byte chip, byte address) {
    //SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    digitalWrite(chip, LOW);
    SPI.transfer(address | READ_FLAG);
    byte data = SPI.transfer(0x00);
    delay(1);
    digitalWrite(chip, HIGH);
    return data;
    //SPI.endTransaction();
}


void read_multiple_registers(byte chip, uint8_t address, uint8_t *buff, unsigned int num_bytes ) {
    unsigned int  i = 0;

    //SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    digitalWrite(chip, LOW);
    SPI.transfer(address | READ_FLAG);
    for(i = 0; i < num_bytes; i++)
        buff[i] = SPI.transfer(0x00);
    digitalWrite(chip, HIGH);
    //SPI.endTransaction();
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






void initialize(){
    next_sample_id = 0;
    mag_memory_count_0 = 0;
    mag_memory_count_1 = 0;

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
        write_register(IMU_SELECT[i], REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG);                                delay(I2C_DELAY);
        write_register(IMU_SELECT[i], REG_I2C_SLV0_REG, MAG_CNTL2);                                       delay(I2C_DELAY);
        write_register(IMU_SELECT[i], REG_I2C_SLV0_DO, 0x01);                                             delay(I2C_DELAY);
        write_register(IMU_SELECT[i], REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);                       delay(I2C_DELAY);

        // SET MAGNETOMETER TO CONTINUOUS MEASUREMENT MODE 2, 100HZ
        write_register(IMU_SELECT[i], REG_I2C_SLV0_REG, MAG_CNTL1);                                       delay(I2C_DELAY);
        write_register(IMU_SELECT[i], REG_I2C_SLV0_DO, 0x16);                                             delay(I2C_DELAY);    // 100hz, 16-bit
//      write_register(IMU_SELECT[i], REG_I2C_SLV0_DO, 0x12);                                             delay(I2C_DELAY);    // 8hz,   16-bit
//      write_register(IMU_SELECT[i], REG_I2C_SLV0_DO, 0x06);                                             delay(I2C_DELAY);    // 100hz, 14-bit
//      write_register(IMU_SELECT[i], REG_I2C_SLV0_DO, 0x02);                                             delay(I2C_DELAY);    // 8hz,   14-bit
//      write_register(IMU_SELECT[i], REG_I2C_SLV0_DO, 0x01);                                             delay(I2C_DELAY);    // single-measurement, 16-bit
        write_register(IMU_SELECT[i], REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);                       delay(I2C_DELAY);

        delay(100);
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
        write_register(IMU_SELECT[i], REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);            delay(I2C_DELAY);
        write_register(IMU_SELECT[i], REG_I2C_SLV0_REG, MAG_ASAX);                                delay(I2C_DELAY);
        write_register(IMU_SELECT[i], REG_I2C_SLV0_CTRL, 0x03 | ENABLE_SLAVE_FLAG);               delay(I2C_DELAY);
        read_multiple_registers(IMU_SELECT[i], REG_EXT_SENS_DATA_00,response + (3*i),3);          delay(I2C_DELAY);
    }

    tx_packet(response, 6);
}



void read_sample(){
    uint8_t resposne_len = 52;
    uint8_t response[resposne_len];
    uint8_t i;
    uint8_t j;



    for (j=0; j < resposne_len; j++) {
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
        write_register(IMU_SELECT[i], REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);    delay(I2C_DELAY);
        write_register(IMU_SELECT[i], REG_I2C_SLV0_REG, 0x00);                            delay(I2C_DELAY);
        write_register(IMU_SELECT[i], REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);       delay(I2C_DELAY);
        response[j++] = read_register(IMU_SELECT[i], REG_EXT_SENS_DATA_00);               delay(I2C_DELAY);

        // DATA READY?
        write_register(IMU_SELECT[i], REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);    delay(I2C_DELAY);
        write_register(IMU_SELECT[i], REG_I2C_SLV0_REG, 0x02);                            delay(I2C_DELAY);
        write_register(IMU_SELECT[i], REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);       delay(I2C_DELAY);
        data_ready = read_register(IMU_SELECT[i], REG_EXT_SENS_DATA_00) & 0x01;           delay(I2C_DELAY);
        response[j++] = data_ready;

        // READ DATA
        //if (data_ready) {
        if (1) {
            write_register(IMU_SELECT[i], REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);    delay(I2C_DELAY);
            write_register(IMU_SELECT[i], REG_I2C_SLV0_REG, 0x03);                            delay(I2C_DELAY);
            write_register(IMU_SELECT[i], REG_I2C_SLV0_CTRL, 0x07 | ENABLE_SLAVE_FLAG);       delay(I2C_DELAY);
            read_multiple_registers(IMU_SELECT[i], REG_EXT_SENS_DATA_00, response + j, 7);    delay(I2C_DELAY);
        }
        j += 7;

    }



    tx_packet(response, resposne_len);

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
                //Timer1.initialize(1000000 / SAMPLE_FREQ_HZ);  // arg in microseconds
                //Timer1.attachInterrupt(read_sample);
                while (1) {
                    read_sample();
                    delay(90);
                }
                break;
            case 's':
                Timer1.detachInterrupt();
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
