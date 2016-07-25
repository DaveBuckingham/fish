#include <SPI.h>
#include "TimerOne.h"

#define SPI_CLOCK               8000000  // 8MHz clock

#define SS_PIN                  10
#define PIN_IMU_CS0             9
#define PIN_IMU_CS1             10

// IMU REGISTERS
#define REG_WHO_AM_I            0x75         // 117
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
#define I2C_ADDRESS_MAG         0x0c

// MAG REGISTERS
#define MAG_HXL                 0x03
#define MAG_CNTL1               0x0A
#define MAG_CNTL2               0x0B
#define MAG_ASAX                0x10

#define ENABLE_SLAVE_FLAG       0x80  // use when specifying data length to SLV0_CTRL
#define READ_FLAG               0x80
 
#define MAG_SENSITIVITY_SCALE   0.15

#define COM_FLAG                0x7E
#define COM_ESCAPE              0X7D
#define COM_XOR                 0X20

#define SAMPLE_FREQ_HZ          100          // 250 seems ok. starts to break around 300.
 

float magnetometer_asa[3];
float mag_data[3];
int16_t mag_data_raw[3];    
int whoami;


#define SERIAL_BUFF_LENGTH             80     // leave room for byte stuffing
byte serial_buffer[SERIAL_BUFF_LENGTH];  // for framing and byte stuffing for tx


void tx_packet(byte *in_buffer, unsigned int num_bytes) {
    byte val;
    byte i;
    byte j = 0;
    serial_buffer[j++] = COM_FLAG;
    for (i = 0; i < num_bytes; i++) {
        if (j >= SERIAL_BUFF_LENGTH - 1) {
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

byte read_register(byte chip, byte address) {
    digitalWrite(chip, LOW);
    SPI.transfer(address | READ_FLAG);
    byte data = SPI.transfer(0x00);
    delay(1);
    digitalWrite(chip, HIGH);
    return data;
}



unsigned int write_register(byte chip, uint8_t WriteAddr, uint8_t WriteData ) {
    unsigned int temp_val;

    digitalWrite(chip, LOW);
    SPI.transfer(WriteAddr);
    temp_val=SPI.transfer(WriteData);
    digitalWrite(chip, HIGH);

    return temp_val;
}


void write_register_2(byte address, byte data) {
    //write_register(PIN_IMU_CS0, address, data);
    write_register(PIN_IMU_CS1, address, data);
}

void read_multiple_registers(byte chip, byte ReadAddr, byte *ReadBuf, unsigned int Bytes ) {
    unsigned int  i = 0;

    digitalWrite(chip, LOW);
    SPI.transfer(ReadAddr | READ_FLAG);
    for(i = 0; i < Bytes; i++)
        ReadBuf[i] = SPI.transfer(0x00);
    digitalWrite(chip, HIGH);

}


void initialize(){
	SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));

    pinMode(PIN_IMU_CS0, OUTPUT);
    digitalWrite(PIN_IMU_CS0, HIGH);
    pinMode(PIN_IMU_CS1, OUTPUT);
    digitalWrite(PIN_IMU_CS1, HIGH);

  
    delay(200);                                    
  
    write_register_2(REG_PWR_MGMT_1, 0x81);   // reset mpu and set clock source
    delay(1);

    // DLPF: GYRO BANDWIDTH = 184HZ, TEMP BANDWIDTH = 188HZ
    // MAG DOESN'T WORK WITHOUT THIS, NOT SURE WHY...
    write_register_2(REG_CONFIG, 0x01);
    write_register_2(REG_USER_CTRL, 0x20);
    write_register_2(REG_I2C_MST_CTRL, 0x0D);

    // SET MAGNETOMETER I2C ADDRESS
    write_register_2( REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG);           delay(1); // DELAYS FOR SLOW I2C

    // SOFT RESET MAGNETOMETER
    write_register_2(REG_I2C_SLV0_REG, MAG_CNTL2);                     delay(1);
    write_register_2(REG_I2C_SLV0_DO, 0x01);                           delay(1);
    write_register_2(REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);     delay(1);

    // SET MAGNETOMETER TO CONTINUOUS MEASUREMENT MODE 2 AND 100HZ
    write_register_2(REG_I2C_SLV0_REG, MAG_CNTL1);                     delay(1);
    write_register_2(REG_I2C_SLV0_DO, 0x16);                           delay(1);
    write_register_2(REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);     delay(1);
}


void calib_mag(){
    uint8_t response[3];
    float data;
    int i;

    // READ 3 BYTES FROM MAGNETOMETERS
    write_register_2(REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);
    write_register_2(REG_I2C_SLV0_REG, MAG_ASAX);
    write_register_2(REG_I2C_SLV0_CTRL, 0x03 | ENABLE_SLAVE_FLAG);

    read_multiple_registers(PIN_IMU_CS1, REG_EXT_SENS_DATA_00,response,3);
    
    for(i = 0; i < 3; i++) {
        data=response[i];
        magnetometer_asa[i] = (((data - 128) / 256) + 1) * MAG_SENSITIVITY_SCALE;
    }
}

void tx_asa(){
    byte response[6];
    float data;
    int i;

    // READ 3 BYTES FROM MAGNETOMETERS
    write_register(PIN_IMU_CS0, REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);
    write_register(PIN_IMU_CS0, REG_I2C_SLV0_REG, MAG_ASAX);
    write_register(PIN_IMU_CS0, REG_I2C_SLV0_CTRL, 0x03 | ENABLE_SLAVE_FLAG);
    read_multiple_registers(PIN_IMU_CS0, REG_EXT_SENS_DATA_00, response, 3);

    // READ 3 BYTES FROM MAGNETOMETERS
    write_register(PIN_IMU_CS1, REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);
    write_register(PIN_IMU_CS1, REG_I2C_SLV0_REG, MAG_ASAX);
    write_register(PIN_IMU_CS1, REG_I2C_SLV0_CTRL, 0x03 | ENABLE_SLAVE_FLAG);
    read_multiple_registers(PIN_IMU_CS1, REG_EXT_SENS_DATA_00, response + 3, 3);

    tx_packet(response, 6);

}




void read_mag(){
    uint8_t response[43];
    float data;
    int i;

    for (i=0; i < 43; i++) {
        response[i] = 0;
    }

    // NEED TO GET 7 BYTES TO ALSO READ ST2 REGISTER
    write_register(PIN_IMU_CS1, REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);
    write_register(PIN_IMU_CS1, REG_I2C_SLV0_REG, MAG_HXL);
    write_register(PIN_IMU_CS1, REG_I2C_SLV0_CTRL, 0x07 | ENABLE_SLAVE_FLAG);

    read_multiple_registers(PIN_IMU_CS1, REG_EXT_SENS_DATA_00, response + 18, 7);
    response[24] = 0;

    tx_packet(response, 42);

}


void imu_whoami() {
    uint8_t response[2];
    response[0] = read_register(PIN_IMU_CS0, REG_WHO_AM_I);
    response[1] = read_register(PIN_IMU_CS1, REG_WHO_AM_I);
    tx_packet(response, 2);
}


void setup() {
	Serial.begin(115200);
    //get_whoami();
	//calib_mag();
    delay(100);;
}

void loop() {
    if (Serial.available() > 0) {
        switch (Serial.read()) {
            case 'b':
                break;
            case 'r':
                Timer1.initialize(1000000 / SAMPLE_FREQ_HZ);  // arg in microseconds
                Timer1.attachInterrupt(read_mag);
                break;
            case 's':
                Timer1.detachInterrupt();
                SPI.endTransaction();
                SPI.end();
                break;
            case 'i':
                initialize();
                break;
            case 'm':
                tx_asa();
                break;
            case 'w':
                imu_whoami();
                break;
            case 't':
                // imu self tests
                break;
            default:
                break;
        }
    }
}
