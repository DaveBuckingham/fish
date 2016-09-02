#include <SPI.h>

#define SPI_CLOCK               1000000  // 8MHz clock

#define SS_PIN                  4

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
 

float magnetometer_asa[3];
float mag_data[3];
int16_t mag_data_raw[3];    
uint8_t mag_id;
uint8_t status_2;
int whoami;


unsigned int write_register( uint8_t WriteAddr, uint8_t WriteData ) {
    unsigned int temp_val;

    digitalWrite(SS_PIN, LOW);
    SPI.transfer(WriteAddr);
    temp_val=SPI.transfer(WriteData);
    digitalWrite(SS_PIN, HIGH);

    return temp_val;
}

void read_registers( uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes ) {
    unsigned int  i = 0;

    digitalWrite(SS_PIN, LOW);
    SPI.transfer(ReadAddr | READ_FLAG);
    for(i = 0; i < Bytes; i++)
        ReadBuf[i] = SPI.transfer(0x00);
    digitalWrite(SS_PIN, HIGH);

}

byte read_register(byte address) {
    digitalWrite(SS_PIN, LOW);
    SPI.transfer(address | READ_FLAG);
    byte data = SPI.transfer(0x00);
    delay(1);
    digitalWrite(SS_PIN, HIGH);
    return data;
}




void initialize(){
    pinMode(SS_PIN, OUTPUT);

    digitalWrite(SS_PIN, HIGH);

  
    delay(200);                                    
  
    write_register(REG_PWR_MGMT_1, 0x81);   // reset mpu and set clock source
    delay(10);

    write_register(REG_CONFIG, 0x01);                           delay(10);               // DLPF: GYRO BANDWIDTH = 184HZ, TEMP BANDWIDTH = 188HZ
    write_register(REG_USER_CTRL, 0x20);                        delay(10);               // RESERVED BIT...WTF
    write_register(REG_I2C_MST_CTRL, 0x0D);                     delay(10);               // SET I2C MASTER CLOCK SPEED TO 400 KHZ

    // SET MAGNETOMETER I2C ADDRESS
    write_register( REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG);           delay(10); // DELAYS FOR SLOW I2C

    // SOFT RESET MAGNETOMETER
    write_register(REG_I2C_SLV0_REG, MAG_CNTL2);                     delay(10);
    write_register(REG_I2C_SLV0_DO, 0x01);                           delay(10);
    write_register(REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);     delay(10);

    // SET MAGNETOMETER TO CONTINUOUS MEASUREMENT MODE 2 AND 100HZ
    write_register(REG_I2C_SLV0_REG, MAG_CNTL1);                     delay(10);
    write_register(REG_I2C_SLV0_DO, 0x16);                           delay(10);
    //write_register(REG_I2C_SLV0_DO, 0x06);                           delay(10);
    write_register(REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);     delay(10);
}


void calib_mag(){
    uint8_t response[3];
    float data;
    int i;

    // READ 3 BYTES FROM MAGNETOMETERS
    write_register(REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG); delay(10);
    write_register(REG_I2C_SLV0_REG, MAG_ASAX);                     delay(10);
    write_register(REG_I2C_SLV0_CTRL, 0x03 | ENABLE_SLAVE_FLAG);    delay(10);

    read_registers(REG_EXT_SENS_DATA_00,response,3);                delay(10);
    
    for(i = 0; i < 3; i++) {
        data=response[i];
        magnetometer_asa[i] = (((data - 128) / 256) + 1) * MAG_SENSITIVITY_SCALE;
    }
}

void read_mag(){
    uint8_t response[7];
    float data;
    int i;

    // MAG DEVICE ID
    write_register(REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG); delay(10);
    write_register(REG_I2C_SLV0_REG, 0x00);                         delay(10);
    write_register(REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);    delay(10);
    mag_id = read_register(REG_EXT_SENS_DATA_00);                   delay(10);


    // NEED TO GET 7 BYTES TO ALSO READ ST2 REGISTER
    write_register(REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG); delay(10);
    write_register(REG_I2C_SLV0_REG, MAG_HXL);                      delay(10);
    write_register(REG_I2C_SLV0_CTRL, 0x07 | ENABLE_SLAVE_FLAG);    delay(10);

    read_registers(REG_EXT_SENS_DATA_00, response, 7);              delay(10);
    for(i = 0; i < 3; i++) {
        mag_data[i] = ((int16_t)response[i*2+1]<<8) | response[i*2];
    }
    status_2 = response[6];
}

void get_whoami() {
    uint8_t response[1];
    read_registers(REG_WHO_AM_I, response, 1);    delay(10);
    whoami = (int)response[0];
}

void setup() {
	Serial.begin(115200);
	SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    initialize();
    get_whoami();
	calib_mag();
    delay(10);;
}

void loop() {
	read_mag();
    Serial.print(whoami);
    Serial.print('\t');
    Serial.print(mag_id);
    Serial.print('\t');
    Serial.print(status_2);
    Serial.print('\t');
    Serial.print(magnetometer_asa[0]);
    Serial.print('\t');
    Serial.print(magnetometer_asa[1]);
    Serial.print('\t');
    Serial.print(magnetometer_asa[2]);
    Serial.print('\t');
	Serial.print(mag_data[0]);
    Serial.print('\t');
	Serial.print(mag_data[1]);
    Serial.print('\t');
	Serial.println(mag_data[2]);
	delay(10);
}
