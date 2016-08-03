#include <SPI.h>
#include "TimerOne.h"

#define SPI_CLOCK               8000000        // 8MHz clock

#define PIN_IMU_CS0             9
#define PIN_IMU_CS1             10

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

#define SAMPLE_FREQ_HZ          50            // 250 seems ok. starts to break around 300.


// COMMUNICATION
#define COM_FLAG                0x7E
#define COM_ESCAPE              0X7D
#define COM_XOR                 0X20


// EMS PINS
#define PIN_EMS_CLK             5
#define PIN_EMS_DATA            6
#define PIN_EMS_CS              7


#define MAG_MEMORY_LIMIT        3


#define SERIAL_BUFF_LENGTH      80             // leave room for byte stuffing
byte serial_buffer[SERIAL_BUFF_LENGTH];        // for framing and byte stuffing for tx
 

int encoder_angle;
unsigned long next_sample_id;            // count sample ids
int whoami;

uint8_t mag_memory_count_0;
uint8_t mag_memory_count_1;

uint8_t mag_memory_0[6];
uint8_t mag_memory_1[6];




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

void write_register_2(uint8_t address, uint8_t data) {
    write_register(PIN_IMU_CS0, address, data);
    write_register(PIN_IMU_CS1, address, data);
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

    pinMode(PIN_IMU_CS0, OUTPUT);
    digitalWrite(PIN_IMU_CS0, HIGH);
    pinMode(PIN_IMU_CS1, OUTPUT);
    digitalWrite(PIN_IMU_CS1, HIGH);
  
    delay(200);                                    
  
    write_register_2(REG_PWR_MGMT_1, 0x81);   // reset mpu and set clock source
    delay(1);

    write_register_2(REG_CONFIG, 0x01);             // DLPF: GYRO BANDWIDTH = 184HZ, TEMP BANDWIDTH = 188HZ
    write_register_2(REG_USER_CTRL, 0x20);          // RESERVED!
    write_register_2(REG_I2C_MST_CTRL, 0x0D);       // SET I2C MASTER CLOCK SPEED TO 400 KHZ

    reset_mag(PIN_IMU_CS0);
    reset_mag(PIN_IMU_CS1);
}

void reset_mag(byte chip) {
    // SET MAGNETOMETER I2C ADDRESS
    write_register(chip, REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG);              delay(10); // DELAYS FOR SLOW I2C

    // SOFT RESET MAGNETOMETER
    write_register(chip, REG_I2C_SLV0_REG, MAG_CNTL2);                     delay(10);
    write_register(chip, REG_I2C_SLV0_DO, 0x01);                           delay(10);
    write_register(chip, REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);     delay(10);

    // SET MAGNETOMETER TO CONTINUOUS MEASUREMENT MODE 2, 100HZ
    write_register(chip, REG_I2C_SLV0_REG, MAG_CNTL1);                     delay(10);
    write_register(chip, REG_I2C_SLV0_DO, 0x16);                           delay(10);
    //write_register(chip, REG_I2C_SLV0_DO, 0x06);                           delay(10);
    write_register(chip, REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);     delay(10);

    delay(100);
}


void tx_asa(){
    int i;
    uint8_t response[6];
    for (i=0; i < 6; i++) {
        response[i] = 0;
    }
    float data;

    // READ 3 BYTES FROM MAGNETOMETERS
    write_register_2(REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);               delay(10);
    write_register_2(REG_I2C_SLV0_REG, MAG_ASAX);                                   delay(10);
    write_register_2(REG_I2C_SLV0_CTRL, 0x03 | ENABLE_SLAVE_FLAG);                  delay(10);
    read_multiple_registers(PIN_IMU_CS1, REG_EXT_SENS_DATA_00,response,3);          delay(10);

    delay(1);

    write_register_2(REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);               delay(10);
    write_register_2(REG_I2C_SLV0_REG, MAG_ASAX);                                   delay(10);
    write_register_2(REG_I2C_SLV0_CTRL, 0x03 | ENABLE_SLAVE_FLAG);                  delay(10);
    read_multiple_registers(PIN_IMU_CS1, REG_EXT_SENS_DATA_00,response + 3,3);      delay(10);
    
    tx_packet(response, 6);
}



void read_sample(){
    uint8_t response[52];
    uint8_t i;
    uint8_t j;

    while(1) {


        for (i=0; i < 52; i++) {
            response[i] = 0;
        }


        i = 0;
        
        // SAMPLE ID
        response[i++] = next_sample_id >> 24;
        response[i++] = next_sample_id >> 16;
        response[i++] = next_sample_id >> 8;
        response[i++] = next_sample_id;
        next_sample_id++;

        // ENCODER VALUE
        read_encoder();
        response[i++] = encoder_angle >> 8;
        response[i++] = encoder_angle;

//i += 21;

        read_multiple_registers(PIN_IMU_CS0, REG_TEMP_OUT_H, response + i, 2);
        i += 2;
        read_multiple_registers(PIN_IMU_CS0, REG_ACCEL_FIRST, response + i, 6);
        i += 6;
        read_multiple_registers(PIN_IMU_CS0, REG_GYRO_FIRST, response + i, 6);
        i += 6;

        // MAG DEVICE ID
        write_register(PIN_IMU_CS0, REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);    delay(10);
        write_register(PIN_IMU_CS0, REG_I2C_SLV0_REG, 0x00);                            delay(10);
        write_register(PIN_IMU_CS0, REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);       delay(10);
        response[i++] = read_register(PIN_IMU_CS0, REG_EXT_SENS_DATA_00);               delay(10);


        write_register(PIN_IMU_CS1, REG_I2C_SLV0_ADDR, 0x02 | READ_FLAG);                 delay(10);
        //write_register(PIN_IMU_CS0, REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);    delay(10);
        write_register(PIN_IMU_CS0, REG_I2C_SLV0_REG, MAG_HXL);                         delay(10);
        write_register(PIN_IMU_CS0, REG_I2C_SLV0_CTRL, 0x07 | ENABLE_SLAVE_FLAG);       delay(10);
        read_multiple_registers(PIN_IMU_CS0, REG_EXT_SENS_DATA_00, response + i, 8);           delay(10);
        i += 8;



        read_multiple_registers(PIN_IMU_CS1, REG_TEMP_OUT_H, response + i, 2);
        i += 2;

        // IMU 1 ACCELEROMETER VALUES
        read_multiple_registers(PIN_IMU_CS1, REG_ACCEL_FIRST, response + i, 6);
        i += 6;

        // IMU 1 GYROSCOPE VALUES
        read_multiple_registers(PIN_IMU_CS1, REG_GYRO_FIRST, response + i, 6);
        i += 6;

        // MAG DEVICE ID
        write_register(PIN_IMU_CS1, REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);    delay(10);
        write_register(PIN_IMU_CS1, REG_I2C_SLV0_REG, 0x00);                            delay(10);
        write_register(PIN_IMU_CS1, REG_I2C_SLV0_CTRL, 0x01 | ENABLE_SLAVE_FLAG);       delay(10);
        response[i++] = read_register(PIN_IMU_CS1, REG_EXT_SENS_DATA_00);               delay(10);
                                                                                                   
                                                                                                   
        // IMU 1 MAGNETOMETER                                                           
        // NEED TO GET 7 BYTES TO ALSO READ ST2 REGISTER                                
        write_register(PIN_IMU_CS1, REG_I2C_SLV0_ADDR, 0x02 | READ_FLAG);                 delay(10);
        //write_register(PIN_IMU_CS1, REG_I2C_SLV0_ADDR, I2C_ADDRESS_MAG | READ_FLAG);    delay(10);
        write_register(PIN_IMU_CS1, REG_I2C_SLV0_REG, MAG_HXL);                         delay(10);
        write_register(PIN_IMU_CS1, REG_I2C_SLV0_CTRL, 0x07 | ENABLE_SLAVE_FLAG);       delay(10);
        read_multiple_registers(PIN_IMU_CS1, REG_EXT_SENS_DATA_00, response + i, 8);           delay(10);
        i += 8;


        tx_packet(response, i);

        delay(10);
    }
}





void imu_whoami() {
    uint8_t response[2];
    delay(100);
    response[0] = read_register(PIN_IMU_CS1, REG_WHO_AM_I);
    delay(100);
    response[1] = read_register(PIN_IMU_CS1, REG_WHO_AM_I);
    tx_packet(response, 2);
    delay(100);
}

void setup() {
	Serial.begin(115200);
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
                read_sample();
                break;
            case 's':
                Timer1.detachInterrupt();
                SPI.endTransaction();
                SPI.end();
                break;
            case 't':
                // imu self tests
                break;
            default:
                break;
        }
    }
}
