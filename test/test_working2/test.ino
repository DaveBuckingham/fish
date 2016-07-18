#include <SPI.h>

#define SPI_CLOCK 8000000  // 8MHz clock works.

#define SS_PIN   10 
#define INT_PIN  3


#ifndef MPU9250_h
#define MPU9250_h
#include "Arduino.h"


// mpu9250 registers
#define MPUREG_CONFIG 0x1A
#define MPUREG_GYRO_CONFIG 0x1B
#define MPUREG_LP_ACCEL_ODR        0x1E
#define MPUREG_FIFO_EN             0x23
#define MPUREG_I2C_MST_CTRL        0x24
#define MPUREG_I2C_SLV0_ADDR       0x25
#define MPUREG_I2C_SLV0_REG        0x26
#define MPUREG_I2C_SLV0_CTRL       0x27
#define MPUREG_I2C_MST_STATUS      0x36
#define MPUREG_INT_PIN_CFG 0x37
#define MPUREG_INT_ENABLE 0x38
#define MPUREG_EXT_SENS_DATA_00    0x49
#define MPUREG_I2C_SLV0_DO         0x63
#define MPUREG_USER_CTRL 0x6A
#define MPUREG_PWR_MGMT_1 0x6B
#define MPUREG_PWR_MGMT_2 0x6C
#define MPUREG_DMP_CFG_1 0x70
#define MPUREG_DMP_CFG_2 0x71
#define MPUREG_FIFO_COUNTH 0x72
#define MPUREG_FIFO_COUNTL 0x73
#define MPUREG_FIFO_R_W 0x74
#define MPUREG_WHOAMI 0x75
 
#define AK8963_I2C_ADDR             0x0c//0x18
#define AK8963_Device_ID            0x48
 
// Read-only Reg
#define AK8963_HXL                  0x03
#define AK8963_HXH                  0x04
#define AK8963_HYL                  0x05
#define AK8963_HYH                  0x06
#define AK8963_HZL                  0x07
#define AK8963_HZH                  0x08
#define AK8963_ST2                  0x09
// Write/Read Reg
#define AK8963_CNTL1                0x0A
#define AK8963_CNTL2                0x0B
#define AK8963_ASTC                 0x0C
#define AK8963_TS1                  0x0D
#define AK8963_TS2                  0x0E
#define AK8963_I2CDIS               0x0F
// Read-only Reg ( ROM )
#define AK8963_ASAX                 0x10
#define AK8963_ASAY                 0x11
#define AK8963_ASAZ                 0x12
 
// Configuration bits mpu9250
#define BIT_H_RESET 0x80
#define MPU_CLK_SEL_PLLGYROX 0x01
#define MPU_CLK_SEL_PLLGYROZ 0x03
#define MPU_EXT_SYNC_GYROX 0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_DLPF_CFG_188HZ         0x01
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_I2C_IF_DIS              0x10
 
#define READ_FLAG   0x80
 
 
 
#define     Magnetometer_Sensitivity_Scale_Factor ((float)0.15f)    
 

class MPU9250 {   
public:
    // constructor. Default low pass filter of 188Hz
    MPU9250(long clock, uint8_t cs, uint8_t low_pass_filter = BITS_DLPF_CFG_188HZ){
        my_clock = clock;
        my_cs = cs;
        my_low_pass_filter = low_pass_filter;
    }
    unsigned int WriteReg(uint8_t WriteAddr, uint8_t WriteData );
    unsigned int ReadReg(uint8_t WriteAddr, uint8_t WriteData );
    void ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes );
 
    bool init(bool calib_gyro = true, bool calib_acc = true);
    void calib_mag();
    void read_mag();
    void calibrate(float *dest1, float *dest2);
    
    
    int calib_data[3];
    float Magnetometer_ASA[3];
 
    float mag_data[3];
    int16_t mag_data_raw[3];    

    float randomstuff[3];   // seemed to be an issue with memory being disturbed so allocated random memory space here

private:
    long my_clock;
    uint8_t my_cs;
    uint8_t my_low_pass_filter;

};
 
#endif
















unsigned int MPU9250::WriteReg( uint8_t WriteAddr, uint8_t WriteData )
{
    unsigned int temp_val;

    SPI.beginTransaction(SPISettings(my_clock, MSBFIRST, SPI_MODE3));
    digitalWrite(my_cs, LOW);
    SPI.transfer(WriteAddr);
    temp_val=SPI.transfer(WriteData);
    digitalWrite(my_cs, HIGH);
    SPI.endTransaction();

    //delayMicroseconds(50);
    return temp_val;
}
unsigned int  MPU9250::ReadReg( uint8_t WriteAddr, uint8_t WriteData )
{
    return WriteReg(WriteAddr | READ_FLAG,WriteData);
}
void MPU9250::ReadRegs( uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes )
{
    unsigned int  i = 0;

    SPI.beginTransaction(SPISettings(my_clock, MSBFIRST, SPI_MODE3));
    digitalWrite(my_cs, LOW);
    SPI.transfer(ReadAddr | READ_FLAG);
    for(i = 0; i < Bytes; i++)
        ReadBuf[i] = SPI.transfer(0x00);
    digitalWrite(my_cs, HIGH);
    SPI.endTransaction();

    //delayMicroseconds(50);
}


#define MPU_InitRegNum 15

bool MPU9250::init(bool calib_gyro, bool calib_acc){
    pinMode(my_cs, OUTPUT);
    digitalWrite(my_cs, HIGH);

  
    delay(200);                                    
    WriteReg(MPUREG_PWR_MGMT_1, 0x00);   // Turn on internal clock source
    WriteReg(MPUREG_USER_CTRL, 0x0C);    // Reset FIFO and DMP
    delay(15);
  

    
    uint8_t i = 0;
    uint8_t MPU_Init_Data[MPU_InitRegNum][2] = {
        {BIT_H_RESET, MPUREG_PWR_MGMT_1},     // Reset Device
        {0x01, MPUREG_PWR_MGMT_1},     // Clock Source
        {0x00, MPUREG_PWR_MGMT_2},     // Enable Acc & Gyro
        {my_low_pass_filter, MPUREG_CONFIG},         // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
        {0x30, MPUREG_INT_PIN_CFG},    //
        {0x20, MPUREG_USER_CTRL},       // I2C Master mode
        {0x0D, MPUREG_I2C_MST_CTRL}, //  I2C configuration multi-master  IIC 400KHz
        
        {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  //Set the I2C slave addres of AK8963 and set for write.

        {AK8963_CNTL2, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
        {0x01, MPUREG_I2C_SLV0_DO}, // Reset AK8963
        {0x81, MPUREG_I2C_SLV0_CTRL},  //Enable I2C and set 1 byte

        {AK8963_CNTL1, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
        {0x16, MPUREG_I2C_SLV0_DO}, // Register value to 100Hz continuous measurement in 16bit
        {0x81, MPUREG_I2C_SLV0_CTRL}  //Enable I2C and set 1 byte
        
    };

    for(i = 0; i < MPU_InitRegNum; i++) {
        WriteReg(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
        delayMicroseconds(1000);  //I2C must slow down the write speed, otherwise it won't work
    }

    
    return 0;
}






void MPU9250::calib_mag(){
    uint8_t response[3];
    float data;
    int i;

    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_ASAX); //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x83); //Read 3 bytes from the magnetometer

    ReadRegs(MPUREG_EXT_SENS_DATA_00,response,3);
    
    for(i = 0; i < 3; i++) {
        data=response[i];
        Magnetometer_ASA[i] = ((data-128)/256+1)*Magnetometer_Sensitivity_Scale_Factor;
    }
}

void MPU9250::read_mag(){
    uint8_t response[7];
    float data;
    int i;

    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 6 bytes from the magnetometer

    ReadRegs(MPUREG_EXT_SENS_DATA_00,response,7);
    //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.
    for(i = 0; i < 3; i++) {
        mag_data_raw[i] = ((int16_t)response[i*2+1]<<8)|response[i*2];
        data = (float)mag_data_raw[i];
        mag_data[i] = data*Magnetometer_ASA[i];
    }
}



MPU9250 mpu(SPI_CLOCK, SS_PIN);

void setup() {
	Serial.begin(115200);

	pinMode(INT_PIN, INPUT);

	SPI.begin();

	mpu.init(true);

	mpu.calib_mag();

        delay(100);;
}

void loop() {
	mpu.read_mag();

	Serial.print(mpu.mag_data[0]);    Serial.print('\t');
	Serial.print(mpu.mag_data[1]);    Serial.print('\t');
	Serial.println(mpu.mag_data[2]);

	delay(10);
}
