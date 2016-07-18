#include <SPI.h>

#define SPI_CLOCK 8000000  // 8MHz clock works.

#define SS_PIN   10 
#define INT_PIN  3




// mpu9250 registers
#define MPUREG_CONFIG 0x1A
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
#define MPUREG_FIFO_COUNTH 0x72
#define MPUREG_FIFO_COUNTL 0x73
#define MPUREG_FIFO_R_W 0x74
 
#define AK8963_I2C_ADDR             0x0c//0x18
 
// Read-only Reg
#define AK8963_HXL                  0x03
#define AK8963_HXH                  0x04
#define AK8963_HYL                  0x05
#define AK8963_HYH                  0x06
#define AK8963_HZL                  0x07
#define AK8963_HZH                  0x08
// Write/Read Reg
#define AK8963_CNTL1                0x0A
#define AK8963_CNTL2                0x0B

// Read-only Reg ( ROM )
#define AK8963_ASAX                 0x10
 
// Configuration bits mpu9250
#define BIT_H_RESET 0x80
#define BITS_FS_250DPS              0x00
#define BITS_FS_2G                  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BIT_I2C_IF_DIS              0x10
 
#define READ_FLAG   0x80
 
 
 
#define     Magnetometer_Sensitivity_Scale_Factor ((float)0.15f)    
 

class MPU9250 {   
public:
    MPU9250(){
    }

    unsigned int WriteReg(uint8_t WriteAddr, uint8_t WriteData );
    unsigned int ReadReg(uint8_t WriteAddr, uint8_t WriteData );
    void ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes );
 
    bool init();
    void calib_mag();
    void read_mag();
    
    
    int calib_data[3];
    float Magnetometer_ASA[3];
 
    float mag_data[3];
    int16_t mag_data_raw[3];    


private:
    uint8_t my_cs;

};
 








unsigned int MPU9250::WriteReg( uint8_t WriteAddr, uint8_t WriteData ) {
    unsigned int temp_val;

    SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    digitalWrite(SS_PIN, LOW);
    SPI.transfer(WriteAddr);
    temp_val=SPI.transfer(WriteData);
    digitalWrite(SS_PIN, HIGH);
    SPI.endTransaction();

    return temp_val;
}
unsigned int  MPU9250::ReadReg( uint8_t WriteAddr, uint8_t WriteData ) {
    return WriteReg(WriteAddr | READ_FLAG,WriteData);
}

void MPU9250::ReadRegs( uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes ) {
    unsigned int  i = 0;

    SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    digitalWrite(SS_PIN, LOW);
    SPI.transfer(ReadAddr | READ_FLAG);
    for(i = 0; i < Bytes; i++)
        ReadBuf[i] = SPI.transfer(0x00);
    digitalWrite(SS_PIN, HIGH);
    SPI.endTransaction();

}


#define MPU_InitRegNum 15

bool MPU9250::init(){
    pinMode(SS_PIN, OUTPUT);
    digitalWrite(SS_PIN, HIGH);

  
    delay(200);                                    
    WriteReg(MPUREG_USER_CTRL, 0x0C);    // Reset FIFO and DMP


  
    
        WriteReg( MPUREG_PWR_MGMT_1    ,  BIT_H_RESET         );     // Reset Device
        delayMicroseconds(1000);
        WriteReg( MPUREG_PWR_MGMT_1    ,  0x01                );     // Clock Source
        delayMicroseconds(1000);
        WriteReg( MPUREG_PWR_MGMT_2    ,  0x00                );     // Enable Acc & Gyro
        delayMicroseconds(1000);
        WriteReg( MPUREG_CONFIG        ,  BITS_DLPF_CFG_188HZ );         // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
        delayMicroseconds(1000);
        WriteReg( MPUREG_INT_PIN_CFG   ,  0x30                );    //
        delayMicroseconds(1000);
        WriteReg( MPUREG_USER_CTRL     ,  0x20                );       // I2C Master mode
        delayMicroseconds(1000);
        WriteReg( MPUREG_I2C_MST_CTRL  ,  0x0D                );   //  I2C configuration multi-master  IIC 400KHz
        delayMicroseconds(1000);
        WriteReg( MPUREG_I2C_SLV0_ADDR ,  AK8963_I2C_ADDR     );  //Set the I2C slave addres of AK8963 and set for write.
        delayMicroseconds(1000);
        WriteReg( MPUREG_I2C_SLV0_REG  ,  AK8963_CNTL2        );  //I2C slave 0 register address from where to begin data transfer
        delayMicroseconds(1000);
        WriteReg( MPUREG_I2C_SLV0_DO   ,  0x01                );   // Reset AK8963
        delayMicroseconds(1000);
        WriteReg( MPUREG_I2C_SLV0_CTRL ,  0x81                ); //Enable I2C and set 1 byte
        delayMicroseconds(1000);
        WriteReg( MPUREG_I2C_SLV0_REG  ,  AK8963_CNTL1        );  //I2C slave 0 register address from where to begin data transfer
        delayMicroseconds(1000);
        WriteReg( MPUREG_I2C_SLV0_DO   ,  0x16                );   // Register value to 100Hz continuous measurement in 16bit
        delayMicroseconds(1000);
        WriteReg( MPUREG_I2C_SLV0_CTRL ,  0x81                ); //Enable I2C and set 1 byte
        delayMicroseconds(1000);
        

    
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
    for(i = 0; i < 3; i++) {
        mag_data_raw[i] = ((int16_t)response[i*2+1]<<8)|response[i*2];
        data = (float)mag_data_raw[i];
        mag_data[i] = data*Magnetometer_ASA[i];
    }
}


MPU9250 mpu;

void setup() {
	Serial.begin(115200);

	pinMode(INT_PIN, INPUT);

	SPI.begin();

	mpu.init();

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
