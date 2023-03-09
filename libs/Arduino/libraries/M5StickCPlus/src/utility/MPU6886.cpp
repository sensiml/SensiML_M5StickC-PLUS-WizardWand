// Modified MPU6886 library to include support for FIFO buffer operation
// C. Rogers - Jan 31 2023

#include "MPU6886.h"
#include <math.h>
#include <Arduino.h>

MPU6886::MPU6886(){

}

void MPU6886::I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *read_Buffer){
    
  Wire1.beginTransmission(driver_Addr);
  Wire1.write(start_Addr);  
  Wire1.endTransmission(false);
  uint8_t i = 0;
  Wire1.requestFrom(driver_Addr,number_Bytes);
  
  //! Put read results in the Rx buffer
  while (Wire1.available()) {
    read_Buffer[i++] = Wire1.read();
  }        
}

void MPU6886::I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer){

  Wire1.beginTransmission(driver_Addr);
  Wire1.write(start_Addr);
  Wire1.write(*write_Buffer);
  Wire1.endTransmission();

}

int MPU6886::Init(void){
  unsigned char tempdata[1];
  unsigned char regdata;
  
  Wire1.begin(21,22);
  
  I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_WHOAMI, 1, tempdata);
  if(tempdata[0] != 0x19)
    return -1;
  delay(1);
  
  regdata = 0x00;
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_PWR_MGMT_1, 1, &regdata);
  delay(10);

  regdata = (0x01<<7);
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_PWR_MGMT_1, 1, &regdata);
  delay(10);

  regdata = (0x01<<0);
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_PWR_MGMT_1, 1, &regdata);
  delay(10);

  regdata = 0x10;
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_CONFIG, 1, &regdata);
  delay(1);

  regdata = 0x18;
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_GYRO_CONFIG, 1, &regdata);
  delay(1);

  regdata = 0x01;
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_CONFIG, 1, &regdata);
  delay(1);

  regdata = 0x05;
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_SMPLRT_DIV, 1,&regdata);
  delay(1);

  regdata = 0x00;
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_INT_ENABLE, 1, &regdata);
  delay(1);

  regdata = 0x00;
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_CONFIG2, 1, &regdata);
  delay(1);

  regdata = 0x00;
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_USER_CTRL, 1, &regdata);
  delay(1);

  regdata = 0x00;
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_FIFO_EN, 1, &regdata);
  delay(1);

  regdata = 0x22;
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_INT_PIN_CFG, 1, &regdata);
  delay(1);

  regdata = 0x01;
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_INT_ENABLE, 1, &regdata);

  delay(100);
  getGres();
  getAres();
  return 0;
}

void MPU6886::getAccelAdc(int16_t* ax, int16_t* ay, int16_t* az){

   uint8_t buf[6];  
   I2C_Read_NBytes(MPU6886_ADDRESS,MPU6886_ACCEL_XOUT_H,6,buf);
   
   *ax=((int16_t)buf[0]<<8)|buf[1];
   *ay=((int16_t)buf[2]<<8)|buf[3];
   *az=((int16_t)buf[4]<<8)|buf[5];

}
void MPU6886::getGyroAdc(int16_t* gx, int16_t* gy, int16_t* gz){

  uint8_t buf[6];
  I2C_Read_NBytes(MPU6886_ADDRESS,MPU6886_GYRO_XOUT_H,6,buf);
  
  *gx=((uint16_t)buf[0]<<8)|buf[1];  
  *gy=((uint16_t)buf[2]<<8)|buf[3];  
  *gz=((uint16_t)buf[4]<<8)|buf[5];
  
}

void MPU6886::getTempAdc(int16_t *t){
  
  uint8_t buf[2];  
  I2C_Read_NBytes(MPU6886_ADDRESS,MPU6886_TEMP_OUT_H,2,buf);
  
  *t=((uint16_t)buf[0]<<8)|buf[1];  
}

//!俯仰，航向，横滚：pitch，yaw，roll，指三维空间中飞行器的旋转状态。
void MPU6886::getAhrsData(float *pitch,float *roll,float *yaw){

  float accX = 0; 
  float accY = 0;
  float accZ = 0;

  float gyroX = 0;
  float gyroY = 0;
  float gyroZ = 0;


  getGyroData(&gyroX,&gyroY,&gyroZ);
  getAccelData(&accX,&accY,&accZ);
  
  MahonyAHRSupdateIMU(gyroX * DEG_TO_RAD, gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD, accX, accY, accZ,pitch,roll,yaw);

}

void MPU6886::getGres(){

   switch (Gyscale)
   {
  // Possible gyro scales (and their register bit settings) are:
     case GFS_250DPS:
           gRes = 250.0/32768.0;
           break;
     case GFS_500DPS:
           gRes = 500.0/32768.0;
           break;
     case GFS_1000DPS:
           gRes = 1000.0/32768.0;
           break;
     case GFS_2000DPS:
           gRes = 2000.0/32768.0;
           break;
   }

}


void MPU6886::getAres(){
   switch (Acscale)
   {
   // Possible accelerometer scales (and their register bit settings) are:
   // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
   // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }

}
 
void MPU6886::SetGyroFsr(Gscale scale)
{
    //return IIC_Write_Byte(MPU_GYRO_CFG_REG,scale<<3);//设置陀螺仪满量程范围
    unsigned char regdata;	
    regdata = (scale<<3);
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_GYRO_CONFIG, 1, &regdata);
    delay(10);

    Gyscale = scale;
    getGres();
}

void MPU6886::SetAccelFsr(Ascale scale)
{
    unsigned char regdata;	
    regdata = (scale<<3);
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_CONFIG, 1, &regdata);
    delay(10);

    Acscale = scale;
    getAres();
}




void MPU6886::getAccelData(float* ax, float* ay, float* az){


  int16_t accX = 0;
  int16_t accY = 0;
  int16_t accZ = 0;
  getAccelAdc(&accX,&accY,&accZ);


  *ax = (float)accX * aRes;
  *ay = (float)accY * aRes;
  *az = (float)accZ * aRes;

}
      
void MPU6886::getGyroData(float* gx, float* gy, float* gz){
  int16_t gyroX = 0;
  int16_t gyroY = 0;
  int16_t gyroZ = 0;
  getGyroAdc(&gyroX,&gyroY,&gyroZ);

  *gx = (float)gyroX * gRes;
  *gy = (float)gyroY * gRes;
  *gz = (float)gyroZ * gRes;
}

void MPU6886::getTempData(float *t){
  
  int16_t temp = 0;
  getTempAdc(&temp);
  
  *t = (float)temp / 326.8 + 25.0;
}

void MPU6886::enableFIFO (Fodr rate){

  unsigned char regdata;
  
  I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_GYRO_CONFIG, 1, &regdata);
  regdata &= 0x1C;	//Clear bits 7:5 and 0:1 of FCHOICE_B to enable sample rate divider and DLPF setting
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_GYRO_CONFIG, 1, &regdata);
  delay(10);

  regdata = rate & 0xFF;	//Set sample rate clock divider based on passed value for sample rate desired
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_SMPLRT_DIV, 1, &regdata);
  delay(10);

  I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_CONFIG, 1, &regdata);
  regdata |= 0x01;	//Set DLPF_CFG to 176Hz DLPF filtering (highest value where sample rate clock divider still works)
  regdata &= 0xBF;	//Clear bit 6 to allow overflow writes to the FIFO - Use it, or lose it!
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_CONFIG, 1, &regdata);
  delay(10);
  
  regdata = 0x18;	//Set GYRO_FIFO_EN and ACCEL_FIFO_EN bits to one in FIFO Enable register to enable FIFO on ALL sensor data
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_FIFO_EN, 1, &regdata);
  delay(10);

  I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_INT_ENABLE, 1, &regdata);
  regdata |= 0x10;	// Set bit 4 to turn on interrupts on FIFO overflow events
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_INT_ENABLE, 1, &regdata);
  delay(10);

  regdata = 0x44;	//Set FIFO_EN and FIFO_RST bits to one in User Control register to enable FIFO mode
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_USER_CTRL, 1, &regdata);
  delay(10);
}

void MPU6886::resetFIFO (void){

  unsigned char regdata;
  
  I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_USER_CTRL, 1, &regdata);
  regdata |= 0x04;	// Set bit 2 to reset FIFO module
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_USER_CTRL, 1, &regdata);
  delay(10);
}

void MPU6886::disableFIFO (void){

  unsigned char regdata;
  
  regdata = 0x00;	//Clear GYRO_FIFO_EN and ACCEL_FIFO_EN bits to zero in FIFO Enable register
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_FIFO_EN, 1, &regdata);
  delay(10);

  I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_INT_ENABLE, 1, &regdata);
  regdata &= 0xEF;	// Clear bit 4 to turn off interrupts on FIFO overflow events
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_INT_ENABLE, 1, &regdata);
  delay(10);

  regdata = 0x00;	//Set FIFO_EN bit to zero in User Control register to dsiable FIFO mode
  I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_USER_CTRL, 1, &regdata);
  delay(10);
}

int MPU6886::getFIFOData(int16_t databuf[]){
	
  uint8_t buf[14];
  uint8_t i; 

  I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_FIFO_R_W, 14, buf);	//Burst read 14 byte sensor data sample array
  
  if ((((uint16_t)buf[0]<<8)|buf[1]) == 0x7F7F) {	// Datasheet suggests 0xFFFF, but not what appears to work
	  return -1;
  }
  databuf[0] = ((int16_t)buf[0]<<8)|buf[1]; //accelX
  databuf[1] = ((int16_t)buf[2]<<8)|buf[3]; //accelY
  databuf[2] = ((int16_t)buf[4]<<8)|buf[5]; //accelZ
  databuf[6] = ((int16_t)buf[6]<<8)|buf[7]; //temp
  databuf[3] = ((int16_t)buf[8]<<8)|buf[9]; //gyroX 
  databuf[4] = ((int16_t)buf[10]<<8)|buf[11]; //gyroY
  databuf[5] = ((int16_t)buf[12]<<8)|buf[13]; //gyroZ
  return 0;
}

int MPU6886::getFIFOData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t *t){
	
  uint8_t buf[14];
  uint8_t i; 

  I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_FIFO_R_W, 14, buf);	//Burst read 14 byte sensor data sample array
  
  if ((((uint16_t)buf[0]<<8)|buf[1]) == 0x7F7F) {	// Datasheet suggests 0xFFFF, but not what appears to work
	  return -1;
  }
  *ax = ((int16_t)buf[0]<<8)|buf[1];
  *ay = ((int16_t)buf[2]<<8)|buf[3];
  *az = ((int16_t)buf[4]<<8)|buf[5];
  *t = ((int16_t)buf[6]<<8)|buf[7];  
  *gx = ((int16_t)buf[8]<<8)|buf[9];  
  *gy = ((int16_t)buf[10]<<8)|buf[11];  
  *gz = ((int16_t)buf[12]<<8)|buf[13];
  return 0;
}

