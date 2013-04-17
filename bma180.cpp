
/*
  Ported from Arduino  to maple using I2C2
  by engeenious 2013
*/
#include "bma180.h"

#if defined( BOARD_STM32VLD )
#define Serial Serial1
#else
#define Serial SerialUSB
#endif


BMA180* BMA180::currentObj = NULL;


BMA180::BMA180(unsigned char a)
{
  BMA180::currentObj = this;
  address = a;
  gSense=G1;
  i2c_master_enable(I2C2, 0);
}

BMA180::BMA180()
{
  BMA180::currentObj = this;
  address=BMA180_DEFAULT_ADDRESS;
  gSense=G1;
  i2c_master_enable(I2C2, 0);
}


void BMA180::SetAddress(int adr)
{
	address=(unsigned char) adr;
}

int BMA180::getRegValue(int adr)
{
  memset(buff, 0, sizeof(buff));
  buff[0] = (uint8)adr;
  msgs[0].addr = address;
  msgs[0].flags = 0;
  msgs[0].length = 1;
  msgs[0].data = buff;
  
  msgs[1].addr = address;
  msgs[1].flags = I2C_MSG_READ;
  msgs[1].length = 1;
  msgs[1].data = &buff[1];

  int ret = i2c_master_xfer(I2C2, msgs, 2, 2); 
  if(ret != 0) {
    /*
    Serial.print("i2c_master_xfer error ");
    Serial.println(ret);
    */
  }
  return buff[1];
}

int BMA180::setRegValue(int regAdr, int val, int maskPreserve)
{
  int preserve  = getRegValue(regAdr);
  int orgval    = preserve & maskPreserve;

  buff[0] = (uint8)regAdr;
  buff[1] = (uint8)orgval|val;

  msgs[0].addr    = address;
  msgs[0].flags   = 0;
  msgs[0].length  = 2;
  msgs[0].data    = buff;

  int ret = i2c_master_xfer(I2C2, msgs, 1, 2);
  return ret;
}

int BMA180::getIDs(int &id, int &version)
{ 

    buff[0] = BMA180_CMD_CHIP_ID;
    msgs[0].addr = address;
    msgs[0].flags = 0;
    msgs[0].length = 1;
    msgs[0].data = buff;
    
    msgs[1].addr = address;
    msgs[1].flags = I2C_MSG_READ;
    msgs[1].length = 2;
    msgs[1].data = &buff[1];

    int ret = i2c_master_xfer(I2C2, msgs, 2, 2);
    id      = buff[1];    
    version = buff[2];
    return ret;
}

void BMA180::readAccel()
{
  int ret = -1;
 
  buff[0] = BMA180_CMD_ACC_X_LSB;
  msgs[0].addr = address;
  msgs[0].flags = 0;
  msgs[0].length = 1;
  msgs[0].data = &buff[0];
  
  msgs[1].addr = address;
  msgs[1].flags = I2C_MSG_READ;
  msgs[1].length = 7;
  msgs[1].data = &buff[1];

  ret = i2c_master_xfer(I2C2, msgs, 2, 1);
  if(ret != 0) {
    Serial.print("i2c_master_xfer error ");
    Serial.println(ret);
  }

  int lsb = buff[1]>>2;
  int msb = buff[2];
  x=(msb<<6)+lsb; 
  if (x&0x2000) x|=0xc000; // set full 2 complement for neg values
  lsb = buff[3]>>2;
  msb = buff[4];
  y=(msb<<6)+lsb;
  if (y&0x2000) y|=0xc000;
  lsb = buff[5]>>2;
  msb = buff[6];
  z=(msb<<6)+lsb;
  if (z&0x2000) z|=0xc000;
  temp = buff[7];
  if (temp&0x80) temp|=0xff00;
}



float BMA180::getGSense()
{
    float result = 1.0;
    switch(gSense)
    {
        case G1: result = 1.0;
        case G15: result = 1.5;
        case G2: result = 2.0;
        case G3: result = 3.0;
        case G4: result = 4.0;
        case G8: result = 8.0;
        case G16: result = 16.0;
    }
    return result;
}

float BMA180::getXValFloat()
{
    // normalize (if x is maximum (8191) and GSENSE=1.0 then 1.0
    return (float)x/8191.0*getGSense();
}
float BMA180::getYValFloat()
{
    // normalize (if x is maximum (8191) and GSENSE=1.0 then 1.0
    return (float)y/8191.0*getGSense();
}
float BMA180::getZValFloat()
{
    // normalize (if x is maximum (8191) and GSENSE=1.0 then 1.0
    return (float)z/8191.0*getGSense();
}



void BMA180::setGSensitivity(GSENSITIVITY maxg) //1, 1.5 2 3 4 8 16
{
    setRegValue(0x35, maxg<<1, 0xF1);
}

void BMA180::SetFilter(FILTER f) // 10,20,40,75,150,300,600,1200, HP 1HZ,BP 0.2-300, higher values not authorized
{
    setRegValue(0x20, f<<4, 0x0F);  
}

void BMA180::SetISRMode() // you must provide a ISR function on the pin selected (pin 2 or 3,. so INT0 or INT1)
{
    //setRegValue(0x21, 2, 0xFD);
    setRegValue(0x21, 0x2, 0x2);
}

void BMA180::SoftReset() // all values will be default
{
    setRegValue(0x10, 0xB6, 0);
    delay(100);
}

void BMA180::SetSMPSkip()
{
    setRegValue(0x35, 1, 0xFE);
}


void BMA180::enableWrite()
{
    //ctrl_reg1 register set ee_w bit to enable writing to regs.
    setRegValue(0x0D,0x10,~0x10);
    delay(10);
}


void BMA180::disableWrite()
{
    setRegValue(0x0D,0x0,~0x10);
    delay(10);
}

bool BMA180::checkResult(int result)
{
	  if(result >= 1)
	  	return false;
	  return true;
}    

void BMA180::doCalibration()
{
  // full calibration
  setRegValue(0x22, 0x03, 0xFC);
  //setRegValue(0x22, 0x03, 0x03);
  // Fine calibration
  //setRegValue(BMA180_CMD_CTRL_REG4, 0x01, 0xFC);

    
  setRegValue(BMA180_CMD_CTRL_REG1, 0x01 << 5, 0x1F);
  while ((getRegValue(BMA180_CMD_CTRL_REG1) & 0xE0)) delay(5);
  
  setRegValue(BMA180_CMD_CTRL_REG1, 0x01 << 6, 0x1F);
  while ((getRegValue(BMA180_CMD_CTRL_REG1) & 0xE0)) delay(5);
  
  setRegValue(BMA180_CMD_CTRL_REG1, 0x01 << 7, 0x1F);
  while ((getRegValue(BMA180_CMD_CTRL_REG1) & 0xE0)) delay(5);
}

void BMA180::doFineCalibration()
{
  // full calibration
  //setRegValue(0x22, 0x03, 0xFC);
  //setRegValue(0x22, 0x03, 0x03);
  // Fine calibration
  setRegValue(BMA180_CMD_CTRL_REG4, 0x01, 0xFC);

    
  setRegValue(BMA180_CMD_CTRL_REG1, 0x01 << 5, 0x1F);
  while ((getRegValue(BMA180_CMD_CTRL_REG1) & 0xE0)) delay(5);
  
  setRegValue(BMA180_CMD_CTRL_REG1, 0x01 << 6, 0x1F);
  while ((getRegValue(BMA180_CMD_CTRL_REG1) & 0xE0)) delay(5);
  
  setRegValue(BMA180_CMD_CTRL_REG1, 0x01 << 7, 0x1F);
  while ((getRegValue(BMA180_CMD_CTRL_REG1) & 0xE0)) delay(5);
}
