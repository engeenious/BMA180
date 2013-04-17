#ifndef _BMA180_h
#define _BMA180_h

#include "wirish.h"
#include "i2c.h"
#include "string.h"



#define BMA180_DEFAULT_ADDRESS 0x40
#define BMA180_CHIP_ID 0x03

//bandwidth / filter mode constants
#define BMA180_BANDWIDTH_10HZ     0x00
#define BMA180_BANDWIDTH_20HZ     0x01
#define BMA180_BANDWIDTH_40HZ     0x02
#define BMA180_BANDWIDTH_75HZ     0x03
#define BMA180_BANDWIDTH_150HZ    0x04
#define BMA180_BANDWIDTH_300HZ    0x05
#define BMA180_BANDWIDTH_600HZ    0x06
#define BMA180_BANDWIDTH_1200HZ   0x07
#define BMA180_BANDWIDTH_HIGHPASS 0x08 // high-pass: 1 Hz
#define BMA180_BANDWIDTH_BANDPASS 0x09 // band-pass: 0.2 Hz ... 300 Hz  

//acceleration sensitivity range constants
#define BMA180_RANGE_1G     0x00
#define BMA180_RANGE_1DOT5G 0x01
#define BMA180_RANGE_2G     0x02
#define BMA180_RANGE_3G     0x03
#define BMA180_RANGE_4G     0x04
#define BMA180_RANGE_8G     0x05
#define BMA180_RANGE_16G    0x06

//pick your mode
#define BMA180_MODE_LOW_NOISE 0x00
#define BMA180_MODE_LOW_POWER 0x03
#define BMA180_MODE_2         0x02


#define BMA180_CMD_CHIP_ID          0x00
#define BMA180_CMD_VERSION          0x01
#define BMA180_CMD_ACC_X_LSB        0x02
#define BMA180_CMD_ACC_X_MSB        0x03
#define BMA180_CMD_ACC_Y_LSB        0x04
#define BMA180_CMD_ACC_Y_MSB        0x05
#define BMA180_CMD_ACC_Z_LSB        0x06
#define BMA180_CMD_ACC_Z_MSB        0x07
#define BMA180_CMD_TEMP             0x08

#define BMA180_CMD_STATUS_REG1      0x09
#define BMA180_CMD_RESET            0x10
#define BMA180_CMD_STATUS_REG2      0x0A
#define BMA180_CMD_STATUS_REG3      0x0B
#define BMA180_CMD_STATUS_REG4      0x0C

#define BMA180_CMD_CTRL_REG0        0x0D
#define BMA180_CMD_CTRL_REG1        0x0E
#define BMA180_CMD_CTRL_REG2        0x0F
#define BMA180_CMD_BW_TCS           0x20
#define BMA180_CMD_CTRL_REG3        0x21
#define BMA180_CMD_CTRL_REG4        0x22



class BMA180
{
    public:
        typedef enum {F10HZ=0,F20HZ=1,F40HZ, F75HZ,F15HZ0,F300HZ,F600HZ,F1200HZ,HIGHPASS,BANDPASS} FILTER;
        typedef enum {G1=0,G15=1,G2,G3,G4,G8,G16}GSENSITIVITY;
    private:
    
        unsigned char   address;
        GSENSITIVITY    gSense;
        static BMA180   *currentObj;

        i2c_msg         msgs[2];
        uint8           buff[8];

    public:

        int16 x,y,z; // yes, public, what the heck
        int temp;

        BMA180(unsigned char a);    
        BMA180();
        
        void    doCalibration();
        void    doFineCalibration();
        void    SetAddress(int val);

        void    readAccel();
        float   getGSense();
        float   getXValFloat();
        float   getYValFloat();
        float   getZValFloat();
        int     setRegValue(int regAdr, int val, int maskPreserve);
        int     getRegValue(int adr);
        void    setGSensitivity(GSENSITIVITY maxg);
        void    SetFilter(FILTER f);
        void    SetISRMode();
        void    SoftReset();
        void    SetSMPSkip();
        int     getIDs(int &id, int &version);
        void    enableWrite();
        void    disableWrite();
        virtual bool checkResult(int result);
};


#endif //_BMA180_h
