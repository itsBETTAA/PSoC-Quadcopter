
#include <barometer.h>
#include <twi.h>
#include <stdlib.h>
#include "project.h"

//static variables/functions
static void reset();

static int32_t  _Pres;
static int32_t  _T;
static int32_t  _dT;
static uint16_t _Co[N_PROM_PARAMS];
static uint32_t _lastTime;

//Things we need to know
#define OSR 2 // 0-3
#define CMD_RESET 0x1E
#define CMD_ADC_READ 0x00
#define CMD_CONV_D1_BASE 0x40
#define CMD_CONV_D2_BASE 0x50
#define CONV_REG_SIZE 0x02
#define CMD_PROM_READ_BASE 0xA2
#define PROM_REG_SIZE 0x02
#define NBYTES_CONV 3
#define NBYTES_PROM 2

// Temperature sampling period threshold [milliseconds]
// Kindly read the comment bellow in getPressure() method
#define T_THR 1000
/*
TODO:
1) Separate OSR for temperature and pressure
2) Timedelay empirical formula for temperature oversampling
3) Precidion pressure calibration for high temperature
4) Default and optional OSR
5) Documentation
*/

void sendCommand(uint8_t cmd)
{
    /*
	Wire.beginTransmission(ADD_MS5611);
	Wire.write(cmd);
	Wire.endTransmission();
*/

    //to write, you want to write:  (1) The address you want to write to
    //                              (2) The cmd you wish to write to the register

    uint8_t Write_Buf[1] = {0};
    Write_Buf[0] = (uint8_t)cmd;

    I2C_MasterWriteBuf(MS5611_ADDRESS, (uint8 *)Write_Buf, sizeof(Write_Buf), I2C_MODE_COMPLETE_XFER);
    while ((I2C_MasterStatus() & I2C_MSTAT_WR_CMPLT) == 0)
    {
    }

    return; //return 1 if everything was successful
}

uint32_t readnBytes(uint8_t nBytes)
{
    if ((0 < nBytes) & (nBytes < 5))
    {
        uint32_t data = 0;
        uint8_t Read_Buf[5] = {0}; //create an array that will store the data read

        I2C_MasterReadBuf(MS5611_ADDRESS, (uint8 *)Read_Buf, nBytes, I2C_MODE_COMPLETE_XFER);
        while ((I2C_MasterStatus() & I2C_MSTAT_RD_CMPLT) == 0)
        {
        } //wait till Master status indicates read is complete
        
        int8_t k;
        for ( k = (nBytes - 1); k>=0 ; k--){
			data |= ( (uint32_t) Read_Buf[(nBytes-1)-k] << (8*k) ); 	// concantenate bytes
        }
        return data;   
    }
    return 0;
}

static void reset(){
    sendCommand(CMD_RESET);
}

void baro_begin()
{
    _T = 0;
    _Pres = 0;
    _lastTime = T_THR;
    uint8_t k;
    for (k = 0; k < N_PROM_PARAMS; k++)
    {
        _Co[k] = 69;
    }

    Wire_begin();
    reset();
    CyDelay(100);
    baro_readCalibration();
}

uint32_t baro_getRawTemperature()
{
    sendCommand(CMD_CONV_D2_BASE + OSR * CONV_REG_SIZE); //read sensor, prepare a data
    CyDelay(1 + 2 * OSR);                                  //wait at least 8.33us
    sendCommand(CMD_ADC_READ);                           //get ready for reading the data
    return readnBytes(NBYTES_CONV);                      //reading the data
}

int32_t baro_getTemperature()
{
    // Code below can be uncommented for slight speedup:
    // NOTE: Be sure what you do! Notice that Delta 1C ~= Delta 2hPa
    //****************
    // if(abs(millis()-_lastTime)<T_THR)
    // 	return _T;
    //_lastTime = millis();
    //****************
    uint32_t D2;
    D2 = baro_getRawTemperature();
    _dT = D2 - ((uint32_t)_Co[5 - 1] * 256); //update '_dT'
    // Below, 'dT' and '_C[6-1]'' must be casted in order to prevent overflow
    // A bitwise division can not be dobe since it is unpredictible for signed integers
    _T = 2000 + ((int64_t)_dT * _Co[6 - 1]) / 8388608;
    return _T;
}

uint32_t baro_getRawPressure() 
{
    sendCommand(CMD_CONV_D1_BASE+OSR*CONV_REG_SIZE);	//read sensor, prepare a data
	CyDelay(1+2*OSR); 									//wait at least 8.33us for full oversampling
	sendCommand(CMD_ADC_READ); 							//get ready for reading the data
	return readnBytes(NBYTES_CONV);						//reading the data
}

int32_t baro_getPressure()
{
    baro_getTemperature(); //updates temperature _dT and _T
    uint32_t D1 = baro_getRawPressure();

    int64_t OFF = (int64_t)_Co[2 - 1] * 65536 + (int64_t)_Co[4 - 1] * _dT / 128;

    int64_t SENS = (int64_t)_Co[1 - 1] * 32768 + (int64_t)_Co[3 - 1] * _dT / 256;
    _Pres = (D1 * SENS / 2097152 - OFF) / 32768;
    return _P;
}

void baro_readCalibration()
{
    uint8_t k;
    for( k=0 ; k<6 ; k++ ){
		sendCommand(CMD_PROM_READ_BASE + k*2);
		_Co[k] = (uint16_t) (readnBytes(NBYTES_PROM) & 0xFFFF); //masking out two LSB
	}
}

void baro_getCalibration(uint16_t *C)
{
    uint8_t k;
    for( k=0 ; k<N_PROM_PARAMS ; k++ )
		C[k]=_Co[k];
	return;
}