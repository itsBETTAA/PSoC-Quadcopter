#include "twi.h"
#include "project.h"

void Wire_begin(){
    I2C_Start(); 
}

void Wire_beginTransmission(uint8_t address, uint8_t R_nW)
{
    /*
    Description: 
        -This function generates a Start condition and sends the slave address with the 
         read/write bit. Disables the I2C interrupt. 
    
    Parameters: 
        -uint8 slaveAddress: Right-justified 7-bit slave address (valid range 0 to 127). 
        -uint8 R_nW: Set to zero, send write command; set to nonzero, send read command. 
    */
    I2C_MasterSendStart(address, R_nW); 
}

int Wire_write(uint8_t address, uint8_t* writeBuffer, uint8_t length, uint8_t type)
{
    /*Wire_beginTransmission(address, 0);
    return (I2C_MasterWriteByte(data)); */

    /*
    Description:
        -This function automatically writes an entire buffer of data to a slave device. After the data
         transfer is initiated by this function, the included ISR manages further data transfer in byte-by
         byte mode. Enables the I2C interrupt.
   
    Parameters: 
        -uint8 slaveAddress: Right-justified 7-bit slave address (valid range 0 to 127).
        -uint8 wrData: Pointer to the buffer of the data to be sent.
        -uint8 cnt: Number of bytes of the buffer to send.
        -uint8 mode: Transfer mode defines: (1) Whether a Start or Restart condition is generated at 
         the beginning of the transfer, and (2) Whether the transfer is completed or halted before the 
         Stop condition is generated on the bus.

              1=>  I2C_MODE_COMPLETE_XFER = Perform complete transfer from Start to Stop. 
              2=>  I2C_MODE_REPEAT_START  = Send Repeat Start instead of Start. 
              3=>  I2C_MODE_NO_STOP       = Execute transfer without a Stop    
    */

    int ret = 0;

    if(type == 1){
        ret = I2C_MasterWriteBuf(address, writeBuffer, length, I2C_MODE_COMPLETE_XFER);
    }

    else if(type == 2){
        ret = I2C_MasterWriteBuf(address, writeBuffer, length, I2C_MODE_REPEAT_START);
    }

    else if(type == 3){
        ret = I2C_MasterWriteBuf(address, writeBuffer, length, I2C_MODE_NO_STOP);
    }
    else{
        ret = 0; //Signifies user inputed error
    }

    return ret;
}

uint8_t Wire_read(uint8_t address)
{
   /*Wire_beginTransmission(address, 1);
    return (I2C_MasterReadByte()); */
     /*
    Description:
        -This function automatically reads an entire buffer of data from a slave device. Once this 
         function initiates the data transfer, the included ISR manages further data transfer in byte by
         byte mode. Enables the I2C interrupt.
   
    Parameters: 
        -uint8 slaveAddress: Right-justified 7-bit slave address (valid range 0 to 127).
        -uint8 rdData: Pointer to the buffer in which to put the data from the slave.
        -uint8 cnt: Number of bytes of the buffer to read.
        -uint8 mode: Transfer mode defines: (1) Whether a Start or Restart condition is generated at 
         the beginning of the transfer, and (2) Whether the transfer is completed or halted before the 
         Stop condition is generated on the bus.

                I2C_MODE_COMPLETE_XFER = Perform complete transfer from Start to Stop. 
                I2C_MODE_REPEAT_START  = Send Repeat Start instead of Start. 
                I2C_MODE_NO_STOP       = Execute transfer without a Stop    
    */

    uint8_t readbuffer[1]; //creates an array of one slot
    Wire_requestFrom(address, sizeof(readbuffer), readbuffer); //do a blocking read
    return readbuffer[0]; //return retrieved buffer data

}

uint8_t Wire_requestFrom(uint8_t slaveAddress, uint8_t length, uint8_t *dataBuffer)
{
    //I2C_I2CMasterClearReadBuf();  /*I2C_MasterClearWriteBuf()*/
    //I2C_I2CMasterClearWriteBuf(); /*Resets the write buffer pointer back to the beginning of the buffer. */

    /*
    Description:
        -This function automatically reads an entire buffer of data from a slave device. Once this 
         function initiates the data transfer, the included ISR manages further data transfer in byte by
         byte mode. Enables the I2C interrupt.
   
    Parameters: 
        -uint8 slaveAddress: Right-justified 7-bit slave address (valid range 0 to 127).
        -uint8 rdData: Pointer to the buffer in which to put the data from the slave.
        -uint8 cnt: Number of bytes of the buffer to read.
        -uint8 mode: Transfer mode defines: (1) Whether a Start or Restart condition is generated at 
         the beginning of the transfer, and (2) Whether the transfer is completed or halted before the 
         Stop condition is generated on the bus.

                I2C_MODE_COMPLETE_XFER = Perform complete transfer from Start to Stop. 
                I2C_MODE_REPEAT_START  = Send Repeat Start instead of Start. 
                I2C_MODE_NO_STOP       = Execute transfer without a Stop    
    */

    I2C_MasterReadBuf(slaveAddress, dataBuffer, length, I2C_MODE_COMPLETE_XFER);

    while ((I2C_MasterStatus() & I2C_MSTAT_RD_CMPLT)==0){}

    return 1/*(I2C_I2CMasterGetReadBufSize())*/; /*Returns the byte count of data read since the 
                                           I2C_MasterClearReadBuf() function was called. */
}

uint8_t Wire_endTransmission()
{
    uint8_t ret = I2C_MasterSendStop();
    return ret;
}