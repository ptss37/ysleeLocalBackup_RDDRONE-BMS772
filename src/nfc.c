/****************************************************************************
 * nxp_bms/BMS_v1/src/nfc.c
 *
 * BSD 3-Clause License
 * 
 * Copyright 2020 NXP
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/i2c/i2c_master.h>
#include "nfc.h"
#include "cli.h"
#include "gpio.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
#define NTAG5_SLAVE_ADR           0x54 
#define SCL_FREQ                  400000

#define USER_MEMORY_BASE_ADR1     0x00  // <ys>
#define USER_MEMORY_BASE_ADR2     0x00  // <ys>
#define SRAM_BASE_ADR1            0x20  // <ys>
#define SRAM_BASE_ADR2            0x20  // <ys>

#define I2C_SLAVE_CONF_REG_ADR1   0x10
#define I2C_SLAVE_CONF_REG_ADR2   0xA9

#define STATUS_REGISTER_ADR1      0x10  // <ys>
#define STATUS_REGISTER_ADR2      0xA0  // <ys>
#define CONFIG_REGISTER_ADR1      0x10  // <ys>
#define CONFIG_REGISTER_ADR2      0xA1  // <ys>
#define ED_CONFIG_REG_ADR1        0x10  // <ys>
#define ED_CONFIG_REG_ADR2        0xA8  // <ys>
#define SYNCH_DATA_BLOCK_REG_ADR1 0x10  // <ys>
#define SYNCH_DATA_BLOCK_REG_ARD2 0xA2  // <ys>

#define I2C_SLAVE_CONF_REG_BYTE   0x0 

#define DEFAULT_NFC_PRIORITY      100 //<ys>
#define DEFAULT_NFC_STACK_SIZE_M  1536 //<ys>


/****************************************************************************
 * Private Variables
 ****************************************************************************/
/*! @brief variable to indicate of it is initialized */
static bool gNfcInitialized = false;  
const char i2c_path_nfc[] = "/dev/i2c0";  

/****************************************************************************
 * Private Functions
 ****************************************************************************/
//<ys
/*!
 * @brief   This Task polls the last bit of NS_REG to check if RF field is detected.
 *          Set led to yellow if RF field is detected and set led to off if not.(No Blinking)
 */
static int nfc_detector(int argc, char *argv[]);
//ys>

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/*!
 * @brief   This function will initialze the NFC
 *          it will test the i2C connection with the chip and read the slave address
 *    
 * @return  0 if ok, -1 if there is an error
 * @example 
 *          if(nfc_initialize())
 *          {
 *            // do something with the error
 *          }
 */
int nfc_initialize(void)
{
  int lvRetValue = 0;
  uint8_t regVal[1] = {0}, writeVal[3]; 
  int fd;
  struct i2c_msg_s i2c_msg[2];  
  struct i2c_transfer_s i2c_transfer;  

  if(!gNfcInitialized)
  {
    cli_printf("SELF-TEST NFC: START\n");

    // set the register address of the i2c slave configuration
    writeVal[0] = I2C_SLAVE_CONF_REG_ADR1;
    writeVal[1] = I2C_SLAVE_CONF_REG_ADR2;
    writeVal[2] = I2C_SLAVE_CONF_REG_BYTE;

    // open the i2c device 
    fd = open(i2c_path_nfc, O_RDONLY);  
    
    // check for errors
    if (fd < 0)  
    { 
      // get the error 
      lvRetValue = -errno;  

      // output to the user
      cli_printfError("nfc ERROR: Can't open i2c device, error: %d\n", lvRetValue);

      // return error
      return lvRetValue; 
    }  
    
    // make the 2 part write message with the register address to read from
    i2c_msg[0].addr   = NTAG5_SLAVE_ADR;  
    i2c_msg[0].flags  = 0;  
    i2c_msg[0].buffer = writeVal;  
    i2c_msg[0].length = 3;  /* Write address of where we want to read to AT24 */  
    i2c_msg[0].frequency = SCL_FREQ;  /* 400K bsp */  
      
    // make the read message
    i2c_msg[1].addr   = NTAG5_SLAVE_ADR;  
    i2c_msg[1].flags  = I2C_M_READ; /* Write command then sequence read data */  
    i2c_msg[1].buffer = regVal;  
    i2c_msg[1].length = 1;  
    i2c_msg[1].frequency = SCL_FREQ;  /* 400K bsp */  
      
    // make the i2C tranfer 
    i2c_transfer.msgv = (struct i2c_msg_s *)i2c_msg;  
    i2c_transfer.msgc = 2;  
      
    // the read register should be equal to the slave address
    /* do the i2C transfer to read the register */  
    lvRetValue = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&i2c_transfer);  

    // check for errors
    if(lvRetValue < 0)  
    {  
      // output to the user
      cli_printfError("nfc ERROR: Can't do i2c tranfer, error: %d\n", lvRetValue);

      // close the file descriptor
      close(fd);  

      // return to the user
      return lvRetValue;  
    }

    // there is a device that reacts to the slave address

    // close the file descriptor
    close(fd);  

    // check if the value is not equal to the slave address
    if((regVal[0] & 0x7F) != NTAG5_SLAVE_ADR)
    {
      // output to the user
      cli_printfError("nfc ERROR: slave address is not equal!\n");

      cli_printfError("Can't verify NFC chip!\n");

      // set the returnvalue 
      lvRetValue = -1;

      // the expected registervalue is not what it should be

      // return to the user
      return lvRetValue;
    }

    //cli_printf("NFC chip (NTAG5) I2C communication verified!\n");

    // say it is initialized
    gNfcInitialized = true;

    cli_printf("SELF-TEST NFC: \e[32mPASS\e[39m\n");

    // <ys> subak.io 쓰기
    nfc_myfunc1(); // <ys>
    
    // nfc_myinit1(); / /<ys>

    /* <ys
    // set the NFC not in HPD mode to test the GPIO
    lvRetValue = nfc_setHPD(false);

    // check if there is an error
    if(lvRetValue)
    {
      // output that it failed
      cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
    }

    
    // put the NFC chip in hard power-down mode because it is not yet used 
    cli_printfWarning("WARNING: putting NFC in hard power-down mode\n");

    
    lvRetValue = nfc_setHPD(true);

    // check if there is an error
    if(lvRetValue)
    {
      // output that it failed
      cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
    }
    ys> */

    lvRetValue = task_create("NfcDetector", DEFAULT_NFC_PRIORITY, DEFAULT_NFC_STACK_SIZE_M, nfc_detector, NULL);
		cli_printf("<ys> NfcDetector PID : %d\n", lvRetValue); // <ys>
    // check for errors

		if(lvRetValue < 0)
	  {
	    // inform user
	    lvRetValue = errno;
	    cli_printfError("NFC ERROR: Failed to start task: %d\n", lvRetValue);
	    return lvRetValue;
	  }

    lvRetValue = !gNfcInitialized;
    
  }

  // return to the user
  return lvRetValue;
}

/*!
 * @brief   This function can be used to set the hard power-down (HPD) mode of the NFC 
 *    
 * @param   HPD if true, the microcontroller will set the NFC chip in hard power-down mode. 
 *          if false, it will disable this mode.
 * @return  0 if ok, -1 if there is an error
 * @example 
 *          if(nfc_setHPD())
 *          {
 *            // do something with the error
 *          }
 */
int nfc_setHPD(bool HPD)
{
  int lvRetValue;

  // set the HPD (hard power-down) pin of the NFC high to consume power
  lvRetValue = gpio_writePin(NFC_HPD, HPD);

  // check for errors
  if(lvRetValue)
  {
    cli_printfError("nfc ERROR: could not set the HPD pin to %d!\n", HPD);
  } 

  // return
  return lvRetValue;
}

/*!
 * @brief   Pass Through(I2C->NFC) 모드 테스트용
 */
static int nfc_detector(int argc, char *argv[])
{
  int lvRetValue = -1;
  uint8_t registerValue[1] = {0}; 
  //uint8_t writeVal[20] = {0x03, 0x0d, 0xd1, 0x01, 0x09, 0x55, 0x03,0x73, 0x75, 0x62,\
   0x61, 0x6b, 0x2e, 0x69, 0x6f, 0x20, 0x00, 0x00, 0x00, 0x00};
  uint8_t writeVal = 0x55;

  cli_printf("<ys> NFC Task is started\n");
  
  while(1)
  {
    do
    {
      // <ys> Status register 읽어서
      nfc_readRegister(STATUS_REGISTER_ADR1, STATUS_REGISTER_ADR2, (uint8_t)0, &registerValue[0]);
      cli_printf("<ys> In loop\n"); // <ys>
      cli_printf("<ys> Status Reg. : 0x%X\n", registerValue[0]); // <ys>
      
    } while ( (registerValue[0]&0x03) != 0x03 ); // <ys> RF 감지되고 VCC ON 이면 빠져나감 
  
    cli_printf("<ys> Field presents & VCC On\n"); // <ys>

    // <ys> 아비터 = PT 모드, SRAM = On, PT 방향 = I2C -> NFC 으로 설정
    nfc_writeRegister(CONFIG_REGISTER_ADR1, CONFIG_REGISTER_ADR2, (uint8_t)1, (uint8_t)0x0F, (uint8_t)0x0A);
    cli_printf("<ys> 아비터 = PT 모드, SRAM = On, PT 방향 = I2C -> NFC 으로 설정\n"); // <ys>

    // <ys> ED = I2C -> NFC PT 모드로 설정
    nfc_writeRegister(ED_CONFIG_REG_ADR1, ED_CONFIG_REG_ADR2, (uint8_t)0, (uint8_t)0x0F, (uint8_t)0x03);
    cli_printf("<ys> ED = I2C -> NFC PT 모드로 설정\n"); // <ys>

    // <ys> SRAM 2000h~2031h까지( 32block x 4byte ) 데이터 씀.
    // <ys> 3Fh(64block * 4byte)인데 왜 31h 까지밖에 못쓰는지 모르겠음. 31쓰고나면 아비터가 잠금
    // <ys> SyncReg. 읽으면 00으로 나옴
    for(int i=0; i < 32; i++)
    {
      //<ys> Lock 상태 읽음
      nfc_readRegister(STATUS_REGISTER_ADR1, STATUS_REGISTER_ADR2, (uint8_t)1, &registerValue[0]);
      if ( (registerValue[0]&0x02)==0x02 ) cli_printf("<ys> I2C Locked");
      else if ( (registerValue[0]&0x01)==0x01 ) cli_printf("<ys> NFC Locked");

      nfc_writeMemory(SRAM_BASE_ADR1, SRAM_BASE_ADR2+i, &writeVal);
    }

      do
      {
        // <ys> Lock 상태 읽음
        nfc_readRegister(STATUS_REGISTER_ADR1, STATUS_REGISTER_ADR2, (uint8_t)1, &registerValue[0]);
        if ( (registerValue[0]&0x02)==0x02 ) cli_printf("<ys> I2C Locked");
        else if ( (registerValue[0]&0x01)==0x01 ) cli_printf("<ys> NFC Locked");

      } while( registerValue[0]&0x02 == 0x02 ); // NFC locked 아니면 빠져나감
    }

    return lvRetValue;

}

/*!
 * @brief   User memory(EEPROM)에 subak.io(NDEF포멧으로) 쓰는 함수.
 */
void nfc_myfunc1(void)
{
  cli_printf("<ys>User memory에 <subak.io> write\n");
  uint8_t writeVal[16] = {0x03, 0x0d, 0xd1, 0x01, 0x09, 0x55, 0x03, 0x73, 0x75, 0x62, 0x61, 0x6b, 0x2e, 0x69, 0x6F, 0x20};

  nfc_writeMemory(USER_MEMORY_BASE_ADR1, USER_MEMORY_BASE_ADR2+1, &writeVal[0]);

  nfc_waitEEPROM();

  nfc_writeMemory(USER_MEMORY_BASE_ADR1, USER_MEMORY_BASE_ADR2+2, &writeVal[4]);

  nfc_waitEEPROM();

  nfc_writeMemory(USER_MEMORY_BASE_ADR1, USER_MEMORY_BASE_ADR2+3, &writeVal[8]);

  nfc_waitEEPROM();

  nfc_writeMemory(USER_MEMORY_BASE_ADR1, USER_MEMORY_BASE_ADR2+4, &writeVal[12]);
  
  nfc_waitEEPROM();
}

/*!
 * @brief ED pin 을 RF감지 모드로 설정하는 함수
 */
void nfc_myinit1(void)
{
  // <ys> ED = RF감지 모드로 설정
  nfc_writeRegister(ED_CONFIG_REG_ADR1, ED_CONFIG_REG_ADR2, (uint8_t)0, (uint8_t)0x0F, (uint8_t)0x01);
}

/*!
 * @brief   This function write 4byte NDEF record to User memory in NTAG
 */
void nfc_writeMemory(uint8_t adr1, uint8_t adr2, uint8_t* wrVal)
{
  int lvRetValue = -1;
  uint8_t writeVal[6]; 
  int fd;
  struct i2c_msg_s i2c_msg[1];  
  struct i2c_transfer_s i2c_transfer; 

  // set the register address of the i2c slave configuration
  writeVal[0] = adr1;
  writeVal[1] = adr2;
  writeVal[2] = *wrVal;
  writeVal[3] = *(wrVal+1);
  writeVal[4] = *(wrVal+2);
  writeVal[5] = *(wrVal+3);

  // open the i2c device 
  fd = open(i2c_path_nfc, O_RDONLY);  
  
  // check for errors
  if (fd < 0)  
  { 
    // get the error 
    lvRetValue = -errno;  

    // output to the user
    cli_printfError("nfc ERROR: Can't open i2c device, error: %d\n", lvRetValue);

    // return error
    return lvRetValue; 
  }
    
  // make the 2 part write message with the register address to read from
  i2c_msg[0].addr   = NTAG5_SLAVE_ADR;  
  i2c_msg[0].flags  = 0;  
  i2c_msg[0].buffer = writeVal;  
  i2c_msg[0].length = 6;  /* Write address of where we want to read to AT24 */  
  i2c_msg[0].frequency = SCL_FREQ;  /* 400K bsp */
    
  // make the i2C tranfer 
  i2c_transfer.msgv = (struct i2c_msg_s *)i2c_msg;  
  i2c_transfer.msgc = 1;  
    
  // the read register should be equal to the slave address
  /* do the i2C transfer to read the register */  
  lvRetValue = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&i2c_transfer);  

  // check for errors
  if(lvRetValue < 0)  
  {  
    // output to the user
    cli_printfError("nfc ERROR: Can't do i2c tranfer, error: %d\n", lvRetValue);

    // close the file descriptor
    close(fd);  

    // return to the user
    return lvRetValue;  
  }
  
  // there is a device that reacts to the slave address

  // close the file descriptor
  close(fd); 
}

/*!
 * @brief   This function set bits in session register
 */
void nfc_writeRegister(uint8_t adr1, uint8_t adr2, uint8_t adrr, uint8_t mask, uint8_t data)
{
  int lvRetValue = -1;
  uint8_t writeVal[5]; 
  int fd;
  struct i2c_msg_s i2c_msg[1];  
  struct i2c_transfer_s i2c_transfer; 

  // set the register address of the i2c slave configuration
  writeVal[0] = adr1;
  writeVal[1] = adr2;
  writeVal[2] = adrr;
  writeVal[3] = mask;
  writeVal[4] = data;

  // open the i2c device 
  fd = open(i2c_path_nfc, O_RDONLY);  

  // check for errors
  if (fd < 0)  
  { 
    // get the error 
    lvRetValue = -errno;  

    // output to the user
    cli_printfError("nfc ERROR: Can't open i2c device, error: %d\n", lvRetValue);

    // return error
    return lvRetValue; 
  }
    
  // make the 2 part write message with the register address to read from
  i2c_msg[0].addr   = NTAG5_SLAVE_ADR;  
  i2c_msg[0].flags  = 0;  
  i2c_msg[0].buffer = writeVal;  
  i2c_msg[0].length = 5;  /* Write address of where we want to read to AT24 */  
  i2c_msg[0].frequency = SCL_FREQ;  /* 400K bsp */
    
  // make the i2C tranfer 
  i2c_transfer.msgv = (struct i2c_msg_s *)i2c_msg;  
  i2c_transfer.msgc = 1;  
    
  // the read register should be equal to the slave address
  /* do the i2C transfer to read the register */  
  lvRetValue = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&i2c_transfer);  

  // check for errors
  if(lvRetValue < 0)  
  {  
    // output to the user
    cli_printfError("nfc ERROR: Can't do i2c tranfer, error: %d\n", lvRetValue);

    // close the file descriptor
    close(fd);  

    // return to the user
    return lvRetValue;  
  }

  // there is a device that reacts to the slave address

  // close the file descriptor
  close(fd); 
}

/*!
 * @brief   This function set bits in session register
 */
void nfc_readRegister(uint8_t adr1, uint8_t adr2, uint8_t adrr, uint8_t* data)
{
  int lvRetValue = -1;
  uint8_t writeVal[3]; 
  int fd;
  struct i2c_msg_s i2c_msg[2];  
  struct i2c_transfer_s i2c_transfer; 

  // set the register address of the i2c slave configuration
  writeVal[0] = adr1;
  writeVal[1] = adr2;
  writeVal[2] = adrr;

  // open the i2c device 
  fd = open(i2c_path_nfc, O_RDONLY);  

  // check for errors
  if (fd < 0)  
  { 
    // get the error 
    lvRetValue = -errno;  

    // output to the user
    cli_printfError("nfc ERROR: Can't open i2c device, error: %d\n", lvRetValue);

    // return error
    return lvRetValue; 
  }
    
  // make the 2 part write message with the register address to read from
  i2c_msg[0].addr   = NTAG5_SLAVE_ADR;  
  i2c_msg[0].flags  = 0;  
  i2c_msg[0].buffer = writeVal;  
  i2c_msg[0].length = 3;  /* Write address of where we want to read to AT24 */  
  i2c_msg[0].frequency = SCL_FREQ;  /* 400K bsp */

  i2c_msg[1].addr   = NTAG5_SLAVE_ADR;  
  i2c_msg[1].flags  = I2C_M_READ;  
  i2c_msg[1].buffer = data;  
  i2c_msg[1].length = 1;  /* Write address of where we want to read to AT24 */  
  i2c_msg[1].frequency = SCL_FREQ;  /* 400K bsp */
    
  // make the i2C tranfer 
  i2c_transfer.msgv = (struct i2c_msg_s *)i2c_msg;  
  i2c_transfer.msgc = 2;  
    
  // the read register should be equal to the slave address
  /* do the i2C transfer to read the register */  
  lvRetValue = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&i2c_transfer);  

  // check for errors
  if(lvRetValue < 0)  
  {  
    // output to the user
    cli_printfError("nfc ERROR: Can't do i2c tranfer, error: %d\n", lvRetValue);

    // close the file descriptor
    close(fd);  

    // return to the user
    return lvRetValue;  
  }

  // there is a device that reacts to the slave address

  // close the file descriptor
  close(fd); 

}

/*!
 * @brief   This function wait for NTAG to write EERPOM
 */
void nfc_waitEEPROM(void)
{
  int lvRetValue = -1;
  uint8_t regVal[1] = {0}, writeVal[3]; 
  int fd;
  struct i2c_msg_s i2c_msg[2];  
  struct i2c_transfer_s i2c_transfer; 
  uint8_t old = 0;
  struct timespec sampleTime;
  struct timespec currentTime;
  uint32_t count = 0;
 
  do
  {
    // set the register address of the i2c slave configuration
    writeVal[0] = STATUS_REGISTER_ADR1;
    writeVal[1] = STATUS_REGISTER_ADR2;
    writeVal[2] = I2C_SLAVE_CONF_REG_BYTE;

    // open the i2c device 
    fd = open(i2c_path_nfc, O_RDONLY);  
    
    // check for errors
    if (fd < 0)  
    { 
      // get the error 
      lvRetValue = -errno;  

      // output to the user
      cli_printfError("nfc ERROR: Can't open i2c device, error: %d\n", lvRetValue);

      // return error
      return lvRetValue; 
    }
      
    // make the 2 part write message with the register address to read from
    i2c_msg[0].addr   = NTAG5_SLAVE_ADR;  
    i2c_msg[0].flags  = 0;  
    i2c_msg[0].buffer = writeVal;  
    i2c_msg[0].length = 3;  /* Write address of where we want to read to AT24 */  
    i2c_msg[0].frequency = SCL_FREQ;  /* 400K bsp */  
      
    // make the read message
    i2c_msg[1].addr   = NTAG5_SLAVE_ADR;  
    i2c_msg[1].flags  = I2C_M_READ; /* Write command then sequence read data */  
    i2c_msg[1].buffer = regVal;  
    i2c_msg[1].length = 1;  
    i2c_msg[1].frequency = SCL_FREQ;  /* 400K bsp */  
      
    // make the i2C tranfer 
    i2c_transfer.msgv = (struct i2c_msg_s *)i2c_msg;  
    i2c_transfer.msgc = 2;  
      
    // the read register should be equal to the slave address
    /* do the i2C transfer to read the register */  
    lvRetValue = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&i2c_transfer);  

    // check for errors
    if(lvRetValue < 0)  
    {  
      // output to the user
      cli_printfError("nfc ERROR: Can't do i2c tranfer, error: %d\n", lvRetValue);

      // close the file descriptor
      close(fd);  

      // return to the user
      return lvRetValue;  
    }
    
    // there is a device that reacts to the slave address

    // close the file descriptor
    close(fd); 
  } while( (regVal[0] & 0x80) == 0x80 );

}


