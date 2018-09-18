/*
 * GREG_Common_COPY.h
 *
 *  Created on: Sep 4, 2018
 *      Author: greg
 */

//#ifndef GREG_LIBRARIES_GREG_COMMON_COPY_H_
#define GREG_LIBRARIES_GREG_COMMON_COPY_H_

/*
 * Common.h
 *
 *  Created on: Jun 19, 2018
 *      Author: greg
 */

#ifndef GREG_COMMON_H_
#define GREG_COMMON_H_

#include "Arduino.h"
#include <stdbool.h>


#define FAULT_STATUS_RANGE                                           149

#define LMCO_COMM_FAULT_STATUS                                       189 /* GLOBAL STATUS */

#define LCD_MENU_DATA_CMD                                         194      /* NEW for LCD */

#define Test_Green_LED     34  // (PC3)
#define Test_Red_LED       32  // (PC5)

/* define standard constants */
#define ON                                 1
#define OFF                                0
#define TRUE                               1
#define FALSE                              0
#define AND                                &&
#define OR                                 ||
#define NOT                                !
#define NE                                 !=
#define EQ                                 ==
#define GE                                 >=
#define LE                                 <=



#define END                                0 /* value used to initiate NO_ACK */
#define CONTINUE                           1 /* value used to initiate ACK */



/* define LMCO_COMM packet offset constants */
#define COMMAND                                  0
#define DATA1                                    2
#define DATA2                                    3
#define DATA3                                    4
#define DATA4                                    5
#define DATA5                                    6
#define DATA6                                    7
#define CHECKSUM                                 8
#define DEVICE_STATUS                            9

#define READ                             0
#define WRITE                            1

#define GENERAL_CALL_ADDRESS             0x00 /* 0:  I2C "general call" address */


//#define SDATA                              digitalRead(20)          			  /* define data line pin */

#define SDATA                              (PIND & (1 << PD1))
//#define SDATA_LOW                          digitalWrite( SDATA, LOW);   	/* macro to drive data line low */
//#define SDATA_HI                           digitalWrite( SDATA, HIGH);  	/* macro to float data line high */
//#define SDATA_LOW                          PORTD &= ~(1<<PORTD1); //clear bit 1
//#define SDATA_HI                           PORTD |= (1<<PORTD1); //set bit 1

//#define SDATA_SET_AS_INPUT                 DDRD &= 0xFD; //clear bit 1
//#define SDATA_SET_AS_OUTPUT                DDRD |= 0x02; //set bit 1


//#define SCLK                               digitalRead(21)                     /* state of clock pin */
#define SCLK                               (PIND & (1 << PD0))

//#define SCLK_LOW                           digitalWrite( SCLK, LOW); 	/* macro to drive clock line low */
//#define SCLK_HI                            digitalWrite( SCLK, HIGH);	/* macro to float clock line high */
//#define SCLK_LOW                           PORTD &= ~(1<<PORTD0); //clear bit 0
//#define SCLK_HI                            PORTD |= (1<<PORTD0);  //set bit 0

// Port for the I2C
#define I2C_DDR DDRD
#define I2C_PIN PIND
#define I2C_PORT PORTD

// Pins to be used in the bit banging
#define I2C_CLK 0
#define I2C_DAT 1

#define I2C_DATA_HI()\
I2C_DDR &= ~ (1 << I2C_DAT);\
I2C_PORT |= (1 << I2C_DAT);
#define I2C_DATA_LO()\
I2C_DDR |= (1 << I2C_DAT);\
I2C_PORT &= ~ (1 << I2C_DAT);

#define I2C_CLOCK_HI()\
I2C_DDR &= ~ (1 << I2C_CLK);\
I2C_PORT |= (1 << I2C_CLK);
#define I2C_CLOCK_LO()\
I2C_DDR |= (1 << I2C_CLK);\
I2C_PORT &= ~ (1 << I2C_CLK);

#define SCLK_LOW 	I2C_CLOCK_LO()
#define SCLK_HI 	I2C_CLOCK_HI()

#define SDATA_LOW 	I2C_DATA_LO()
#define SDATA_HI 	I2C_DATA_HI()

#define I2C_CLOCK_HI()\
I2C_DDR &= ~ (1 << I2C_CLK);\
I2C_PORT |= (1 << I2C_CLK);
#define I2C_CLOCK_LO()\
I2C_DDR |= (1 << I2C_CLK);\
I2C_PORT &= ~ (1 << I2C_CLK);



/* variable definitions */
bool I2C_Bus_Busy_Flag;			/* I2C bus state is busy */
bool I2C_Clock_Timeout_Flag; 	/* I2C clock timeout or failure */
bool I2C_Acknowledge_Flag; 		/* I2C acknowledge error or not ACK */

/* function prototypes */
void Delay_5_Us ( void );
void Delay_50_Us ( void );
unsigned char I2C_Read(unsigned char *buffer, char acknowledge); /* Read byte from SSPBUF register */
void I2C_Restart(void); /* Generate bus repeat start condition */
void I2C_Start(void); /* Generate bus start condition */
void I2C_Stop(void); /* Generate bus stop condition */
unsigned char I2C_Write(char data_out); /* Write byte to SSPBUF register */


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************************************
*  FUNCTION NAME: Delay_25_Usec ()
*  PURPOSE: Function provides given delay when compiled for 16 MHz.
*  AUTHOR: Jerome Sirotnik
*  COPYRIGHT: (C) Copyright, 2015, Aviation Ground Equipment Corp.  All Rights Reserved.
*  NOTES: Compile code with full optimization.
**************************************************************************************************/
void Delay_25_Us ( void )
{
   delayMicroseconds(5);
   delayMicroseconds(5);
   delayMicroseconds(5);
   delayMicroseconds(5);
   delayMicroseconds(5);
}
/**************************************************************************************************
*  FUNCTION NAME: Delay_400_Usec ()
*  PURPOSE: Function provides given delay when compiled for 16 MHz.
*  AUTHOR: Jerome Sirotnik
*  COPYRIGHT: (C) Copyright, 2015, Aviation Ground Equipment Corp.  All Rights Reserved.
*  NOTES: Compile code with full optimization.
**************************************************************************************************/
void Delay_400_Us ( void )
{
   Delay_50_Us ();
   Delay_50_Us ();
   Delay_50_Us ();
   Delay_50_Us ();
   Delay_50_Us ();
   Delay_50_Us ();
   Delay_50_Us ();
   Delay_50_Us ();
}
/**************************************************************************************************
*  FUNCTION NAME: Delay_5_Us ( void )
*  PURPOSE: Function provides given delay when compiled for 16 MHz.
*  AUTHOR: Jerome Sirotnik
*  COPYRIGHT: (C) Copyright, 2015, Aviation Ground Equipment Corp.  All Rights Reserved.
*  NOTES: Compile code with full optimization.
**************************************************************************************************/
void Delay_5_Us ( void )
{
   /* define local variable */
//   unsigned char i;
//   i = 20; /* init loop counter */
   /* loop */
//   do
//   {
//      __asm__("nop\n\t"); /* assembly language no-operation */
//   } while(--i);

	delayMicroseconds(5);

}
/**************************************************************************************************
*  FUNCTION NAME: Delay_50_Usec ()
*  PURPOSE: Function provides given delay when compiled for 16 MHz.
*  AUTHOR: Jerome Sirotnik
*  COPYRIGHT: (C) Copyright, 2015, Aviation Ground Equipment Corp.  All Rights Reserved.
*  NOTES: Compile code with full optimization.
**************************************************************************************************/
void Delay_50_Us ( void )
{
	delayMicroseconds(50);
}
/**************************************************************************************************
*  FUNCTION NAME: Delay_Ms ( void )
*  PURPOSE: Function provides specified delay when compiled for 16 MHz.
*  AUTHOR: Jerome Sirotnik
*  COPYRIGHT: (C) Copyright, 2015, Aviation Ground Equipment Corp.  All Rights Reserved.
*  NOTES:
**************************************************************************************************/
void Delay_Ms(unsigned milliseconds)
{
	delay(milliseconds);
}
/**************************************************************************************************
*  FUNCTION NAME: char I2C_Read ( acknowledge )
*  PURPOSE: Function reads single byte from the I2C bus and initiates ACK or NO_ACK as instructed.
*           Returns error if bus error occurs.
*  AUTHOR: Jerome Sirotnik
*  COPYRIGHT: (C) Copyright, 2015, Aviation Ground Equipment Corp.  All Rights Reserved.
*  NOTES: "near *buffer" is a pointer to the passed memory location.  "near" signifies that the
*         pointer is 8 bits long and can only access ram.
**************************************************************************************************/
unsigned char I2C_Read ( unsigned char *buffer, char acknowledge )
{
   /* declare local variable */
   unsigned char bit_counter;

   I2C_Bus_Busy_Flag = 0; /* reset error flags */
   I2C_Clock_Timeout_Flag = 0; /* reset error flags */
   I2C_Acknowledge_Flag = 0; /* reset error flags */

   bit_counter = 8; /* set bit count for byte */

   /* perform the following for each bit */
   do
   {
      SCLK_LOW; /* drive clock line low */
      SDATA_HI; /* release data line to float high */

      delayMicroseconds(15); /* minimum low period of clock Tlow */

      SCLK_HI; /* release clock line to float high */

      delayMicroseconds(15); /* minimum high period of clock Thigh */

      if ( !SCLK ) /* test for clock low */
      {

         /* Rev 2.07 Added clock stretch function here and removed call */
         // I2C_Clock_Test(); /* clock stretch routine */

         Delay_50_Us (); /* Rev 2.07 added wait to stretch clock */
         Delay_50_Us (); /* Rev 2.07 added wait to stretch clock */
         Delay_50_Us (); /* Rev 2.07 added wait to stretch clock */

         /* Rev 2.07 added check for clock stretch */
         if ( !SCLK ) /* if clock is still low after wait */
         {
            I2C_Clock_Timeout_Flag = ON; /* set clock error flag */
         }

         if ( I2C_Clock_Timeout_Flag ) /* if clock timeout error */
         {
            return ( 1 ); /* return with error condition */
         }
      }

      *buffer <<= 1;	/* clear bit 0 */

      /* perform triple check for HI or LO value. */
      if ( SDATA )  /* do if data line high */
      {
         if ( SDATA )  /* do if data line still high */
         {
            if ( SDATA )  /* do if data line still high */
            {
               /* data check for 1 passed */
               *buffer |= 0x01;  /* set bit 0 to logic 1 */
            }
            else  /* data check for 1 failed */
            {
               return ( 1 ); /* return with error condition */
            }
         }
         else /* data check for 1 failed */
         {
            return ( 1 ); /* return with error condition */
         }
      }
      else if ( SDATA ) /* data check for 0 failed */
      {
         return ( 1 ); /* return with error condition */
      }
      else if ( SDATA ) /* data check for 0 failed */
      {
         return ( 1 ); /* return with error condition */
      }

   } while (--bit_counter); /* end do while */

   SCLK_LOW; /* drive clock line low */

   /* initiate ACK if more data to follow */
   if ( acknowledge EQ CONTINUE )
   {
      SDATA_LOW; /* drive data line low */
   }
   else /* else do not ACK to signal end */
   {
      SDATA_HI; /* release data line to float high */
   }

   delayMicroseconds(15); /* minimum low period of clock Tlow */

   SCLK_HI; /* release clock line to float high */

   delayMicroseconds(15); /* minimum high period of clock Thigh */

   return ( 0 ); /* return with no error */
}
/**************************************************************************************************
*  FUNCTION NAME: I2C_Restart ()
*  PURPOSE: Function initiates repeated start I2C bus start condition.
*  AUTHOR: Jerome Sirotnik
*  COPYRIGHT: (C) Copyright, 2015, Aviation Ground Equipment Corp.  All Rights Reserved.
*  NOTES:
**************************************************************************************************/
void I2C_Restart ( void )
{
   /* do the following to complete the 9th clock pulse from the last write/read */
   SCLK_LOW; /* drive clock line low */

   delayMicroseconds(15); /* minimum bus free time between start and last clock pulse */

   SDATA_HI; /* release data line to float high */

   SCLK_HI; /* release clock line to float high */

   delayMicroseconds(15); /* minimum bus free time between start and stop condition Tbuf */

   SDATA_LOW; /* drive data line low */

   delayMicroseconds(15); /* minimum start condition hold time */
}
/**************************************************************************************************
*  FUNCTION NAME: I2C_Start ()
*  PURPOSE: Function initiates I2C bus start condition.
*  AUTHOR: Jerome Sirotnik
*  COPYRIGHT: (C) Copyright, 2015, Aviation Ground Equipment Corp.  All Rights Reserved.
*  NOTES:
**************************************************************************************************/
void I2C_Start ( void )
{
   SDATA_HI; /* release data line to float high */

   SCLK_HI; /* release clock line to float high */

   delayMicroseconds(10); /* minimum bus free time between start and stop condition Tbuf */

   SDATA_LOW; /* drive data line low */

   delayMicroseconds(10); /* minimum start condition hold time */
}
/**************************************************************************************************
*  FUNCTION NAME: I2C_Stop ()
*  PURPOSE: Function initiates I2C bus stop condition.
*  AUTHOR: Jerome Sirotnik
*  COPYRIGHT: (C) Copyright, 2015, Aviation Ground Equipment Corp.  All Rights Reserved.
*  NOTES:
**************************************************************************************************/
void I2C_Stop ( void )
{
   SCLK_LOW; /* drive clock line low */

   SDATA_LOW; /* drive data line low */

   delayMicroseconds(15); /* minimum low period of clock Tlow */

   SCLK_HI; /* release clock line to float high */

   delayMicroseconds(15); /* minimum set-up time for stop condtion */

   SDATA_HI; /* release data line to float high */
}
/**************************************************************************************************
*  FUNCTION NAME: char I2C_Write ( acknowledge )
*  PURPOSE: Function writes single byte to the I2C bus and checks for ACK.  Returns error if bus
*           error occurs.
*  AUTHOR: Jerome Sirotnik
*  COPYRIGHT: (C) Copyright, 2015, Aviation Ground Equipment Corp.  All Rights Reserved.
*  NOTES:
**************************************************************************************************/
unsigned char I2C_Write ( char data_out )
{


	//if(data_out EQ 0x12)
	{
		//Wire.beginTransmission(0x12); // transmit to LCD Display, Address 0x12
	}
	//else
	{
		Wire.write( &data_out, 8);
	}


	Wire.endTransmission();    // stop transmitting


    return(0);
}


#endif /* GREG_COMMON_H_ */
