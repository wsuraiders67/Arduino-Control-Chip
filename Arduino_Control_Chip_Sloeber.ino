#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24_config.h>
#include <SPI.h>
#include "RF24.h"

//#include <conio.h>
//#include <sys.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "/home/greg/eclipse/GREG_libraries/GREG_Common.h"



//#include <dht.h> // DHT22 sensor library from adafruit


//The setup function is called once at startup of the sketch
//void setup()
//{
//  Wire.begin(); // join i2c bus (address optional for master)
//}

//byte x = 0;

//void loop()
//{
//  Wire.beginTransmission(8); // transmit to device #8
//  Wire.write("x is ");        // sends five bytes
//  Wire.write(x);              // sends one byte
//  Wire.endTransmission();    // stop transmitting

//  x++;
//  delay(500);
//}



/****************** Rigol 1054Z SPI Decode scope settings ***************************
Mode = CS (Chip select mode) [NOTE: Timeout mode also works with timeout= 2.0 us]
CS Polarity = LOW
Edge (Clock edge) = Going high
Data Polarity = 1 is high
Width = 8
Order = MSB

***********************************************************************************/

/****************** NRF24 User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 1;
#define ce_pin  46
#define csn_pin 53 //10



/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 46 & 53 */
RF24 radio(ce_pin, csn_pin);

byte addresses[][6] = {"1Node","2Node"};

// Used to control whether this node is sending or receiving
bool role = 1;


/****************************************************************/

/****************** DHT22 Sensor User Config ***************************/
#define DHTPIN 2        // what digital pin we're connected to

#define DHTTYPE DHT22

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

/***********************************************************************/


/* define data array location and structure */
#define DATA_ARRAY_16_BIT_SIZE                          15
static unsigned short Data_Array_16_Bit [ DATA_ARRAY_16_BIT_SIZE ];
#define Event_Description                               Data_Array_16_Bit [0]
#define LMCO_COMM_Command_Old                           Data_Array_16_Bit [1] /* LMCO Command code */
#define LMCO_Fault                                      Data_Array_16_Bit [2] /* LMCO_Fault code */
#define Front_Panel_Status                              Data_Array_16_Bit [3]

#define Elapsed_Time_Minutes_Timer                      Data_Array_16_Bit [6]
#define AC_Phase_A_B_Volts_Input                        Data_Array_16_Bit [7]
#define AC_Phase_B_C_Volts_Input                        Data_Array_16_Bit [8]
#define AC_Phase_C_A_Volts_Input                        Data_Array_16_Bit [9]
#define AC_Phase_A_Current_Input                        Data_Array_16_Bit [10]
#define AC_Phase_B_Current_Input                        Data_Array_16_Bit [11]
#define AC_Phase_C_Current_Input                        Data_Array_16_Bit [12]
#define IPC_AC_Input_Output_Frequency                   Data_Array_16_Bit [13]
#define LCD_Comm_Error_Count                            Data_Array_16_Bit [14]


/****************** I2C Config ***************************/
/* declare LMCO_COMM variables */
static unsigned char LMCO_COMM_Packet [ 12 ]; /* set packet size = size + 1 */
static unsigned char LMCO_COMM_Command;
static unsigned char LMCO_COMM_Error;
static unsigned char LMCO_Comm_Fault;
static unsigned char LMCO_Warning;
static bool LMCO_COMM_Read_Write_Flag;
static unsigned char I2C_Byte_One;
static unsigned char I2C_Byte_Two;
static unsigned char I2C_Byte_Three;
static unsigned char I2C_Byte_Four;

#define I2C_Data_Pin 	20
#define I2C_Clock_Pin 	21

// Addresses
#define LCD_DISPLAY_I2C_ADDR            0x24 /* 12: LCD Display Board address (7 MSB) */


static unsigned char LCD_Status;

static unsigned char LCD_Data_Array_Index;


/***********************************************************************/

/****************** Function Prototypes ***************************/

    // check sensors  // write a function for this
    // communicate with slaves  // write a function for this

    //unsigned char Read_Temp_Humidity_Press_Sensor ( void )
    //{
    //}
void Initialize_LMCO_COMM_Packet ( void );
void Generate_Packet_Checksum ( void );
char LMCO_COMM_Write ( unsigned char address );
char LMCO_COMM_Read ( unsigned char address );
unsigned char Test_Packet_Checksum ( void );
unsigned char Transmit_Command ( unsigned char slave_address );

//#include <GREG_AGEC_I2C_Comm_Mega2560.cpp>


/****************** Global Variable ***************************/
int Serial_TestValue_1;
int Serial_TestValue_2;


/**************************************************************************************************
*  FUNCTION NAME: Test_Packet_Checksum
*  PURPOSE:
*  AUTHOR: Jerome Sirotnik
*  COPYRIGHT: (C) Copyright, 2015, Aviation Ground Equipment Corp.  All Rights Reserved.
*  NOTES:
**************************************************************************************************/
unsigned char Test_Packet_Checksum ( void )
{
   unsigned char checksum;
   unsigned char error;

   error = 0;

   checksum = LMCO_COMM_Packet [ COMMAND ] +
              LMCO_COMM_Packet [ DATA1 ] +
              LMCO_COMM_Packet [ DATA2 ] +
              LMCO_COMM_Packet [ DATA3 ] +
              LMCO_COMM_Packet [ DATA4 ] +
              LMCO_COMM_Packet [ DATA5 ] +
              LMCO_COMM_Packet [ DATA6 ];

   checksum = ~ checksum; /* invert checksum */

   if (( checksum EQ LMCO_COMM_Packet [ DATA6 + 1 ] ))
   {
      error = 0;
   }
   else
   {
      error = 1; /* set mismatch error */
   }

   return ( error );
}
/**************************************************************************************************
*  FUNCTION NAME: Initialize_LMCO_COMM_Packet ( void )
*  PURPOSE:
*  AUTHOR: Jerome Sirotnik
*  COPYRIGHT: (C) Copyright, 2015, Aviation Ground Equipment Corp.  All Rights Reserved.
*  NOTES:
**************************************************************************************************/
void Initialize_LMCO_COMM_Packet ( void )
{
   unsigned char i;

   /* init LMCO_COMM packet data */

   for ( i = COMMAND; i < DEVICE_STATUS + 2; i++ )
   {
      LMCO_COMM_Packet [ i ] = 0; /* clear value */
   }
}
/**************************************************************************************************
*  FUNCTION NAME: Generate_Packet_Checksum
*  PURPOSE:
*  AUTHOR: Jerome Sirotnik
*  COPYRIGHT: (C) Copyright, 2015, Aviation Ground Equipment Corp.  All Rights Reserved.
*  NOTES:
**************************************************************************************************/
void Generate_Packet_Checksum ( void )
{
   unsigned char checksum;
   LMCO_COMM_Packet [ COMMAND + 1 ] = LMCO_COMM_Packet [ COMMAND ]; /* copy command for double send */

   checksum = LMCO_COMM_Packet [ COMMAND ] +
              LMCO_COMM_Packet [ DATA1 ] +
              LMCO_COMM_Packet [ DATA2 ] +
              LMCO_COMM_Packet [ DATA3 ] +
              LMCO_COMM_Packet [ DATA4 ] +
              LMCO_COMM_Packet [ DATA5 ] +
              LMCO_COMM_Packet [ DATA6 ];

   checksum = ~ checksum; /* invert checksum */

   LMCO_COMM_Packet [ DATA6 + 1 ] = checksum;
}
/**************************************************************************************************
*  FUNCTION NAME: Process_Data ( void )
*  PURPOSE: Reads received data and status and saves to memory.
*  AUTHOR: Jerome Sirotnik
*  COPYRIGHT: (C) Copyright, 2015, Aviation Ground Equipment Corp.  All Rights Reserved.
*  NOTES: All LMCO_COMM commands < 100 return real time data values.
**************************************************************************************************/
void Process_Data ( unsigned char slave_address )
{
   /* define temp local variables */
   unsigned char temp;
   unsigned char temp2;
   unsigned short temp_short;
   unsigned long temp_long;
   unsigned long temp_long2;
   float temp_float;
   unsigned char packet_number;
   unsigned char temp_warning;
   unsigned char ecb_test_data_packet_number;

   static unsigned short hz400_frequency_old1 = 0;
   static unsigned short hz400_frequency_old2 = 0;
   static unsigned short hz400_frequency_old3 = 0;
   static unsigned char lcd_packet_number;

   unsigned char iob_data_array_index = 0;


   /* do if master write */
// if ( LMCO_COMM_Packet [ DEVICE_STATUS ] < 100 )
// {
//   /* master write only, do not save data */   //// fault?
// }
// /* do if master read command */
// else


   if ( LMCO_COMM_Packet [ COMMAND ] < 100 ) /* save real time data */
   {

	   /* execute state machine based on slave address */
      switch ( slave_address )
      {
         case LCD_DISPLAY_I2C_ADDR:
         {
            packet_number = LMCO_COMM_Packet [ DATA1 ] & 0b00000111;

            switch ( packet_number )
            {
               case 0:
               {
//                  LCD_Screen = LMCO_COMM_Packet [ DATA3 ]; /* read value */
//                  LCD_Adjustments_Screen_Listbox_Item = LMCO_COMM_Packet [ DATA4 ]; /* read value */

                  temp =  LMCO_COMM_Packet [ DATA5 ]; /* read value */

                  if ( temp & 0x01 )
                  {
//                        LCD_Adjustments_Screen_Radiobutton = 1;
                  }
                  else
                  {
//                        LCD_Adjustments_Screen_Radiobutton = 0;
                  }

//                  Spare_Array_8 = LMCO_COMM_Packet [ DATA6 ]; /* read value */
                  break;
               }
               case 1:
               {

//                  Spare_Array_9 = LMCO_COMM_Packet [ DATA2 ]; /* read value */
temp_short =  LMCO_COMM_Packet [ DATA3 ]; /* read value */
temp_short = temp_short << 8;
temp_short = temp_short + LMCO_COMM_Packet [ DATA4 ]; /* read value */
//Spare_Array_4 = temp_short;

temp_short =  LMCO_COMM_Packet [ DATA5 ]; /* read value */
temp_short = temp_short << 8;
temp_short = temp_short + LMCO_COMM_Packet [ DATA6 ]; /* read value */
//Spare_Array_5 = temp_short;

                  break;
               }
               case 2:
               {
                  //Spare_Array_52 = LMCO_COMM_Packet [ DATA3 ]; /* read value */
                  //Spare_Array_53 = LMCO_COMM_Packet [ DATA4 ]; /* read value */
                  //Spare_Array_54 = LMCO_COMM_Packet [ DATA5 ]; /* read value */
                  //Spare_Array_55 = LMCO_COMM_Packet [ DATA6 ]; /* read value */

                  break;
               }
               case 3:
               {
                  //Spare_Array_52 = LMCO_COMM_Packet [ DATA3 ]; /* read value */
                  //Spare_Array_53 = LMCO_COMM_Packet [ DATA4 ]; /* read value */
                  //Spare_Array_54 = LMCO_COMM_Packet [ DATA5 ]; /* read value */
                  //Spare_Array_55 = LMCO_COMM_Packet [ DATA6 ]; /* read value */

                  break;
               }
               default:
               {
                  break;
               }

            }
            break;
         }

         default:
         {


        	 break;
         }
      }
   }


}
/**************************************************************************************************
*  FUNCTION NAME: Transmit_Command ( void )
*  PURPOSE: Loads any needed command data and sends command on LMCO_COMM.
*  AUTHOR: Greg Chaney
*  COPYRIGHT:
*  NOTES:
**************************************************************************************************/
unsigned char Transmit_Command ( unsigned char slave_address )
{
   /* define temp local variables */
   unsigned char comm_fault;
   unsigned char i;
   unsigned char temp;
   unsigned char index;
   unsigned short temp_short;

   static unsigned char lcd_packet_number = 0;

// LMCO_COMM_Enable_1_Out = ON; /* connect LMCO_COMM to I2C bus */

   comm_fault = 0; /* init comm error value */

   /* init LMCO_COMM packet data */
   Initialize_LMCO_COMM_Packet ();

   LMCO_COMM_Packet [ COMMAND ] = LMCO_COMM_Command; /* load command value */

   LMCO_COMM_Read_Write_Flag = WRITE; /* init flag for master data write to slave */

   /* execute state machine based on command */
   switch ( LMCO_COMM_Packet [ COMMAND ] )
   {
		/* AGEC REFERENCE BELOW
			  case CNV_DC270V_MAIN_MANUAL_LINE_DROP_ADJ_VALUE_CMD:
			  {
				 temp = ( CNV_DC270V_MAIN_BOARD_I2C_ADDR << 2 ) & 0b11111000; 	// keep bits 5:1 of address and shift
				 temp = temp | 0b00000001; 	// set data packet number
				 LMCO_COMM_Packet [ DATA1 ] = temp;

				 LMCO_COMM_Packet [ DATA2 ] = Config_DC270V_Line_Drop_Adjust_Value; 	// load value
				 break;
			  }
        */
   	   case LCD_MENU_DATA_CMD:
   	   {

   		      lcd_packet_number = 1;

   		      temp = ( LCD_DISPLAY_I2C_ADDR << 2 ) & 0b11111000; /* keep bits 5:1 of address and shift */
			  temp = temp | ( lcd_packet_number & 0b00000111); /* set data packet number */
			  LMCO_COMM_Packet [ DATA1 ] = temp;

			  lcd_packet_number = 1;

			  /* send all data array values based on index */
/*			  if ( LCD_Data_Array_Index < ( DATA_ARRAY_16_BIT_SIZE - 1 ))
			  {
				  LCD_Data_Array_Index ++;
			  }
			  else
			  {
				  LCD_Data_Array_Index = 0;
			  }

			  LMCO_COMM_Packet [ DATA2 ] = LCD_Data_Array_Index;

			  temp_short = Data_Array_16_Bit [ LCD_Data_Array_Index ];
			  LMCO_COMM_Packet [ DATA3 ] = (temp_short >> 8); // return high byte of data
			  LMCO_COMM_Packet [ DATA4 ] = temp_short; // return low byte of data
*/
			  LMCO_COMM_Packet [ DATA2 ] = Data_Array_16_Bit [0];
			  LMCO_COMM_Packet [ DATA3 ] = Data_Array_16_Bit [1];
			  LMCO_COMM_Packet [ DATA4 ] = Data_Array_16_Bit [2];




			  break;
   	   }
      default:
      {
         /* all other commands are master read, set flag = READ */
         LMCO_COMM_Read_Write_Flag = READ; /* set flag for master data read from slave */
         break;
      }
   }

   /* copy command packet information for double send */
   LMCO_COMM_Packet [ COMMAND + 1 ] = LMCO_COMM_Packet [ COMMAND ]; /* copy command for double send */


   /* do if master write command */
   if ( LMCO_COMM_Read_Write_Flag EQ WRITE )
   {
      /* copy packet data information for double send */
      Generate_Packet_Checksum ();

      comm_fault = LMCO_COMM_Write ( slave_address ); /* master write command to LMCO_COMM */
      //comm_fault = 0;
   }
   else
   {
      comm_fault = LMCO_COMM_Read ( slave_address ); /* master read command to LMCO_COMM */
      comm_fault = 0;
   }

   /* do if master read command */
   if ( LMCO_COMM_Packet [ COMMAND ] < 100 ) /* save real time data */
   {
      if ((( LMCO_COMM_Packet [ DATA1 ] & 0b11111000 ) >> 2 ) NE slave_address )
      {
         comm_fault ++;
//       error = 3; /* set address error */
      }
   }

   return ( comm_fault ); /* return comm fault */
}
/**************************************************************************************************
*  FUNCTION NAME: LMCO_COMM_Send_Packet_Command
*  PURPOSE: Perform LMCO_COMM protocol packet command write to slave.
*  AUTHOR: Jerome Sirotnik
*  COPYRIGHT: (C) Copyright, 2015, Aviation Ground Equipment Corp.  All Rights Reserved.
*  NOTES:  All LMCO_COMM commands, data and, statuses are all double sent/received.
**************************************************************************************************/
unsigned char LMCO_COMM_Send_Packet_Command ( unsigned char address )
{
   unsigned char error;

   I2C_Start (); /* send I2C start bit */
   error = I2C_Write ( address ); /* send address */


   /* do if not I2C error */
   if ( NOT error )
   {
      error = I2C_Write ( LMCO_COMM_Packet [ COMMAND ] ); /* write command */
   }

   /* do if not I2C error */
   if ( NOT error )
   {
      error = I2C_Write ( LMCO_COMM_Packet [ COMMAND + 1 ] ); /* resend command */
   }
   return ( error );
}
/**************************************************************************************************
*  FUNCTION NAME: LMCO_COMM_Write
*  PURPOSE: Perform LMCO_COMM protocol write to slave.
*  AUTHOR: Jerome Sirotnik
*  COPYRIGHT: (C) Copyright, 2015, Aviation Ground Equipment Corp.  All Rights Reserved.
*  NOTES:  All LMCO_COMM commands, data and, statuses are all double sent/received.
**************************************************************************************************/
char LMCO_COMM_Write ( unsigned char address )
{
   /* define temp local variables */
   unsigned char i;
   unsigned char error;

   error = LMCO_COMM_Send_Packet_Command ( address );

   /* do if not I2C error */
// if ( NOT error )
// {
//    I2C_Restart (); /* send I2C restart bit */
//    error = I2C_Write ( address ); /* send address to indicate write operation */
// }

   i = DATA1; /* init index */

   /* do while no error and not done */
   while ( NOT error AND ( i < DEVICE_STATUS ))
   {

	   error = I2C_Write ( LMCO_COMM_Packet [ i ] ); /* write data */
      i++; /* increment index */

      Serial_TestValue_1 = error;
   }

   /* do if not I2C error */
   if ( NOT error )
   {
      if ( address EQ GENERAL_CALL_ADDRESS )
      {
         I2C_Stop (); /* send I2C stop bit */
      }
      else
      {
         I2C_Restart (); /* send I2C restart bit */
         error = I2C_Write ( address + 1); /* send address + 1 to indicate read */

         if ( NOT error )
         {
            error = LMCO_COMM_Read_Packet_Status ( address );
         }
      }
   }
   return ( error );
}
/**************************************************************************************************
*  FUNCTION NAME: LMCO_COMM_Read
*  PURPOSE: Perform LMCO_COMM protocol read from slave.
*  AUTHOR: Jerome Sirotnik
*  COPYRIGHT: (C) Copyright, 2015, Aviation Ground Equipment Corp.  All Rights Reserved.
*  NOTES:  All LMCO_COMM commands, data and, statuses are all double sent/received.
**************************************************************************************************/
char LMCO_COMM_Read ( unsigned char address )
{
   /* define temp local variables */
   unsigned char i;
   unsigned char temp_buffer;
   unsigned char error;

   error = LMCO_COMM_Send_Packet_Command ( address );

   /* do if not I2C error */
   if ( NOT error )
   {
      I2C_Restart (); /* send I2C restart bit */
      error = I2C_Write ( address + 1 ); /* write address + 1 to indicate read */
   }

   i = DATA1; /* init index */


   /* do while no error and not done */
   while ( NOT error AND ( i < DEVICE_STATUS ))
   {
      if ( i EQ ( DATA6 + 1 ))
      {
         /* added delay when communicating over LMCO_COMM */
         SCLK_LOW; /* drive clock line low */
         Delay_25_Us (); /* minimum low period of clock Tlow */

         error = I2C_Read ( &temp_buffer, END ); /* read value */
      }
      else
      {
         /* added delay when communicating over LMCO_COMM */
         SCLK_LOW; /* drive clock line low */
         Delay_25_Us (); /* minimum low period of clock Tlow */

         error = I2C_Read ( &temp_buffer, CONTINUE ); /* read value */
      }

      LMCO_COMM_Packet [ i ] = temp_buffer; /* store value in array */
      i++; /* increment index */
   }

   if ( NOT error )
   {
      I2C_Restart (); /* send I2C restart bit */
      error = I2C_Write ( address + 1); /* send address + 1 to indicate read */


      if ( NOT error )
      {
         error = LMCO_COMM_Read_Packet_Status ( address );
      }

   }
   return ( error );
}
/**************************************************************************************************
*  FUNCTION NAME: LMCO_COMM_Read_Packet_Status
*  PURPOSE: Perform LMCO_COMM protocol packet status read from slave.
*  AUTHOR: Jerome Sirotnik
*  COPYRIGHT: (C) Copyright, 2015, Aviation Ground Equipment Corp.  All Rights Reserved.
*  NOTES:  All LMCO_COMM commands, data and, statuses are all double sent/received.
**************************************************************************************************/
unsigned char LMCO_COMM_Read_Packet_Status ( unsigned char address )
{
   unsigned char temp_buffer;
   unsigned char error;

   /* added delay when communicating over LMCO_COMM */
   SCLK_LOW; /* drive clock line low */
   Delay_25_Us (); /* minimum low period of clock Tlow */

   error = I2C_Read ( &temp_buffer, CONTINUE ); /* read value */

   LMCO_COMM_Packet [ DEVICE_STATUS ] = temp_buffer; /* store value in array */


   /* do if not I2C error */
   if ( NOT error )
   {
      /* added delay when communicating over LMCO_COMM */
      SCLK_LOW; /* drive clock line low */
      Delay_25_Us (); /* minimum low period of clock Tlow */

      error = I2C_Read ( &temp_buffer, END); /* read last value and send "end" bit */
      LMCO_COMM_Packet [ DEVICE_STATUS + 1 ] = temp_buffer; /* store value in array */
      I2C_Stop (); /* send I2C stop bit */
   }

   /* do if any byte mismatch or invalid status */
   error = Test_Packet_Checksum ();

   if (( LMCO_COMM_Packet [ DATA1 ] & 0b10000000 ) EQ 0 )
   {
//    error = 2; /* set frame error */
   }


   if ((( LMCO_COMM_Packet [ DATA1 ] & 0b01111000 ) >> 2 ) NE address )
   {
//    if (address NE U23_PROCESSOR_ADDR)
      {
//       error = 3; /* set address error */
      }
   }

   return ( error );
}
/**************************************************************************************************
*  FUNCTION NAME: Process_Master_LMCO_Commands
*  PURPOSE: Sends LMCO_COMM command to all slaves.  Sends/receives all command data.  Loads LMCO_COMM
         and LMCO system faults.
*  AUTHOR: Jerome Sirotnik
*  COPYRIGHT: (C) Copyright, 2015, Aviation Ground Equipment Corp.  All Rights Reserved.
*  NOTES:
**************************************************************************************************/
void Process_Master_LMCO_Commands ( void )
{
	   /* define temp local variables */
	   unsigned char comm_fault;
	   static unsigned char lmco_comm_command_old;
	   static unsigned char command = 0;

	   unsigned char i;

	   LMCO_Comm_Fault = 0; /* init comm fault value */

//	   Get_Next_Master_LMCO_Command (); /* load next command to be sent on LMCO_COMM */

	   /* disable both external I2C busses */
//	   LMCO_COMM_Enable_1_Out = 0;
//	   LMCO_COMM_Enable_2_Out = 0;



	   /* get command data if needed and send command on LMCO_COMM */
//	   comm_fault = Transmit_Command ( LCD_DISPLAY_I2C_ADDR ); /* transmit to slave */

	   /* do if no comm fault */
//	   if ( NOT comm_fault )
//	   {
//		  LCD_Status = LMCO_COMM_Packet [ DEVICE_STATUS ]; /* save slave device status */
//		  Process_Data ( LCD_DISPLAY_I2C_ADDR ); /* process slave data */
//	   }
//	   else
//	   {
//		  LCD_Status = LMCO_COMM_FAULT_STATUS; /* save comm fault status*/
//		  LCD_Comm_Error_Count ++;
//	   }

		Delay_25_Us ();


		//LCD_Status = 199;  // xxxxyyyy   uncommenting this causes random errors
//		if ( LCD_Status LE FAULT_STATUS_RANGE )
//		{
		//Test_A4_Out = 1;
//		}


		LMCO_COMM_Command = LCD_MENU_DATA_CMD;

		// Updating LCD PIC32 micro data array

		if ( LCD_Status NE LMCO_COMM_FAULT_STATUS )
		{

		   lmco_comm_command_old = LMCO_COMM_Command;

		   /* Data_Array Information from 'Control Chip' to 'LCD Chip' */  // xxx
		   LMCO_COMM_Command = LCD_MENU_DATA_CMD;

		   /* get command data if needed and send command on LMCO_COMM */
		   comm_fault = Transmit_Command ( LCD_DISPLAY_I2C_ADDR ); /* transmit to slave */

		   /* do if no comm fault */
		   if ( NOT comm_fault )
		   {
			  LCD_Status = LMCO_COMM_Packet [ DEVICE_STATUS ]; /* save slave device status */
			  Process_Data ( LCD_DISPLAY_I2C_ADDR ); /* process slave data */

			  //digitalWrite(Test_Red_LED, LOW);
			  //digitalWrite(Test_Green_LED, HIGH);
		   }
		   else
		   {
			  LCD_Status = LMCO_COMM_FAULT_STATUS; /* save comm fault status*/
			  LCD_Comm_Error_Count ++;

			  //digitalWrite(Test_Red_LED, HIGH);
			  //digitalWrite(Test_Green_LED, LOW);
		   }

		   LMCO_COMM_Command = lmco_comm_command_old;
		}





}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{

  unsigned char i;

  pinMode(Test_Green_LED, OUTPUT);
  pinMode(Test_Red_LED, OUTPUT);

  //pinMode(I2C_Data_Pin, OUTPUT);
  //pinMode(I2C_Clock_Pin, OUTPUT);

//  PORTD &= ~ ((1 << 1) | (1 << 0));

    //I2C_CLOCK_HI();
//    DDRD &= ~ (1 << 0);
//    PORTD |= (1 << 0);

    //I2C_DATA_HI();
//    DDRD &= ~ (1 << 1);
//    PORTD |= (1 << 1);

   delay(1);


  //TWCR |= 0x04;


  pinMode(19, OUTPUT);

  digitalWrite(Test_Green_LED, LOW);
  digitalWrite(Test_Red_LED, LOW);

  pinMode(7, INPUT);


  Serial.begin(9600);

  dht.begin(); // Start DHT22 Sensor Data collection


  for ( i = 0; i < sizeof ( Data_Array_16_Bit ); i++ )
  {
        Data_Array_16_Bit [i] = 0;
  }

  Wire.begin(); // join i2c bus (address optional for master)

  //LMCO_Fault = 5;	//TEST xxx

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{


//digitalWrite(Test_Green_LED, HIGH);
//digitalWrite(Test_Red_LED, LOW);
//delay(1000);


//digitalWrite(Test_Red_LED, HIGH);
//digitalWrite(Test_Green_LED, LOW);
//delay(1000);

	digitalWrite(19, HIGH);
	delayMicroseconds(5);
	if((PIND & (1 << PD2)))
	{
		//digitalWrite(Test_Green_LED, HIGH);
	}
	else
	{
		//digitalWrite(Test_Green_LED, LOW);
	}

	Process_Master_LMCO_Commands();
	delay(100);		//delay 100ms


/*
	  Wire.beginTransmission(0x12); // transmit to LCD Display, Address 0x12
	  Wire.write(LCD_MENU_DATA_CMD);              // sends one byte
	  Wire.write(LCD_MENU_DATA_CMD);              // sends one byte
	  Wire.write(5);              // sends one byte
	  Wire.write(5);
	  Wire.write(1);
	  Wire.write(2);
	  Wire.write(3);
	  Wire.endTransmission();    // stop transmitting
*/

/****************** Change Roles via Serial Commands ***************************/

/*
  if ( Serial.available() )
  {
    char c = toupper(Serial.read());
    if ( c == 'T' && role == 0 ){
      Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
      role = 1;                  // Become the primary transmitter (ping out)

   }else
    if ( c == 'R' && role == 1 ){
      Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));
       role = 0;                // Become the primary receiver (pong back)
       radio.startListening();

    }
  }
*/

///*
	  //if ( Serial.available() )
	  //{
		  // print the results to the serial monitor:

//check how to do no-ops properly on avr

		  //digitalWrite(Test_Green_LED, HIGH);
		  Serial.print("sensor = ");
		  Serial.print(Serial_TestValue_1);
		  Serial.print("\t output = ");
		  Serial.println(Serial_TestValue_2);
	  //}
//*/

} // Loop


