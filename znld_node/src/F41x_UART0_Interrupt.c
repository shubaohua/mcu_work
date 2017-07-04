//-----------------------------------------------------------------------------
// F41x_UART0_Interrupt.c
//-----------------------------------------------------------------------------
// Copyright 2014 Silicon Laboratories, Inc.
// http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt
//
// Program Description:
//
// This program demonstrates how to configure the C8051F410 to write to and read
// from the UART interface. The program reads a word using the UART interrupts
// and outputs that word to the screen, with all characters in uppercase
//
// How To Test:
//
// 1) Ensure shorting blocks are place on the following:
//    J9:   VUNREG (9V adapter) or 5VEC3 (Debugger) 
//    J10:  +3VD/+3VD
//    J12:  +3VD/VREGIN
//          VREGIN_EN
//    J17:  5VEC3/VIO or +3VD/VIO
//          VIO_EN
//    J27:  TX/P0.4
//          RX/P0.5
// 2) Ensure that the serial cable is connected to the RS232 connector
//    on the target board.
// 3) Specify the target baudrate in the constant <BAUDRATE>.
// 4) Compile and download code to the C8051F410-TB development board by
//    selecting Run -> Debug from the menus, clicking the Debug button in the
//    quick menu, or pressing F11.
// 5) Run the code by selecting Run -> Resume from the menus, clicking the
//    Resume button in the quick menu, or pressing F8. 
// 6) Open Hyperterminal, or a similar program, and connect to the target
//    board's serial port.
// 7) Type up to 64 characters into the Terminal and press Enter.  The MCU
//    will then print back the characters that were typed in uppercase.
//
//
// Target:         C8051F41x
// Tool chain:     Simplicity Studio / Keil C51 9.51
// Command Line:   None
//
// Release 1.2 (BL)
//    - Updated Description / How to Test
//    - Replaced C51 specific code with compiler agnostic code from 
//      compiler_defs.h.
//    - 21 JAN 2014
//
// Release 1.1 / 11 MAR 2010 (GP)
//    -Tested with Raisonance
//
// Release 1.0
//    -Initial Revision (SM)
//    -5 JUN 2007
//

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include <SI_C8051F410_Register_Enums.h>
#include <stdio.h>

//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------

#define SYSCLK      24500000           	// SYSCLK frequency in Hz
#define BAUDRATE        9600           	// Baud rate of UART in bps
#define AUX			   P1_B7			// Descriptive name for P1.7

//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------
INTERRUPT_PROTO(UART0_Interrupt, 4);

void SYSCLK_Init (void);
void UART0_Init (void);
void PORT_Init (void);
void Timer2_Init (S16);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

#define UART_BUFFERSIZE 64
U8 UART_Buffer[UART_BUFFERSIZE];
U8 UART_Buffer_Size = 0;
U8 UART_Input_First = 0;
U8 UART_Output_First = 0;
U8 TX_Ready =1;
static char Byte;

//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------

void main (void)
{
   PCA0MD &= ~0x40;                    // WDTE = 0 (clear watchdog timer
                                       // enable)
   PORT_Init();                        // Initialize Port I/O
   SYSCLK_Init ();                     // Initialize Oscillator
   UART0_Init();

   IE_EA = 1;

   while(1)
   {
      // If the complete word has been entered via the terminal followed by
      // carriage return
      if((TX_Ready == 1) && (UART_Buffer_Size != 0) && (Byte == '\r' || Byte == '\n'))
      {
         TX_Ready = 0;                 // Set the flag to zero
         SCON0_TI = 1;                      // Set transmit flag to 1
      }
   }
}

//-----------------------------------------------------------------------------
// Initialization Subroutines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// PORT_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Configure the Crossbar and GPIO ports.
//
// P0.0	  digital   push-pull    IDA0 (Pin17)
// P0.1   digital   push-pull    IDA1 (Pin18)
// P0.2	  digital   push-pull    TX0 (Pin21)
// P0.3   digital   push-pull    MISO (Pin20)
// P0.4   digital   push-pull    UART TX (Pin21)
// P0.5   digital   open-drain   UART RX (Pin22)
// P0.6   digital	push-pull	 MOSI (Pin23)
// P0.7   digital   push-pull    CS (Pin24)
// P1.0   analog
// P1.1   analog
// P1.2   digital   open-drain   SDA (Pin11)
// P1.3   digital   open-drain	 SCL (Pin12)
// P1.4   digital   push-pull    CEX0 (Pin13)
// P1.5   digital   push-pull    CEX1 (Pin14)
// P1.6   digital   push-pull    IO (Pin15)
// P1.7   digital   open-drain   E32 AUX
// P2.0	  digital   push-pull	 Realy1
// P2.1   digital   push-pull    Relay2
//
//-----------------------------------------------------------------------------

void PORT_Init (void)
{
   P0MDOUT 	= P0MDOUT_B0__PUSH_PULL | P0MDOUT_B1__PUSH_PULL | P0MDOUT_B2__PUSH_PULL
		   	| P0MDOUT_B3__PUSH_PULL | P0MDOUT_B4__PUSH_PULL | P0MDOUT_B5__OPEN_DRAIN
		    | P0MDOUT_B6__PUSH_PULL | P0MDOUT_B7__PUSH_PULL;
   P1MDIN   = P1MDIN_B0__ANALOG  | P1MDIN_B1__ANALOG  | P1MDIN_B2__DIGITAL
            | P1MDIN_B3__DIGITAL | P1MDIN_B4__DIGITAL | P1MDIN_B5__DIGITAL
            | P1MDIN_B6__DIGITAL | P1MDIN_B7__DIGITAL;
   P1MDOUT  = P1MDOUT_B0__OPEN_DRAIN | P1MDOUT_B1__OPEN_DRAIN | P1MDOUT_B2__OPEN_DRAIN
		    | P1MDOUT_B3__OPEN_DRAIN | P1MDOUT_B4__PUSH_PULL  | P1MDOUT_B5__PUSH_PULL
            | P1MDOUT_B6__PUSH_PULL  | P1MDOUT_B7__OPEN_DRAIN;
   // Analog inputs pins should be skipped by the crossbar
   P1SKIP   = P1SKIP_B0__SKIPPED | P1SKIP_B1__SKIPPED | P1SKIP_B2__NOT_SKIPPED
            | P1SKIP_B3__NOT_SKIPPED | P1SKIP_B4__NOT_SKIPPED | P1SKIP_B5__NOT_SKIPPED
            | P1SKIP_B6__NOT_SKIPPED | P1SKIP_B7__NOT_SKIPPED;
   P2MDOUT 	= P2MDOUT_B0__PUSH_PULL | P2MDOUT_B1__PUSH_PULL;

   // Enable UART on P0.4(TX) and P0.5(RX)，I2C & SPI
   XBR0     = XBR0_URT0E__ENABLED | XBR0_SPI0E__ENABLED | XBR0_SMB0E__ENABLED;

   // Enable crossbar, weak pull-ups and CEX0/CEX1
   XBR1     = XBR1_XBARE__ENABLED | XBR1_WEAKPUD__PULL_UPS_ENABLED | XBR1_PCA0ME__CEX0_CEX1;
}

//-----------------------------------------------------------------------------
// SYSCLK_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// This routine initializes the system clock to use the internal oscillator
// at its maximum frequency.
// Also enables the Missing Clock Detector.
//-----------------------------------------------------------------------------

void SYSCLK_Init (void)
{
   OSCICN = 0x87;                      // configure internal oscillator for
                                       // 24.5MHz
   RSTSRC = 0x04;                      // enable missing clock detector
}

//-----------------------------------------------------------------------------
// UART0_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Configure the UART0 using Timer1, for <BAUDRATE> and 8-N-1.
//-----------------------------------------------------------------------------

void UART0_Init (void)
{
   SCON0 = 0x10;                       // SCON0: 8-bit variable bit rate
                                       //        level of STOP bit is ignored
                                       //        RX enabled
                                       //        ninth bits are zeros
                                       //        clear SCON0_RI and SCON0_TI bits
   if (SYSCLK/BAUDRATE/2/256 < 1) {
      TH1 = -(SYSCLK/BAUDRATE/2);
      CKCON |=  0x08;                  // T1M = 1; SCA1:0 = xx
   } else if (SYSCLK/BAUDRATE/2/256 < 4) {
      TH1 = -(SYSCLK/BAUDRATE/2/4);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 01
      CKCON |=  0x01;
   } else if (SYSCLK/BAUDRATE/2/256 < 12) {
      TH1 = -(SYSCLK/BAUDRATE/2/12);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 00
   } else if (SYSCLK/BAUDRATE/2/256 < 48) {
      TH1 = -(SYSCLK/BAUDRATE/2/48);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 10
      CKCON |=  0x02;
   } else {
      while (1);                       // Error.  Unsupported baud rate
   }

   TL1 = TH1;                          // init Timer1
   TMOD &= ~0xf0;                      // TMOD: timer 1 in 8-bit autoreload
   TMOD |=  0x20;
   TCON_TR1 = 1;                       // START Timer1
   TX_Ready = 1;                       // Flag showing that UART can transmit
   IP |= 0x10;						   // Make UART high priority
   IE_ES0 = 1;						   // Enable UART0 interrupts
}


//-----------------------------------------------------------------------------
// CRC16 Service Routines
//-----------------------------------------------------------------------------
void crc16_init(unsigned int seed)
{
}

void crc16_add(unsigned short byte)
{
}

unsigned short crc16_msb(void)
{
    return msb;
}

unsigned short crc16_lsb(void)
{
    return lsb;
}


//-----------------------------------------------------------------------------
// Frame Process Routines
// When data are ready in UART_RX_buffer, this function will be called.
// Depending on different roles (STA=0, RELAY=1) of node, deal with the data in
// different way.
// STA: process only frames with dest_id == self or dest_id == broadcast_id.
// RELAY: in addition to what STA does, it will relay broadcast frames and frames
// to other nodes (dest_id != self).
//-----------------------------------------------------------------------------
void rx_frame_process(unsigned short role)
{
}


//-----------------------------------------------------------------------------
// Interrupt Service Routines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// UART0_Interrupt
//-----------------------------------------------------------------------------
//
// UART receiving routine for protocol frame receiving only
// The process is,
// 1. detect & receive until get HEADER correctly 
// 2. continue receiving bytes until the whole frame is received (22 bytes currently)
// 3. calculate CRC16, if match return the whole frame to RX protocol processing
//-----------------------------------------------------------------------------

#define FRAME_HEADER 0x55
#define FRAME_CRC_S 20
#define FRAME_LEN 22
INTERRUPT(UART0_Interrupt, 4)
{
    static unsigned short idx = 0;
    IE_ES0 = 0;   // disable UART interrupt

    if (SCON0_RI == 1)
    {
        SCON0_RI = 0;                           // Clear interrupt flag
        byte = SBUF0;                      // Read a character from UART
        if (idx == 0) {
            if (byte == FRAME_HEADER) {
                UART_RX_buffer[idx++] = byte;
                crc16_init(0xFFFF);
                crc16_add(byte);
            }
        } else if (idx == 1) {
            if (byte == FRAME_HEADER) {
                UART_RX_buffer[idx++] = byte;
                crc16_add(byte);
            } else
                idx = 0;
        } else if (idx >= 2 && idx < FRAME_CRC_S) {
            UART_RX_buffer[idx++] = byte;
            crc16_add(byte);
        } else if (idx < FRAME_LEN && idx >= FRAME_CRC_S) {
            UART_RX_buffer[idx++] = byte;
        }

        if (idx == FRAME_LEN) {
            if (crc16_msb() == UART_RX_buffer[FRAME_CRC_S] &&
                   crc16_lsb() == UART_RX_buffer[FRAME_CRC_S + 1])
               rx_frame_process(role);
            idx = 0 
        }
    }

    IE_ES0 = 1;   // enable UART interrupt

    /* sample codes for reference only
       if (SCON0_RI == 1)
       {
       if( UART_Buffer_Size == 0)  {      // If new word is entered
       UART_Input_First = 0;    }

       SCON0_RI = 0;                           // Clear interrupt flag

       Byte = SBUF0;                      // Read a character from UART

       if (UART_Buffer_Size < UART_BUFFERSIZE)
       {
       UART_Buffer[UART_Input_First] = Byte; // Store in array

       UART_Buffer_Size++;             // Update array's size

       UART_Input_First++;             // Update counter
       }
       }

       if (SCON0_TI == 1)                   // Check if transmit flag is set
       {
       SCON0_TI = 0;                           // Clear interrupt flag

       if (UART_Buffer_Size != 1)         // If buffer not empty
       {
    // If a new word is being output
    if ( UART_Buffer_Size == UART_Input_First ) {
    UART_Output_First = 0;  }

    // Store a character in the variable byte
    Byte = UART_Buffer[UART_Output_First];

    if ((Byte >= 0x61) && (Byte <= 0x7A)) { // If upper case letter
    Byte -= 32; }

    SBUF0 = Byte;                   // Transmit to Hyperterminal

    UART_Output_First++;            // Update counter

    UART_Buffer_Size--;             // Decrease array size

    }
    else
    {
    UART_Buffer_Size = 0;            // Set the array size to 0
    TX_Ready = 1;                    // Indicate transmission complete
    }
    }
    */ 
}

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
