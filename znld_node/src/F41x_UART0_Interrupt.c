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
#define FRAME_SRC_ID_S  2	// src address start index
#define FRAME_DEST_ID_S 8	// dest address start index


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

#define UART_BUFFERSIZE 22
U8 UART_Buffer[UART_BUFFERSIZE];
U8 UART_Buffer_Size = 0;
U8 UART_Input_First = 0;
U8 UART_Output_First = 0;
U8 TX_Ready =1;
static char Byte;

//add code @2017-07-08 from eric S
#define FRAME_ID_LEN    6
unsigned char self_id[FRAME_ID_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x05};

//#define FRAME_ID_LEN    6 //move up @2017-07-08 from eric S
unsigned char UART_RX_buffer[UART_BUFFERSIZE];//add define @2017-07-08 from eric S

//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------

void main (void)
{
   PCA0MD &= ~0x40;                    // WDTE = 0 (clear watchdog timer
                                       // enable)

   SYSCLK_Init ();                     // Initialize Oscillator
   PORT_Init();                        // Initialize Port I/O
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
	   P0MDOUT |= 0x10;                    // Enable UTX as push-pull output
	   XBR0     = 0x01;                    // Enable UART on P0.4(TX) and P0.5(RX)
	   XBR1     = 0x40;                    // Enable crossbar and weak pull-ups
	/*
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
   */
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
	unsigned short i;
    U8 msb=0;

	CRC0CN = 0x1d; 		// CRC initial valve 0xFF and enable
	for (i=0; i<20; i++) {
		CRC0IN = UART_RX_buffer[i];
	}

	msb = CRC0DAT;
    return msb;
}

unsigned short crc16_lsb(void)
{
	U8 lsb;

	CRC0CN = 0x14;
	lsb = CRC0DAT;
    return lsb;
}

//-----------------------------------------------------------------------------
// process related add some function defien here @2017-07-08 from Eric S
//-----------------------------------------------------------------------------
void do_nack_response()
{
	return ;
}

void do_poll_response()
{
	return ;
}

void do_lamp_ctrl_response()
{
	return ;
}


//-----------------------------------------------------------------------------
// TX Routines
//-----------------------------------------------------------------------------

// send UART_TX_buffer to UART byte by byte
void send_frame()
{
	return;
}

// duplicate UART_RX_buffer into UART_TX_buffer
void forward_preparation()
{
}

//-----------------------------------------------------------------------------
// rx processing misc routines
//-----------------------------------------------------------------------------
#define RECV_SRC_ID (& UART_RX_buffer[FRAME_SRC_ID_S])
#define RECV_DEST_ID (& UART_RX_buffer[FRAME_DEST_ID_S])
unsigned short is_my_frame()
{
    unsigned short result = 0;
    short i;

    for (i=0; i<FRAME_ID_LEN; i++)
        if (self_id[i] != RECV_DEST_ID[i]) break;
    if (i == FRAME_ID_LEN) result = 1;
    return result;
}

unsigned short is_broadcast_frame()
{
    unsigned short result = 0;
    short i;

    for (i=0; i<FRAME_ID_LEN; i++)
        if (0 != RECV_DEST_ID[i]) break;
    if (i == FRAME_ID_LEN) result = 1;
    return result;
}

void sleep(unsigned short seconds)
{
}

// return a random value in [0, range], note range is included.
unsigned short random(unsigned short range)
{
    unsigned short value = 0;
    return value;
}

#define FRAME_TAG_S     15
#define FRAME_VALUE_S   16
#define FRAME_CRC_S     20
#define RECV_TAG (UART_RX_buffer[FRAME_TAG_S])
#define RECV_VALUE (& UART_RX_buffer[FRAME_VALUE_S])
#define RECV_CRC (& UART_RX_buffer[FRAME_CRC_S])

// Protocol TAG definitions
#define PTAG_ACK        1
#define PTAG_NACK       2
#define PTAG_POLL       3
#define PTAG_POLL_ACK   4
#define PTAG_LAMP_CTRL  5
void rx_cmd_process(unsigned short isBroadcastFrame)
{
    switch (RECV_TAG) {
        // all *_resonse() below need to fill in UART_TX_buffer
        case PTAG_POLL:
            do_poll_response();
            break;
        case PTAG_LAMP_CTRL:
            do_lamp_ctrl_response();
            break;
        default:
            do_nack_response();
    }
    send_frame();
}

//-----------------------------------------------------------------------------
// Frame Process Routines
// When data are ready in UART_RX_buffer, this function will be called.
// Depending on different roles (STA=0, RELAY=1) of node, deal with the data in
// different way.
// STA: process only frames with dest_id == self or dest_id == broadcast_id.
// RELAY: in addition to what STA does, it will relay broadcast frames and frames
// to other nodes (dest_id != self) with some delay.
//-----------------------------------------------------------------------------

//unsigned short self_id[6]; // move up @2017-07-08 from eric

unsigned short role; // 0: STA; 1: RELAY
unsigned short sn; // frame sequence number
#define FRAME_SN 14 // add define @2017-07-8 from eric
#define RECV_SN (UART_RX_buffer[FRAME_SN])
void rx_frame_process(unsigned short role)
{
    unsigned short isMyFrame = is_my_frame();
    unsigned short isBroadcastFrame = is_broadcast_frame();
    unsigned short need_forward = 0;

    // common cmd process for STA & RELAY
    if (isMyFrame || isBroadcastFrame) {
        if (RECV_SN > sn || (RECV_SN == 0 && sn != 0)) {
            sn = RECV_SN;
            need_forward = 1;
            rx_cmd_process(isBroadcastFrame);
        }
    }

    // RELAY specific things
    if (role == 1)
        if (isBroadcastFrame && need_forward) { // change isBroadcastFrame() to isBroadcastFrame @2017-07-08 from eric s
            sleep(random(3));
            forward_preparation();
            send_frame();
        } else if (!isMyFrame) {
            if (RECV_SN > sn || (RECV_SN == 0 && sn != 0)) {
                sn = RECV_SN;
                sleep(1 + random(3));
                forward_preparation();
                send_frame();
            }
        }
// del 1 '}'  @2017-07-08 from eric
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
#define FRAME_SN 14
#define FRAME_LEN 22
INTERRUPT(UART0_Interrupt, 4)
{
	static unsigned short idx = 0;
	IE_ES0 = 0;   // disable UART interrupt

	if (SCON0_RI == 1)
	   {
	      SCON0_RI = 0;                           // Clear interrupt flag

	      Byte = SBUF0;                      // Read a character from UART

	      if (idx == 0) {
	                  if (Byte == FRAME_HEADER) {
	                      UART_RX_buffer[idx++] = Byte;
	                      crc16_init(0xFFFF);
	                      crc16_add(Byte);
	                  }
	              } else if (idx == 1) {
	                  if (Byte == FRAME_HEADER) {
	                      UART_RX_buffer[idx++] = Byte;
	                      crc16_add(Byte);
	                  } else
	                      idx = 0;
	              } else if (idx >= 2 && idx < FRAME_CRC_S) {
	                  UART_RX_buffer[idx++] = Byte;
	                  crc16_add(Byte);
	              } else if (idx < FRAME_LEN && idx >= FRAME_CRC_S) {
	                  UART_RX_buffer[idx++] = Byte;
	              }

	              if (idx == FRAME_LEN) {
	                  if (crc16_msb() == UART_RX_buffer[FRAME_CRC_S] &&
	                         crc16_lsb() == UART_RX_buffer[FRAME_CRC_S + 1])
	                     rx_frame_process(role);
	                  idx = 0;
	              }
	   }

    IE_ES0 = 1;   // enable UART interrupt
}

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
