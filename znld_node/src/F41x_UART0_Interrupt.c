/**
 * @file F41x_UART0_Interrupt.c
 *
 * @brief ZNLD node application
 *
 * This program provide all functionality for ZNLD node application.
 *
 * Target:         C8051F41x
 * Tool chain:     Simplicity Studio / Keil C51 9.53
 * Command Line:   None
 *
 * @author  Mike Qin (laigui)
 * @author  Wayne Chen (wayne2000)
 * @author  Eric Shu (shubaohua)
 * @author  Hongbo Lu (lu7120bobo)
 * @version 1.0
 * @date    2017-07-10
 * @bug     AUX not functional.
 *
 * Revision history:
 * $Log:$
 *
 */

#include <SI_C8051F410_Register_Enums.h>
#include <stdio.h>
#include <stdlib.h>         // for rand() usage


#define SYSCLK              24500000    ///< SYSCLK frequency in Hz
#define BAUDRATE            9600        ///< Baud rate of UART in bps
#define AUX                 P1_B7       ///< Descriptive name for P1.7
#define FRAME_SRC_ID_S      2           ///< src address start index
#define FRAME_DEST_ID_S     8           ///< dest address start index
#define FRAME_SN            14      
#define RECV_SN             (UART_RX_buffer[FRAME_SN])
#define FRAME_TAG_S         15
#define FRAME_VALUE_S       16
#define FRAME_CRC_S         20
#define RECV_TAG            (UART_RX_buffer[FRAME_TAG_S])
#define RECV_VALUE          (& UART_RX_buffer[FRAME_VALUE_S])
#define RECV_CRC            (& UART_RX_buffer[FRAME_CRC_S])
#define CMD_LAMP_ALL_OFF    0x00
#define CMD_LAMP_LINE1_ON   0x01
#define CMD_LAMP_LINE2_ON   0x02
#define CMD_LAMP_ALL_ON     0x03
#define CMD_X_IDX           0
#define CMD_Y_IDX           1
#define CMD_Z_IDX           2
#define POLL_BACK_TAG       0X04
#define UART_BUFFERSIZE     22
#define FRAME_ID_LEN        6
#define PACKAGE_HEAD_BYTe_1 0x55
#define PACKAGE_HEAD_BYTe_2 0x55
#define REAPTOR_WAITING_TIME_DURATION 0x03		// normally should be 3 second for repeator delay


INTERRUPT_PROTO(UART0_Interrupt, 4);

void SYSCLK_Init (void);
void UART0_Init (void);
void PORT_Init (void);
void Timer2_Init (S16);
void rx_frame_process(unsigned short role);
void send_frame(void);
U8 random(unsigned char range);
void sleep(unsigned char seconds);


//U8 UART_Buffer[UART_BUFFERSIZE]; // no using so -->
U8 UART_Buffer_Size = 0;
U8 UART_Input_First = 0;
U8 UART_Output_First = 0;
U8 RX_Ready =0; // 0: no receive frm ready in UART_RX_buffer or. no need to do process; 1: need to do receive frm process
static char Byte;


unsigned char self_id[FRAME_ID_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x05};
unsigned char source_id[FRAME_ID_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01};

unsigned char poll_package[UART_BUFFERSIZE];      ///< send back package buffer
unsigned char repeator_package[UART_BUFFERSIZE];  ///< repeator pckage buffer
unsigned char UART_RX_buffer[UART_BUFFERSIZE];
unsigned char role; //0: that is normal node; 1: that is repeator ndoe;  0: STA; 1: RELAY
unsigned char sn; // frame sequence number

//unsigned char flag_receive_frm_ready; // 0: no receive frm ready in UART_RX_buffer or. no need to do process; 1: need to do receive frm process
unsigned char flag_send_bck_frm_ready; // 0: no need send message to source destination; 1: need to send message to source destination
unsigned char flag_send_repeat_frm_ready; //0: no need send reapeat frm; 1: need to send repeat frm

/**
 * @brief Main loop
 *
 * @return void
 */
void main (void)
{
  PCA0MD &= ~0x40;                    // WDTE = 0 (clear watchdog timer enable)

  SYSCLK_Init();                      // Initialize Oscillator
  PORT_Init();                        // Initialize Port I/O
  UART0_Init();

  //initialize flag
  //flag_receive_frm_ready = 0;
  flag_send_bck_frm_ready = 0;
  flag_send_repeat_frm_ready = 0;

  IE_EA = 1;

  while (1)
  {
    // If the complete word has been entered via the terminal followed by
    // carriage return
    if (RX_Ready == 1) {
      RX_Ready = 0;                   // Set the flag to zero, means complete all cmd processing
      rx_frame_process(role);
    }

    send_frame();  // handling send message involving send back & repeat frm together

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
// P0.0   digital   push-pull    IDA0 (Pin17)
// P0.1   digital   push-pull    IDA1 (Pin18)
// P0.2   digital   push-pull    TX0 (Pin21)
// P0.3   digital   push-pull    MISO (Pin20)
// P0.4   digital   push-pull    UART TX (Pin21)
// P0.5   digital   open-drain   UART RX (Pin22)
// P0.6   digital push-pull  MOSI (Pin23)
// P0.7   digital   push-pull    CS (Pin24)
// P1.0   analog
// P1.1   analog
// P1.2   digital   open-drain   SDA (Pin11)
// P1.3   digital   open-drain  SCL (Pin12)
// P1.4   digital   push-pull    CEX0 (Pin13)
// P1.5   digital   push-pull    CEX1 (Pin14)
// P1.6   digital   push-pull    IO (Pin15)
// P1.7   digital   open-drain   E32 AUX
// P2.0   digital   push-pull  Realy1
// P2.1   digital   push-pull    Relay2
//
//-----------------------------------------------------------------------------

void PORT_Init (void)
{
  P0MDOUT |= 0x10;                    // Enable UTX as push-pull output
  P2MDOUT  = P2MDOUT_B0__PUSH_PULL | P2MDOUT_B1__PUSH_PULL;
  // Enable UART on P0.4(TX) and P0.5(RX)，I2C & SPI
  XBR0     = XBR0_URT0E__ENABLED | XBR0_SPI0E__ENABLED | XBR0_SMB0E__ENABLED;
  // Analog inputs pins should be skipped by the crossbar
  P0SKIP   = 0x03;
  P1SKIP   = P1SKIP_B0__SKIPPED | P1SKIP_B1__SKIPPED | P1SKIP_B2__NOT_SKIPPED
    | P1SKIP_B3__NOT_SKIPPED | P1SKIP_B4__NOT_SKIPPED | P1SKIP_B5__NOT_SKIPPED
    | P1SKIP_B6__NOT_SKIPPED | P1SKIP_B7__NOT_SKIPPED; // 0x03
  // Enable crossbar, weak pull-ups and CEX0/CEX1
  XBR1     = XBR1_XBARE__ENABLED | XBR1_WEAKPUD__PULL_UPS_ENABLED | XBR1_PCA0ME__CEX0_CEX1;
  REF0CN   = 0x18;
  PCA0CN   = 0x40;
  PCA0MD   = 0;
  PCA0CPM0 = 0x42;
  PCA0CPM1 = 0x42;
  PCA0CPL0 = 0;
  PCA0CPH0 = 0;
  PCA0CPL1 = 0;
  PCA0CPH1 = 0;
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
  RX_Ready = 0;                       // Flag showing that UART can transmit
  IP |= 0x10;         // Make UART high priority
  IE_ES0 = 1;         // Enable UART0 interrupts
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

  CRC0CN = 0x1d;   // CRC initial valve 0xFF and enable
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
void do_nack_response(void)
{
  //unsigned char send_back_frm_number = 0;
  unsigned char i;

  poll_package[0] = PACKAGE_HEAD_BYTe_1;
  poll_package[1] = PACKAGE_HEAD_BYTe_2;

  poll_package[2] = self_id[0];
  poll_package[3] = self_id[1];
  poll_package[4] = self_id[2];
  poll_package[5] = self_id[3];
  poll_package[6] = self_id[4];
  poll_package[7] = self_id[5];

  poll_package[8] =  source_id[0];
  poll_package[9] =  source_id[1];
  poll_package[10] =  source_id[2];
  poll_package[11] =  source_id[3];
  poll_package[12] =  source_id[4];
  poll_package[13] =  source_id[5];

  poll_package[14] =  RECV_SN + 0x01; // frame number add 1

  poll_package[15] =  2;//NACK_TAG;  // response tag number

  poll_package[16] =  0x00;//RECV_VALUE[CMD_X_IDX];
  poll_package[17] =  0x00;//RECV_VALUE[CMD_Y_IDX];
  poll_package[18] =  0x00;//RECV_VALUE[CMD_Z_IDX];
  poll_package[19] =  0x00;


  CRC0CN = 0x1d;   // CRC initial valve 0xFF and enable
  for (i=0; i<20; i++) {
    CRC0IN = poll_package[i];
  }
  poll_package[20] =  CRC0DAT;
  CRC0CN = 0x14;
  poll_package[21] =  CRC0DAT;

  return ;
}

void do_poll_response(void)
{
  unsigned char send_back_frm_number = 0;
  unsigned char i;

  poll_package[0] = PACKAGE_HEAD_BYTe_1;
  poll_package[1] = PACKAGE_HEAD_BYTe_2;

  poll_package[2] = self_id[0];
  poll_package[3] = self_id[1];
  poll_package[4] = self_id[2];
  poll_package[5] = self_id[3];
  poll_package[6] = self_id[4];
  poll_package[7] = self_id[5];

  poll_package[8] =  source_id[0];
  poll_package[9] =  source_id[1];
  poll_package[10] =  source_id[2];
  poll_package[11] =  source_id[3];
  poll_package[12] =  source_id[4];
  poll_package[13] =  source_id[5];

  poll_package[14] =  RECV_SN + 0x01; // frame number add 1

  poll_package[15] =  POLL_BACK_TAG;  // response tag number

  poll_package[16] =  RECV_VALUE[CMD_X_IDX];
  poll_package[17] =  RECV_VALUE[CMD_Y_IDX];
  poll_package[18] =  RECV_VALUE[CMD_Z_IDX];
  poll_package[19] =  0x00;


  CRC0CN = 0x1d;   // CRC initial valve 0xFF and enable
  for (i=0; i<20; i++) {
    CRC0IN = poll_package[i];
  }
  poll_package[20] =  CRC0DAT;
  CRC0CN = 0x14;
  poll_package[21] =  CRC0DAT;

  //poll_package[8:13] = source_id
  return ;
}

void do_lamp_ctrl_response()
{
  switch (RECV_VALUE[CMD_X_IDX]) {
    case CMD_LAMP_ALL_ON:
      P2 = (P2_B0__BMASK & P2_B0__LOW) | (P2_B1__BMASK & P2_B1__LOW);  // low active
      PCA0CPL0 = ~(RECV_VALUE[CMD_Y_IDX]);
      PCA0CPH0 = ~(RECV_VALUE[CMD_Y_IDX]);
      PCA0CPL1 = ~(RECV_VALUE[CMD_Z_IDX]);
      PCA0CPH1 = ~(RECV_VALUE[CMD_Z_IDX]);
      break;
    case CMD_LAMP_ALL_OFF:
      P2 = (P2_B0__BMASK & P2_B0__HIGH) | (P2_B1__BMASK & P2_B1__HIGH);  // high in-active
      PCA0CPL0 = ~(RECV_VALUE[CMD_Y_IDX]);
      PCA0CPH0 = ~(RECV_VALUE[CMD_Y_IDX]);
      PCA0CPL1 = ~(RECV_VALUE[CMD_Z_IDX]);
      PCA0CPH1 = ~(RECV_VALUE[CMD_Z_IDX]);
      break;
    case CMD_LAMP_LINE1_ON:
    case CMD_LAMP_LINE2_ON:
      P2 = (P2_B0__BMASK & P2_B0__LOW) | (P2_B1__BMASK & P2_B1__LOW);
      PCA0CPL0 = ~(RECV_VALUE[CMD_Y_IDX]);
      PCA0CPH0 = ~(RECV_VALUE[CMD_Y_IDX]);
      PCA0CPL1 = ~(RECV_VALUE[CMD_Z_IDX]);
      PCA0CPH1 = ~(RECV_VALUE[CMD_Z_IDX]);
      break;
    default:
      break;
  }
  return ;
}


/**
 * @brief random number generation in range
 *
 * @param range [0 ~ 256)
 * @return a random value in [0, range], note range is included
 */
unsigned char random(unsigned char range)
{
  unsigned char value;

  value = rand() / ( 32768 / ( range + 1 ) );

  return value;
}


//-----------------------------------------------------------------------------
// TX Routines
//-----------------------------------------------------------------------------

// send UART_TX_buffer to UART byte by byte
void send_frame(void)
{
  unsigned char i = 0;
  // while (AUX & 1){
  //
  //}


  // send back frm hanlding
  if ((flag_send_bck_frm_ready == 1) && (AUX == 1)){
    for (i=0; i<22; i++){
      SBUF0 = poll_package[i];
      while (SCON0_TI != 1){

      }
      SCON0_TI = 0;
    } 
    flag_send_bck_frm_ready = 0; // send back message already so clear the flag
  }

  // repeat frm handling
  if ((flag_send_repeat_frm_ready== 1) && (AUX == 1)){
  	if (is_broadcast_frame()== 1)
		sleep(random(REAPTOR_WAITING_TIME_DURATION));  // waiting random second then send out to avoid broadcast storm
	else
		sleep(1 + random(REAPTOR_WAITING_TIME_DURATION));
    for (i=0; i<22; i++){
      SBUF0 = repeator_package[i];
      while (SCON0_TI != 1){

      }
      SCON0_TI = 0;
    } 
    flag_send_repeat_frm_ready = 0; // send repeat message already so clear the flag
  }
  return;
}

// duplicate UART_RX_buffer into UART_TX_buffer
void forward_preparation()
{
  unsigned char i = 0;
  for (i=0; i<22; i++){
    repeator_package[i] = UART_RX_buffer[i];
  }

  return;

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


/**
 * @brief sleep/delay certain seconds
 *
 * The amount of time it takes for a single NOP on the target (24.5MHz) is about
 * 0.5 us, so use NOP with 2M times for one section delay or sleep.
 *
 * @param seconds
 * @return void
 */
void sleep(unsigned char seconds)
{
  // temp try using for cycle, will use timer in finial code
  unsigned char i = 0;
  unsigned int j = 0;
  unsigned int k = 0;

  for (i=0; i<seconds; i++) {
    for (j=0; j<2000;j++) {
      for (k=0; k<1000; k++) {
        _nop_ ();
      }
    }
  }

  return;
}


// Protocol TAG definitions
#define PTAG_SN_CLR		0
#define PTAG_ACK        1
#define PTAG_NACK       2
#define PTAG_POLL       3
#define PTAG_POLL_ACK   4
#define PTAG_LAMP_CTRL  5
void rx_cmd_process(unsigned short isBroadcastFrame)
{
  switch (RECV_TAG) {     // CMD tag
    // all *_resonse() below need to fill in UART_TX_buffer
    case PTAG_SN_CLR:		// that is sn clear frm, do nothing
		break;
    case PTAG_POLL:
      do_poll_response();   // poll package ready
      flag_send_bck_frm_ready = 1;
      //send_frame();
      break;
    case PTAG_LAMP_CTRL:   // lamp open/close control
      do_lamp_ctrl_response();
      if (!isBroadcastFrame){
        do_poll_response();
        //send_frame();
        flag_send_bck_frm_ready = 1;
      }
      break;
    default:
      do_nack_response();
  }

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
    if (isBroadcastFrame && need_forward) { // change isBroadcastFrame() to isBroadcastFrame @2017-07-08 from eric s -- handling broadcast frm
      //sleep(random(3));
      forward_preparation(); // prepare repeat  frm package
      flag_send_repeat_frm_ready = 1;
      //send_frame();
    } else if (!isMyFrame) { // hanlding single node frm
      if (RECV_SN > sn || (RECV_SN == 0 && sn != 0)) {
        sn = RECV_SN;
        //sleep(1 + random(3));
        forward_preparation(); // only prepare package buffer for repeator
        //send_frame();
        flag_send_repeat_frm_ready = 1;
      }
    }
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
        RX_Ready = 1;
      idx = 0;
    }
  }

  IE_ES0 = 1;   // enable UART interrupt
}
