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
#define FRAME_HEADER        0x55
#define FRAME_CRC_S         20
#define FRAME_SN            14
#define FRAME_LEN           22
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
#define RECV_SRC_ID         (& UART_RX_buffer[FRAME_SRC_ID_S])
#define RECV_DEST_ID        (& UART_RX_buffer[FRAME_DEST_ID_S])
#define CMD_LAMP_ALL_OFF    0x00
#define CMD_LAMP_LINE1_ON   0x01
#define CMD_LAMP_LINE2_ON   0x02
#define CMD_LAMP_ALL_ON     0x03
#define CMD_X_IDX           0
#define CMD_Y_IDX           1
#define CMD_Z_IDX           2
#define UART_BUFFERSIZE     FRAME_LEN
#define FRAME_ID_LEN        6
#define PACKAGE_HEAD_BYTe_1 0x55
#define PACKAGE_HEAD_BYTe_2 0x55
#define RELAY_RANDOM_DELAY_MAX 3
#define RELAY_FIXED_DELAY      1


INTERRUPT_PROTO(UART0_Interrupt, 4);

void SYSCLK_Init (void);
void UART0_Init (void);
void PORT_Init (void);
void Timer2_Init (S16);
void rx_frame_process(unsigned char role);
void send_frame(void);
U8 random(unsigned char range);
void sleep(unsigned char seconds);

///////////////  NEED MODIFICATION HERE FOR DIFFERENT NODES /////////
unsigned char role = 0; //0: that is normal node; 1: that is repeator ndoe;  0: STA; 1: RELAY
unsigned char self_id[FRAME_ID_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x05};
/////////////////////////////////////////////////////////////////////

U8 lamp_cmd_x = 0;
U8 lamp_cmd_y = 0;
U8 lamp_cmd_x = 0;
U8 RX_Ready =0; // 0: no receive frm ready in UART_RX_buffer or. no need to do process; 1: need to do receive frm process
unsigned char sn = 0; // frame sequence number

unsigned char UART_RX_buffer[UART_BUFFERSIZE];
unsigned char UART_TX_buffer[UART_BUFFERSIZE];

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

    IE_EA = 1;                          // enable all interrupts

    while (1)
    {
        if (RX_Ready == 1) {
            rx_frame_process(role);
            RX_Ready = 0;                   // Set the flag to zero, means complete all cmd processing
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
    IP |= 0x10;         // Make UART high priority
    IE_ES0 = 1;         // Enable UART0 interrupts
}


//-----------------------------------------------------------------------------
// CRC16 Service Routines
//-----------------------------------------------------------------------------
void crc16_init()
{
    CRC0CN = 0x1d;   // CRC initial valve 0xFF and enable
}

void crc16_add(unsigned char byte)
{
    CRC0IN = byte
}

unsigned char crc16_msb(void)
{
    CRC0CN = 0x15;
    return CRC0DAT;
}

unsigned char crc16_lsb(void)
{
    CRC0CN = 0x14;
    return CRC0DAT;
}

//-----------------------------------------------------------------------------
// process related add some function defien here @2017-07-08 from Eric S
//-----------------------------------------------------------------------------
void do_nack_response(void)
{
    //unsigned char send_back_frm_number = 0;
    unsigned char i;

    UART_TX_Buffer[0] = PACKAGE_HEAD_BYTe_1;
    UART_TX_Buffer[1] = PACKAGE_HEAD_BYTe_2;

    UART_TX_Buffer[2] = self_id[0];
    UART_TX_Buffer[3] = self_id[1];
    UART_TX_Buffer[4] = self_id[2];
    UART_TX_Buffer[5] = self_id[3];
    UART_TX_Buffer[6] = self_id[4];
    UART_TX_Buffer[7] = self_id[5];

    UART_TX_Buffer[8] =  RECV_SRC_ID[0];
    UART_TX_Buffer[9] =  RECV_SRC_ID[1];
    UART_TX_Buffer[10] =  RECV_SRC_ID[2];
    UART_TX_Buffer[11] =  RECV_SRC_ID[3];
    UART_TX_Buffer[12] =  RECV_SRC_ID[4];
    UART_TX_Buffer[13] =  RECV_SRC_ID[5];

    UART_TX_Buffer[14] =  RECV_SN + 1; // frame number add 1

    UART_TX_Buffer[15] =  PTAG_NACK;  // response tag number

    UART_TX_Buffer[16] =  0x00;
    UART_TX_Buffer[17] =  0x00;
    UART_TX_Buffer[18] =  0x00;
    UART_TX_Buffer[19] =  0x00;

#if 0
    CRC0CN = 0x1d;   // CRC initial valve 0xFF and enable
    for (i=0; i<FRAME_CRC_S; i++) {
        CRC0IN = UART_TX_Buffer[i];
    }
    UART_TX_Buffer[i++] =  CRC0DAT;
    CRC0CN = 0x14;
    UART_TX_Buffer[i] =  CRC0DAT;
#endif

    crc16_init();
    for (i=0; i<FRAME_CRC_S; i++)   crc16_add(UART_TX_Buffer[i]);
    UART_TX_Buffer[i++] = crc16_msb();
    UART_TX_Buffer[i] = crc16_lsb();
}

void do_poll_response(void)
{
    unsigned char send_back_frm_number = 0;
    unsigned char i;

    UART_TX_Buffer[0] = PACKAGE_HEAD_BYTe_1;
    UART_TX_Buffer[1] = PACKAGE_HEAD_BYTe_2;

    UART_TX_Buffer[2] = self_id[0];
    UART_TX_Buffer[3] = self_id[1];
    UART_TX_Buffer[4] = self_id[2];
    UART_TX_Buffer[5] = self_id[3];
    UART_TX_Buffer[6] = self_id[4];
    UART_TX_Buffer[7] = self_id[5];

    UART_TX_Buffer[8] =  RECV_SRC_ID[0];
    UART_TX_Buffer[9] =  RECV_SRC_ID[1];
    UART_TX_Buffer[10] =  RECV_SRC_ID[2];
    UART_TX_Buffer[11] =  RECV_SRC_ID[3];
    UART_TX_Buffer[12] =  RECV_SRC_ID[4];
    UART_TX_Buffer[13] =  RECV_SRC_ID[5];

    UART_TX_Buffer[14] =  RECV_SN + 1; // frame number add 1

    UART_TX_Buffer[15] =  PTAG_POLL_ACK;  // response tag number

    UART_TX_Buffer[16] =  lamp_cmd_x;
    UART_TX_Buffer[17] =  lamp_cmd_y;
    UART_TX_Buffer[18] =  lamp_cmd_z;
    UART_TX_Buffer[19] =  0x00;

#if 0
    CRC0CN = 0x1d;   // CRC initial valve 0xFF and enable
    for (i=0; i<FRAME_CRC_S; i++) {
        CRC0IN = UART_TX_Buffer[i];
    }
    UART_TX_Buffer[i++] =  CRC0DAT;
    CRC0CN = 0x14;
    UART_TX_Buffer[i] =  CRC0DAT;
#endif

    crc16_init();
    for (i=0; i<FRAME_CRC_S; i++)   crc16_add(UART_TX_Buffer[i]);
    UART_TX_Buffer[i++] = crc16_msb();
    UART_TX_Buffer[i] = crc16_lsb();
}

void do_lamp_ctrl_response()
{
    lamp_cmd_x = RECV_VALUE[CMD_X_IDX];
    lamp_cmd_y = RECV_VALUE[CMD_Y_IDX];
    lamp_cmd_x = RECV_VALUE[CMD_Z_IDX];

    switch (lamp_cmd_x) {
        case CMD_LAMP_ALL_ON:
            P2 = (P2_B0__BMASK & P2_B0__LOW) | (P2_B1__BMASK & P2_B1__LOW);  // low active
            PCA0CPL0 = ~(lamp_cmd_y);
            PCA0CPH0 = ~(lamp_cmd_y);
            PCA0CPL1 = ~(lamp_cmd_z);
            PCA0CPH1 = ~(lamp_cmd_z);
            break;
        case CMD_LAMP_ALL_OFF:
            P2 = (P2_B0__BMASK & P2_B0__HIGH) | (P2_B1__BMASK & P2_B1__HIGH);  // high in-active
            PCA0CPL0 = ~(lamp_cmd_y);
            PCA0CPH0 = ~(lamp_cmd_y);
            PCA0CPL1 = ~(lamp_cmd_z);
            PCA0CPH1 = ~(lamp_cmd_z);
            break;
        case CMD_LAMP_LINE1_ON:
        case CMD_LAMP_LINE2_ON:
            P2 = (P2_B0__BMASK & P2_B0__LOW) | (P2_B1__BMASK & P2_B1__LOW);
            PCA0CPL0 = ~(lamp_cmd_y);
            PCA0CPH0 = ~(lamp_cmd_y);
            PCA0CPL1 = ~(lamp_cmd_z);
            PCA0CPH1 = ~(lamp_cmd_z);
            break;
        default:
            break;
    }
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

    while (AUX); // waiting for E32 ready before TX

    for (i=0; i<FRAME_LEN; i++){
        SBUF0 = UART_TX_buffer[i];
        while (SCON0_TI != 1);
        SCON0_TI = 0;
    } 
}

// duplicate UART_RX_buffer into UART_TX_buffer
void forward_preparation()
{
    unsigned char i = 0;

    for (i=0; i<FRAME_LEN; i++){
        UART_TX_buffer[i] = UART_RX_buffer[i];
    }
}

//-----------------------------------------------------------------------------
// rx processing misc routines
//-----------------------------------------------------------------------------
unsigned char is_my_frame()
{
    unsigned char result = 0;
    unsigned char i;

    for (i=0; i<FRAME_ID_LEN; i++)
        if (self_id[i] != RECV_DEST_ID[i]) break;
    if (i == FRAME_ID_LEN) result = 1;
    return result;
}

unsigned char is_broadcast_frame()
{
    unsigned char result = 0;
    unsigned char i;

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
        // assuming clock is 24MHz and one _nop_ takes 12 cpu cycles.
        for (j=0; j<2000;j++) {
            for (k=0; k<1000; k++) {
                _nop_ ();
            }
        }
    }

    return;
}


// Protocol TAG definitions
#define PTAG_ACK        1
#define PTAG_NACK       2
#define PTAG_POLL       3
#define PTAG_POLL_ACK   4
#define PTAG_LAMP_CTRL  5
void rx_cmd_process(unsigned char isBroadcastFrame)
{
    switch (RECV_TAG) {     // CMD tag
        // all *_resonse() below need to fill in UART_TX_buffer
        case PTAG_POLL:
            do_poll_response();   // poll package ready
            send_frame();
            break;
        case PTAG_LAMP_CTRL:   // lamp open/close control
            do_lamp_ctrl_response();
            if (!isBroadcastFrame){
                do_poll_response();
                send_frame();
            }
            break;
        default:
            if (!isBroadcastFrame){
                do_nack_response();
                send_frame();
            }
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
void rx_frame_process(unsigned char role)
{
    unsigned char isMyFrame = is_my_frame();
    unsigned char isBroadcastFrame = is_broadcast_frame();
    unsigned char need_forward = 0;

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
        if (isBroadcastFrame && need_forward) { 
            sleep(random(RELAY_RANDOM_DELAY_MAX));
            forward_preparation(); // prepare repeat  frm package
            send_frame();
        } else if (!isMyFrame) { // hanlding single node frm
            if (RECV_SN > sn || (RECV_SN == 0 && sn != 0)) {
                sn = RECV_SN;
                sleep(RELAY_FIXED_DELAY + random(RELAY_RANDOM_DELAY_MAX));
                forward_preparation(); // only prepare package buffer for repeator
                send_frame();
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

INTERRUPT(UART0_Interrupt, 4)
{
    static unsigned char idx = 0;
    char Byte;
    IE_ES0 = 0;   // disable UART interrupt

    if (SCON0_RI == 1)
    {
        SCON0_RI = 0;                           // Clear interrupt flag
        Byte = SBUF0;                      // Read a character from UART

        if (RX_Ready == 0) {
            if (idx == 0) {
                if (Byte == FRAME_HEADER) {
                    UART_RX_buffer[idx++] = Byte;
                    crc16_init();
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
    }

    IE_ES0 = 1;   // enable UART interrupt
}
