/**
 * File:  MRF24J40_lib.c
 *
 * Wireless communications library functions for MRF24J40 RF module.
 *
 * Acknowledgements:
 * Derived from "mrf24j.cpp" (C++ Arduino lib.), Karl Palsson, 2011
 * by M.J.Bauer, 2017 [www.mjbauer.biz]
 * 
 * Note:  Host MCU platform dependencies are defined in "MRF24J40_lib.h".
 *
 * Reference:  IEEE 802.15.4 standard for LR-WPAN wireless comm's devices.
 */
#include <xc.h>
#include <string.h>
#include "MRF24J40_lib.h"

// aMaxPHYPacketSize = 127, from the 802.15.4-2006 standard.
static uint8  rx_buf[128];

// Essential for obtaining the data frame only...
// bytes_MHR = 2 Frame control + 1 sequence number + 2 panid + 2 shortAddr Destination + 2 shortAddr Source
static uint8  bytes_MHR;     // MAC header len = 9
static uint8  bytes_FCS;     // FCS length = 2
static uint8  bytes_nodata;  // no_data bytes in PHY payload, header len + FCS
static uint8  ignoreBytes;   // bytes to ignore, some modules behaviour (typ. 0))
static bool   bufPHY;        // flag = true to buffer all bytes in PHY Payload

volatile uint8  flag_got_rx;   // True => packet received
volatile uint8  flag_got_tx;   // True => ready to send pkt

static rx_info_t  rx_info;
static tx_info_t  tx_info;


/**
 * Execute hardware reset operation.
 */
void  MRF_Reset(void)
{
    MRF_INT_DISABLE();
    MRF_CS_HIGH();

    MRF_RESET_LO();
    Delay_ms(10);
    MRF_RESET_HI();
    Delay_ms(20);
    
    MRF_WAKE();
    Delay_ms(10);
}

/**
 * Read MRF register using the "short" (6 bit) addressing mode.
 * Arg address must be in the range 0..63 (0x0..0x3F).
 */
uint8  MRF_ReadShort(uint8 address)
{
    uint8  retval;

    MRF_CS_LOW();
    SPI_Exchange8bit((address << 1) & 0b01111110);
    retval = SPI_Exchange8bit(0);
    MRF_CS_HIGH();

    return  retval;
}

/**
 * Read MRF register using the "long" (10 bit) addressing mode.
 * Arg address must be in the range 0..1023 (0x0..0x3FF).
 */
uint8  MRF_ReadLong(uint16 address)
{
    uint8  retval;
    uint8  ahigh = address >> 3;
    uint8  alow = address << 5;

    MRF_CS_LOW();
    SPI_Exchange8bit(0x80 | ahigh);  // high bit for long
    SPI_Exchange8bit(alow);
    retval = SPI_Exchange8bit(0);
    MRF_CS_HIGH();

    return  retval;
}

/**
 * Write MRF register using the "short" (6 bit) addressing mode.
 * Arg address must be in the range 0..63 (0x0..0x3F).
 */
void  MRF_WriteShort(uint8 address, uint8 data)
{
    MRF_CS_LOW();
    SPI_Exchange8bit(((address << 1) & 0b01111110) | 0x01);
    SPI_Exchange8bit(data);
    MRF_CS_HIGH();
}

/**
 * Write MRF register using the "long" (10 bit) addressing mode.
 * Arg address must be in the range 0..1023 (0x0..0x3FF).
 */
void  MRF_WriteLong(uint16 address, uint8 data)
{
    uint8 ahigh = address >> 3;
    uint8 alow = address << 5;

    MRF_CS_LOW();
    SPI_Exchange8bit(0x80 | ahigh);  // high bit for long
    SPI_Exchange8bit(alow | 0x10);   // last bit for write
    SPI_Exchange8bit(data);
    MRF_CS_HIGH();
}

/**
 *
 */
uint16  MRF_GetPAN(void)
{
    uint8  panh = MRF_ReadShort(MRF_PANIDH);

    return  (uint16) ((panh << 8) | MRF_ReadShort(MRF_PANIDL));
}

/**
 *
 */
void  MRF_SetPAN(uint16 panid)
{
    MRF_WriteShort(MRF_PANIDH, panid >> 8);
    MRF_WriteShort(MRF_PANIDL, panid & 0xff);
}

/**
 *
 */
void  MRF_Address16Write(uint16 address16)
{
    MRF_WriteShort(MRF_SADRH, address16 >> 8);
    MRF_WriteShort(MRF_SADRL, address16 & 0xff);
}

/**
 *
 */
uint16  MRF_Address16Read(void)
{
    uint8  a16h = MRF_ReadShort(MRF_SADRH);

    return  (uint16) ((a16h << 8) | MRF_ReadShort(MRF_SADRL));
}

/**
 * Transmit a data packet using 16-bit node addresses, with ACK. Local PAN only.
 * 
 * Entry args:  dest16 = destination node, short address; 
 *              data   = pointer to TX data buffer
 *              len    = length of data packet (user payload), maximum = ??? bytes
 */
void  MRF_TransmitData(uint16 dest16, char *data, uint8 len)
{
    uint16  src16, panid;      // source address (short) and PAN ID
    uint16  q, i = 0;

    MRF_WriteLong(i++, bytes_MHR); // header length
    // +ignoreBytes is because some module seems to ignore 2 bytes after the header?!.
    // default: ignoreBytes = 0;
    MRF_WriteLong(i++, bytes_MHR + ignoreBytes + len);

    // 0 | PAN compression | ack | no security | no data pending | data frame[3 bits]
    MRF_WriteLong(i++, 0b01100001); // first byte of Frame Control
    // 16 bit source, 802.15.4 (2003), 16 bit dest,
    MRF_WriteLong(i++, 0b10001000); // second byte of frame control
    MRF_WriteLong(i++, 1);  // sequence number 1

    panid = MRF_GetPAN();
    MRF_WriteLong(i++, panid & 0xff);   // dest panid
    MRF_WriteLong(i++, panid >> 8);
    MRF_WriteLong(i++, dest16 & 0xff);  // dest16 low
    MRF_WriteLong(i++, dest16 >> 8);    // dest16 high

    src16 = MRF_Address16Read();
    MRF_WriteLong(i++, src16 & 0xff);   // src16 low
    MRF_WriteLong(i++, src16 >> 8);     // src16 high

    // All testing seems to indicate that the next two bytes are ignored.
    // 2 bytes on FCS appended by TXMAC
    i += ignoreBytes;
    for (q = 0;  q < len;  q++)
    {
        MRF_WriteLong(i++, data[q]);
    }
    // ACK on, and go!
    MRF_WriteShort(MRF_TXNCON, (1 << MRF_TXNACKREQ | 1 << MRF_TXNTRIG));
}

/**
 *
 */
void  MRF_SetInterruptReg(void)
{
    // interrupts for rx and tx normal complete
    MRF_WriteShort(MRF_INTCON, 0b11110110);
}

/** 
 * Set the 802.15.4 channel number
 */
void  MRF_SetChannel(uint8 channel)
{
    MRF_WriteLong(MRF_RFCON0, (((channel - 11) << 4) | 0x03));
}

/**
 *
 */
void  MRF_Init(void)
{
    SPI_Initialize(SPI_MODE0);
    MRF_INT_DISABLE(); 
/*
    // Include following software reset code only if hardware reset not implemented...
    MRF_WriteShort(MRF_SOFTRST, 0x7); // from manual
    while (read_short(MRF_SOFTRST) & 0x7 != 0)
    {
        ;;;  // wait for soft reset to finish
    }
*/
    MRF_WriteShort(MRF_PACON2, 0x98);  // FIFOEN = 1 and TXONTS = 0x6.
    MRF_WriteShort(MRF_TXSTBL, 0x95);  // RFSTBL = 0x9.

    MRF_WriteLong(MRF_RFCON0, 0x03);   // RFOPT = 0x03.
    MRF_WriteLong(MRF_RFCON1, 0x01);   // VCOOPT = 0x02.
    MRF_WriteLong(MRF_RFCON2, 0x80);   // Enable PLL (PLLEN = 1).
    MRF_WriteLong(MRF_RFCON6, 0x90);   // TXFIL = 1 and 20MRECVR = 1.
    MRF_WriteLong(MRF_RFCON7, 0x80);   // SLPCLKSEL = 0x2 (100 kHz Internal oscillator).
    MRF_WriteLong(MRF_RFCON8, 0x10);   // RFVCO = 1.
    MRF_WriteLong(MRF_SLPCON1, 0x21);  // CLKOUTEN = 1 and SLPCLKDIV = 0x01.

    //  Configuration for non-beacon-enabled devices 
    //  (see Section 3.8 - Beacon-Enabled and Non-Beacon-Enabled Networks)
    MRF_WriteShort(MRF_BBREG2, 0x80);  // Set CCA mode to ED
    MRF_WriteShort(MRF_CCAEDTH, 0x60); // Set CCA ED threshold.
    MRF_WriteShort(MRF_BBREG6, 0x40);  // Set appended RSSI value to RXFIFO.
    MRF_SetInterruptReg();
    MRF_SetChannel(12);
    // max power is by default.. just leave it...
    // Set transmitter power - See REGISTER 2-62: RF CONTROL 3 REGISTER (ADDRESS: 0x203)”.
    MRF_WriteShort(MRF_RFCTL, 0x04);   // Reset RF state machine.
    MRF_WriteShort(MRF_RFCTL, 0x00);   // part 2

    Delay_x10us(20);    // delay >= 192us
    
    bytes_MHR = 9;      // MAC header len = 9
    bytes_FCS = 2;      // FCS length = 2
    bytes_nodata = bytes_MHR + bytes_FCS;  // no_data bytes in PHY payload, header len + FCS
    ignoreBytes = 0;    // bytes to ignore, some modules behaviour
    bufPHY = 0;         // flag = true to buffer all bytes in PHY Payload
    
    flag_got_tx = 1;    // ready to send
    flag_got_rx = 0;    // rx packet available

    MRF_INT_ENABLE(); 
}

/**
 * Call this from within an interrupt handler connected to the MRFs output
 * interrupt pin.  It handles reading in any data from the module, and letting it
 * continue working.
 * Only the most recent data is ever kept.
 */
void  MRF_InterruptHandler(void)
{
    uint8   frame_len, data_len;
	uint16  i, rb_ptr, rd_ptr;
    uint8   last_interrupt = MRF_ReadShort(MRF_INTSTAT);

    if (last_interrupt & MRF_I_RXIF)
    {
        flag_got_rx++;

        MRF_INT_DISABLE();  // All IRQ's, or just MRF ???
        MRF_RxDisable();
        
        // read start of rxfifo, has 2 bytes more added by FCS. frame length = m + n + 2
        frame_len = MRF_ReadLong(0x300);
        // buffer all bytes in PHY Payload, from 0x301 to (0x301 + frame_len -1)
        if (bufPHY)
		{
            for (rb_ptr = 0, i = 0;  i < frame_len;  i++)
			{
                rx_buf[rb_ptr++] = MRF_ReadLong(0x301 + i);
            }
        }

        data_len = (uint8) (frame_len - bytes_nodata);
        // buffer data bytes, from (0x301 + bytes_MHR) to (0x301 + frame_len - bytes_nodata - 1)
        for (rd_ptr = 0, i = 0;  i < data_len;  i++)
		{
            rx_info.rx_data[rd_ptr++] = MRF_ReadLong(0x301 + bytes_MHR + i);
        }

        rx_info.frame_length = frame_len;
        // same as datasheet 0x301 + (m + n + 2) <-- frame_len
        rx_info.lqi = MRF_ReadLong(0x301 + frame_len);
        // same as datasheet 0x301 + (m + n + 3) <-- frame_len + 1
        rx_info.rssi = MRF_ReadLong(0x301 + frame_len + 1);

        MRF_RxEnable();
        MRF_INT_ENABLE();  // All IRQ's, or just MRF ???
    }

    if (last_interrupt & MRF_I_TXNIF)
	{
        uint8  tmp = MRF_ReadShort(MRF_TXSTAT);

        flag_got_tx++;
        // 1 means it failed, we want 1 to mean it worked.
        tx_info.tx_ok = !(tmp & ~(1 << TXNSTAT));
        tx_info.retries = tmp >> 6;
        tx_info.channel_busy = (tmp & (1 << CCAFAIL));
    }
}

#ifdef USE_MRF_LEGACY_CODE
/**
 * Call this function periodically, it will invoke your nominated handlers
 */
void  MRF_CheckFlags(void (*rx_handler)(void), void (*tx_handler)(void))
{
    // TODO - we could check whether the flags are > 1 here, indicating data was lost?

    if (flag_got_rx)
	{
        flag_got_rx = 0;
        rx_handler();
    }

    if (flag_got_tx)
	{
        flag_got_tx = 0;
        tx_handler();
    }
}
#endif

/**
 * Function returns TRUE if a data packet has been received since the previous call.
 * The application is expected to call this function frequently and, if it returns true,
 * process the received data before a subsequent packet is received.
 * 
 * The (private) flag, flag_got_rx, is always cleared on exit.
 */
bool  MRF_RxDataAvailable(void)
{
    bool  result = flag_got_rx;
    
    flag_got_rx = 0;
    
    return  result;
}

/**
 * Function returns TRUE if a data packet has been transmitted since the previous call.
 * The application is expected to call this function frequently after sending a packet
 * to check that transmission is complete and, if it returns true, this indicates that
 * a subsequent packet may be transmitted.
 * 
 * The (private) flag, flag_got_tx, is always cleared on exit.
 */
bool  MRF_TxDataSent(void)
{
    bool  result = flag_got_tx;
    
    flag_got_tx = 0;
    
    return  result;
}

/**
 * Set RX mode to promiscuous, or normal
 */
void  MRF_SetPromiscuous(bool enabled)
{
    if (enabled)  MRF_WriteShort(MRF_RXMCR, 0x01);
    else  MRF_WriteShort(MRF_RXMCR, 0x00);
}

/**
 * Function returns a pointer to the receiver information data structure.
 * The application should call MRF_RxDataAvailable() prior to accessing data 
 * to check that a new data packet is available.
 * 
 * The RX packet payload is accessible at rx_info->rx_data[].
 */
rx_info_t  *MRF_RxInfoGet(void)
{
    return  &rx_info;
}

/**
 * Function returns a pointer to the transmitter information data structure.
 */
tx_info_t  *MRF_TxInfoGet(void)
{
    return  &tx_info;
}

/**
 * Function returns a (byte) pointer to the receiver raw frame buffer, rx_buf.
 * The RX buffer contains all header and footer fields as well as the user payload.
 */
uint8  *MRF_RxBufferGet(void)
{
    return  rx_buf;
}

/**
 * Function returns the length of the received data packet, ie. the user payload.
 */
uint8  MRF_RxDataLength(void)
{
    return  (rx_info.frame_length - bytes_nodata);
}

/**
 * Some RF module variants (revisions?) behave diffently from others.
 * This function sets the number of bytes to be skipped in the RX frame at the point...
 * [TBA].
 */
void  MRF_IgnoreBytesValSet(int ib)
{
    // some modules behaviour
    ignoreBytes = ib;
}

/**
 * Set bufPHY flag true to buffer *all* bytes in PHY Payload
 */
void  MRF_SetFlagBufPHY(bool bp)
{
    bufPHY = bp;
}

/**
 * Get current bufPHY flag state.
 */
bool  MRF_GetFlagBufPHY(void)
{
    return  bufPHY;
}

/**
 * Set PA/LNA external control...
 * PA = power amplifier (transmitter), LNA = low-noise pre-amp (receiver).
 */
void  MRF_SetPALNA(bool enabled)
{
    if (enabled)
        MRF_WriteLong(MRF_TESTMODE, 0x07);  // Enable PA/LNA on MRF24J40MB module.
    else
        MRF_WriteLong(MRF_TESTMODE, 0x00);  // Disable PA/LNA on MRF24J40MB module.
}

/**
 * Flush the RX FIFO in the MRF24J module.
 */
void  MRF_RxFlush(void)
{
    MRF_WriteShort(MRF_RXFLUSH, 0x01);
}

/**
 * Disable the reciever in the MRF24J module.
 */
void  MRF_RxDisable(void)
{
    MRF_WriteShort(MRF_BBREG1, 0x04);  // RXDECINV - disable receiver
}

/**
 * Enable the reciever in the MRF24J module.
 */
void  MRF_RxEnable(void)
{
    MRF_WriteShort(MRF_BBREG1, 0x00);  // RXDECINV - enable receiver
}

