/**
 * mrf24j.cpp, Karl Palsson, 2011, karlp@tweak.net.au
 * modifed bsd license / apache license
 */

#include "WProgram.h"
#include "mrf24j.h"



Mrf24j::Mrf24j(int pin_reset, int pin_chip_select, int pin_interrupt) {
    _pin_reset = pin_reset;
    _pin_cs = pin_chip_select;
    _pin_int = pin_interrupt;

    pinMode(_pin_reset, OUTPUT);
    pinMode(_pin_cs, OUTPUT);
    pinMode(_pin_int, INPUT);

    SPI.begin();
    // arguably should not be here...
    mrf_reset();
    mrf_init();
}

void Mrf24j::mrf_reset(void) {
    digitalWrite(_pin_reset, LOW);
    delay(10);  // just my gut
    digitalWrite(_pin_reset, HIGH);
    delay(20);  // from manual
}

byte Mrf24j::mrf_read_short(byte address) {
    digitalWrite(_pin_cs, LOW);
    // 0 top for short addressing, 0 bottom for read
    SPI.transfer(address<<1 & 0b01111110);
    byte ret = SPI.transfer(0x0);
    digitalWrite(_pin_cs, HIGH);
    return ret;
}

byte Mrf24j::mrf_read_long(word address) {
    digitalWrite(_pin_cs, LOW);
    byte ahigh = address >> 3;
    byte alow = address << 5;
    SPI.transfer(0x80 | ahigh);  // high bit for long
    SPI.transfer(alow);
    byte ret = SPI.transfer(0);
    digitalWrite(_pin_cs, HIGH);
    return ret;
}


void Mrf24j::mrf_write_short(byte address, byte data) {
    digitalWrite(_pin_cs, LOW);
    // 0 for top address, 1 bottom for write
    SPI.transfer((address<<1 & 0b01111110) | 0x01);
    SPI.transfer(data);
    digitalWrite(_pin_cs, HIGH);
}

void Mrf24j::mrf_write_long(word address, byte data) {
    digitalWrite(_pin_cs, LOW);
    byte ahigh = address >> 3;
    byte alow = address << 5;
    SPI.transfer(0x80 | ahigh);  // high bit for long
    SPI.transfer(alow | 0x10);  // last bit for write
    SPI.transfer(data);
    digitalWrite(_pin_cs, HIGH);
}

word Mrf24j::mrf_pan_read(void) {
    byte panh = mrf_read_short(MRF_PANIDH);
    return panh << 8 | mrf_read_short(MRF_PANIDL);
}

void Mrf24j::mrf_pan_write(word panid) {
    mrf_write_short(MRF_PANIDH, panid >> 8);
    mrf_write_short(MRF_PANIDL, panid & 0xff);
}

void Mrf24j::mrf_address16_write(word address16) {
    mrf_write_short(MRF_SADRH, address16 >> 8);
    mrf_write_short(MRF_SADRL, address16 & 0xff);
}

word Mrf24j::mrf_address16_read(void) {
    byte a16h = mrf_read_short(MRF_SADRH);
    return a16h << 8 | mrf_read_short(MRF_SADRL);
}

/**
 * Simple send 16, with acks, not much of anything.. assumes src16 and local pan only.
 * @param data
 */
void Mrf24j::mrf_send16(word dest16, byte len, char * data) {

    int i = 0;
    mrf_write_long(i++, 9);  // header length
    mrf_write_long(i++, 9+2+len); //+2 is because module seems to ignore 2 bytes after the header?!

// 0 | pan compression | ack | no security | no data pending | data frame[3 bits]
    mrf_write_long(i++, 0b01100001); // first byte of Frame Control
// 16 bit source, 802.15.4 (2003), 16 bit dest,
    mrf_write_long(i++, 0b10001000); // second byte of frame control
    mrf_write_long(i++, 1);  // sequence number 1

    word panid = mrf_pan_read();

    mrf_write_long(i++, panid & 0xff);  // dest panid
    mrf_write_long(i++, panid >> 8);
    mrf_write_long(i++, dest16 & 0xff);  // dest16 low
    mrf_write_long(i++, dest16 >> 8); // dest16 high

    word src16 = mrf_address16_read();
    mrf_write_long(i++, src16 & 0xff); // src16 low
    mrf_write_long(i++, src16 >> 8); // src16 high

    i+=2;  // All testing seems to indicate that the next two bytes are ignored.
    for (int q = 0; q < len; q++) {
        mrf_write_long(i++, data[q]);
    }
    // ack on, and go!
    mrf_write_short(MRF_TXNCON, (1<<MRF_TXNACKREQ | 1<<MRF_TXNTRIG));
}

void Mrf24j::mrf_set_interrupts(void) {
    // interrupts for rx and tx normal complete
    mrf_write_short(MRF_INTCON, 0b11110110);
}

// Set the channel to 12, 2.41Ghz, xbee channel 0xC
void Mrf24j::mrf_set_channel(void) {
    mrf_write_long(MRF_RFCON0, 0x13);
}

void Mrf24j::mrf_init(void) {
/*
 // Seems a bit ridiculous when I use reset pin anyway
    mrf_write_short(MRF_SOFTRST, 0x7); // from manual
    while (mrf_read_short(MRF_SOFTRST) & 0x7 != 0) {
        ; // wait for soft reset to finish
    }
*/
    mrf_write_short(MRF_PACON2, 0x98); // – Initialize FIFOEN = 1 and TXONTS = 0x6.
    mrf_write_short(MRF_TXSTBL, 0x95); // – Initialize RFSTBL = 0x9.

    mrf_write_long(MRF_RFCON0, 0x03); // – Initialize RFOPT = 0x03.
    mrf_write_long(MRF_RFCON1, 0x01); // – Initialize VCOOPT = 0x02.
    mrf_write_long(MRF_RFCON2, 0x80); // – Enable PLL (PLLEN = 1).
    mrf_write_long(MRF_RFCON6, 0x90); // – Initialize TXFIL = 1 and 20MRECVR = 1.
    mrf_write_long(MRF_RFCON7, 0x80); // – Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator).
    mrf_write_long(MRF_RFCON8, 0x10); // – Initialize RFVCO = 1.
    mrf_write_long(MRF_SLPCON1, 0x21); // – Initialize CLKOUTEN = 1 and SLPCLKDIV = 0x01.

    //  Configuration for nonbeacon-enabled devices (see Section 3.8 “Beacon-Enabled and
    //  Nonbeacon-Enabled Networks”):
    mrf_write_short(MRF_BBREG2, 0x80); // Set CCA mode to ED
    mrf_write_short(MRF_CCAEDTH, 0x60); // – Set CCA ED threshold.
    mrf_write_short(MRF_BBREG6, 0x40); // – Set appended RSSI value to RXFIFO.
    mrf_set_interrupts();
    mrf_set_channel();
    // max power is by default.. just leave it...
    //Set transmitter power - See “REGISTER 2-62: RF CONTROL 3 REGISTER (ADDRESS: 0x203)”.
    mrf_write_short(MRF_RFCTL, 0x04); //  – Reset RF state machine.
    mrf_write_short(MRF_RFCTL, 0x00); // part 2
    delay(1); // delay at least 192usec
}



