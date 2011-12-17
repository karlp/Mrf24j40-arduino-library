/**
 * Example code for using a microchip mrf24j40 module to receive only
 * packets using plain 802.15.4
 * Requirements: 3 pins for spi, 3 pins for reset, chip select and interrupt
 * notifications
 * This example file is considered to be in the public domain
 * Originally written by Karl Palsson, karlp@tweak.net.au, March 2011
 */
#include <SPI.h>
#include <mrf24j.h>

const int pin_reset = 6;
const int pin_cs = 10; // default CS pin on ATMEGA8
const int pin_interrupt = 2; // default interrupt pin on ATMEGA8

Mrf24j mrf(pin_reset, pin_cs, pin_interrupt);

void setup() {
  Serial.begin(9600);

  mrf.set_pan(0xcafe);
  // This is _our_ address
  mrf.address16_write(0x6001); 
  mrf.set_channel(12);

  // uncomment if you want to receive any packet on this channel
  // mrf.set_promiscuous(true);

  attachInterrupt(0, interrupt_routine, CHANGE); // interrupt 0 equivalent to pin 2 on ATMEGA8
  interrupts();
}

void interrupt_routine() {
  mrf.interrupt_handler();
}

void loop() {
    mrf.check_flags(&handle_rx, &handle_tx);
}

void handle_rx() {
    Serial.print("received a packet ");Serial.print(mrf.get_rxinfo()->frame_length, DEC);Serial.println(" bytes long");
    
    Serial.println("Packet data:");
    for (int i = 0; i < mrf.rx_datalength(); i++) {
        Serial.write(mrf.get_rxinfo()->rx_data[i]);
    }
    
    Serial.print("\r\nLQI/RSSI=");
    Serial.print(mrf.get_rxinfo()->lqi, DEC);
    Serial.print("/");
    Serial.println(mrf.get_rxinfo()->rssi, DEC);
}

void handle_tx() {
    // code to transmit, nothing to do
}