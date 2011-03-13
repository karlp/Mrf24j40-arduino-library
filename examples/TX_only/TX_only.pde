/**
 * Example code for using a microchip mrf24j40 module to send simple packets
 *
 * Requirements: 3 pins for spi, 3 pins for reset, chip select and interrupt
 * notifications
 * This example file is considered to be in the public domain
 * Originally written by Karl Palsson, karlp@tweak.net.au, March 2011
 */
#include <SPI.h>
#include <mrf24j.h>

const int pin_reset = 6;
const int pin_cs = 7;
const int pin_interrupt = 5;

Mrf24j mrf(pin_reset, pin_cs, pin_interrupt);

long last_time;
long tx_interval = 1000;

void setup() {
  Serial.begin(9600);

  mrf.set_pan(0xcafe);
  // This is _our_ address
  mrf.address16_write(0x6001); 

  attachInterrupt(0, interrupt_routine, CHANGE);   

  last_time = millis();
  interrupts();
}

void interrupt_routine() {
    // read and clear interrupt flags from the radio
    byte last_interrupt = mrf.read_short(MRF_INTSTAT);
    // we don't care about the rx acks, but we need them to be flushed so it can keep receiving
    if (last_interrupt & MRF_I_RXIF) {
        mrf.write_short(MRF_RXFLUSH, 0x01);
    }
}


void loop() {
    int tmp;
    unsigned long current_time = millis();
    if (current_time - last_time > tx_interval) {
        last_time = current_time;
        Serial.println("txxxing...");
        char * msg = "hello world";
        mrf.send16(0x4202, strlen(msg), msg);
    }
  
}
