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
const int pin_cs = 10; // default CS pin on ATmega8/168/328
const int pin_interrupt = 2; // default interrupt pin on ATmega8/168/328

Mrf24j mrf(pin_reset, pin_cs, pin_interrupt);

long last_time;
long tx_interval = 1000;

void setup() {
  Serial.begin(9600);
  
  mrf.reset();
  mrf.init();
  
  mrf.set_pan(0xcafe);
  // This is _our_ address
  mrf.address16_write(0x6001); 
  
  // uncomment if you want to enable PA/LNA external control
  //mrf.set_palna(true);

  attachInterrupt(0, interrupt_routine, CHANGE); // interrupt 0 equivalent to pin 2(INT0) on ATmega8/168/328
  last_time = millis();
  interrupts();
}

void interrupt_routine() {
    mrf.interrupt_handler(); // mrf24 object interrupt routine
}

void loop() {
    mrf.check_flags(&handle_rx, &handle_tx);
}

void handle_rx() {
    // data to receive, nothing to do
}

void handle_tx() {
    if (mrf.get_txinfo()->tx_ok) {
        Serial.println("TX went ok, got ack");
    } else {
        Serial.print("TX failed after ");Serial.print(mrf.get_txinfo()->retries);Serial.println(" retries\n");
    }
}