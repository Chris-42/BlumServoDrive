#include "blum_servo_drive.h"
#include <Bounce2.h>

#define CSN GPIO_NUM_0
#define CE GPIO_NUM_1
#define BUTTON GPIO_NUM_9

#define LED_ON digitalWrite(15, LOW)
#define LED_OFF digitalWrite(15, HIGH)

RF24 radio(CE, CSN);
BlumServoDrive blum(&radio);
Bounce2::Button button = Bounce2::Button();

void callback(BlumServoDrive::event_t event, int peer_id) {
  switch(event) {
    case BlumServoDrive::SYNC_END:
      LED_OFF;
      break;
  }
}

void setup() {
  Serial.begin(115200);
  SPI.begin(SCK, MISO, MOSI);
  SPI.setFrequency(1000000);
  // initialize the transceiver on the SPI bus
  bool radio_ok = radio.begin(&SPI);
  if(radio_ok) {
    Serial.println(F("radio ok"));
  } else {
    Serial.println(F("radio hardware is not responding!!"));
    while(1);
  }

  blum.begin(RF24_PA_LOW);
  blum.setCallback(callback);

  button.attach(BUTTON, INPUT);
  button.interval(5); 
  button.setPressedState(LOW); 
  if(blum.getPeerCount()) {
    Serial.printf(F("Door 0 state is %d\r\n"), blum.getPeerState(0));
  }
}

void loop() {
  button.update();
  if(button.released()) {
    if(button.previousDuration() > 2000) {
      LED_ON;
      blum.startSync();
    } else if(button.previousDuration() > 1000) {
      blum.sendSyncIdentify(0);
    } else {
      blum.toggleState(0);
    }
  }
}