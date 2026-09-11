// bt_passthrough.cpp - talk to the Bluetooth module. NOT the competition firmware.
//
// THE ROBOT DOES NOT MOVE. Nothing but the two UARTs is compiled: no motors,
// no encoders, no sensors.
//
// Bridges the USB serial monitor to the module on Serial2. At boot it sends
// "AT" and prints the reply, which settles in one line whether the module has
// power, is wired the right way round and is listening at MODULE_BAUD. After
// that, whatever you type is sent to the module as one whole line when you
// press Enter, and every byte the module sends comes back to the monitor.
//
// The competition firmware talks to the module at BLUETOOTH_BAUD (main.cpp),
// 9600 unless changed, which is the factory rate of both modules - so a new
// module normally needs nothing from here. This is for when it does not
// answer, was configured by someone before, or you want to rename it, change
// its PIN, or run it faster (then change BLUETOOTH_BAUD to match).
//
// BUILD AND RUN
//   pio run -e bt_passthrough -t upload
//   pio device monitor -e bt_passthrough      (115200 baud)
//
// WIRING (Arduino Mega)
//   module RX <- pin 16 (TX2)     module TX -> pin 17 (RX2)     VCC 5 V, GND
//   The competition firmware only needs pin 16; this test needs both.
//
// The module ignores AT commands while a phone is connected to it: the LED
// must be blinking, not solid, before any of this answers.
//
// HC-06 (JY-MCU carrier, no KEY/EN pin, LED blinks fast)
//   MODULE_BAUD 9600 from the factory, LINE_ENDING "" (it rejects CR/LF).
//     AT            -> OK
//     AT+VERSION    -> OKlinvorV1.8   (or similar)
//     AT+NAMEwro26  -> OKsetname      the name the phone will show
//     AT+PIN1234    -> OKsetPIN       the pairing PIN
//     AT+BAUD8      -> OK115200       permanent. The module now talks at
//                                     115200: set MODULE_BAUD here and
//                                     BLUETOOTH_BAUD in main.cpp to match.
//
// HC-05 (ZS-040 carrier, has a KEY or EN pin and a small button)
//   Hold KEY/EN high (or the button down) while powering up: the LED then
//   blinks slowly, about every 2 s, and the module is in AT mode at 38400.
//   MODULE_BAUD 38400, LINE_ENDING "\r\n".
//     AT                  -> OK
//     AT+UART?            -> +UART:9600,0,0
//     AT+UART=115200,0,0  -> OK        permanent; then BLUETOOTH_BAUD to match
//     AT+ROLE?            -> +ROLE:0   0 = slave. A phone can only find a slave.
//     AT+ORGL             -> OK        factory defaults, if it was configured before
//   Power-cycle with KEY/EN low to return to data mode.
//
// Lines are sent whole because the HC-06 wants a complete command within about
// a second, and the PlatformIO monitor sends every key the moment it is pressed.

#include <Arduino.h>

static const unsigned long MODULE_BAUD = 9600;   // 9600 HC-06 factory, 38400 HC-05 AT mode, 115200 once set
static const char          LINE_ENDING[] = "";   // "" for the HC-06, "\r\n" for the HC-05

// Copies everything the module has sent to the monitor. Returns true if there
// was anything.
static bool relayFromModule() {
  bool any = false;
  while (Serial2.available()) {
    Serial.write(Serial2.read());
    any = true;
  }
  return any;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(MODULE_BAUD);
  delay(1000);   // the module boots more slowly than the Mega

  Serial.print(F("bt_passthrough: module port at "));
  Serial.print(MODULE_BAUD);
  Serial.println(F(" baud. Probing with AT ..."));

  Serial2.print(F("AT"));
  Serial2.print(LINE_ENDING);

  bool answered = false;
  unsigned long t0 = millis();
  while (millis() - t0 < 1500) {
    if (relayFromModule()) answered = true;
  }

  if (answered) {
    Serial.println();
    Serial.println(F("<- the module answered: power, wiring and MODULE_BAUD are right."));
  } else {
    Serial.println(F("<- NO REPLY. Check, in this order: LED blinking (power, and not connected to"));
    Serial.println(F("   a phone); module TX really on pin 17; MODULE_BAUD matches the module;"));
    Serial.println(F("   an HC-05 must be in AT mode (KEY/EN high at power-up, slow blink)."));
  }
  Serial.println(F("Type a command and press Enter. It is sent whole, with LINE_ENDING."));
}

void loop() {
  static char    line[64];
  static uint8_t len = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (len > 0) {
        Serial2.write(line, len);
        Serial2.print(LINE_ENDING);
        Serial.print(F("-> "));
        Serial.write(line, len);
        Serial.println();
        len = 0;
      }
    } else if (len < sizeof(line)) {
      line[len++] = c;
    }
  }

  relayFromModule();
}
