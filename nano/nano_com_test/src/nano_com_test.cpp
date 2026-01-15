#include <AltSoftSerial.h>

AltSoftSerial link; // Nano: TX=9, RX=8 (fest)
const uint32_t USB_BAUD = 115200;
const uint32_t LINK_BAUD = 57600;
const char *ID = "NANO";

unsigned long lastTx = 0;
uint32_t counter = 0;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(USB_BAUD);
  link.begin(LINK_BAUD);
  Serial.println("Nano ready (AltSoftSerial TX=D9 RX=D8 @57600).");
}

void loop()
{
  // receive from UNO -> output to serial
  static char rbuf[96];
  static uint8_t rix = 0;
  while (link.available())
  {
    char c = link.read();
    if (c == '\n')
    {
      rbuf[rix] = 0;
      Serial.print("RX←UNO: ");
      Serial.println(rbuf);
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      rix = 0;
    }
    else if (c != '\r')
    {
      if (rix < sizeof(rbuf) - 1)
        rbuf[rix++] = c;
      else
        rix = 0;
    }
  }

  // send something every second
  unsigned long now = millis();
  if (now - lastTx >= 1000)
  {
    lastTx = now;
    char msg[96];
    snprintf(msg, sizeof(msg), "FROM %s: cnt=%lu, ms=%lu",
             ID, (unsigned long)counter++, now);
    Serial.print("TX→UNO: ");
    Serial.println(msg);
    link.print(msg);
    link.print('\n');
  }
}
