#include <Wire.h>
#include <Arduino.h>

#define TCA_ADDR 0x70
#define I2C_MIN 0x03
#define I2C_MAX 0x77

void tcaSelect(uint8_t channel)
{
  if (channel > 7)
    return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

bool tcaPresent()
{
  Wire.beginTransmission(TCA_ADDR);
  return (Wire.endTransmission() == 0);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  }

  Wire.begin();
  Wire.setClock(100000); // 100kHz standard bus speed

  Serial.println(F("\n=== TCA9548A I2C-Multichannel-Scanner ==="));
  Serial.print(F("TCA-Address: 0x"));
  Serial.println(TCA_ADDR, HEX);

  if (!tcaPresent())
  {
    Serial.println(F("[WARN] TCA9548A not reachable. Check wiring and address"));
  }
}

void loop()
{
  for (uint8_t ch = 0; ch < 8; ch++)
  {
    tcaSelect(ch);
    delay(2);

    Serial.print(F("\nChannel "));
    Serial.print(ch);
    Serial.println(F(":"));

    uint8_t found = 0;

    for (uint8_t addr = I2C_MIN; addr <= I2C_MAX; addr++)
    {
      if (addr == TCA_ADDR)
        continue; // Skipt TCA9548A Module

      Wire.beginTransmission(addr);
      uint8_t error = Wire.endTransmission();

      if (error == 0)
      {
        Serial.print(F("  -> found at 0x"));
        if (addr < 16)
          Serial.print('0');
        Serial.print(addr, HEX);
        Serial.print(F(" (dec "));
        Serial.print(addr, DEC);
        Serial.println(F(")"));
        found++;
      }
      else if (error == 4)
      {
        Serial.print(F("[WARNING] Unknown error at address 0x"));
        if (addr < 16)
          Serial.print('0');
        Serial.println(addr, HEX);
      }
    }

    if (found == 0)
    {
      Serial.println(F("  (no devices)"));
    }
  }

  Serial.println(F("\nScan complete. Re-scanning in 5 seconds"));
  delay(5000);
}
