#include <Wire.h>
#include <Adafruit_AS5600.h>

#define TCA_ADDR     0x70
#define ENCODER_CH   0 // Adjust according to Encoder's mounted channel at TCA9548A
#define AS5600_ADDR  0x36 // AS5600 I2C-Address

// AS5600 statusregisters
#define AS5600_REG_STATUS 0x0B
#define AS5600_STATUS_MD  (1 << 5)
#define AS5600_STATUS_ML  (1 << 4)
#define AS5600_STATUS_MH  (1 << 3)

Adafruit_AS5600 as5600;

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// read 8 bits using i2c
uint8_t i2cRead8(uint8_t addr, uint8_t reg){
  Wire.beginTransmission(addr); Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

static inline float ticksToDeg(uint16_t ticks){
  return ticks * (360.0f / 4096.0f);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin();
  Wire.setClock(100000); // 100 kHz standard bus speed

  tcaSelect(ENCODER_CH);
  delay(2);

  if (!as5600.begin(AS5600_ADDR)) {
    Serial.println(F("[ERROR] AS5600 not found. Check TCA channel, wiring, address."));
    while (1) delay(100);
  }

  Serial.println(F("AS5600 found on TCA channel 1. Start logging status.."));
}

void loop() {
  tcaSelect(ENCODER_CH);

  // Raw angle (0..4095) and 0..360 deg
  uint16_t rawTicks = as5600.getRawAngle() & 0x0FFF;
  float angleDeg = ticksToDeg(rawTicks);

  // Field strength / status
  uint8_t agc     = as5600.getAGC();        // 0..255 (AS5600 uses 0..255)
  uint16_t mag    = as5600.getMagnitude();  // 0..4095
  uint8_t status  = i2cRead8(AS5600_ADDR, AS5600_REG_STATUS);
  bool MD = status & AS5600_STATUS_MD;      // Magnet found
  bool ML = status & AS5600_STATUS_ML;      // Field strength too low
  bool MH = status & AS5600_STATUS_MH;      // Field strength too high

  Serial.print(F("ch")); Serial.print(ENCODER_CH);
  Serial.print(F("  angle=")); Serial.print(angleDeg, 2); Serial.print(F("°"));
  Serial.print(F("  raw="));   Serial.print(rawTicks);
  Serial.print(F("  AGC="));   Serial.print((int)agc);
  Serial.print(F("  MAG="));   Serial.print(mag);
  Serial.print(F("  ["));
  if (MD) Serial.print(F("MD "));
  if (ML) Serial.print(F("ML "));
  if (MH) Serial.print(F("MH "));
  Serial.println(F("]"));
  
  if (ML) Serial.println(F("Hint: Field strength too low"));
  if (MH) Serial.println(F("Hint: Field strength too high"));

  delay(100); // ~10 Hz
}
