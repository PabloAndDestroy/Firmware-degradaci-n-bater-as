// ============================================================================
// Firmware aparte: solo el ADS @0x49, una muestra cada 20 ms.
// Mismos pines que el datalogger principal. No lee el ADS 0x48 ni el DHT11.
//
//   ADS @0x49 (ADDR -> VDD)
//     A0 = voltaje de celda
//     A1 = sensor de corriente K6D (50 A/V)
//   SD  CS = PA4, SCK = PA5, MISO = PA6, MOSI = PA7
//   I2C SDA = PB7, SCL = PB6
//   LED PC13, activo en LOW
//
// Cada arranque crea VIxxxx.CSV. El sync de la SD va cada 25 filas (500 ms)
// para que la lectura pueda seguir el periodo de 20 ms.
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"
#include <math.h>

static constexpr auto PIN_SD_CS = PA4;
static constexpr auto PIN_LED = PC13;
static constexpr auto PIN_SDA = PB7;
static constexpr auto PIN_SCL = PB6;

static const uint8_t ADS_VI_ADDR = 0x49;
static const uint32_t SAMPLE_PERIOD_MS = 20UL;
static const uint8_t SYNC_EVERY = 25;

static const float ADS_VOLTS_PER_COUNT = 6.144f / 32768.0f;
static const float CURRENT_SLOPE = 50.0f;
static const float CURRENT_OFFSET = 0.0f;

class Ads1115
{
public:
  explicit Ads1115(uint8_t address) : addr_(address) {}

  bool readSingleEnded(uint8_t channel, int16_t &counts)
  {
    const uint16_t config =
        0x8000
        | (uint16_t)((0x04 | channel) << 12)
        | 0x0000   // PGA +-6.144 V
        | 0x0100   // single-shot
        | 0x00E0   // data rate maxima
        | 0x0003;

    if (!writeReg(0x01, config))
      return false;

    const uint32_t t0 = millis();
    uint16_t status = 0;
    while (true)
    {
      delayMicroseconds(200);
      if (!readReg(0x01, status))
        return false;
      if (status & 0x8000)
        break;
      if (millis() - t0 > 5)
        return false;
    }

    uint16_t raw = 0;
    if (!readReg(0x00, raw))
      return false;
    counts = (int16_t)raw;
    return true;
  }

private:
  uint8_t addr_;

  bool writeReg(uint8_t reg, uint16_t value)
  {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write((uint8_t)(value >> 8));
    Wire.write((uint8_t)(value & 0xFF));
    return Wire.endTransmission() == 0;
  }

  bool readReg(uint8_t reg, uint16_t &value)
  {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    if (Wire.endTransmission() != 0)
      return false;
    if (Wire.requestFrom((uint8_t)addr_, (uint8_t)2) != 2)
      return false;
    const uint8_t hi = Wire.read();
    const uint8_t lo = Wire.read();
    value = ((uint16_t)hi << 8) | lo;
    return true;
  }
};

static Ads1115 adcVI(ADS_VI_ADDR);
static SdFat sd;
static SdFile csv;
static char logName[13];
static bool sdOk = false;
static bool adcOk = false;
static uint8_t i2cFailStreak = 0;
static uint8_t rowsSinceSync = 0;
static uint32_t t0Ms = 0;
static uint32_t nextSampleMs = 0;

class LineBuffer
{
public:
  void clear() { len_ = 0; }
  const char *data() const { return buf_; }
  size_t size() const { return len_; }

  void putChar(char c)
  {
    if (len_ < sizeof(buf_))
      buf_[len_++] = c;
  }
  void putText(const char *s)
  {
    while (*s)
      putChar(*s++);
  }
  void putUint(uint32_t v)
  {
    char tmp[10];
    uint8_t n = 0;
    do
    {
      tmp[n++] = (char)('0' + (v % 10));
      v /= 10;
    } while (v != 0);
    while (n)
      putChar(tmp[--n]);
  }
  void putFixed(float v, uint8_t decimals)
  {
    if (!isfinite(v) || fabsf(v) > 1.0e6f)
    {
      putText("nan");
      return;
    }
    if (v < 0.0f)
    {
      v = -v;
      float probe = v;
      for (uint8_t i = 0; i < decimals; ++i)
        probe *= 10.0f;
      if (probe >= 0.5f)
        putChar('-');
    }
    uint32_t scale = 1;
    for (uint8_t i = 0; i < decimals; ++i)
      scale *= 10;
    const uint32_t scaled = (uint32_t)(v * (float)scale + 0.5f);
    putUint(scaled / scale);
    if (decimals == 0)
      return;
    putChar('.');
    const uint32_t frac = scaled % scale;
    for (uint32_t d = scale / 10; d > 0; d /= 10)
      putChar((char)('0' + (frac / d) % 10));
  }

private:
  char buf_[96];
  size_t len_ = 0;
};

static LineBuffer line;

static const char CSV_HEADER[] =
    "time_ms;voltage_V;current_sensor_V;current_A;error\n";

static void updateLed()
{
  const uint32_t ms = millis();
  bool on;
  if (!sdOk)
    on = ((ms / 100) % 2) == 0;
  else
  {
    const uint32_t p = ms % 1000;
    if (!adcOk)
      on = (p < 80) || (p >= 200 && p < 280);
    else
      on = (p < 60);
  }
  digitalWrite(PIN_LED, on ? LOW : HIGH);
}

static void i2cBegin()
{
  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.begin();
  Wire.setClock(100000);
}

static void i2cRecover()
{
  Wire.end();
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, INPUT_PULLUP);
  delayMicroseconds(10);
  for (uint8_t i = 0; i < 9; ++i)
  {
    pinMode(PIN_SCL, OUTPUT);
    digitalWrite(PIN_SCL, LOW);
    delayMicroseconds(5);
    pinMode(PIN_SCL, INPUT_PULLUP);
    delayMicroseconds(5);
  }
  pinMode(PIN_SDA, OUTPUT);
  digitalWrite(PIN_SDA, LOW);
  delayMicroseconds(5);
  pinMode(PIN_SCL, INPUT_PULLUP);
  delayMicroseconds(5);
  pinMode(PIN_SDA, INPUT_PULLUP);
  delayMicroseconds(5);
  i2cBegin();
}

static float computeVolts(int16_t counts)
{
  return (float)counts * ADS_VOLTS_PER_COUNT;
}

static bool sdWrite(const char *data, size_t n, bool forceSync)
{
  if (csv.write(data, n) != n)
    return false;
  if (!forceSync && ++rowsSinceSync < SYNC_EVERY)
    return true;
  rowsSinceSync = 0;
  return csv.sync();
}

static bool sdStartNewFile()
{
  if (!sd.begin(PIN_SD_CS, SD_SCK_MHZ(4)))
    return false;

  for (uint16_t n = 1; n <= 9999; ++n)
  {
    logName[0] = 'V';
    logName[1] = 'I';
    logName[2] = (char)('0' + (n / 1000) % 10);
    logName[3] = (char)('0' + (n / 100) % 10);
    logName[4] = (char)('0' + (n / 10) % 10);
    logName[5] = (char)('0' + n % 10);
    logName[6] = '.';
    logName[7] = 'C';
    logName[8] = 'S';
    logName[9] = 'V';
    logName[10] = 0;

    if (sd.exists(logName))
      continue;
    if (!csv.open(logName, O_WRONLY | O_CREAT | O_EXCL))
      return false;
    return sdWrite(CSV_HEADER, sizeof(CSV_HEADER) - 1, true);
  }
  return false;
}

static bool sdReopen()
{
  if (csv.isOpen())
    csv.close();
  if (!sd.begin(PIN_SD_CS, SD_SCK_MHZ(4)))
    return false;
  rowsSinceSync = 0;
  return csv.open(logName, O_WRONLY | O_AT_END);
}

static void takeSample(uint32_t elapsedMs)
{
  float voltage = NAN;
  float sensorV = NAN;
  float current = NAN;
  bool i2cError = false;
  int16_t counts;

  if (adcVI.readSingleEnded(0, counts))
    voltage = computeVolts(counts);
  else
    i2cError = true;

  if (!i2cError && adcVI.readSingleEnded(1, counts))
  {
    sensorV = computeVolts(counts);
    current = CURRENT_SLOPE * sensorV + CURRENT_OFFSET;
    if (current < 0.0f)
      current = 0.0f;
  }
  else
    i2cError = true;

  adcOk = !i2cError;
  if (i2cError)
  {
    if (++i2cFailStreak >= 3)
    {
      i2cRecover();
      i2cFailStreak = 0;
    }
  }
  else
    i2cFailStreak = 0;

  line.clear();
  line.putUint(elapsedMs);
  line.putChar(';');
  line.putFixed(voltage, 4);
  line.putChar(';');
  line.putFixed(sensorV, 4);
  line.putChar(';');
  line.putFixed(current, 2);
  line.putChar(';');
  if (i2cError)
    line.putText("ERROR no lee ADS 0x49 por I2C");
  line.putChar('\n');

  if (!sdWrite(line.data(), line.size(), false))
    sdOk = sdReopen() && sdWrite(line.data(), line.size(), true);
  else
    sdOk = true;
}

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
  SPI.begin();

  while (!(sdOk = sdStartNewFile()))
  {
    const uint32_t t = millis();
    while (millis() - t < 1000)
      updateLed();
  }

  i2cBegin();
  t0Ms = millis();
  nextSampleMs = t0Ms;
}

void loop()
{
  updateLed();
  const uint32_t now = millis();
  if ((int32_t)(now - nextSampleMs) < 0)
    return;

  takeSample(now - t0Ms);
  nextSampleMs += SAMPLE_PERIOD_MS;
  if ((int32_t)(millis() - nextSampleMs) >= 0)
    nextSampleMs = millis() + SAMPLE_PERIOD_MS;
}
