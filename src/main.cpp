// ============================================================================
// Datalogger STM32 -> SD  (sin Serial)
//   ADS @0x48 : 4 NTC 100k (A0..A3), 5 V -- NTC -- nodo -- 47k -- GND
//   ADS @0x49 : A0 = voltaje de celda, A1 = sensor K6D (50 A/V)
//   DHT11 (PA8)   : humedad y temperatura ambiente
//   SD (SPI, CS = PA4)
//
// Cambios clave respecto a la version anterior:
//   * Cada arranque crea un archivo nuevo (LOG0001.CSV, LOG0002.CSV, ...).
//     El tiempo siempre empieza en 0 y crece con millis(); ya no se lee el
//     archivo viejo para "continuar" el tiempo (esa logica era fragil).
//   * Cada fila se arma en un buffer propio (sin Print::print(float)) y se
//     escribe de una sola vez + sync.
//   * Driver ADS1115 propio con timeout: la libreria de Adafruit puede
//     quedarse colgada para siempre si el ADS deja de responder.
//   * Si un ADS falla, se reintenta en cada muestra y se recupera el bus I2C.
//   * Se registran las columnas adc1_ok / adc2_ok para diagnosticar despues.
//   * El LED PC13 indica el estado (no hay Serial).
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"
#include <DHT.h>
#include <math.h>

// ----------------------------- Configuracion -------------------------------
static constexpr auto PIN_SD_CS = PA4;
static constexpr auto PIN_DHT = PA8;
static constexpr auto PIN_LED = PC13; // activo en LOW
static constexpr auto PIN_SDA = PB7;
static constexpr auto PIN_SCL = PB6;

static const uint8_t ADS_NTC_ADDR = 0x48; // ADDR -> GND
static const uint8_t ADS_VI_ADDR = 0x49;  // ADDR -> VDD

static const uint32_t SAMPLE_PERIOD_MS = 20UL;
static const uint32_t DHT_PERIOD_MS = 2000UL;
static const bool USE_DHT = true; // poner true cuando conectes el DHT11

// 5 V -- NTC -- nodo ADS -- 47 kOhm -- GND. NTC 100k a 25 C, beta 3950.
// La tabla anterior era de una NTC de 10 kOhm: no corresponde a este sensor.
static const float NTC_BETA = 3950.0f;
static const float NTC_T0 = 298.15f;
static const float NTC_R0 = 100000.0f;
static const float NTC_RF = 47000.0f;
static const float NTC_VIN = 5.0f;

// GAIN_TWOTHIRDS, fondo de escala +-6.144 V.
// Igual que Adafruit computeVolts(): 6.144/32768. En un ADS1015 los 4 bits
// bajos del registro vienen en 0, asi que la cuenta de 12 bits da el mismo voltaje.
static const float ADS_VOLTS_PER_COUNT = 6.144f / 32768.0f;

// K6D-P2S6-200: 0 V -> 0 A, 4 V -> 200 A.
static const float CURRENT_SLOPE = 50.0f;
static const float CURRENT_OFFSET = 0.0f;

// Muestras por canal. Se guarda la mediana, no una lectura suelta.
static constexpr uint8_t FILTER_SAMPLES = 5;

// ------------------------------ Driver ADS1115 -----------------------------
class Ads1115
{
public:
  explicit Ads1115(uint8_t address) : addr_(address) {}

  // Lectura single-ended de un canal (0..3). Devuelve false si hay error I2C
  // o si la conversion no termina a tiempo.
  bool readSingleEnded(uint8_t channel, int16_t &counts)
  {
    const uint16_t config =
        0x8000                               // OS: iniciar conversion
        | (uint16_t)((0x04 | channel) << 12) // MUX: AINx vs GND
        | 0x0000                             // PGA: +-6.144 V
        | 0x0100                             // MODE: single-shot
        | 0x00E0                             // DR: 860 SPS
        | 0x0003;                            // comparador desactivado

    if (!writeReg(REG_CONFIG, config))
      return false;

    const uint32_t t0 = millis();
    uint16_t status = 0;
    while (true)
    {
      delayMicroseconds(300);
      if (!readReg(REG_CONFIG, status))
        return false;
      if (status & 0x8000)
        break;
      if (millis() - t0 > 25)
        return false;
    }

    uint16_t raw = 0;
    if (!readReg(REG_CONVERSION, raw))
      return false;
    counts = (int16_t)raw;
    return true;
  }

private:
  static const uint8_t REG_CONVERSION = 0x00;
  static const uint8_t REG_CONFIG = 0x01;
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

// ------------------------------- Globales ----------------------------------
static Ads1115 adcNtc(ADS_NTC_ADDR);
static Ads1115 adcVI(ADS_VI_ADDR);
static SdFat sd;
static SdFile csv;
static DHT dht(PIN_DHT, DHT11);

static char logName[13]; // "LOG0001.CSV"
static bool sdOk = false;
static bool adcOk = false; // true si ambos ADS respondieron
static uint8_t i2cFailStreak = 0;

static uint32_t t0Ms = 0;
static uint32_t nextSampleMs = 0;
static uint32_t lastDhtMs = 0;
static float airTempC = NAN;
static float humidityPct = NAN;

// ------------------------------- LED de estado -----------------------------
// SD con error     : parpadeo rapido continuo
// ADS no responde  : doble pulso cada segundo
// Todo OK          : un pulso corto cada segundo
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

// ------------------------------- I2C ---------------------------------------
static void i2cBegin()
{
  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.begin();
  Wire.setClock(100000);
}

// Libera un bus I2C trabado (un esclavo sujetando SDA en bajo)
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
  // Condicion STOP
  pinMode(PIN_SDA, OUTPUT);
  digitalWrite(PIN_SDA, LOW);
  delayMicroseconds(5);
  pinMode(PIN_SCL, INPUT_PULLUP);
  delayMicroseconds(5);
  pinMode(PIN_SDA, INPUT_PULLUP);
  delayMicroseconds(5);

  i2cBegin();
}

// ------------------------------- Escala y filtro ---------------------------
static float computeVolts(int16_t counts)
{
  return (float)counts * ADS_VOLTS_PER_COUNT;
}

static int16_t medianOf(int16_t *v, uint8_t n)
{
  for (uint8_t i = 1; i < n; ++i)
  {
    const int16_t x = v[i];
    uint8_t j = i;
    while (j > 0 && v[j - 1] > x)
    {
      v[j] = v[j - 1];
      --j;
    }
    v[j] = x;
  }
  return v[n / 2];
}

// false si el ADS deja de responder por I2C.
static bool readFilteredCounts(Ads1115 &adc, uint8_t channel, int16_t &counts)
{
  int16_t samples[FILTER_SAMPLES];
  for (uint8_t i = 0; i < FILTER_SAMPLES; ++i)
  {
    if (!adc.readSingleEnded(channel, samples[i]))
      return false;
  }
  counts = medianOf(samples, FILTER_SAMPLES);
  return true;
}

// ------------------------------- NTC ---------------------------------------
// Rntc = Rf * (Vin - Vadc) / Vadc
static float temperaturaNTC(float voltaje)
{
  if (voltaje <= 0.01f || voltaje >= (NTC_VIN - 0.01f))
    return NAN;

  const float rNtc = NTC_RF * (NTC_VIN - voltaje) / voltaje;
  const float invT = (1.0f / NTC_T0) + (logf(rNtc / NTC_R0) / NTC_BETA);
  return (1.0f / invT) - 273.15f;
}

// ------------------------------- Formato de fila ---------------------------
// Se arma la fila a mano: no depende de printf con floats ni de
// Print::print(float).
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
      // evita "-0.0"
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
  char buf_[192];
  size_t len_ = 0;
};

static LineBuffer line;

// ------------------------------- SD ----------------------------------------
static const char CSV_HEADER[] =
    "time_ms;T1_C;T2_C;T3_C;T4_C;V_NTC1;V_NTC2;V_NTC3;V_NTC4;"
    "voltage_V;current_sensor_V;current_A;humidity_percent;air_temp_C;"
    "adc1_ok;adc2_ok;error\n";

static bool sdWriteAndSync(const char *data, size_t n)
{
  if (csv.write(data, n) != n)
    return false;
  return csv.sync();
}

// Crea un archivo nuevo LOGxxxx.CSV con encabezado
static bool sdStartNewFile()
{
  if (!sd.begin(PIN_SD_CS, SD_SCK_MHZ(4)))
    return false;

  for (uint16_t n = 1; n <= 9999; ++n)
  {
    logName[0] = 'L';
    logName[1] = 'O';
    logName[2] = 'G';
    logName[3] = (char)('0' + (n / 1000) % 10);
    logName[4] = (char)('0' + (n / 100) % 10);
    logName[5] = (char)('0' + (n / 10) % 10);
    logName[6] = (char)('0' + n % 10);
    logName[7] = '.';
    logName[8] = 'C';
    logName[9] = 'S';
    logName[10] = 'V';
    logName[11] = 0;

    if (sd.exists(logName))
      continue;
    if (!csv.open(logName, O_WRONLY | O_CREAT | O_EXCL))
      return false;
    return sdWriteAndSync(CSV_HEADER, sizeof(CSV_HEADER) - 1);
  }
  return false;
}

// Reabre el archivo actual en modo append (por si la SD se solto un momento)
static bool sdReopen()
{
  if (csv.isOpen())
    csv.close();
  if (!sd.begin(PIN_SD_CS, SD_SCK_MHZ(4)))
    return false;
  return csv.open(logName, O_WRONLY | O_AT_END);
}

// ------------------------------- Muestreo ----------------------------------
static void takeSample(uint32_t elapsedMs)
{
  float temps[4] = {NAN, NAN, NAN, NAN};
  float vNtc[4] = {NAN, NAN, NAN, NAN};
  float voltage = NAN, sensorV = NAN, current = NAN;
  uint8_t ok1 = 0, ok2 = 0;
  bool i2cError = false;

  // ADS 0x48 : termistores. Mediana de varias lecturas y luego computeVolts.
  for (uint8_t ch = 0; ch < 4; ++ch)
  {
    int16_t counts;
    if (readFilteredCounts(adcNtc, ch, counts))
    {
      vNtc[ch] = computeVolts(counts);
      temps[ch] = temperaturaNTC(vNtc[ch]);
      ++ok1;
    }
    else
    {
      i2cError = true;
      break; // si el chip no responde, no insistir en los demas
    }
  }

  // ADS 0x49 : voltaje de celda (A0) y sensor de corriente (A1)
  {
    int16_t c0, c1;
    if (readFilteredCounts(adcVI, 0, c0))
    {
      voltage = computeVolts(c0);
      ++ok2;
      if (readFilteredCounts(adcVI, 1, c1))
      {
        sensorV = computeVolts(c1);
        current = CURRENT_SLOPE * sensorV + CURRENT_OFFSET;
        if (current < 0.0f)
          current = 0.0f;
        ++ok2;
      }
      else
        i2cError = true;
    }
    else
      i2cError = true;
  }

  adcOk = (ok1 == 4 && ok2 == 2);

  // Si el bus falla varias veces seguidas, se intenta liberarlo
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

  // Armar fila (separador ';')
  line.clear();
  line.putUint(elapsedMs);
  for (uint8_t i = 0; i < 4; ++i)
  {
    line.putChar(';');
    line.putFixed(temps[i], 2);
  }
  for (uint8_t i = 0; i < 4; ++i)
  {
    line.putChar(';');
    line.putFixed(vNtc[i], 4);
  }
  line.putChar(';');
  line.putFixed(voltage, 4);
  line.putChar(';');
  line.putFixed(sensorV, 4);
  line.putChar(';');
  line.putFixed(current, 2);
  line.putChar(';');
  line.putFixed(humidityPct, 1);
  line.putChar(';');
  line.putFixed(airTempC, 1);
  line.putChar(';');
  line.putUint(ok1);
  line.putChar(';');
  line.putUint(ok2);
  line.putChar(';');
  if (ok1 != 4 && ok2 != 2)
    line.putText("ERROR no lee ADS 0x48 ni ADS 0x49 por I2C");
  else if (ok1 != 4)
    line.putText("ERROR no lee ADS 0x48 por I2C");
  else if (ok2 != 2)
    line.putText("ERROR no lee ADS 0x49 por I2C");
  line.putChar('\n');

  // Escribir; si falla, reabrir el archivo y reintentar una vez
  if (!sdWriteAndSync(line.data(), line.size()))
  {
    sdOk = sdReopen() && sdWriteAndSync(line.data(), line.size());
  }
  else
    sdOk = true;
}

static void updateDht()
{
  if (!USE_DHT)
    return;
  // Lectura bloqueante (~25 ms); se hace justo despues de muestrear.
  const float t = dht.readTemperature();
  const float h = dht.readHumidity();
  if (isfinite(t))
    airTempC = t;
  if (isfinite(h))
    humidityPct = h;
}

// ------------------------------- Arduino -----------------------------------
void setup()
{
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
  SPI.begin();

  // Esperar a que la SD este lista (LED parpadea rapido mientras tanto)
  while (!(sdOk = sdStartNewFile()))
  {
    const uint32_t t = millis();
    while (millis() - t < 1000)
      updateLed();
  }

  i2cBegin();
  if (USE_DHT)
  {
    dht.begin();
    delay(1000); // el DHT11 necesita ~1 s tras energizar
    updateDht();
  }

  t0Ms = millis();
  lastDhtMs = t0Ms;
  nextSampleMs = t0Ms; // primera muestra inmediata, tiempo = 0
}

void loop()
{
  updateLed();

  const uint32_t now = millis();
  if ((int32_t)(now - nextSampleMs) >= 0)
  {
    takeSample(now - t0Ms);

    // Siguiente instante fijo; si nos atrasamos, resincronizar
    nextSampleMs += SAMPLE_PERIOD_MS;
    const uint32_t after = millis();
    if ((int32_t)(after - nextSampleMs) >= 0)
      nextSampleMs = after + SAMPLE_PERIOD_MS;

    if (after - lastDhtMs >= DHT_PERIOD_MS)
    {
      lastDhtMs = after;
      updateDht();
    }
  }
}