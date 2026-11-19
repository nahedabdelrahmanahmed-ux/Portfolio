/*
 * =====================================================================
 *  GREEN BOX v7.0 - Industrial Predictive Maintenance Edge Unit  (investor-demo build)
 *  Target : ESP32-S3 (also runs on classic ESP32, e.g. Wokwi "ESP32 DevKit")
 *  Core   : Arduino-ESP32 2.x or 3.x
 * =====================================================================
 *
 *  SENSOR CHANNELS (5)
 *    VIB  vibration  : ADXL345 accelerometer, I2C, dynamic RMS (g)
 *    TMP  temperature: DS18B20, One-Wire (deg C)
 *    CUR  motor current : ACS712 (or any AC current sensor), ADC, RMS (A)
 *    RPM  shaft speed   : Hall / optical tach pulse, interrupt (rpm)
 *    ACO  acoustic level: analog microphone (MAX9814 / KY-037), ADC, RMS (% FS)
 *
 *  ARCHITECTURE
 *    +-------------------+  frames (100 ms)  +-----------------------------+
 *    | acqTask  (1 kHz)  | ----------------> | loop()                      |
 *    |  ADXL345 200 Hz   |   FreeRTOS queue  |  anomaly detection (5 ch)   |
 *    |  current/mic 1 kHz|                   |  DS18B20 + tach             |
 *    +-------------------+                   |  relay, LED, logs, commands |
 *                                            +-----------------------------+
 *
 *  SAFETY FEATURES
 *    - Moving-window z-score spike detector per channel (+ debounce)
 *    - Absolute hard limits (immediate trip), checked even while learning
 *    - Sensor-failure watchdog (I2C loss, DS18B20 disconnected / 85 C glitch)
 *    - Acquisition-stall watchdog + hardware task watchdog (TWDT)
 *    - Latched trip, persisted in NVS: survives power cycles
 *    - Unexpected reset (panic / WDT / brownout) => starts TRIPPED
 *    - Anomalous samples never enter the baseline window
 *    - Production mode: ALLOW_SIMULATION=false refuses to run on missing sensors
 *
 *  PINS (default)                  ESP32-S3          classic ESP32
 *    ADXL345 SDA / SCL             GPIO8 / GPIO9     GPIO21 / GPIO22
 *    DS18B20 DATA (4.7k pull-up)   GPIO4             GPIO4
 *    Relay IN                      GPIO5             GPIO5
 *    Tach pulse input              GPIO6             GPIO18
 *    Current sensor (analog)       GPIO1             GPIO34
 *    Microphone (analog)           GPIO2             GPIO35
 *    Status LED                    GPIO15            GPIO2
 *    Re-arm button (to GND)        GPIO0 (BOOT)      GPIO0 (BOOT)
 *
 *  LIBRARIES : OneWire, DallasTemperature   (Wokwi: add to libraries.txt)
 *
 *  ARDUINO IDE SETTINGS (tested with Arduino-ESP32 3.3.12)
 *    Board: ESP32S3 Dev Module (or ESP32 Dev Module)
 *    Tools > Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"  <- needed with BLE + WiFi
 *    (with COMM_ENABLE_TLS = 0 it also fits the default partition on the ESP32-S3)
 *    Tools > USB CDC On Boot: Enabled   (ESP32-S3 with the native USB port)
 *
 *  FAULT DIAGNOSIS + REMAINING USEFUL LIFE (new in v3)
 *    - On every mechanical trip the unit classifies the failure signature:
 *        BEARING    impulsive vibration (high crest factor / kurtosis) [+ gradual heating]
 *        BELT_1X    vibration locked to shaft speed (Goertzel at 1X RPM): belt / misalignment / balance
 *        GEAR_LUBE  sharp temperature rise with unchanged vibration: gear wear / oil loss
 *        OVERLOAD   current surge        BELT_SLIP  speed deviation
 *    - Remaining life per component = 100 - operating-hours / expected-life x stress
 *      (hours counted only while the shaft turns; stress rises with vibration,
 *       current and temperature above the learned healthy baseline)
 *    - Report is printed for the engineer and emitted as JSON lines for the PWA:
 *        @TEL {...} live telemetry (toggle with 'j')   @DIAG {...} diagnosis
 *        @LIFE {...} component life
 *
 *  COMMUNICATIONS (new in v4): BLE -> WiFi -> SIM failover with app ACK
 *    Critical messages (trip diagnosis) are delivered like this:
 *      1) BLE notify to the PWA; the app must answer {"ack":id}   (3 s window)
 *      2) no ACK / no BLE  -> HTTP POST to the server; the server holds the
 *         reply until the PWA acknowledges (app-level ACK, 8 s window)
 *      3) still no app ACK -> SMS through the SIM modem (SIM800L / SIM7600)
 *      The chain is retried every minute until an app confirms.
 *    Telemetry / life data are best-effort (BLE if connected, else WiFi).
 *    Remote re-arm is NOT possible by design: only the local button / serial.
 *
 *  INVESTOR DEMO (new in v6)
 *    Press the BOOT button (or send 'p'): starts a progressive bearing-wear story.
 *      Health index falls -> EARLY WARNING (machine still running, ETA to trip shown)
 *      -> emergency stop -> diagnosis -> alert chain BLE -> WiFi -> SMS.
 *    Hold BOOT 1 s while tripped (or send 'r') to re-arm.  'x' runs a pre-flight self-test.
 *    EVERYTHING IS SIMULATED unless real sensors are detected: the dashboard says so.
 *
 *  SERIAL COMMANDS (115200 baud)
 *    p start wear story   x self-test
 *    t send a test alert through the full BLE->WiFi->SIM chain   c comm status
45 / DS18B20 are auto-detected and simulated. Current,
 *    tach and mic are simulated unless USE_REAL_* is set to true below.
 * =====================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <stdarg.h>

#define FW_VERSION "7.0.0"

// ---- Communication transports (set to 0 to compile one out) ----
#define COMM_ENABLE_BLE    1
#define COMM_ENABLE_WIFI   1
#define COMM_ENABLE_SIM    1
#define COMM_ENABLE_TLS    0      // 1 = https to the server (needs the Huge APP partition; REQUIRED for internet use)
#define COMM_ALLOW_INSTALL 1      // PWA may register a new part over BLE (PIN protected)

#if COMM_ENABLE_BLE
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEUtils.h>
  #include <BLE2902.h>
#endif
#if COMM_ENABLE_WIFI
  #include <WiFi.h>
  #if COMM_ENABLE_TLS
    #include <WiFiClientSecure.h>
  #endif
  #include <HTTPClient.h>
#endif

// =====================================================================
//  CONFIG
// =====================================================================

// ---- Operating mode ----
constexpr bool     ALLOW_SIMULATION       = true;   // PRODUCTION: set false
constexpr bool     PERSIST_TRIP           = true;   // keep trip latched across reboots
constexpr bool     TRIP_ON_UNEXPECTED_RST = true;   // panic / WDT / brownout => tripped

// ---- Investor demo ----
constexpr bool     DEMO_MODE              = true;    // PRODUCTION: false (long baselines, no story controls)
constexpr bool     DEMO_AUTOPLAY          = false;   // true = booth mode: story loops unattended
constexpr uint32_t DEMO_AUTOPLAY_DELAY_MS = 12000;   // armed time before the story starts (booth mode)
constexpr uint32_t DEMO_REARM_DELAY_MS    = 25000;   // booth mode: automatic re-arm after a trip
constexpr float    DEMO_WEAR_RAMP_S       = 60.0f;   // bearing wear 0 -> 100 % over this time
constexpr bool     DEMO_PRELOAD_LIFE      = true;    // first boot: machine already has operating hours
constexpr float    HEALTH_WATCH           = 78.0f;   // health below this (2 s) -> EARLY WARNING
constexpr float    HEALTH_CLEAR           = 90.0f;   // health above this (4 s) -> warning cleared
constexpr float    HEALTH_Z0              = 1.5f;    // |z| where abnormality starts to count
// The fast detector adapts to slow changes by design, so a FROZEN healthy reference is kept as well:
// gradual wear is caught as drift away from it. 0 = never adapts (re-learn with 'r' after maintenance).
constexpr float    REF_ADAPT_ALPHA        = 0.0f;
constexpr uint32_t DIAG_SETTLE_MS       = 700;     // after a trip: wait for a complete signature before reporting
constexpr uint8_t  DRIFT_HOLD_S           = DEMO_MODE ? 3 : 30;   // seconds of sustained drift before the trip
constexpr uint32_t REARM_LOCKOUT_MS       = 2000;   // min time between trip and re-arm
constexpr uint32_t WDT_TIMEOUT_S          = 5;      // hardware task watchdog

// ---- Which analog/pulse sensors are physically wired ----
constexpr bool USE_REAL_CURRENT  = false;
constexpr bool USE_REAL_ACOUSTIC = false;
constexpr bool USE_REAL_TACH     = false;

// ---- Pins ----
#if CONFIG_IDF_TARGET_ESP32S3
constexpr int PIN_I2C_SDA = 8,  PIN_I2C_SCL = 9;
constexpr int PIN_ONEWIRE = 4,  PIN_RELAY   = 5;
constexpr int PIN_TACH    = 6,  PIN_CURRENT = 1, PIN_ACOUSTIC = 2;
constexpr int PIN_LED     = 15, PIN_RESET_BTN = 0;
constexpr int PIN_BUZZER  = 7;                  // active buzzer (-1 = none)
#else
constexpr int PIN_I2C_SDA = 21, PIN_I2C_SCL = 22;
constexpr int PIN_ONEWIRE = 4,  PIN_RELAY   = 5;
constexpr int PIN_TACH    = 18, PIN_CURRENT = 34, PIN_ACOUSTIC = 35;
constexpr int PIN_LED     = 2,  PIN_RESET_BTN = 0;
constexpr int PIN_BUZZER  = 19;                 // active buzzer (-1 = none)
#endif

// ---- Relay polarity (fail-safe wiring: swap levels and use the NC contact) ----
constexpr uint8_t RELAY_TRIP_LEVEL = HIGH;
constexpr uint8_t RELAY_RUN_LEVEL  = LOW;

// ---- ADXL345 ----
constexpr uint8_t ADXL_ADDR        = 0x53;
constexpr uint8_t ADXL_REG_DEVID   = 0x00;
constexpr uint8_t ADXL_REG_BWRATE  = 0x2C;
constexpr uint8_t ADXL_REG_PWRCTL  = 0x2D;
constexpr uint8_t ADXL_REG_DATAFMT = 0x31;
constexpr uint8_t ADXL_REG_DATAX0  = 0x32;
constexpr float   ADXL_LSB_G       = 0.0039f;   // full-res ~3.9 mg/LSB

// ---- Acquisition timing ----
constexpr uint16_t FRAME_TICKS     = 100;       // 1 tick = 1 ms -> 100 ms frames
constexpr uint8_t  ADXL_DIV        = 5;         // ADXL345 every 5 ms (200 Hz)
constexpr uint8_t  ADXL_PER_FRAME  = FRAME_TICKS / ADXL_DIV;
constexpr uint32_t TEMP_PERIOD_MS  = 1000;
constexpr uint32_t TEMP_CONV_MS    = 450;       // 11-bit conversion ~375 ms
constexpr uint32_t TACH_GATE_MS    = 500;
constexpr uint32_t STATUS_PERIOD_MS         = 1000;
constexpr uint32_t STATUS_PERIOD_TRIPPED_MS = 5000;

// ---- Analog scaling ----
constexpr float ANALOG_CENTER_MV   = 1650.0f;   // mid-rail bias of AC sensors
constexpr float CUR_MV_PER_A       = 66.0f;     // ACS712-20A (100 mV/A) via 2:3 divider to 3.3 V
constexpr float ACO_FULLSCALE_MV   = 1650.0f;   // 100 % FS = +/- 1.65 V swing

// ---- Tach ----
constexpr uint8_t  TACH_PPR           = 1;      // pulses per revolution
constexpr uint32_t TACH_MIN_PERIOD_US = 2000;   // debounce
constexpr uint32_t TACH_MAX_PERIOD_US = 2000000;

// ---- Sensor watchdogs ----
constexpr uint8_t  ACCEL_FAIL_LIMIT = 20;       // consecutive failed reads
constexpr uint8_t  TEMP_FAIL_LIMIT  = 3;        // consecutive bad temp reads
constexpr uint32_t ACQ_TIMEOUT_MS   = 500;      // no frame => acquisition stall
constexpr uint8_t  SETTLE_FRAMES    = 3;        // frames ignored after re-arm

// ---- Simulation model ----
constexpr float SIM_TEMP_BASE_C = 45.0f;
constexpr float SIM_AMBIENT_C   = 30.0f;
constexpr float SIM_RPM_NOM     = 1500.0f;
constexpr float SIM_CUR_NOM_A   = 4.0f;
constexpr float SIM_ACO_NOM_MV  = 40.0f;

// ---- Diagnosis & component life ----
constexpr float    RUN_RPM_MIN         = 100.0f;  // shaft above this speed = "machine running"
constexpr float    LIFE_TIME_SCALE     = DEMO_MODE ? 360.0f : 1.0f;   // demo: 1 real second = 6 operating minutes
constexpr uint32_t LIFE_SAVE_PERIOD_S  = 600;     // flash save interval (real seconds of operation)
constexpr uint16_t DIAG_BASE_FRAMES    = DEMO_MODE ? 20 : 50;   // frames to learn the signature baseline
constexpr float    DIAG_MIN_SCORE      = 0.25f;   // below this => "unclassified"
constexpr uint32_t TELEMETRY_PERIOD_MS = 1000;    // @TEL period while streaming
constexpr bool     PWA_STREAM_DEFAULT  = false;   // true: stream @TEL JSON from boot
constexpr float    RUL_WARN_PCT        = 15.0f;   // one-time warning when a part drops below this

// ---- Communications: credentials & timing (CHANGE THESE) ----
#define BLE_NAME_PREFIX "GreenBox"
#define WIFI_SSID       "Wokwi-GUEST"
#define WIFI_PASS       ""
#define SERVER_URL      "http://192.168.1.10:8080/api/ingest"   // https:// needs COMM_ENABLE_TLS=1 + SERVER_CA_PEM (or TLS_INSECURE)
#define DEVICE_TOKEN    "change-me-device-token"
#define SERVER_CA_PEM   ""                                       // root CA of your server (PEM string)
#define TLS_INSECURE    0                                        // 1 = skip certificate check (development only!)
#define COMM_PIN        "1234"                                   // PIN for BLE 'install' commands
#define SIM_PHONE_1     "+201000000000"                          // engineer numbers (international format)
#define SIM_PHONE_2     ""

constexpr int      PIN_SIM_RX          = 16;      // ESP32 RX  <- modem TX
constexpr int      PIN_SIM_TX          = 17;      // ESP32 TX  -> modem RX
constexpr uint32_t SIM_BAUD            = 115200;
constexpr uint32_t COMM_ACK_BLE_MS     = 3000;    // wait for the app's ACK over BLE
constexpr uint32_t COMM_ACK_WIFI_MS    = 8000;    // WiFi connect + server hold for the app's ACK
constexpr uint32_t COMM_RETRY_MS       = 60000;   // retry period for an unconfirmed critical alert
constexpr uint8_t  COMM_MAX_RETRY      = 10;
constexpr uint32_t COMM_TEL_PERIOD_MS  = 5000;
constexpr uint32_t COMM_LIFE_PERIOD_MS = 60000;
constexpr uint32_t WIFI_RETRY_MS       = 30000;
constexpr size_t   BLE_CHUNK_BYTES     = 20;      // safe for every default MTU; raise after testing

// =====================================================================
//  CHANNEL TABLE - one place to tune every threshold
// =====================================================================

enum Ch : uint8_t { CH_VIB, CH_TEMP, CH_CUR, CH_RPM, CH_ACO, CH_COUNT };

struct ChannelCfg {
  const char *tag;        // short label for logs
  const char *name;       // long name for trip reasons
  const char *unit;
  uint8_t  decimals;
  uint8_t  win;           // window length (samples of healthy history)
  float    zThr;          // sigmas that count as a violation
  float    sigmaFloor;    // minimum sigma (avoids over-sensitivity on quiet signals)
  uint8_t  strikes;       // consecutive violations to trip
  bool     twoSided;      // true: deviations in both directions
  float    hardLow;       // immediate trip below (NAN = off)
  float    hardHigh;      // immediate trip above (NAN = off)
  float    driftZ;        // sustained deviation from the FROZEN healthy reference (sigmas) that trips
};

constexpr size_t MAX_WIN = 64;

// =====================================================================
//  COMPONENT TABLE - edit names / expected life / stress sensitivity
// =====================================================================

enum Comp : uint8_t { COMP_BEARING, COMP_BELT, COMP_GEAR, COMP_COUNT };

struct CompCfg {
  const char *id;          // short id for JSON
  const char *name;        // shown to the engineer
  float lifeHours;         // expected life at nominal stress
  float kVib, kTemp, kCur; // how strongly vibration / heat / current age this part
};

static const CompCfg COMPS[COMP_COUNT] = {
  { "bearing", "Drive-end bearing (SKF 6205)", 2000.0f, 1.0f, 0.6f, 0.2f },
  { "belt",    "Drive belt",                   3000.0f, 0.3f, 0.2f, 1.0f },
  { "gear",    "Gearbox / gear mesh",          5000.0f, 0.4f, 0.8f, 0.5f },
};

static const ChannelCfg CFG[CH_COUNT] = {
  // tag   name           unit  dec win  z     sigma  k  2side hardLow hardHigh driftZ
  { "VIB", "VIBRATION",   "g",   3, DEMO_MODE ? 20 : 50, 4.0f, 0.02f, 3, false, NAN,    1.50f,   12.0f },
  { "TMP", "TEMPERATURE", "C",   1, DEMO_MODE ?  8 : 30, 4.0f, 0.30f, 3, false, NAN,    85.0f,   30.0f },
  { "CUR", "MOTOR CURRENT","A",  2, DEMO_MODE ? 20 : 50, 4.0f, 0.15f, 3, false, NAN,    12.0f,   20.0f },
  { "RPM", "SHAFT SPEED", "rpm", 0, DEMO_MODE ?  6 : 20, 4.0f, 15.0f, 3, true,  300.0f, 3600.0f, 30.0f },
  { "ACO", "ACOUSTIC",    "%",   1, DEMO_MODE ? 20 : 50, 4.0f, 0.50f, 3, false, NAN,    60.0f,   24.0f },
};

// =====================================================================
//  TYPES & GLOBALS
// =====================================================================

enum class UnitState : uint8_t { WARMUP, RUN, TRIPPED };
enum class TripKind  : uint8_t { MECHANICAL, SENSOR, SYSTEM };

enum FaultClass : uint8_t { FC_BEARING, FC_BELT_1X, FC_GEAR_LUBE, FC_OVERLOAD, FC_BELT_SLIP, FC_UNKNOWN, FC_COUNT };

struct Ev { float zv, zt, zc, zr, za, slope; };           // evidence latched at the moment of the trip
struct DiagFeat  { float crest, kurt, oneX; };           // vibration signature per 0.5 s window
struct Diagnosis {
  uint8_t cls;  uint8_t conf;                            // class, confidence 0..99
  float   score[FC_COUNT];
  float   imp, x1, slope, zCur, zRpm;                    // evidence shown in the report
};

// Simulated-fault bit mask (written by main, read by acquisition task)
enum : uint32_t {
  FAULT_VIB        = 1u << 0,
  FAULT_THERMAL    = 1u << 1,
  FAULT_CURRENT    = 1u << 2,
  FAULT_SPEED      = 1u << 3,
  FAULT_ACOUSTIC   = 1u << 4,
  FAULT_VIB_SHOCK  = 1u << 5,
  FAULT_ACCEL_COMM = 1u << 6,
  FAULT_TEMP_PROBE = 1u << 7,
  FAULT_TEMP_OVER  = 1u << 8,
  FAULT_1X         = 1u << 9,
};

struct Frame {            // one 100 ms feature frame from the acquisition task
  float   vibRms;
  float   curA;
  float   acoPct;
  uint8_t accelFails;
  bool    vibValid;
  bool    diagOk;      // signature features valid
  float   crest, kurt, oneX;
};

// ---- Types used in function signatures MUST be declared before the first function:
//      the Arduino IDE inserts auto-generated prototypes above everything else. ----
constexpr uint16_t DIAG_N  = 100;      // 0.5 s signature window at 200 Hz
constexpr float    ADXL_FS = 200.0f;

enum MsgPrio : uint8_t { PRIO_TEL = 0, PRIO_INFO = 1, PRIO_CRITICAL = 2 };
constexpr size_t COMM_MAX_JSON = 1100;

struct OutMsg {
  uint32_t id;
  uint8_t  prio;
  uint8_t  attempts;
  bool     smsSent;
  uint16_t len;
  char     json[COMM_MAX_JSON];
  char     sms[161];
};

struct JBuf {
  char   b[900];
  size_t n = 0;
  JBuf() { b[0] = 0; }
  void add(const char *fmt, ...) {
    if (n >= sizeof(b) - 1) return;
    va_list ap; va_start(ap, fmt);
    int k = vsnprintf(b + n, sizeof(b) - n, fmt, ap);
    va_end(ap);
    if (k > 0) n += ((size_t)k < sizeof(b) - n) ? (size_t)k : sizeof(b) - n - 1;
  }
  void num(float v, int dec) { if (isnan(v)) add("null"); else add("%.*f", dec, v); }
};

OneWire           oneWire(PIN_ONEWIRE);
DallasTemperature tempSensor(&oneWire);
Preferences       prefs;

QueueHandle_t frameQ = nullptr;

// Shared with the acquisition task
volatile uint32_t g_faults         = 0;
volatile bool     g_machineRunning = true;
volatile float    g_simRun         = 1.0f;
volatile uint32_t g_droppedFrames  = 0;
bool              simulated[CH_COUNT] = {};   // immutable after setup()

// Main-thread state
UnitState state      = UnitState::WARMUP;
bool      bootFault  = false;
char      tripReason[112] = "";
uint32_t  tripAtMs   = 0;
uint32_t  tripCount  = 0;
uint32_t  bootCount  = 0;
uint8_t   settleFrames = 0;
uint32_t  lastFrameMs  = 0;
uint32_t  lastStatusMs = 0;
uint32_t  runSinceMs   = 0;

float lastVal[CH_COUNT];

// Temperature state machine
bool     tempRequested   = false;
uint32_t tempRequestedMs = 0;
uint32_t lastTempCycleMs = 0;
uint8_t  tempFails       = 0;
float    simTempC        = SIM_TEMP_BASE_C;

// Tach (ISR-shared)
static portMUX_TYPE tachMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t tachLastUs    = 0;
volatile uint32_t tachPeriodSum = 0;
volatile uint32_t tachPeriodCnt = 0;
uint32_t lastTachMs = 0;

uint32_t rngMain = 0x9E3779B9u;

volatile float g_wear = 0.0f;           // simulated bearing wear 0..1 (story), read by the acquisition task
bool      storyActive = false;
uint32_t  storyStartMs = 0;
float     refMean[CH_COUNT], refSd[CH_COUNT];   // frozen healthy reference per channel
bool      refReady[CH_COUNT];
float     vSm[CH_COUNT];                        // smoothed value per channel (EMA)
bool      vSmReady[CH_COUNT];
uint8_t   driftCnt[CH_COUNT];
float     g_health = 100.0f;            // machine health index 0..100
bool      g_watch  = false;             // early warning active
float     g_eta    = NAN;               // estimated seconds to trip (linear trend), NAN = unknown
float     hHist[8];
uint8_t   hHistN = 0, watchCnt = 0, clearCnt = 0;
uint32_t  lastHealthMs = 0;
bool      diagPending = false;
uint32_t  diagDueMs = 0;
volatile float g_rpm = 0.0f;            // latest shaft speed (for 1X analysis in the acquisition task)
TripKind  tripKind   = TripKind::SYSTEM;
Diagnosis lastDiag;
bool      haveDiag   = false;
bool      pendingNew = false;           // 'n' pressed, waiting for part letter
bool      streamJson = PWA_STREAM_DEFAULT;
uint32_t  lastLifeMs = 0, lastTelMs = 0, lifeSaveTick = 0;

// =====================================================================
//  SMALL HELPERS
// =====================================================================

// NOTE: this macro defines a function, so it must stay BELOW every type declaration: the Arduino IDE puts
// its auto-generated prototypes above the first function in the file.
#if defined(SET_LOOP_TASK_STACK_SIZE)
SET_LOOP_TASK_STACK_SIZE(12 * 1024);   // diagnosis + JSON builders need more than the default 8 KB
#endif

/** xorshift32 noise in [-1, 1). Each task owns its own seed (thread-safe). */
static inline float frand(uint32_t &s) {
  s ^= s << 13; s ^= s >> 17; s ^= s << 5;
  return (float)((int32_t)s) / 2147483648.0f;
}

/** Phase-accumulator oscillator (no precision loss on long uptimes). */
struct Osc {
  float ph = 0, step = 0;
  void init(float hz, float dt) { step = 2.0f * PI * hz * dt; }
  float next() { ph += step; if (ph > 2.0f * PI) ph -= 2.0f * PI; return sinf(ph); }
};

static void logTs() { Serial.printf("[%8.1fs] ", millis() / 1000.0f); }

static const char *stateName(UnitState s) {
  switch (s) {
    case UnitState::WARMUP: return "WARMUP ";
    case UnitState::RUN:    return "RUN    ";
    default:                return "TRIPPED";
  }
}

static const char *resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWER-ON";
    case ESP_RST_EXT:       return "EXTERNAL";
    case ESP_RST_SW:        return "SOFTWARE";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT-WDT";
    case ESP_RST_TASK_WDT:  return "TASK-WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEP-SLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    default:                return "OTHER";
  }
}

static bool isUnexpectedReset(esp_reset_reason_t r) {
  return r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT ||
         r == ESP_RST_WDT   || r == ESP_RST_BROWNOUT;
}

// =====================================================================
//  MOVING-WINDOW SPIKE DETECTOR
// =====================================================================

/**
 * z-score of each new sample against a window of recent HEALTHY samples.
 * Samples flagged as anomalous are not added to the window, so a developing
 * fault cannot slowly poison the baseline.
 */
class SpikeDetector {
public:
  float   z = 0;
  uint8_t strikes = 0;

  void reset() { count_ = 0; head_ = 0; strikes = 0; z = 0; mean_ = 0; sd_ = 0; }
  bool   ready(const ChannelCfg &c) const { return count_ >= c.win; }
  size_t fill() const { return count_; }
  float  mean() const { return mean_; }
  float  sd()   const { return sd_; }

  /** Returns true once the anomaly is confirmed (>= c.strikes in a row). */
  bool update(float x, const ChannelCfg &c) {
    computeStats();
    if (count_ < c.win) {            // learning phase
      push(x, c.win);
      z = 0;
      return false;
    }
    float denom = (sd_ > c.sigmaFloor) ? sd_ : c.sigmaFloor;
    z = (x - mean_) / denom;
    float dev = c.twoSided ? fabsf(z) : z;
    if (dev > c.zThr) {
      if (strikes < 255) strikes++;
    } else {
      strikes = 0;
      push(x, c.win);                // only healthy data updates the baseline
    }
    return strikes >= c.strikes;
  }

private:
  float  buf_[MAX_WIN] = {0};
  size_t head_ = 0, count_ = 0;
  float  mean_ = 0, sd_ = 0;

  void push(float v, size_t win) {
    buf_[head_] = v;
    head_ = (head_ + 1) % win;
    if (count_ < win) count_++;
  }
  void computeStats() {
    if (count_ == 0) { mean_ = 0; sd_ = 0; return; }
    float sum = 0;
    for (size_t i = 0; i < count_; i++) sum += buf_[i];
    mean_ = sum / count_;
    float ss = 0;
    for (size_t i = 0; i < count_; i++) { float d = buf_[i] - mean_; ss += d * d; }
    sd_ = sqrtf(ss / count_);
  }
};

SpikeDetector det[CH_COUNT];

// =====================================================================
//  RELAY, LED, PERSISTENCE
// =====================================================================

static void relaySetRun()  { digitalWrite(PIN_RELAY, RELAY_RUN_LEVEL);  }
static void relaySetTrip() { digitalWrite(PIN_RELAY, RELAY_TRIP_LEVEL); }

/** LED: slow blink = learning, short heartbeat = running, solid = tripped. */
static void serviceLed() {
  uint32_t now = millis();
  bool on;
  switch (state) {
    case UnitState::TRIPPED: on = true; break;
    case UnitState::WARMUP:  on = ((now / 500) % 2) == 0; break;
    default:                 on = (now % 2000) < 80; break;
  }
  digitalWrite(PIN_LED, on ? HIGH : LOW);
}

static void persistTrip() {
  if (!PERSIST_TRIP) return;
  prefs.putBool("latched", true);
  prefs.putString("reason", tripReason);
  prefs.putUInt("trips", tripCount);
  prefs.putUChar("dkind", (uint8_t)tripKind);
  prefs.putUChar("dcls",  lastDiag.cls);
  prefs.putUChar("dconf", lastDiag.conf);
}

// =====================================================================
//  COMMUNICATIONS CORE: BLE -> WiFi -> SIM failover with application ACK
// =====================================================================
//  All slow I/O runs in commTask (low priority) so the safety loop is never blocked.
//  Envelope (one JSON line):  {"v":1,"dev":"GB-xxxxxx","boot":N,"id":N,"type":"diag","r":1,"t":12.3,"d":{...}}
//  "r":1 = the sender expects {"ack":id} from the app.


static QueueHandle_t   commQ = nullptr;
static char            g_devId[16] = "GB-000000";
static uint32_t        g_msgId = 0;
static volatile uint32_t g_ackId = 0;              // last ACK received over BLE
static volatile bool   g_bleConnected = false;
static volatile bool   g_bleJustConnected = false;
static volatile int8_t g_cmdInstall = -1;          // set by BLE task, executed by the main loop
static volatile bool   g_cmdGetLife = false, g_cmdGetDiag = false, g_cmdGetHello = false;
static volatile uint8_t g_lastWifiState = 0;

static void commLog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void commLog(const char *fmt, ...) {
  char b[200];
  va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap);
  Serial.printf("[%8.1fs] COMM  %s\n", millis() / 1000.0f, b);
}

/** Queue a message for delivery. Main thread only (uses a static buffer). */
static bool commPost(const char *type, uint8_t prio, const char *dataJson, const char *sms = nullptr) {
  if (!commQ) return false;
  static OutMsg m;
  memset(&m, 0, sizeof(m));
  m.id   = ++g_msgId;
  m.prio = prio;
  int n = snprintf(m.json, sizeof(m.json),
                   "{\"v\":1,\"dev\":\"%s\",\"boot\":%u,\"id\":%u,\"type\":\"%s\",\"r\":%d,\"t\":%.1f,\"d\":%s}\n",
                   g_devId, (unsigned)bootCount, (unsigned)m.id, type, prio >= PRIO_INFO ? 1 : 0,
                   millis() / 1000.0f, dataJson);
  if (n <= 0 || (size_t)n >= sizeof(m.json)) return false;
  m.len = (uint16_t)n;
  if (sms) snprintf(m.sms, sizeof(m.sms), "%s", sms);
  return xQueueSend(commQ, &m, 0) == pdTRUE;
}

// ---- tiny JSON field readers (commands from the app are flat objects) ----
static bool jsonStr(const char *s, const char *key, char *out, size_t n) {
  char pat[24]; snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char *p = strstr(s, pat); if (!p) return false;
  p += strlen(pat);
  while (*p == ' ' || *p == ':') p++;
  if (*p != '"') return false;
  p++;
  size_t i = 0;
  while (*p && *p != '"' && i < n - 1) out[i++] = *p++;
  out[i] = 0;
  return true;
}
static bool jsonNum(const char *s, const char *key, uint32_t &v) {
  char pat[24]; snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char *p = strstr(s, pat); if (!p) return false;
  p += strlen(pat);
  while (*p == ' ' || *p == ':') p++;
  if (*p < '0' || *p > '9') return false;
  v = (uint32_t)strtoul(p, nullptr, 10);
  return true;
}

/** One complete line received from the app over BLE. */
__attribute__((unused)) static void commHandleAppLine(const char *line) {
  uint32_t id;
  if (jsonNum(line, "ack", id)) { g_ackId = id; return; }
  char cmd[16];
  if (!jsonStr(line, "cmd", cmd, sizeof(cmd))) return;
  if (!strcmp(cmd, "get")) {
    char what[12] = "";
    jsonStr(line, "what", what, sizeof(what));
    if      (!strcmp(what, "life")) g_cmdGetLife = true;
    else if (!strcmp(what, "diag")) g_cmdGetDiag = true;
    else                            g_cmdGetHello = true;
  } else if (!strcmp(cmd, "install")) {
#if COMM_ALLOW_INSTALL
    char part[12] = "", pin[16] = "";
    jsonStr(line, "part", part, sizeof(part));
    jsonStr(line, "pin", pin, sizeof(pin));
    if (strcmp(pin, COMM_PIN) != 0) { commLog("install command rejected: wrong PIN"); return; }
    int comp = !strcmp(part, "bearing") ? COMP_BEARING : !strcmp(part, "belt") ? COMP_BELT :
               !strcmp(part, "gear")    ? COMP_GEAR : -1;
    if (comp >= 0) g_cmdInstall = (int8_t)comp;
#else
    commLog("install command ignored (COMM_ALLOW_INSTALL=0)");
#endif
  }
}

// ---------------------------------------------------------------- BLE ----
#if COMM_ENABLE_BLE
#define BLE_UUID_SVC "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_UUID_RX  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // app -> device (write)
#define BLE_UUID_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // device -> app (notify)

static BLECharacteristic *bleTx = nullptr;
static char   bleRxBuf[200];
static size_t bleRxN = 0;

class GbServerCb : public BLEServerCallbacks {
  void onConnect(BLEServer *) override    { g_bleConnected = true; g_bleJustConnected = true; }
  void onDisconnect(BLEServer *) override { g_bleConnected = false; BLEDevice::startAdvertising(); }
};
class GbRxCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    auto v = c->getValue();                       // std::string (core 2.x) or String (core 3.x)
    for (size_t i = 0; i < v.length(); i++) {
      char ch = v[i];
      if (ch == '\n') { bleRxBuf[bleRxN] = 0; if (bleRxN) commHandleAppLine(bleRxBuf); bleRxN = 0; }
      else if (ch != '\r' && bleRxN < sizeof(bleRxBuf) - 1) bleRxBuf[bleRxN++] = ch;
    }
  }
};

static void bleBegin() {
  char name[24]; snprintf(name, sizeof(name), "%s-%s", BLE_NAME_PREFIX, g_devId + 3);
  BLEDevice::init(name);
  BLEDevice::setMTU(185);
  BLEServer *srv = BLEDevice::createServer();
  srv->setCallbacks(new GbServerCb());
  BLEService *svc = srv->createService(BLE_UUID_SVC);
  bleTx = svc->createCharacteristic(BLE_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
#if !defined(CONFIG_NIMBLE_ENABLED)
  bleTx->addDescriptor(new BLE2902());          // Bluedroid needs it; NimBLE adds it automatically
#endif
  BLECharacteristic *rx = svc->createCharacteristic(BLE_UUID_RX,
                          BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new GbRxCb());
  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_UUID_SVC);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

static bool bleSend(const char *data, size_t len) {
  if (!g_bleConnected || !bleTx) return false;
  for (size_t i = 0; i < len; i += BLE_CHUNK_BYTES) {
    size_t n = (len - i < BLE_CHUNK_BYTES) ? (len - i) : BLE_CHUNK_BYTES;
    bleTx->setValue((uint8_t *)(data + i), n);
    bleTx->notify();
    vTaskDelay(pdMS_TO_TICKS(8));
    if (!g_bleConnected) return false;
  }
  return true;
}
#endif

/** Tier 1. Returns true when the app acknowledged (or, if !waitAck, when sent). */
static bool bleDeliver(const OutMsg &m, bool waitAck) {
#if COMM_ENABLE_BLE
  if (!g_bleConnected) return false;
  g_ackId = 0;                                   // reset BEFORE sending: the ACK can be instant
  if (!bleSend(m.json, m.len)) return false;
  if (!waitAck) return true;
  uint32_t t0 = millis();
  while (millis() - t0 < COMM_ACK_BLE_MS) {
    if (g_ackId == m.id) return true;
    if (!g_bleConnected) return false;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
#else
  (void)m; (void)waitAck;
#endif
  return false;
}

// --------------------------------------------------------------- WiFi ----
__attribute__((unused)) static uint32_t lastWifiTry = 0;

__attribute__((unused)) static bool wifiEnsure(uint32_t waitMs) {
#if COMM_ENABLE_WIFI
  if (WiFi.status() == WL_CONNECTED) return true;
  if (waitMs == 0) return false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  lastWifiTry = millis();
  uint32_t t0 = millis();
  while (millis() - t0 < waitMs) {
    if (WiFi.status() == WL_CONNECTED) return true;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
#else
  (void)waitMs;
#endif
  return false;
}

/** Keeps WiFi associated for telemetry without ever blocking. */
static void wifiMaintain() {
#if COMM_ENABLE_WIFI
  if (WiFi.status() != WL_CONNECTED && (lastWifiTry == 0 || millis() - lastWifiTry > WIFI_RETRY_MS)) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    lastWifiTry = millis();
  }
#endif
}

/** Tier 2. Returns 0 = failed, 1 = server got it but no app confirmed, 2 = app confirmed. */
static int wifiPost(const OutMsg &m, bool wait) {
#if COMM_ENABLE_WIFI
  if (!wifiEnsure(wait ? COMM_ACK_WIFI_MS : 0)) return 0;
  String url = SERVER_URL;
  if (wait) url += "?wait=1";
  WiFiClient  plain;
  WiFiClient *cli = &plain;
#if COMM_ENABLE_TLS
  WiFiClientSecure secure;
  if (url.startsWith("https")) {
    if (strlen(SERVER_CA_PEM)) secure.setCACert(SERVER_CA_PEM);
    else if (TLS_INSECURE)     secure.setInsecure();
    else { commLog("WiFi: https needs SERVER_CA_PEM or TLS_INSECURE=1"); return 0; }
    cli = &secure;
  }
#else
  if (url.startsWith("https")) { commLog("WiFi: https needs COMM_ENABLE_TLS=1 (and the Huge APP partition)"); return 0; }
#endif
  HTTPClient http;
  if (!http.begin(*cli, url)) return 0;
  http.setTimeout(COMM_ACK_WIFI_MS + 3000);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-Device-Token", DEVICE_TOKEN);
  int code = http.POST((uint8_t *)m.json, m.len);
  int result = 0;
  if (code >= 200 && code < 300) {
    String body = http.getString();
    result = (body.indexOf("\"app\":true") >= 0) ? 2 : 1;
  } else {
    commLog("WiFi: HTTP %d", code);
  }
  http.end();
  return result;
#else
  (void)m; (void)wait;
  return 0;
#endif
}

// ---------------------------------------------------------------- SIM ----
#if COMM_ENABLE_SIM
static HardwareSerial SimSerial(1);

static void simFlush() { while (SimSerial.available()) SimSerial.read(); }

static bool simRead(char *buf, size_t n, const char *expect, uint32_t ms) {
  size_t i = 0; uint32_t t0 = millis(); buf[0] = 0;
  while (millis() - t0 < ms) {
    while (SimSerial.available() && i < n - 1) { buf[i++] = (char)SimSerial.read(); buf[i] = 0; }
    if (expect && strstr(buf, expect)) return true;
    if (strstr(buf, "ERROR")) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return expect == nullptr;
}
static bool simCmd(const char *cmd, const char *expect = "OK", uint32_t ms = 2000) {
  char b[160];
  simFlush(); SimSerial.print(cmd); SimSerial.print("\r");
  return simRead(b, sizeof(b), expect, ms);
}
static bool smsOne(const char *number, const char *text) {
  char b[200];
  simFlush();
  SimSerial.printf("AT+CMGS=\"%s\"\r", number);
  if (!simRead(b, sizeof(b), ">", 5000)) return false;
  SimSerial.print(text);
  SimSerial.write((uint8_t)0x1A);                  // Ctrl+Z ends the message
  return simRead(b, sizeof(b), "+CMGS:", 30000);
}
#endif

/** Tier 3. Returns true if at least one SMS was accepted by the network. */
static bool smsSend(const char *text) {
#if COMM_ENABLE_SIM
  bool alive = false;
  for (int i = 0; i < 3 && !alive; i++) alive = simCmd("AT");
  if (!alive) { commLog("SIM: modem does not answer"); return false; }
  simCmd("ATE0");
  bool reg = false; uint32_t t0 = millis();
  while (!reg && millis() - t0 < 20000) {          // wait for network registration
    char b[120]; simFlush(); SimSerial.print("AT+CREG?\r");
    simRead(b, sizeof(b), "OK", 1000);
    reg = strstr(b, ",1") || strstr(b, ",5");
    if (!reg) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  if (!reg) { commLog("SIM: not registered on the network"); return false; }
  simCmd("AT+CMGF=1");
  simCmd("AT+CSCS=\"GSM\"");
  bool any = false;
  if (SIM_PHONE_1[0]) { bool ok = smsOne(SIM_PHONE_1, text); commLog("SIM: SMS to %s %s", SIM_PHONE_1, ok ? "sent" : "FAILED"); any |= ok; }
  if (SIM_PHONE_2[0]) { bool ok = smsOne(SIM_PHONE_2, text); commLog("SIM: SMS to %s %s", SIM_PHONE_2, ok ? "sent" : "FAILED"); any |= ok; }
  return any;
#else
  (void)text;
  return false;
#endif
}

// ------------------------------------------------------ failover logic ----

/**
 * Delivers one message. TEL: best effort. INFO/CRITICAL: BLE (app ACK) ->
 * WiFi (app ACK through the server) -> SMS (CRITICAL only). Returns true when an
 * app confirmed (or when a TEL message was sent).
 */
static bool commDeliver(OutMsg &m) {
  m.attempts++;
  if (m.prio == PRIO_TEL) {
    if (bleDeliver(m, false)) return true;
    return wifiPost(m, false) > 0;
  }
  const char *kind = m.prio == PRIO_CRITICAL ? "CRITICAL" : "info";
  commLog("msg #%u (%s, try %u): BLE -> WiFi -> SIM", (unsigned)m.id, kind, (unsigned)m.attempts);

  if (bleDeliver(m, true)) { commLog("msg #%u delivered over BLE, app ACK received", (unsigned)m.id); return true; }
  commLog("BLE: %s", g_bleConnected ? "no ACK from the app" : "no app connected");

  int r = wifiPost(m, true);
  if (r == 2) { commLog("msg #%u delivered over WiFi, app ACK received", (unsigned)m.id); return true; }
  commLog(r == 1 ? "WiFi: server reached but no app acknowledged" : "WiFi: unreachable");

  if (m.prio == PRIO_CRITICAL) {
    if (m.smsSent) commLog("SIM: SMS already sent for this alert");
    else if (smsSend(m.sms)) { m.smsSent = true; commLog("msg #%u alerted by SMS (no app ACK possible)", (unsigned)m.id); }
    else commLog("SIM: SMS failed");
  }
  return false;
}

static void commTask(void *) {
  static OutMsg cur, pend;
  bool     havePend  = false;
  uint32_t nextRetry = 0;

#if COMM_ENABLE_BLE
  bleBegin();
#endif
#if COMM_ENABLE_SIM
  SimSerial.begin(SIM_BAUD, SERIAL_8N1, PIN_SIM_RX, PIN_SIM_TX);
#endif
#if COMM_ENABLE_WIFI
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
#endif

  for (;;) {
    wifiMaintain();
    if (xQueueReceive(commQ, &cur, pdMS_TO_TICKS(200)) == pdTRUE) {
      bool done = commDeliver(cur);
      if (cur.prio == PRIO_CRITICAL) {
        if (!done) { pend = cur; havePend = true; nextRetry = millis() + COMM_RETRY_MS; }
        else havePend = false;
      }
    } else if (havePend && (int32_t)(millis() - nextRetry) >= 0) {
      if (pend.attempts >= COMM_MAX_RETRY) {
        commLog("msg #%u: giving up after %u tries", (unsigned)pend.id, (unsigned)pend.attempts);
        havePend = false;
      } else if (commDeliver(pend)) {
        havePend = false;
      } else {
        nextRetry = millis() + COMM_RETRY_MS;
      }
    }
  }
}

/** Creates the queue and starts the communication task. Call once from setup(). */
static void commBegin() {
  uint64_t mac = ESP.getEfuseMac();
  snprintf(g_devId, sizeof(g_devId), "GB-%06X", (unsigned)(mac & 0xFFFFFF));
  commQ = xQueueCreate(6, sizeof(OutMsg));
#if CONFIG_FREERTOS_UNICORE
  xTaskCreate(commTask, "comm", 12288, nullptr, 1, nullptr);
#else
  xTaskCreatePinnedToCore(commTask, "comm", 12288, nullptr, 1, nullptr, 0);
#endif
}

// =====================================================================
//  FAULT DIAGNOSIS (rule-based signature classifier) & COMPONENT LIFE
// =====================================================================

struct FaultInfo {
  const char *code, *title, *part, *action;
  int8_t comp;                       // component whose life is affected (-1 = none)
};

static const FaultInfo FAULTS[FC_COUNT] = {
  { "BEARING",   "Impulsive high-frequency vibration",
    "Main motor bearing (drive-end / pillow block)",
    "Inspect lubrication and bearing condition within 48 h; plan replacement.", COMP_BEARING },
  { "BELT_1X",   "Vibration locked to shaft speed (1X)",
    "Drive belt tension / shaft misalignment / rotor imbalance",
    "Check belt tension, shaft alignment and coupling within 72 h.", COMP_BELT },
  { "GEAR_LUBE", "Sharp temperature rise with stable vibration",
    "Gearbox: gear-mesh wear or lubricant loss",
    "Check oil level and quality NOW; do not restart until verified.", COMP_GEAR },
  { "OVERLOAD",  "Motor current surge",
    "Mechanical overload or jam on the driven load",
    "Clear the jam / inspect the driven load before restart.", -1 },
  { "BELT_SLIP", "Shaft speed deviation",
    "Drive belt slip or coupling loss",
    "Inspect belt tension and wear.", COMP_BELT },
  { "UNKNOWN",   "Unclassified anomaly",
    "Undetermined - manual inspection required",
    "Manual inspection required.", -1 },
};

static const char *CH_ID[CH_COUNT] = { "vib", "tmp", "cur", "rpm", "aco" };

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float clamp01(float v) { return clampf(v, 0.0f, 1.0f); }

static const char *stateTrim() {
  switch (state) {
    case UnitState::WARMUP: return "WARMUP";
    case UnitState::RUN:    return "RUN";
    default:                return "TRIPPED";
  }
}

// ---- signature history & baselines ------------------------------------
constexpr uint8_t DIAG_HIST = 10;            // 1 s of peak-hold history
static DiagFeat  dHist[DIAG_HIST];
static uint8_t   dHistN = 0, dHistHead = 0;
static DiagFeat  dBase  = {0, 0, 0};
static uint16_t  dBaseN = 0;
static float     tHist[4];
static uint8_t   tHistN = 0;

static void diagPush(const DiagFeat &f) {
  dHist[dHistHead] = f;
  dHistHead = (dHistHead + 1) % DIAG_HIST;
  if (dHistN < DIAG_HIST) dHistN++;
}

/** Slow EMA baseline of the healthy signature; only fed while nothing is flagged. */
static void diagLearn(const DiagFeat &f) {
  if (dBaseN == 0) { dBase = f; dBaseN = 1; return; }
  if (dBaseN >= DIAG_BASE_FRAMES && REF_ADAPT_ALPHA <= 0.0f) return;           // frozen after learning
  float a = (dBaseN < DIAG_BASE_FRAMES) ? 1.0f / (dBaseN + 1) : REF_ADAPT_ALPHA;
  dBase.crest += a * (f.crest - dBase.crest);
  dBase.kurt  += a * (f.kurt  - dBase.kurt);
  dBase.oneX  += a * (f.oneX  - dBase.oneX);
  if (dBaseN < 65535) dBaseN++;
}

static bool anyStrikes() {
  for (int i = 0; i < CH_COUNT; i++) if (det[i].strikes) return true;
  return false;
}

static void tempPush(float t) {
  if (tHistN < 4) { tHist[tHistN++] = t; return; }
  for (int i = 0; i < 3; i++) tHist[i] = tHist[i + 1];
  tHist[3] = t;
}
static float tempSlope() {                    // deg C per second (1 Hz readings)
  return tHistN < 2 ? 0.0f : (tHist[tHistN - 1] - tHist[0]) / (tHistN - 1);
}

/** z-score of the latest value against the learned healthy window. */
static float refZ(Ch ch, float v) {
  float sd = refSd[ch];
  if (sd < CFG[ch].sigmaFloor) sd = CFG[ch].sigmaFloor;
  return (v - refMean[ch]) / sd;
}
static float zOf(Ch ch) {
  if (isnan(lastVal[ch])) return 0.0f;
  if (refReady[ch]) {                                  // frozen reference: immune to slow adaptation
    float zr = refZ(ch, lastVal[ch]);
    float zs = vSmReady[ch] ? refZ(ch, vSm[ch]) : zr;
    return fabsf(zr) > fabsf(zs) ? zr : zs;
  }
  float sd = det[ch].sd();
  if (sd < CFG[ch].sigmaFloor) sd = CFG[ch].sigmaFloor;
  return (lastVal[ch] - det[ch].mean()) / sd;
}

/**
 * Rule-based classifier. Every class gets a 0..1 evidence score from the
 * signature; the best one wins if it reaches DIAG_MIN_SCORE. Confidence is
 * best/sum(all) scaled by evidence strength: a heuristic, not a calibrated
 * probability.
 */
static Ev liveEv() {
  Ev e;
  e.zv = zOf(CH_VIB); e.zt = zOf(CH_TEMP); e.zc = zOf(CH_CUR); e.zr = zOf(CH_RPM); e.za = zOf(CH_ACO);
  e.slope = tempSlope();
  return e;
}

static Diagnosis classify(const Ev &ev) {
  Diagnosis d;
  memset(&d, 0, sizeof(d));
  d.cls = FC_UNKNOWN;

  DiagFeat pk = {0, 0, 0};                    // peak-hold over the last second
  for (uint8_t i = 0; i < dHistN; i++) {
    pk.crest = fmaxf(pk.crest, dHist[i].crest);
    pk.kurt  = fmaxf(pk.kurt,  dHist[i].kurt);
    pk.oneX  = fmaxf(pk.oneX,  dHist[i].oneX);
  }
  bool  baseOk = dBaseN >= DIAG_BASE_FRAMES;
  float crX = (baseOk && dBase.crest > 0.1f) ? pk.crest / dBase.crest : 1.0f;
  float kuX = (baseOk && dBase.kurt  > 0.1f) ? pk.kurt  / dBase.kurt  : 1.0f;
  float oxR = baseOk ? pk.oneX / fmaxf(dBase.oneX, 0.005f) : 1.0f;

  d.imp   = fmaxf(clamp01((crX - 1.3f) / 1.5f), clamp01((kuX - 1.5f) / 2.5f));
  d.x1    = clamp01((oxR - 2.0f) / 6.0f);
  d.slope = ev.slope;
  d.zCur  = ev.zc;
  d.zRpm  = ev.zr;

  float sharp   = clamp01((d.slope - 0.5f) / 0.7f);
  float gradual = (d.slope > 0.05f && d.slope <= 0.5f) ? clamp01(d.slope / 0.15f) : 0.0f;
  float tz      = clamp01((ev.zt - 4.0f) / 10.0f);
  float vibLvl  = clamp01((ev.zv - 4.0f) / 10.0f);
  float aco     = clamp01((ev.za - 4.0f) / 10.0f);
  float vibAbn  = fmaxf(d.imp, fmaxf(d.x1, vibLvl));

  // A fault onset inside the 0.5 s window looks impulsive too, so a dominant 1X component discounts BEARING.
  d.score[FC_BEARING]   = 0.65f * d.imp * (1.0f - 0.7f * d.x1) + 0.25f * gradual + 0.10f * aco;
  d.score[FC_BELT_1X]   = d.x1 * (1.0f - 0.5f * d.imp);
  d.score[FC_GEAR_LUBE] = (0.7f * sharp + 0.3f * tz) * (1.0f - vibAbn);
  d.score[FC_OVERLOAD]  = clamp01((d.zCur - 4.0f) / 10.0f);
  d.score[FC_BELT_SLIP] = clamp01((fabsf(d.zRpm) - 4.0f) / 10.0f);

  float best = 0, sum = 0; uint8_t bi = FC_UNKNOWN;
  for (uint8_t i = 0; i < FC_UNKNOWN; i++) {
    sum += d.score[i];
    if (d.score[i] > best) { best = d.score[i]; bi = i; }
  }
  if (best >= DIAG_MIN_SCORE) {
    d.cls  = bi;
    d.conf = (uint8_t)fminf(99.0f, 100.0f * (best / sum) * clamp01(best / 0.6f) + 0.5f);
  }
  return d;
}
static Diagnosis classify() { return classify(liveEv()); }
static Ev tripEv;                                      // evidence at trip time (set in emergencyTrip)

// ---- component life ------------------------------------------------------

struct LifeRec {
  uint32_t magic;
  uint32_t installs[COMP_COUNT];
  double   opSec[COMP_COUNT];      // real operating seconds since install
  double   wSec[COMP_COUNT];       // stress-weighted seconds since install
};
constexpr uint32_t LIFE_MAGIC = 0x4C494645u;
static LifeRec life;
static bool    rulWarned[COMP_COUNT];

static void lifeInit() {
  memset(&life, 0, sizeof(life));
  life.magic = LIFE_MAGIC;
  for (int i = 0; i < COMP_COUNT; i++) life.installs[i] = 1;
}
static void loadLife() {
  if (prefs.getBytesLength("life") == sizeof(life)) {
    prefs.getBytes("life", &life, sizeof(life));
    if (life.magic != LIFE_MAGIC) lifeInit();
  } else {
    lifeInit();
  }
}
static void saveLife() { prefs.putBytes("life", &life, sizeof(life)); }

static float rulPct(int i)   { return clampf(100.0f - (float)(life.wSec[i] / 3600.0) / COMPS[i].lifeHours * 100.0f, 0.0f, 100.0f); }
static float rulHours(int i) { return clampf(COMPS[i].lifeHours - (float)(life.wSec[i] / 3600.0), 0.0f, COMPS[i].lifeHours); }

static float ratioOf(Ch ch, float minDen) {
  if (isnan(lastVal[ch])) return 1.0f;
  float m = refReady[ch] ? refMean[ch] : det[ch].mean();
  float v = vSmReady[ch] ? vSm[ch] : lastVal[ch];
  return m > minDen ? v / m : 1.0f;
}

/** Ageing multiplier (1.0 = nominal). Rises when vibration / current / heat exceed the healthy baseline. */
static float stressFactor(int comp) {
  if (state != UnitState::RUN) return 1.0f;
  const CompCfg &c = COMPS[comp];
  float xv = clampf((ratioOf(CH_VIB, 0.01f) - 1.25f) / 1.0f, 0.0f, 3.0f);
  float xc = clampf((ratioOf(CH_CUR, 0.10f) - 1.15f) / 0.5f, 0.0f, 3.0f);
  float dT = isnan(lastVal[CH_TEMP]) ? 0.0f : lastVal[CH_TEMP] - (refReady[CH_TEMP] ? refMean[CH_TEMP] : det[CH_TEMP].mean());
  float xt = clampf(dT / 5.0f, 0.0f, 3.0f);
  return 1.0f + c.kVib * xv + c.kCur * xc + c.kTemp * xt;
}

/** Once per second: count operating time only while the shaft is turning. */
static void serviceLife() {
  uint32_t now = millis();
  if (now - lastLifeMs < 1000) return;
  lastLifeMs = now;
  if (state == UnitState::TRIPPED) return;
  float rpm = lastVal[CH_RPM];
  if (isnan(rpm) || rpm < RUN_RPM_MIN) return;
  for (int i = 0; i < COMP_COUNT; i++) {
    life.opSec[i] += LIFE_TIME_SCALE;
    life.wSec[i]  += LIFE_TIME_SCALE * stressFactor(i);
  }
  if (++lifeSaveTick >= LIFE_SAVE_PERIOD_S) { lifeSaveTick = 0; saveLife(); }
  for (int i = 0; i < COMP_COUNT; i++) {              // one-time low-life warning per part
    if (!rulWarned[i] && rulPct(i) < RUL_WARN_PCT) {
      rulWarned[i] = true;
      char d[200];
      snprintf(d, sizeof(d), "{\"comp\":\"%s\",\"name\":\"%s\",\"rul_pct\":%.1f,\"rul_h\":%.0f}",
               COMPS[i].id, COMPS[i].name, rulPct(i), rulHours(i));
      commPost("rul_low", PRIO_INFO, d);
      logTs(); Serial.printf("WARNING: %s below %.0f%% remaining life\n", COMPS[i].name, RUL_WARN_PCT);
    }
  }
}

// ---- JSON helper (no external library) -------------------------------------


static void buildLifeJson(JBuf &j) {
  j.add("{\"comps\":[");
  for (int i = 0; i < COMP_COUNT; i++) {
    j.add("%s{\"id\":\"%s\",\"name\":\"%s\",\"op_h\":%.1f,\"rul_pct\":%.1f,\"rul_h\":%.0f,\"life_h\":%.0f,\"installs\":%u,\"stress\":%.2f}",
          i ? "," : "", COMPS[i].id, COMPS[i].name, life.opSec[i] / 3600.0, rulPct(i), rulHours(i),
          COMPS[i].lifeHours, (unsigned)life.installs[i], stressFactor(i));
  }
  j.add("]}");
}
static void printLifeJson() { JBuf j; buildLifeJson(j); Serial.print("@LIFE "); Serial.println(j.b); }

static void buildDiagJson(JBuf &j) {
  j.add("{\"t\":%.1f,\"state\":\"%s\",\"trip\":%u,", millis() / 1000.0f, stateTrim(), (unsigned)tripCount);
  const char *kind = tripKind == TripKind::MECHANICAL ? "mechanical" : (tripKind == TripKind::SENSOR ? "sensor" : "system");
  if (state == UnitState::TRIPPED && !(tripKind == TripKind::MECHANICAL && haveDiag)) {
    j.add("\"kind\":\"%s\",\"reason\":\"%s\"}", kind, tripReason);
  } else {
    Diagnosis live; const Diagnosis *pd = &lastDiag;
    if (state != UnitState::TRIPPED) { live = classify(); pd = &live; }
    const FaultInfo &fi = FAULTS[pd->cls];
    j.add("\"kind\":\"mechanical\",\"reason\":\"%s\",\"code\":\"%s\",\"title\":\"%s\",\"part\":\"%s\",\"conf\":%u,",
          state == UnitState::TRIPPED ? tripReason : "", fi.code, fi.title, fi.part, (unsigned)pd->conf);
    j.add("\"scores\":{");
    for (int i = 0; i < FC_UNKNOWN; i++) j.add("%s\"%s\":%.2f", i ? "," : "", FAULTS[i].code, pd->score[i]);
    j.add("},\"comp\":");
    if (fi.comp >= 0) j.add("\"%s\"", COMPS[fi.comp].id); else j.add("null");
    j.add(",\"action\":\"%s\"}", fi.action);
  }
}
static void printDiagJson() { JBuf j; buildDiagJson(j); Serial.print("@DIAG "); Serial.println(j.b); }

static void buildTelJson(JBuf &j) {
  j.add("{\"t\":%.1f,\"state\":\"%s\",\"relay\":\"%s\",\"ch\":{", millis() / 1000.0f, stateTrim(),
        state == UnitState::TRIPPED ? "TRIPPED" : "RUN");
  for (int i = 0; i < CH_COUNT; i++) { j.add("%s\"%s\":", i ? "," : "", CH_ID[i]); j.num(lastVal[i], CFG[i].decimals); }
  j.add("},\"z\":{");
  for (int i = 0; i < CH_COUNT; i++) { j.add("%s\"%s\":", i ? "," : "", CH_ID[i]); j.num(det[i].ready(CFG[i]) ? det[i].z : NAN, 1); }
  j.add("},\"zr\":{");
  for (int i = 0; i < CH_COUNT; i++) {
    j.add("%s\"%s\":", i ? "," : "", CH_ID[i]);
    j.num((refReady[i] && vSmReady[i]) ? refZ((Ch)i, vSm[i]) : NAN, 1);
  }
  j.add("},\"health\":%.0f,\"watch\":%d,\"sim\":%d,\"eta\":", g_health, g_watch ? 1 : 0,
        (int)(simulated[0] + simulated[1] + simulated[2] + simulated[3] + simulated[4]));
  j.num(g_eta, 0);
  j.add("}");
}
static void printTelemetry() { JBuf j; buildTelJson(j); Serial.print("@TEL "); Serial.println(j.b); }

// ---- human-readable report -------------------------------------------------

static void printLifeTable(int suspect) {
  Serial.println("--- Component life (operating hours, stress-weighted RUL) ---");
  for (int i = 0; i < COMP_COUNT; i++) {
    Serial.printf(" %c %-30s op %7.1f h | RUL %5.1f%% (~%.0f h left) | installs %u%s\n",
                  i == suspect ? '*' : ' ', COMPS[i].name, life.opSec[i] / 3600.0, rulPct(i), rulHours(i),
                  (unsigned)life.installs[i], i == suspect ? "  <-- SUSPECT" : "");
  }
}

static void printReport() {
  Serial.println("==================================================");
  Serial.println(" FAULT DIAGNOSIS & REMAINING-LIFE REPORT (Green Box)");
  Serial.println("==================================================");
  Serial.printf(" Status       : %s\n", stateTrim());
  if (state == UnitState::TRIPPED) Serial.printf(" Trip cause   : %s\n", tripReason);

  Diagnosis live; const Diagnosis *pd = nullptr;
  if (state == UnitState::TRIPPED) { if (tripKind == TripKind::MECHANICAL && haveDiag) pd = &lastDiag; }
  else { live = classify(); pd = &live; }

  int suspect = -1;
  if (!pd && tripKind == TripKind::MECHANICAL) {
    Serial.println(" Diagnosis    : analysing failure signature...");
  } else if (!pd) {
    Serial.println(" Diagnosis    : non-mechanical trip (sensor / system) - no part classification");
  } else if (pd->cls == FC_UNKNOWN) {
    Serial.println(" Diagnosis    : no dominant fault signature (unclassified or no anomaly)");
  } else {
    const FaultInfo &fi = FAULTS[pd->cls];
    suspect = fi.comp;
    Serial.printf(" Diagnosis    : %s\n", fi.title);
    Serial.printf(" Suspect part : %s\n", fi.part);
    Serial.printf(" Confidence   : %u%% (rule-based signature score, not a calibrated probability)\n", (unsigned)pd->conf);
    Serial.printf(" Evidence     : impulsive %.2f | 1X %.2f | heating %+.2f C/s | current z %+.1f | speed z %+.1f\n",
                  pd->imp, pd->x1, pd->slope, pd->zCur, pd->zRpm);
  }
  printLifeTable(suspect);
  if (pd && pd->cls != FC_UNKNOWN) {
    Serial.printf(" Action       : %s\n", FAULTS[pd->cls].action);
    if (suspect >= 0 && rulPct(suspect) < 20.0f)
      Serial.println(" NOTE         : suspect part is near end of its expected life - replace rather than repair.");
  }
  Serial.println("==================================================");
}

static void installNewPart(int comp) {
  life.opSec[comp] = 0;
  life.wSec[comp]  = 0;
  life.installs[comp]++;
  rulWarned[comp] = false;
  saveLife();
  logTs(); Serial.printf("New part registered: %s. Life counter reset (installs: %u).\n",
                         COMPS[comp].name, (unsigned)life.installs[comp]);
  printLifeJson();
}


// ---- main-thread glue: build payloads and hand them to the comm task ----

static void commPostHello() {
  JBuf j;
  j.add("{\"fw\":\"%s\",\"channels\":[", FW_VERSION);
  for (int i = 0; i < CH_COUNT; i++) {
    const ChannelCfg &c = CFG[i];
    j.add("%s{\"id\":\"%s\",\"tag\":\"%s\",\"name\":\"%s\",\"unit\":\"%s\",\"dec\":%u,\"hh\":", i ? "," : "",
          CH_ID[i], c.tag, c.name, c.unit, (unsigned)c.decimals);
    j.num(c.hardHigh, c.decimals);
    j.add(",\"hl\":"); j.num(c.hardLow, c.decimals);
    j.add(",\"sim\":%d}", simulated[i] ? 1 : 0);
  }
  j.add("]}");
  commPost("hello", PRIO_TEL, j.b);
}
static void commPostLife() { JBuf j; buildLifeJson(j); commPost("life", PRIO_TEL, j.b); }
static void commPostTel()  { JBuf j; buildTelJson(j);  commPost("tel",  PRIO_TEL, j.b); }
static void commPostSnapshot() { JBuf j; buildDiagJson(j); commPost("snap", PRIO_TEL, j.b); }

/** Critical alert: full BLE -> WiFi -> SIM chain. Also builds the SMS text (ASCII, <= 160 chars). */
static void commPostTrip(const char *type = "diag") {
  JBuf j; buildDiagJson(j);
  char sms[161];
  if (tripKind == TripKind::MECHANICAL && haveDiag && lastDiag.cls != FC_UNKNOWN) {
    const FaultInfo &fi = FAULTS[lastDiag.cls];
    char rs[24] = "";
    if (fi.comp >= 0) snprintf(rs, sizeof(rs), " RUL %.0f%%", rulPct(fi.comp));
    snprintf(sms, sizeof(sms), "GreenBox %s TRIP#%u: %s %u%%%s. %.80s", g_devId, (unsigned)tripCount,
             fi.code, (unsigned)lastDiag.conf, rs, tripReason);
  } else {
    snprintf(sms, sizeof(sms), "GreenBox %s TRIP#%u: %.110s", g_devId, (unsigned)tripCount, tripReason);
  }
  commPost(type, PRIO_CRITICAL, j.b, sms);
}

/** Periodic data + requests coming from the app. Call every loop(). */
static void commService() {
  if (!commQ) return;
  static uint32_t lastTel = 0, lastLifeP = 0;
  static bool first = true;
  uint32_t now = millis();

  if (g_cmdInstall >= 0) { int c = g_cmdInstall; g_cmdInstall = -1; installNewPart(c); commPostLife(); }
  if (first || g_bleJustConnected || g_cmdGetHello) {
    first = false; g_bleJustConnected = false; g_cmdGetHello = false;
    commPostHello(); commPostLife(); commPostSnapshot();
  }
  if (g_cmdGetLife) { g_cmdGetLife = false; commPostLife(); }
  if (g_cmdGetDiag) { g_cmdGetDiag = false; commPostSnapshot(); }
  if (now - lastTel   >= COMM_TEL_PERIOD_MS)  { lastTel = now;   commPostTel(); }
  if (now - lastLifeP >= COMM_LIFE_PERIOD_MS) { lastLifeP = now; commPostLife(); }
}

static void commStatus() {
  logTs(); Serial.printf("Device %s | BLE %s | WiFi %s | SIM %s\n", g_devId,
#if COMM_ENABLE_BLE
        g_bleConnected ? "app connected" : "advertising",
#else
        "disabled",
#endif
#if COMM_ENABLE_WIFI
        WiFi.status() == WL_CONNECTED ? "connected" : "down",
#else
        "disabled",
#endif
#if COMM_ENABLE_SIM
        SIM_PHONE_1[0] ? "armed (SMS)" : "no phone number"
#else
        "disabled"
#endif
  );
}

static void commTest() {
  char d[96], sms[161];
  snprintf(d, sizeof(d), "{\"state\":\"%s\",\"kind\":\"test\",\"reason\":\"communication test\",\"trip\":0}", stateTrim());
  snprintf(sms, sizeof(sms), "GreenBox %s: communication test", g_devId);
  logTs(); Serial.println("Sending test alert through BLE -> WiFi -> SIM ...");
  commPost("test", PRIO_CRITICAL, d, sms);
}


// =====================================================================
//  HEALTH INDEX, EARLY WARNING, DEMO STORY, SELF-TEST
// =====================================================================

static void rearm();                 // defined below (TRIP / RE-ARM section)
static void emergencyTrip(TripKind kind, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

static void resetHealth() {
  g_health = 100.0f; g_watch = false; g_eta = NAN;
  hHistN = 0; watchCnt = 0; clearCnt = 0;
}

static void commPostWatch(bool active, const Diagnosis &dg) {
  JBuf j;
  const FaultInfo &fi = FAULTS[dg.cls];
  j.add("{\"active\":%d,\"health\":%.0f,\"code\":\"%s\",\"conf\":%u,\"comp\":", active ? 1 : 0, g_health,
        fi.code, (unsigned)dg.conf);
  if (fi.comp >= 0) j.add("\"%s\",\"rul_pct\":%.1f", COMPS[fi.comp].id, rulPct(fi.comp));
  else              j.add("null,\"rul_pct\":null");
  j.add(",\"eta_s\":"); j.num(g_eta, 0);
  j.add(",\"action\":\"%s\"}", fi.action);
  commPost("watch", PRIO_INFO, j.b);
}

/** Once per second: health index (0-100), trend, and the early-warning state machine. */
static void serviceHealth() {
  uint32_t now = millis();
  if (now - lastHealthMs < 1000) return;
  lastHealthMs = now;
  if (state == UnitState::TRIPPED) { g_health = 0; g_watch = false; g_eta = NAN; return; }
  if (state != UnitState::RUN)     { g_health = 100; return; }

  float worst = 0;
  for (int i = 0; i < CH_COUNT; i++) {
    if (!refReady[i] || !vSmReady[i]) continue;
    float z = refZ((Ch)i, vSm[i]);
    if (!CFG[i].twoSided && z < 0) z = 0;
    z = fabsf(z);
    worst = fmaxf(worst, clamp01((z - HEALTH_Z0) / (CFG[i].driftZ - HEALTH_Z0)));
    if (z >= CFG[i].driftZ) {                                  // sustained drift away from the healthy reference
      if (++driftCnt[i] >= DRIFT_HOLD_S) {
        const ChannelCfg &c = CFG[i];
        emergencyTrip(TripKind::MECHANICAL, "%s DRIFT (%.*f%s vs healthy reference %.*f%s, z=%+.1f)", c.name,
                      (int)c.decimals, vSm[i], c.unit, (int)c.decimals, refMean[i], c.unit, refZ((Ch)i, vSm[i]));
        return;
      }
    } else driftCnt[i] = 0;
  }
  Diagnosis dg = classify();                                   // signature channels react before RMS does
  worst = fmaxf(worst, 0.6f * clamp01((fmaxf(dg.imp, dg.x1) - 0.15f) / 0.85f));
  float raw = 100.0f * (1.0f - worst);
  g_health += 0.4f * (raw - g_health);

  if (hHistN < 8) hHist[hHistN++] = g_health;                  // linear trend over the last 8 s
  else { for (int i = 0; i < 7; i++) hHist[i] = hHist[i + 1]; hHist[7] = g_health; }
  g_eta = NAN;
  if (hHistN >= 5) {
    float slope = (hHist[hHistN - 1] - hHist[0]) / (hHistN - 1);       // health points per second
    if (slope < -0.4f) g_eta = g_health / (-slope);
  }

  if (!g_watch) {
    watchCnt = (g_health < HEALTH_WATCH) ? watchCnt + 1 : 0;
    if (watchCnt >= 2) {
      g_watch = true; clearCnt = 0;
      logTs(); Serial.printf("EARLY WARNING: health %.0f%% - %s (%u%%)%s\n", g_health, FAULTS[dg.cls].title, (unsigned)dg.conf,
                             isnan(g_eta) ? "" : " - machine still running");
      if (!isnan(g_eta)) { logTs(); Serial.printf("Estimated time to emergency stop (linear trend): ~%.0f s\n", g_eta); }
      commPostWatch(true, dg);
    }
  } else {
    clearCnt = (g_health > HEALTH_CLEAR) ? clearCnt + 1 : 0;
    if (clearCnt >= 4) {
      g_watch = false; watchCnt = 0;
      logTs(); Serial.println("Early warning cleared: health recovered.");
      commPostWatch(false, dg);
    }
  }
}

static void startStory() {
  if (!DEMO_MODE)                { logTs(); Serial.println("Story is disabled (DEMO_MODE=false)."); return; }
  if (!simulated[CH_VIB])        { logTs(); Serial.println("Story needs the simulated vibration channel."); return; }
  if (state != UnitState::RUN)   { logTs(); Serial.println("Story: wait until detection is ARMED (state RUN)."); return; }
  if (storyActive)               { logTs(); Serial.println("Story already running."); return; }
  storyActive = true; storyStartMs = millis(); g_wear = 0.0f;
  logTs(); Serial.printf("STORY: bearing wear starts (simulated), full wear in %.0f s.\n", DEMO_WEAR_RAMP_S);
}

static void serviceStory() {
  uint32_t now = millis();
  if (storyActive && state != UnitState::TRIPPED)
    g_wear = clamp01((now - storyStartMs) / 1000.0f / DEMO_WEAR_RAMP_S);
  if (DEMO_MODE && DEMO_AUTOPLAY) {
    if (state == UnitState::RUN && !storyActive && (now - runSinceMs) > DEMO_AUTOPLAY_DELAY_MS) startStory();
    if (state == UnitState::TRIPPED && (now - tripAtMs) > DEMO_REARM_DELAY_MS) rearm();
  }
}

/** Buzzer: 2 short beeps at early warning, long beep + intermittent after a trip. */
static void serviceBuzzer() {
  if (PIN_BUZZER < 0) return;
  uint32_t now = millis();
  bool on = false;
  if (state == UnitState::TRIPPED) {
    uint32_t t = now - tripAtMs;
    on = (t < 1500) || ((t % 3000) < 120);
  } else if (g_watch) {
    on = ((now % 4000) < 90) || (((now % 4000) > 220) && ((now % 4000) < 310));
  }
  digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
}

/** Pre-flight checklist: run it before a presentation ('x'). */
static void selfTest() {
  int fails = 0, warns = 0;
  auto row = [&](const char *name, const char *res, int level) {     // 0 pass, 1 warn, 2 fail
    Serial.printf("  [%s] %-22s %s\n", level == 0 ? "PASS" : level == 1 ? "WARN" : "FAIL", name, res);
    if (level == 1) warns++; else if (level == 2) fails++;
  };
  Serial.println("---------------- SELF-TEST ----------------");
  prefs.putUInt("selftest", millis());
  row("Flash storage (NVS)", prefs.getUInt("selftest", 0) ? "read/write ok" : "read-back failed", prefs.getUInt("selftest", 0) ? 0 : 2);
  for (int i = 0; i < CH_COUNT; i++) {
    char nm[24]; snprintf(nm, sizeof(nm), "Sensor %s", CFG[i].tag);
    if (!simulated[i]) row(nm, "real sensor", 0);
    else row(nm, DEMO_MODE ? "SIMULATED (demo)" : "SIMULATED - not allowed in production", DEMO_MODE ? 1 : 2);
  }
  row("Acquisition task", ((millis() - lastFrameMs) < ACQ_TIMEOUT_MS) ? "frames flowing" : "NO FRAMES", ((millis() - lastFrameMs) < ACQ_TIMEOUT_MS) ? 0 : 2);
  char b[64]; snprintf(b, sizeof(b), "%u dropped", (unsigned)g_droppedFrames);
  row("Frame queue", b, g_droppedFrames == 0 ? 0 : 1);
  if (DEMO_MODE && state != UnitState::TRIPPED && !bootFault) {     // never click the relay of a real, running machine
    relaySetTrip(); delay(180); relaySetRun();
    row("Relay", "clicked once (demo)", 0);
    if (PIN_BUZZER >= 0) { digitalWrite(PIN_BUZZER, HIGH); delay(120); digitalWrite(PIN_BUZZER, LOW); }
  } else row("Relay", "not exercised (production / tripped)", 1);
#if COMM_ENABLE_BLE
  row("Bluetooth (BLE)", g_bleConnected ? "app connected" : "advertising", 0);
#else
  row("Bluetooth (BLE)", "disabled in firmware", 1);
#endif
#if COMM_ENABLE_WIFI
  row("WiFi", WiFi.status() == WL_CONNECTED ? "connected" : "not connected (will connect on demand)", WiFi.status() == WL_CONNECTED ? 0 : 1);
#else
  row("WiFi", "disabled in firmware", 1);
#endif
#if COMM_ENABLE_SIM
  row("SIM / SMS", SIM_PHONE_1[0] ? "phone number configured" : "no phone number", SIM_PHONE_1[0] ? 0 : 1);
#else
  row("SIM / SMS", "disabled in firmware", 1);
#endif
  snprintf(b, sizeof(b), "%u bytes free", (unsigned)ESP.getFreeHeap());
  row("Memory", b, ESP.getFreeHeap() > 60000 ? 0 : 1);
  Serial.printf("RESULT: %s (%d warning%s, %d failure%s)\n", fails ? "FAIL" : "PASS", warns, warns == 1 ? "" : "s", fails, fails == 1 ? "" : "s");
  Serial.println("-------------------------------------------");
}

// =====================================================================
//  TRIP / RE-ARM
// =====================================================================

static void printSnapshot() {
  for (int i = 0; i < CH_COUNT; i++) {
    const ChannelCfg &c = CFG[i];
    if (isnan(lastVal[i])) Serial.printf("%s=--- ", c.tag);
    else Serial.printf("%s=%.*f%s ", c.tag, (int)c.decimals, lastVal[i], c.unit);
  }
  Serial.println();
}

static void emergencyTrip(TripKind kind, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void finishTripReport();

/** Latched emergency shutdown. Safe to call repeatedly; acts only once. */
static void emergencyTrip(TripKind kind, const char *fmt, ...) {
  if (state == UnitState::TRIPPED) return;
  relaySetTrip();                          // act first, log second
  g_machineRunning = false;
  g_watch   = false;
  state     = UnitState::TRIPPED;
  tripAtMs  = millis();
  tripKind  = kind;
  tripCount++;

  va_list ap;
  va_start(ap, fmt);
  vsnprintf(tripReason, sizeof(tripReason), fmt, ap);
  va_end(ap);

  Serial.println();
  logTs(); Serial.printf("!!!!!!!!!! EMERGENCY SHUTDOWN (trip #%u) !!!!!!!!!!\n", (unsigned)tripCount);
  logTs(); Serial.printf("Reason  : %s\n", tripReason);
  logTs(); Serial.print ("Snapshot: "); printSnapshot();
  logTs(); Serial.println("Relay latched. Send 'r' or hold BOOT 1 s to re-arm.");

  memset(&lastDiag, 0, sizeof(lastDiag));
  lastDiag.cls = FC_UNKNOWN;
  haveDiag = false;
  saveLife();
  persistTrip();                                   // the latch survives a power cut even before the diagnosis is ready
  if (kind == TripKind::MECHANICAL) {
    tripEv = liveEv();                             // freeze the evidence: the machine is about to coast down
    diagPending = true; diagDueMs = millis() + DIAG_SETTLE_MS;
    logTs(); Serial.println("Analysing the failure signature (0.7 s)...");
  } else {
    finishTripReport();
  }
}

/** Diagnosis + report + alert chain. Runs DIAG_SETTLE_MS after a mechanical trip (the relay is already open). */
static void finishTripReport() {
  diagPending = false;
  if (tripKind == TripKind::MECHANICAL) { lastDiag = classify(tripEv); haveDiag = true; persistTrip(); }
  Serial.println();
  printReport();
  printDiagJson();
  printLifeJson();
  Serial.println();
  commPostTrip();
}

/** Re-arm: clears baselines and returns to the learning phase. */
static void rearm() {
  if (bootFault) {
    logTs(); Serial.println("Re-arm REFUSED: required sensors are missing. Fix wiring and reboot.");
    return;
  }
  if (state == UnitState::TRIPPED && (millis() - tripAtMs) < REARM_LOCKOUT_MS) {
    logTs(); Serial.printf("Re-arm refused: wait %u ms after a trip.\n", (unsigned)REARM_LOCKOUT_MS);
    return;
  }
  for (int i = 0; i < CH_COUNT; i++) { det[i].reset(); lastVal[i] = NAN; }
  dHistN = 0; dHistHead = 0; dBaseN = 0; tHistN = 0; haveDiag = false;
  g_wear = 0.0f; storyActive = false; resetHealth(); diagPending = false;
  for (int i = 0; i < CH_COUNT; i++) { refReady[i] = false; vSmReady[i] = false; driftCnt[i] = 0; }
  g_faults        = 0;
  tempFails       = 0;
  tempRequested   = false;
  simTempC        = SIM_TEMP_BASE_C;       // simulate a warmed-up restart
  tripReason[0]   = '\0';
  settleFrames    = SETTLE_FRAMES;         // drop transient frames around the restart
  uint32_t now    = millis();
  lastFrameMs = lastTachMs = lastTempCycleMs = now;
  if (frameQ) xQueueReset(frameQ);
  g_machineRunning = true;
  relaySetRun();
  if (PERSIST_TRIP) prefs.putBool("latched", false);
  state = UnitState::WARMUP;
  logTs(); Serial.println("System re-armed. Learning new baseline...");
}

// =====================================================================
//  ADXL345 (raw I2C, no library)
// =====================================================================

static bool adxlWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool adxlReadBytes(uint8_t reg, uint8_t *dst, uint8_t len) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)ADXL_ADDR, (int)len) != len) return false;
  for (uint8_t i = 0; i < len; i++) dst[i] = Wire.read();
  return true;
}

static bool adxlInit() {
  uint8_t id = 0;
  if (!adxlReadBytes(ADXL_REG_DEVID, &id, 1) || id != 0xE5) return false;
  bool ok = true;
  ok &= adxlWrite(ADXL_REG_BWRATE,  0x0B);   // 200 Hz output data rate
  ok &= adxlWrite(ADXL_REG_DATAFMT, 0x09);   // full resolution, +/-4 g
  ok &= adxlWrite(ADXL_REG_PWRCTL,  0x08);   // measurement mode
  return ok;
}

static bool adxlReadG(float &x, float &y, float &z) {
  uint8_t raw[6];
  if (!adxlReadBytes(ADXL_REG_DATAX0, raw, 6)) return false;
  x = (int16_t)((raw[1] << 8) | raw[0]) * ADXL_LSB_G;
  y = (int16_t)((raw[3] << 8) | raw[2]) * ADXL_LSB_G;
  z = (int16_t)((raw[5] << 8) | raw[4]) * ADXL_LSB_G;
  return true;
}

// =====================================================================
//  TACHOMETER ISR
// =====================================================================

static void IRAM_ATTR tachIsr() {
  uint32_t now = micros();
  uint32_t dt  = now - tachLastUs;
  if (dt < TACH_MIN_PERIOD_US) return;                  // debounce
  portENTER_CRITICAL_ISR(&tachMux);
  if (tachLastUs != 0 && dt < TACH_MAX_PERIOD_US) {
    tachPeriodSum = tachPeriodSum + dt;
    tachPeriodCnt = tachPeriodCnt + 1;
  }
  tachLastUs = now;
  portEXIT_CRITICAL_ISR(&tachMux);
}

// =====================================================================
//  ACQUISITION TASK (1 kHz) - vibration, current, microphone
// =====================================================================


/**
 * Signature features from the last 0.5 s of 3-axis acceleration:
 *  - crest factor and kurtosis (impulsiveness: bearing defects)
 *  - amplitude at the shaft frequency 1X via a Hann-windowed Goertzel filter
 * NOTE: at 200 Hz sampling this sees events below 100 Hz only. Real bearing
 * defect frequencies are far higher: see the notes for an upgrade path.
 */
static void computeDiag(const float ring[3][DIAG_N], uint16_t head, float f1x, const float *hann, DiagFeat &out) {
  const bool  use1x = (f1x >= 2.0f && f1x <= 0.45f * ADXL_FS);
  const float coeff = use1x ? 2.0f * cosf(2.0f * PI * f1x / ADXL_FS) : 0.0f;
  float oneXpow = 0, bestM2 = -1, crest = 0, kurt = 0;
  for (int a = 0; a < 3; a++) {
    float mean = 0;
    for (uint16_t i = 0; i < DIAG_N; i++) mean += ring[a][i];
    mean /= DIAG_N;
    float m2 = 0, m4 = 0, pk = 0, s1 = 0, s2 = 0;
    for (uint16_t i = 0; i < DIAG_N; i++) {
      float x  = ring[a][(head + i) % DIAG_N] - mean;     // chronological order
      float x2 = x * x;
      m2 += x2; m4 += x2 * x2;
      float ax = fabsf(x); if (ax > pk) pk = ax;
      if (use1x) { float s0 = x * hann[i] + coeff * s1 - s2; s2 = s1; s1 = s0; }
    }
    m2 /= DIAG_N; m4 /= DIAG_N;
    if (use1x) {
      float p = s1 * s1 + s2 * s2 - coeff * s1 * s2;
      if (p < 0) p = 0;
      float amp = 4.0f * sqrtf(p) / DIAG_N;               // Hann coherent gain 0.5
      oneXpow += 0.5f * amp * amp;                        // RMS^2 of the 1X component
    }
    if (m2 > bestM2) { bestM2 = m2; crest = pk / sqrtf(m2 + 1e-9f); kurt = m4 / (m2 * m2 + 1e-12f); }
  }
  out.crest = crest; out.kurt = kurt; out.oneX = sqrtf(oneXpow);
}

static void acqTask(void *) {
  uint32_t rng = 0xC0FFEEu ^ esp_random();
  Osc oscVib, oscCur, oscAco;
  oscCur.init(50.0f,  0.001f);     // advanced once per tick (1 kHz)
  oscAco.init(120.0f, 0.001f);

  float hann[DIAG_N];
  for (uint16_t i = 0; i < DIAG_N; i++) hann[i] = 0.5f * (1.0f - cosf(2.0f * PI * i / (DIAG_N - 1)));

  float    run = 1.0f;             // simulated machine speed factor 0..1
  float    ax[3][ADXL_PER_FRAME];  // per-axis samples of the current 100 ms frame
  float    ring[3][DIAG_N];        // per-axis 0.5 s history for the signature
  memset(ring, 0, sizeof(ring));
  uint16_t rHead = 0, rCount = 0;
  uint8_t  nAx = 0, accelFails = 0;
  float    curSum = 0, curSq = 0, acoSum = 0, acoSq = 0;
  uint16_t tick = 0, nAn = 0;

  TickType_t period = pdMS_TO_TICKS(1);
  if (period == 0) period = 1;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, period);
    tick++;

    // Simulated machine: stops smoothly after a trip, restarts instantly on re-arm
    if (g_machineRunning) run = 1.0f; else run -= run * 0.0004f;     // coasts down over a few seconds
    g_simRun = run;
    uint32_t faults = g_faults;
    float wear = g_wear;

    // ---- Analog channels (every tick, AC component only) ----
    float curMv, acoMv;
    if (simulated[CH_CUR]) {
      float irms = (SIM_CUR_NOM_A + 0.6f * wear + ((faults & FAULT_CURRENT) ? 5.0f : 0.0f)) * run;
      curMv = 1.41421356f * irms * CUR_MV_PER_A * oscCur.next() + frand(rng) * 3.0f;
    } else {
      curMv = (float)analogReadMilliVolts(PIN_CURRENT) - ANALOG_CENTER_MV;
    }
    if (simulated[CH_ACO]) {
      float arms = (SIM_ACO_NOM_MV + 60.0f * wear + ((faults & FAULT_ACOUSTIC) ? 200.0f : 0.0f)) * run;
      acoMv = 1.41421356f * arms * oscAco.next() + frand(rng) * 5.0f;
    } else {
      acoMv = (float)analogReadMilliVolts(PIN_ACOUSTIC) - ANALOG_CENTER_MV;
    }
    curSum += curMv; curSq += curMv * curMv;
    acoSum += acoMv; acoSq += acoMv * acoMv;
    nAn++;

    // ---- Accelerometer (every ADXL_DIV ticks) ----
    if (tick % ADXL_DIV == 0) {
      float x = 0, y = 0, z = 0;
      bool ok = true;
      if (faults & FAULT_ACCEL_COMM) {
        ok = false;
      } else if (simulated[CH_VIB]) {
        oscVib.step = 2.0f * PI * (SIM_RPM_NOM / 60.0f) * run * 0.005f;   // 1X tracks shaft speed
        float amp = (0.05f + 0.04f * wear) * run;
        if (faults & FAULT_1X)        amp += 0.5f * run;                  // belt / misalignment: strong 1X
        if (faults & FAULT_VIB_SHOCK) amp += 2.0f * run;                  // violent shock
        float v = amp * oscVib.next();
        x = v * 0.7f + frand(rng) * 0.015f;
        y = v * 0.4f + frand(rng) * 0.015f;
        z = 1.0f + v + frand(rng) * 0.015f;
        float impA = (faults & FAULT_VIB) ? 0.9f : 0.0f;                   // bearing: random impulses (~20 %)
        if (wear > 0.0f) { float wa = 0.9f * powf(wear, 2.2f); if (wa > impA) impA = wa; }   // progressive wear
        if (impA > 0.0f && run > 0.5f && frand(rng) < -0.6f) {
          x += frand(rng) * impA; y += frand(rng) * impA; z += frand(rng) * impA;
        }
      } else {
        ok = adxlReadG(x, y, z);
      }
      if (ok) {
        accelFails = 0;
        if (nAx < ADXL_PER_FRAME) { ax[0][nAx] = x; ax[1][nAx] = y; ax[2][nAx] = z; nAx++; }
        ring[0][rHead] = x; ring[1][rHead] = y; ring[2][rHead] = z;
        rHead = (rHead + 1) % DIAG_N;
        if (rCount < DIAG_N) rCount++;
      } else if (accelFails < 255) {
        accelFails++;
      }
    }

    // ---- Frame complete (100 ms) ----
    if (tick >= FRAME_TICKS) {
      Frame f;
      f.accelFails = accelFails;
      f.vibValid   = nAx >= ADXL_PER_FRAME / 2;
      f.vibRms     = 0;
      if (f.vibValid) {
        float tot = 0;                                 // orientation-independent: sum of per-axis variances
        for (int a = 0; a < 3; a++) {
          float m = 0;
          for (uint8_t i = 0; i < nAx; i++) m += ax[a][i];
          m /= nAx;
          float ss = 0;
          for (uint8_t i = 0; i < nAx; i++) { float d = ax[a][i] - m; ss += d * d; }
          tot += ss / nAx;
        }
        f.vibRms = sqrtf(tot);                         // dynamic RMS: gravity removed
      }
      f.diagOk = f.vibValid && rCount >= DIAG_N;
      f.crest = f.kurt = f.oneX = 0;
      if (f.diagOk) {
        DiagFeat d;
        computeDiag(ring, rHead, g_rpm / 60.0f, hann, d);
        f.crest = d.crest; f.kurt = d.kurt; f.oneX = d.oneX;
      }
      float n = (float)nAn;
      float cm = curSum / n, cv = curSq / n - cm * cm; if (cv < 0) cv = 0;
      float am = acoSum / n, av = acoSq / n - am * am; if (av < 0) av = 0;
      f.curA   = sqrtf(cv) / CUR_MV_PER_A;
      f.acoPct = sqrtf(av) / ACO_FULLSCALE_MV * 100.0f;

      if (xQueueSend(frameQ, &f, 0) != pdTRUE) g_droppedFrames = g_droppedFrames + 1;

      tick = 0; nAn = 0; nAx = 0;
      curSum = curSq = acoSum = acoSq = 0;
    }
  }
}

// =====================================================================
//  DETECTION GLUE
// =====================================================================

/** Hard limits first (immediate), then statistical spike detection. */
static void evaluate(Ch ch, float x) {
  const ChannelCfg &c = CFG[ch];
  lastVal[ch] = x;
  {                                                       // smoothed value (EMA), tuned to each channel's update rate
    float a = (ch == CH_TEMP) ? 0.3f : (ch == CH_RPM) ? 0.2f : 0.05f;
    if (!vSmReady[ch]) { vSm[ch] = x; vSmReady[ch] = true; } else vSm[ch] += a * (x - vSm[ch]);
  }
  if (state == UnitState::TRIPPED) return;

  if (!isnan(c.hardHigh) && x > c.hardHigh) {
    emergencyTrip(TripKind::MECHANICAL, "%s HARD LIMIT (%.*f%s > %.*f%s)", c.name,
                  (int)c.decimals, x, c.unit, (int)c.decimals, c.hardHigh, c.unit);
    return;
  }
  if (!isnan(c.hardLow) && x < c.hardLow) {
    emergencyTrip(TripKind::MECHANICAL, "%s BELOW MINIMUM (%.*f%s < %.*f%s)", c.name,
                  (int)c.decimals, x, c.unit, (int)c.decimals, c.hardLow, c.unit);
    return;
  }
  if (det[ch].update(x, c)) {
    emergencyTrip(TripKind::MECHANICAL, "%s ANOMALY (%.*f%s, z=%+.1f, baseline %.*f%s)", c.name,
                  (int)c.decimals, x, c.unit, det[ch].z,
                  (int)c.decimals, det[ch].mean(), c.unit);
  }
}

static void processFrame(const Frame &f) {
  if (settleFrames) { settleFrames--; return; }
  if (f.accelFails >= ACCEL_FAIL_LIMIT) {
    emergencyTrip(TripKind::SENSOR, "ACCELEROMETER COMMUNICATION LOSS (%u failed reads)", (unsigned)f.accelFails);
    return;
  }
  DiagFeat df = { f.crest, f.kurt, f.oneX };
  if (f.diagOk) diagPush(df);                  // history first: the classifier may run inside evaluate()
  if (f.vibValid) evaluate(CH_VIB, f.vibRms);
  evaluate(CH_CUR, f.curA);
  evaluate(CH_ACO, f.acoPct);
  if (state != UnitState::TRIPPED && f.diagOk && !anyStrikes()) diagLearn(df);
}

// ---- Temperature -----------------------------------------------------

static float simTempStep(float dt) {
  if (g_machineRunning) {
    if (g_faults & FAULT_THERMAL) {
      simTempC += 1.2f * dt;                                   // lubrication loss: sharp ramp
    } else if (g_faults & FAULT_VIB) {
      simTempC += 0.12f * dt;                                  // bearing wear: gradual heating
    } else {
      float target = SIM_TEMP_BASE_C + 0.8f * sinf(millis() / 40000.0f) + 6.0f * g_wear;
      simTempC += (target - simTempC) * (dt / 5.0f);
    }
    simTempC += frand(rngMain) * 0.08f;
    if (g_faults & FAULT_TEMP_OVER) return simTempC + 45.0f;   // hard-limit test offset
  } else {
    simTempC += (SIM_AMBIENT_C - simTempC) * (dt / 60.0f);     // machine cools after trip
  }
  return simTempC;
}

static void handleTempReading(float t) {
  if (g_faults & FAULT_TEMP_PROBE) t = DEVICE_DISCONNECTED_C;
  // 85.0 C is the DS18B20 power-on / not-ready value: treat as a bad read.
  bool bad = isnan(t) || t == DEVICE_DISCONNECTED_C || t < -55.0f || t > 125.0f ||
             fabsf(t - 85.0f) < 0.001f;
  if (bad) {
    if (++tempFails >= TEMP_FAIL_LIMIT)
      emergencyTrip(TripKind::SENSOR, "TEMPERATURE SENSOR FAILURE (%u bad reads)", (unsigned)tempFails);
    return;
  }
  tempFails = 0;
  tempPush(t);
  evaluate(CH_TEMP, t);
}

/** Non-blocking DS18B20: request -> wait -> read. */
static void serviceTemperature() {
  uint32_t now = millis();
  if (!tempRequested) {
    if (now - lastTempCycleMs < TEMP_PERIOD_MS) return;
    lastTempCycleMs = now;
    if (simulated[CH_TEMP]) {
      handleTempReading(simTempStep(TEMP_PERIOD_MS / 1000.0f));
    } else {
      tempSensor.requestTemperatures();
      tempRequested   = true;
      tempRequestedMs = now;
    }
    return;
  }
  if (now - tempRequestedMs < TEMP_CONV_MS) return;
  tempRequested = false;
  handleTempReading(tempSensor.getTempCByIndex(0));
}

// ---- Tach -------------------------------------------------------------

static void serviceTach() {
  uint32_t now = millis();
  if (now - lastTachMs < TACH_GATE_MS) return;
  lastTachMs = now;

  float rpm;
  if (simulated[CH_RPM]) {
    float k = (g_faults & FAULT_SPEED) ? 0.85f : 1.0f;         // belt slip
    rpm = SIM_RPM_NOM * g_simRun * k + frand(rngMain) * 4.0f;
    if (rpm < 0) rpm = 0;
  } else {
    uint32_t sum, cnt;
    portENTER_CRITICAL(&tachMux);
    sum = tachPeriodSum; cnt = tachPeriodCnt;
    tachPeriodSum = 0;   tachPeriodCnt = 0;
    portEXIT_CRITICAL(&tachMux);
    rpm = (cnt > 0) ? 60.0e6f / ((float)sum / cnt) / TACH_PPR : 0.0f;  // no pulses = stopped
  }
  g_rpm = rpm;
  evaluate(CH_RPM, rpm);
}

// =====================================================================
//  STATUS LOGGING
// =====================================================================

static void printChannel(Ch ch) {
  const ChannelCfg &c = CFG[ch];
  if (isnan(lastVal[ch])) { Serial.printf("%s ---", c.tag); return; }
  Serial.printf("%s %.*f%s", c.tag, (int)c.decimals, lastVal[ch], c.unit);
  if (state == UnitState::TRIPPED) return;
  if (det[ch].ready(c)) {
    Serial.printf(" z%+.1f", det[ch].z);
    if (det[ch].strikes) Serial.printf(" !k%u", (unsigned)det[ch].strikes);
  } else {
    Serial.printf(" L%u/%u", (unsigned)det[ch].fill(), (unsigned)c.win);
  }
}

static void logStatus() {
  logTs();
  Serial.printf("%s", stateName(state));
  for (int i = 0; i < CH_COUNT; i++) { Serial.print(" | "); printChannel((Ch)i); }
  if (state != UnitState::WARMUP) Serial.printf(" | HEALTH %.0f%s", g_health, g_watch ? " WATCH" : "");
  if (state == UnitState::TRIPPED) Serial.printf(" | RELAY TRIPPED | cause: %s\n", tripReason);
  else                             Serial.println(" | RELAY RUN");
}

static void printLimit(float v, int dec) {
  if (isnan(v)) Serial.print("--"); else Serial.printf("%.*f", dec, v);
}

static void printDetail() {
  Serial.println("---------------- DETAIL ----------------");
  Serial.printf("State %s| uptime %.0fs | trips %u | boots %u | dropped frames %u | faults 0x%02X | heap %u\n",
                stateName(state), millis() / 1000.0f, (unsigned)tripCount, (unsigned)bootCount,
                (unsigned)g_droppedFrames, (unsigned)g_faults, (unsigned)ESP.getFreeHeap());
  for (int i = 0; i < CH_COUNT; i++) {
    const ChannelCfg &c = CFG[i];
    Serial.printf("%s %-14s %-4s ", c.tag, c.name, simulated[i] ? "SIM" : "REAL");
    if (isnan(lastVal[i])) Serial.print("value ---     ");
    else Serial.printf("value %.*f%-3s ", (int)c.decimals, lastVal[i], c.unit);
    Serial.printf("mu %.*f sd %.*f z %+.1f k%u/%u win %u/%u limits [",
                  (int)c.decimals + 1, det[i].mean(), (int)c.decimals + 1, det[i].sd(),
                  det[i].z, (unsigned)det[i].strikes, (unsigned)c.strikes,
                  (unsigned)det[i].fill(), (unsigned)c.win);
    printLimit(c.hardLow, c.decimals); Serial.print(" .. "); printLimit(c.hardHigh, c.decimals);
    Serial.println("]");
  }
  if (state == UnitState::TRIPPED) Serial.printf("Trip cause: %s\n", tripReason);
  Serial.println("----------------------------------------");
}

static void printHelp() {
  Serial.println("Commands: r re-arm | s detail | h help | d diagnosis | l component life | j JSON stream | 0 clear faults");
  Serial.println("  p start wear story (demo) | x self-test | t test alert via BLE->WiFi->SIM | c comm status");
  Serial.println("  n + b/l/g : new part installed (bearing / belt / gear) -> resets its life counter");
  Serial.println("  Fault injection (simulated channels): 1 bearing wear  2 lubrication loss  3 overcurrent");
  Serial.println("    4 speed loss  5 acoustic  6 vib hard-limit shock  7 temp hard-limit  m shaft-speed (1X)");
  Serial.println("    8 ADXL345 comm loss  9 DS18B20 loss      (f = 1)");
}

// =====================================================================
//  USER INPUT
// =====================================================================

static void injectFault(uint32_t bit, int needSim, const char *what) {
  if (state == UnitState::TRIPPED) {
    logTs(); Serial.println("Ignored: unit is TRIPPED. Send 'r' to re-arm first.");
    return;
  }
  if (needSim >= 0 && !simulated[needSim]) {
    logTs(); Serial.printf("Ignored: %s channel uses a REAL sensor.\n", CFG[needSim].tag);
    return;
  }
  g_faults |= bit;
  logTs(); Serial.printf("Simulated fault INJECTED: %s\n", what);
}

static void handleCommand(char c) {
  if (pendingNew) {                                   // second key of the "n" sequence
    if (c == '\r' || c == '\n' || c == ' ') return;
    pendingNew = false;
    int comp = (c == 'b' || c == 'B') ? COMP_BEARING :
               (c == 'l' || c == 'L') ? COMP_BELT :
               (c == 'g' || c == 'G') ? COMP_GEAR : -1;
    if (comp >= 0) installNewPart(comp);
    else { logTs(); Serial.println("New-part command cancelled."); }
    return;
  }
  switch (c) {
    case 'r': case 'R': rearm(); break;
    case 's': case 'S': printDetail(); break;
    case 'h': case 'H': case '?': printHelp(); break;
    case 'd': case 'D': printReport(); printDiagJson(); break;
    case 'l': case 'L': printLifeTable(-1); printLifeJson(); break;
    case 'j': case 'J': streamJson = !streamJson; logTs(); Serial.printf("JSON telemetry stream %s\n", streamJson ? "ON" : "OFF"); break;
    case 'p': case 'P': startStory(); break;
    case 'x': case 'X': selfTest(); break;
    case 't': case 'T': commTest(); break;
    case 'c': case 'C': commStatus(); break;
    case 'n': case 'N': pendingNew = true; logTs(); Serial.println("New part? press b = bearing, l = belt, g = gear (other key cancels)"); break;
    case '0': g_faults = 0; logTs(); Serial.println("Simulated faults cleared."); break;
    case '1': case 'f': case 'F': injectFault(FAULT_VIB,      CH_VIB,  "bearing wear (impulsive vibration, gradual heating)"); break;
    case '2': injectFault(FAULT_THERMAL,   CH_TEMP, "lubrication loss (sharp temperature rise)"); break;
    case '3': injectFault(FAULT_CURRENT,   CH_CUR,  "mechanical jam (overcurrent)"); break;
    case '4': injectFault(FAULT_SPEED,     CH_RPM,  "belt slip (speed loss)"); break;
    case '5': injectFault(FAULT_ACOUSTIC,  CH_ACO,  "acoustic emission increase"); break;
    case '6': injectFault(FAULT_VIB_SHOCK, CH_VIB,  "vibration shock (hard-limit test)"); break;
    case '7': injectFault(FAULT_TEMP_OVER, CH_TEMP, "temperature offset +45 C (hard-limit test)"); break;
    case '8': injectFault(FAULT_ACCEL_COMM, -1, "ADXL345 communication loss"); break;
    case '9': injectFault(FAULT_TEMP_PROBE, -1, "DS18B20 disconnected"); break;
    case 'm': case 'M': injectFault(FAULT_1X, CH_VIB, "strong 1X vibration (belt tension / misalignment)"); break;
    default: break;   // ignore CR/LF/space
  }
}

static void serviceInputs() {
  while (Serial.available()) handleCommand((char)Serial.read());

  // BOOT button: short press = start the wear story (demo), hold 1 s while tripped = re-arm
  static uint32_t pressedSince = 0;
  static bool     longDone = false;
  uint32_t now = millis();
  if (digitalRead(PIN_RESET_BTN) == LOW) {
    if (pressedSince == 0) { pressedSince = now; longDone = false; }
    if (state == UnitState::TRIPPED && !longDone && now - pressedSince > 1000) { longDone = true; rearm(); }
  } else {
    if (pressedSince && !longDone && (now - pressedSince) > 40 && (now - pressedSince) < 900 && state != UnitState::TRIPPED)
      startStory();
    pressedSince = 0;
  }
}

// =====================================================================
//  WATCHDOG (Arduino-ESP32 2.x / 3.x compatible)
// =====================================================================

static void watchdogBegin() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg;
  cfg.timeout_ms     = WDT_TIMEOUT_S * 1000;
  cfg.idle_core_mask = 0;
  cfg.trigger_panic  = true;
  if (esp_task_wdt_reconfigure(&cfg) != ESP_OK) esp_task_wdt_init(&cfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);   // subscribe the Arduino loop task
}

// =====================================================================
//  SETUP
// =====================================================================

void setup() {
  // --- 1. Decide the relay level BEFORE anything slow happens -----------
  prefs.begin("greenbox", false);
  bool persistedLatched = PERSIST_TRIP && prefs.getBool("latched", false);
  tripCount = prefs.getUInt("trips", 0);
  bootCount = prefs.getUInt("boots", 0) + 1;
  loadLife();
  if (DEMO_PRELOAD_LIFE && !prefs.getBool("preload", false)) {      // demo: a machine that already has hours on it
    const float hours[COMP_COUNT] = { 1620.0f, 900.0f, 1200.0f };
    for (int i = 0; i < COMP_COUNT; i++) { life.opSec[i] = hours[i] * 3600.0; life.wSec[i] = hours[i] * 3600.0; }
    saveLife();
    prefs.putBool("preload", true);
  }
  prefs.putUInt("boots", bootCount);
  esp_reset_reason_t rr = esp_reset_reason();
  bool unexpected = TRIP_ON_UNEXPECTED_RST && isUnexpectedReset(rr);

  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, (persistedLatched || unexpected) ? RELAY_TRIP_LEVEL : RELAY_RUN_LEVEL);
  pinMode(PIN_LED, OUTPUT);
  if (PIN_BUZZER >= 0) { pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, LOW); }
  pinMode(PIN_RESET_BTN, INPUT_PULLUP);
  for (int i = 0; i < CH_COUNT; i++) lastVal[i] = NAN;

  // --- 2. Serial banner -------------------------------------------------
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);
  delay(200);

  Serial.println();
  Serial.println("==================================================");
  Serial.printf ("  GREEN BOX v%s - Predictive Maintenance Edge Unit\n", FW_VERSION);
  Serial.println("  5-channel monitor | moving-window anomaly detection");
  if (DEMO_MODE) Serial.println("  *** DEMO BUILD: sensors simulated unless detected ***");
  Serial.println("==================================================");
  Serial.printf ("Chip   : %s rev %d, %d core(s) @ %u MHz\n", ESP.getChipModel(),
                 (int)ESP.getChipRevision(), (int)ESP.getChipCores(), (unsigned)getCpuFrequencyMhz());
  Serial.printf ("Boot   : #%u, reset reason %s, lifetime trips %u\n",
                 (unsigned)bootCount, resetReasonStr(rr), (unsigned)tripCount);

  // --- 3. Sensors -------------------------------------------------------
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(20);                       // never hang on a stuck I2C bus
  bool adxlOk = adxlInit();

  tempSensor.begin();
  tempSensor.setWaitForConversion(false);    // non-blocking conversions
  bool dsOk = tempSensor.getDeviceCount() > 0;
  if (dsOk) tempSensor.setResolution(11);

  analogReadResolution(12);
  if (USE_REAL_CURRENT)  analogSetPinAttenuation(PIN_CURRENT,  ADC_11db);
  if (USE_REAL_ACOUSTIC) analogSetPinAttenuation(PIN_ACOUSTIC, ADC_11db);
  if (USE_REAL_TACH) {
    pinMode(PIN_TACH, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_TACH), tachIsr, RISING);
  }

  simulated[CH_VIB]  = !adxlOk;
  simulated[CH_TEMP] = !dsOk;
  simulated[CH_CUR]  = !USE_REAL_CURRENT;
  simulated[CH_RPM]  = !USE_REAL_TACH;
  simulated[CH_ACO]  = !USE_REAL_ACOUSTIC;

  const char *srcName[CH_COUNT] = { "ADXL345 (I2C)", "DS18B20 (1-Wire)", "ACS712 (ADC)",
                                    "Hall tach (IRQ)", "Microphone (ADC)" };
  for (int i = 0; i < CH_COUNT; i++)
    Serial.printf("Sensor : %s %-16s -> %s\n", CFG[i].tag, srcName[i],
                  simulated[i] ? "SIMULATED" : "REAL");

  bool anySim = false;
  for (int i = 0; i < CH_COUNT; i++) anySim |= simulated[i];
  if (!ALLOW_SIMULATION && anySim) bootFault = true;

  printLifeTable(-1);

  // --- 4. Acquisition task ---------------------------------------------
  frameQ = xQueueCreate(8, sizeof(Frame));
#if CONFIG_FREERTOS_UNICORE
  xTaskCreate(acqTask, "acq", 8192, nullptr, 5, nullptr);
#else
  xTaskCreatePinnedToCore(acqTask, "acq", 8192, nullptr, 5, nullptr, 0);
#endif

  watchdogBegin();
  lastFrameMs = lastTachMs = lastTempCycleMs = millis();

  commBegin();                                   // BLE + WiFi + SIM tasks (never blocks the safety loop)
  Serial.printf("Comms  : device %s | BLE %s | WiFi %s | SIM %s\n", g_devId,
                COMM_ENABLE_BLE ? "on" : "off", COMM_ENABLE_WIFI ? "on" : "off", COMM_ENABLE_SIM ? "on" : "off");

  // --- 5. Initial state --------------------------------------------------
  if (bootFault) {
    emergencyTrip(TripKind::SYSTEM, "REQUIRED SENSOR(S) UNAVAILABLE (ALLOW_SIMULATION=false)");
  } else if (persistedLatched) {
    String r = prefs.getString("reason", "unknown");
    snprintf(tripReason, sizeof(tripReason), "%s", r.c_str());
    state = UnitState::TRIPPED;
    g_machineRunning = false;
    tripAtMs = millis();
    logTs(); Serial.println("PERSISTED TRIP RESTORED - machine stays stopped until re-armed.");
    logTs(); Serial.printf("Last cause: %s\n", tripReason);
    tripKind = (TripKind)prefs.getUChar("dkind", (uint8_t)TripKind::SYSTEM);
    memset(&lastDiag, 0, sizeof(lastDiag));
    lastDiag.cls  = prefs.getUChar("dcls", FC_UNKNOWN);
    lastDiag.conf = prefs.getUChar("dconf", 0);
    haveDiag = (tripKind == TripKind::MECHANICAL);
    if (haveDiag && lastDiag.cls < FC_COUNT) {
      logTs(); Serial.printf("Last diagnosis: %s (%u%%) - suspect: %s\n",
                             FAULTS[lastDiag.cls].title, (unsigned)lastDiag.conf, FAULTS[lastDiag.cls].part);
    }
  } else if (unexpected) {
    emergencyTrip(TripKind::SYSTEM, "UNEXPECTED RESET (%s) - operator acknowledgement required", resetReasonStr(rr));
  } else {
    state = UnitState::WARMUP;
  }

  Serial.println("--------------------------------------------------");
  printHelp();
  Serial.println("--------------------------------------------------");
}

// =====================================================================
//  MAIN LOOP
// =====================================================================

void loop() {
  esp_task_wdt_reset();

  serviceInputs();

  // Consume feature frames from the acquisition task
  Frame f;
  while (xQueueReceive(frameQ, &f, 0) == pdTRUE) {
    lastFrameMs = millis();
    processFrame(f);
  }

  serviceTemperature();
  serviceTach();
  serviceLife();
  commService();

  // Acquisition watchdog
  if (state != UnitState::TRIPPED && (millis() - lastFrameMs) > ACQ_TIMEOUT_MS) {
    emergencyTrip(TripKind::SYSTEM, "ACQUISITION STALL (no data for %u ms)", (unsigned)(millis() - lastFrameMs));
  }

  // WARMUP -> RUN once every baseline is learned
  if (state == UnitState::WARMUP) {
    bool all = true;
    for (int i = 0; i < CH_COUNT; i++) all &= det[i].ready(CFG[i]);
    if (all) {
      state = UnitState::RUN;
      runSinceMs = millis();
      for (int i = 0; i < CH_COUNT; i++) {                    // freeze the healthy reference
        refMean[i] = det[i].mean(); refSd[i] = det[i].sd(); refReady[i] = true; driftCnt[i] = 0;
      }
      logTs(); Serial.println("Baselines learned on all 5 channels. Anomaly detection ARMED.");
    }
  }

  serviceStory();
  serviceHealth();
  if (diagPending && state == UnitState::TRIPPED && (int32_t)(millis() - diagDueMs) >= 0) finishTripReport();
  serviceLed();
  serviceBuzzer();

  uint32_t now = millis();
  uint32_t period = (state == UnitState::TRIPPED) ? STATUS_PERIOD_TRIPPED_MS : STATUS_PERIOD_MS;
  if (now - lastStatusMs >= period) {
    lastStatusMs = now;
    logStatus();
  }

  if (streamJson && now - lastTelMs >= TELEMETRY_PERIOD_MS) {
    lastTelMs = now;
    printTelemetry();
  }

  delay(1);
}
