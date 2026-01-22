#include <time.h>
#include "wifi.h"   // Includes multiple networks: WIFI_SSIDS[], WIFI_PASSWORDS[], etc.
#include <Ticker.h>

// ----------------------
// Pin and constant definitions
// ----------------------
#define LEDBUILTIN 2      // Pin for the LED (indicates active transmission)
#define ANTENNAPIN 18     // Pin for the antenna connection (through a 1kΩ resistor, then GND)

// If you want the device to run continuously, uncomment the following line:
//#define CONTINUOUSMODE

// ----------------------
// Global variables
// ----------------------
struct tm timeinfo;          // Structure for storing local time
const int pwmChannel = 0;    // PWM channel for ledc

Ticker tickerDecisec;        // Ticker object to call DcfOut function every 100 ms

// Array of pulses to form the DCF77 signal (60 seconds)
// Made volatile because accessed from the timer callback and main context
volatile uint8_t impulseArray[60];
volatile int impulseCount = 0;
volatile int actualSecond = 0;

volatile int actualHours = 0, actualMinutes = 0, actualDay = 0, actualMonth = 0, actualYear = 0, DayOfW = 0;

// The total time we allow for WiFi connection or initial active period
unsigned long dontGoToSleep = 0UL;                // ESP32 startup time (in milliseconds)
const unsigned long onTimeAfterReset = 1200000UL;  // 20 minutes in milliseconds
int timeRunningContinuous = 0;          // Counter for continuous transmission mode

// ----------------------
// Forward declarations
// ----------------------
void DcfOut();
void CodeTime();

// ----------------------
// DCF77 signal generation (timer-safe minimal handler)
// ----------------------

// DcfOut is called every 100 ms by the Ticker. It must be non-blocking and avoid
// any heavy system calls. All heavy work (getLocalTime, Serial prints, CodeTime)
// is done in the main loop.
void DcfOut() {
  // Read volatile values into local copies for stability during this 100ms slot
  int sec = actualSecond;
  int state = impulseArray[sec];

  switch (impulseCount++) {
    case 0:
      if (state != 0) {
        digitalWrite(LEDBUILTIN, LOW);
        ledcWrite(pwmChannel, 0);
      } else {
        // For second 59 (state == 0) we keep carrier on; but leaving as is
      }
      break;
    case 1:
      if (state == 1) {
        digitalWrite(LEDBUILTIN, HIGH);
        ledcWrite(pwmChannel, 127);
      }
      break;
    case 2:
      digitalWrite(LEDBUILTIN, HIGH);
      ledcWrite(pwmChannel, 127);
      break;
    case 9:
      impulseCount = 0;
      // Keep all logging and heavy operations out of this function
      break;
  }
}

// ----------------------
// Convert a decimal number to BCD (non-blocking helper)
// ----------------------
int Bin2Bcd(int dato) {
  int msb, lsb;
  if (dato < 10)
    return dato;
  msb = (dato / 10) << 4;
  lsb = dato % 10;
  return msb + lsb;
}

// The CodeTime() function forms the impulseArray for the DCF77 signal.
// It constructs the new array in a local buffer and then swaps it into the
// shared impulseArray while the Ticker is detached to avoid races.
void CodeTime() {
  uint8_t newImpulse[60];
  // Defaults
  for (int i = 0; i < 60; i++) newImpulse[i] = 0;

  // Determine the day of the week (0 -> 7 for DCF77)
  int dow = timeinfo.tm_wday;
  if (dow == 0) dow = 7;

  int thour    = timeinfo.tm_hour;
  int tminute  = timeinfo.tm_min;
  int tday     = timeinfo.tm_mday;
  int tmonth   = timeinfo.tm_mon + 1;
  int tyear    = timeinfo.tm_year - 100;  // 2-digit year

  int actualHours_local  = thour;
  int actualMinutes_local = tminute + 1;
  if (actualMinutes_local >= 60) {
    actualMinutes_local = 0;
    actualHours_local++;
  }
  int actualSecond_local = timeinfo.tm_sec;
  if (actualSecond_local == 60) actualSecond_local = 0;

  // First 20 seconds – logical "0" (100 ms pulse)
  for (int n = 0; n < 20; n++) {
    newImpulse[n] = 1;
  }

  // Set bits for DST
  if (timeinfo.tm_isdst == 0) {
    newImpulse[18] = 2;  // DST OFF
  } else {
    newImpulse[17] = 2;  // DST ON
  }

  // Bit 20 – active time indicator
  newImpulse[20] = 2;

  // Form bits for minutes (bits 21..27) and parity bit (28)
  int ParityCount = 0;
  int TmpIn = Bin2Bcd(actualMinutes_local);
  for (int n = 21; n < 28; n++) {
    int Tmp = TmpIn & 1;
    newImpulse[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  newImpulse[28] = ((ParityCount & 1) == 0) ? 1 : 2;

  // Hours (29..34) and parity (35)
  ParityCount = 0;
  TmpIn = Bin2Bcd(actualHours_local);
  for (int n = 29; n < 35; n++) {
    int Tmp = TmpIn & 1;
    newImpulse[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  newImpulse[35] = ((ParityCount & 1) == 0) ? 1 : 2;

  // Date: day, day of week, month, year and parity (58)
  ParityCount = 0;
  TmpIn = Bin2Bcd(tday);
  for (int n = 36; n < 42; n++) {
    int Tmp = TmpIn & 1;
    newImpulse[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  TmpIn = Bin2Bcd(dow);
  for (int n = 42; n < 45; n++) {
    int Tmp = TmpIn & 1;
    newImpulse[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  TmpIn = Bin2Bcd(tmonth);
  for (int n = 45; n < 50; n++) {
    int Tmp = TmpIn & 1;
    newImpulse[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  TmpIn = Bin2Bcd(tyear);
  for (int n = 50; n < 58; n++) {
    int Tmp = TmpIn & 1;
    newImpulse[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  newImpulse[58] = ((ParityCount & 1) == 0) ? 1 : 2;

  // Last second – no pulse
  newImpulse[59] = 0;

  // Swap into the shared buffer safely: detach ticker, copy, reset counters, reattach
  tickerDecisec.detach();
  for (int i = 0; i < 60; i++) {
    impulseArray[i] = newImpulse[i];
  }
  impulseCount = 0; // reset timing for new second series
  // Update the shared actualSecond and related values
  actualSecond = actualSecond_local;
  actualHours = actualHours_local;
  actualMinutes = actualMinutes_local;
  actualDay = tday;
  actualMonth = tmonth;
  actualYear = tyear;
  DayOfW = dow;
  tickerDecisec.attach_ms(100, DcfOut);
}

// ----------------------
// Sync windows and deep sleep logic (minor fixes to prints and types)
// ----------------------

struct SyncWindow { int hour; int minute; };
const SyncWindow syncWindows[] = {
  {0, 0}, {1, 30}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {9, 30}, {17, 30}
};
const int numSyncWindows = sizeof(syncWindows) / sizeof(syncWindows[0]);

bool isSyncWindowActive() {
  int nowMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;
  const int windowLen = 10; // minutes
  for (int i = 0; i < numSyncWindows; i++) {
    int start = syncWindows[i].hour * 60 + syncWindows[i].minute;
    int end = (start + windowLen) % (24 * 60);
    bool active = false;
    if (start <= end) {
      active = (nowMinutes >= start && nowMinutes < end);
    } else {
      // window wraps midnight
      active = (nowMinutes >= start || nowMinutes < end);
    }
    if (active) {
      int endHour = end / 60;
      int endMin = end % 60;
      Serial.printf("Sync window active: %02d:%02d to %02d:%02d\n",
                    syncWindows[i].hour, syncWindows[i].minute, endHour, endMin);
      return true;
    }
  }
  return false;
}

unsigned long secondsToNextSyncWindow() {
  int nowMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;
  int minDiff = 24 * 60; // maximum value for a day
  for (int i = 0; i < numSyncWindows; i++) {
    int start = syncWindows[i].hour * 60 + syncWindows[i].minute;
    int diff = start - nowMinutes;
    if (diff < 0) diff += 24 * 60; // if the window has already passed, add a day
    if (diff < minDiff) {
      minDiff = diff;
    }
  }
  Serial.printf("Next sync window in %d minutes (~%lu seconds)\n", minDiff, (unsigned long)minDiff * 60UL);
  return (unsigned long)minDiff * 60UL; // convert minutes to seconds
}

void checkSleep() {
#ifndef CONTINUOUSMODE
  if (millis() - dontGoToSleep > onTimeAfterReset) {
    if (!isSyncWindowActive()) {
      unsigned long sleepSeconds = secondsToNextSyncWindow();
      Serial.printf("Outside sync window. Going to deep sleep for %lu seconds...\n", sleepSeconds);
      ESP.deepSleep(sleepSeconds * 1000000ULL);
    } else {
      Serial.println("Within sync window. Staying awake.");
    }
  } else {
    Serial.println("Initial 20-minute active period. Staying awake.");
  }
#else
  Serial.println("Continuous mode enabled. Skipping sleep check.");
#endif
}

// ----------------------
// setup() and loop()
// ----------------------
void setup() {
  // Disable wake-up from other sources
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  Serial.begin(115200);
  Serial.println();
  Serial.println("=== DCF77 Transmitter with Scheduled Sync Windows (patched) ===");

  // Record the time the device was started (not from deep sleep)
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) {
    dontGoToSleep = millis();
    Serial.printf("Device started at millis: %lu\n", dontGoToSleep);
  }

  // Keep trying to connect to WiFi for up to 20 minutes
  bool connected = false;
  while ((millis() - dontGoToSleep) < onTimeAfterReset) {
    if (WiFi_on()) {
      connected = true;
      break;
    } else {
      Serial.println("Will try again in 5 seconds...");
      delay(5000);
    }
  }

  if (!connected) {
    Serial.println("No WiFi connection after 20 minutes. Going to deep sleep...");
    ESP.deepSleep(3600ULL * 1000000ULL);
  }

  // Otherwise, if we are connected, proceed with NTP sync
  getNTP();
  WiFi_off();
  show_time();

#ifndef CONTINUOUSMODE
  checkSleep();
#else
  Serial.println("Continuous mode active. Device will not enter deep sleep.");
#endif

  // Configure PWM for the DCF77 signal
  ledcSetup(pwmChannel, 77500, 8); // 77.5 kHz, 8-bit resolution
  ledcAttachPin(ANTENNAPIN, pwmChannel);
  ledcWrite(pwmChannel, 0);

  pinMode(LEDBUILTIN, OUTPUT);
  digitalWrite(LEDBUILTIN, LOW);

  // Build the initial DCF77 pulse array (safe swap inside CodeTime)
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Error obtaining time on startup");
  }
  CodeTime();

  // Synchronize with the start of a second for accurate transmission (non-blocking-ish)
  Serial.print("Syncing with start of a second... ");
  int startSecond = timeinfo.tm_sec;
  long count = 0;
  // Use a bounded wait to avoid indefinite blocking
  while (count < 5000) {
    count++;
    if (!getLocalTime(&timeinfo)) {
      // don't restart from here — handle in main loop instead
      break;
    }
    if (timeinfo.tm_sec != startSecond) break;
    delay(1);
  }
  Serial.print("Synced after ");
  Serial.print(count);
  Serial.println(" checks.");

  // Start the Ticker which calls DcfOut() every 100 ms
  tickerDecisec.attach_ms(100, DcfOut);
}

void loop() {
#ifndef CONTINUOUSMODE
  static unsigned long lastCheck = 0UL;
  if (millis() - lastCheck > 30000UL) {
    lastCheck = millis();
    Serial.println("Periodic check of sync window...");
    if (millis() - dontGoToSleep > onTimeAfterReset) {
      if (!isSyncWindowActive()) {
        Serial.println("Sync window ended. Preparing to enter deep sleep.");
        if (getLocalTime(&timeinfo)) {
          checkSleep();
        }
      } else {
        Serial.println("Still within sync window. Continuing operation.");
      }
    } else {
      Serial.println("Within the initial 20-minute active period.");
    }
  }
#endif

  // Time update handling (once per second) — safe, non-blocking
  static int lastSec = -1;
  static int failCount = 0;
  if (getLocalTime(&timeinfo)) {
    if (timeinfo.tm_sec != lastSec) {
      lastSec = timeinfo.tm_sec;
      failCount = 0;
      // Update the shared arrays and timing
      CodeTime();
      // Optional: print current second info in main context
      Serial.printf("Updated time: %02d:%02d:%02d\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    }
  } else {
    failCount++;
    Serial.println("Warning: getLocalTime() failed in loop");
    if (failCount >= 3) {
      Serial.println("Too many time failures — restarting");
      delay(100);
      ESP.restart();
    }
  }

  // all other work is performed in Ticker ISR for output only
}