
#include "Arduino.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"

#include "esp32/ulp.h"
#include "ulp_main.h"
#include "ulptool.h"

#define RAINFACTOR        0.2794      // 0.2794 mm per pulse defined by the Rain Gauge
#define ULPSLEEP          4000        // amount in microseconds the ULP co-processor sleeps
#define TIMEFACTOR        1000000     // factor between seconds and microseconds
#define TIMESLEEP         10          // amount in seconds the ESP32 sleeps
#define PIN_RAIN          GPIO_NUM_13

bool disp = true;
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int timeSleep = 0;
unsigned long startMillis = millis();

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

void setup() {
  Serial.begin(115200);
  setCpuFrequencyMhz(80);
  ++bootCount;

  initULP();
  windTest();
    
  Serial.println(F("Waiting instead of sleeping."));
}

void loop() {
  if (getElapsed(startMillis) > TIMESLEEP * 1000) {
    windTest();
    initSleep();
  }
}

void windTest() {
  float rain = 0;
  bool rainSuccess = false;
  float rainDownpour = 0;
  bool rainDownpourSuccess = false;

  // Wind Speed, Rain, Wind Gust Speed and Rain Downpour
  if (timeSleep > 0) {
    
    // Wind Speed and Rain
    int pulsesRaw = getPulseCount();
    if (pulsesRaw >= 0) {
      Serial.print("Pulse raw value: ");
      Serial.print(pulsesRaw);
      Serial.print(" pulses in ");
      Serial.print(timeSleep);
      Serial.println(" seconds.");
      
      rain = (float)pulsesRaw * RAINFACTOR;
      rainSuccess = rain < (float)timeSleep * RAINFACTOR;
    }

    // Wind Gust Speed and Rain Downpour
    int shortestPulseRaw = getShortestPulse();
    if (pulsesRaw > 1 && shortestPulseRaw > ULPSLEEP * 5 && shortestPulseRaw < timeSleep * TIMEFACTOR) {
      Serial.print("Shortest Pulse raw value: ");
      Serial.print(shortestPulseRaw);
      Serial.print(" microseconds | ");
      Serial.print(((float)shortestPulseRaw / (float)TIMEFACTOR));
      Serial.println(" seconds.");
      
      rainDownpour = ((float)timeSleep / ((float)shortestPulseRaw / (float)TIMEFACTOR)) * RAINFACTOR;
      rainDownpourSuccess = rainDownpour < (float)timeSleep * RAINFACTOR;
    }
  }

  Serial.print(F("Rain: "));
  if (rainSuccess) Serial.print(rain);
  Serial.println(F("mm"));
  
  Serial.print(F("Rain Downpour: "));
  if (rainDownpourSuccess) Serial.print(rainDownpour);
  Serial.println(F("mm"));

  if (timeSleep == TIMESLEEP) timeSleep = TIMESLEEP * 2;
  else if (timeSleep == TIMESLEEP * 2) timeSleep = TIMESLEEP * 3;
  else timeSleep = TIMESLEEP;
}

/****************\
|*    Sleep     *|
\****************/

void initSleep(void) {
  Serial.println(F("Preparing deep sleep now"));

  Serial.println("Set timer for sleep-wakeup every " + String(timeSleep) + " seconds");
  esp_sleep_enable_timer_wakeup(timeSleep * TIMEFACTOR);

  esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);

  Serial.println(F("Going into deep sleep now"));
  esp_deep_sleep_start();
}

/****************\
|*     ULP      *|
\****************/

static void initULP(void)
{
  if (timeSleep > 0) return;
  esp_err_t err_load = ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
  ESP_ERROR_CHECK(err_load);

  /* GPIO used for pulse counting. */
  gpio_num_t gpio_num = PIN_RAIN;
  assert(rtc_gpio_desc[gpio_num].reg && "GPIO used for pulse counting must be an RTC IO");

  /* Initialize some variables used by ULP program.
   * Each 'ulp_xyz' variable corresponds to 'xyz' variable in the ULP program.
   * These variables are declared in an auto generated header file,
   * 'ulp_main.h', name of this file is defined in component.mk as ULP_APP_NAME.
   * These variables are located in RTC_SLOW_MEM and can be accessed both by the
   * ULP and the main CPUs.
   *
   * Note that the ULP reads only the lower 16 bits of these variables. */
  ulp_debounce_counter = 5;
  ulp_debounce_max_count = 5;
  ulp_pulse_edge = 1;
  ulp_next_edge = 1;
  ulp_io_number = rtc_gpio_desc[gpio_num].rtc_num; /* map from GPIO# to RTC_IO# */

  /* Initialize selected GPIO as RTC IO, enable input, sets pullup and pulldown */
  rtc_gpio_init(gpio_num);
  rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pulldown_en(gpio_num);
  rtc_gpio_pullup_dis(gpio_num);
  rtc_gpio_hold_en(gpio_num);

  /* Set ULP wake up period to T = 4ms.
   * Minimum pulse width has to be T * (ulp_debounce_counter + 1) = 24ms. */
  ulp_set_wakeup_period(0, ULPSLEEP);

  /* Start the program */
  esp_err_t err_run = ulp_run(&ulp_entry - RTC_SLOW_MEM);
  ESP_ERROR_CHECK(err_run);
}

static uint32_t getPulseCount(void) {
  /* ULP program counts signal edges, convert that to the number of pulses */
  uint32_t pulse_count_from_ulp = (ulp_edge_count & UINT16_MAX) / 2;
  
  /* In case of an odd number of edges, keep one until next time */
  ulp_edge_count = ulp_edge_count % 2;
  
  return pulse_count_from_ulp;
}

static uint32_t getShortestPulse(void) {
  /* ULP program saves shortes pulse */
  uint32_t pulse_time_min = (ulp_pulse_min & UINT16_MAX) * ULPSLEEP;
  
  /* Reset shortest edge */
  ulp_pulse_min = 0;
  
  return pulse_time_min;
}

long getElapsed(unsigned long compareMillis) {
  unsigned long currentMillis = millis();
  unsigned long elapsedMillis = currentMillis - compareMillis;
  return abs(elapsedMillis);
}
