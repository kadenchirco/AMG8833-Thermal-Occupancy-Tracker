#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include "process.h"
#include <driver/rtc_io.h>

Adafruit_AMG88xx amg;
thermal_processor processor;

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

// RTC memory survives deep sleep, so the background and count only reset on a real power-on
RTC_DATA_ATTR float background[AMG88xx_PIXEL_ARRAY_SIZE];
RTC_DATA_ATTR bool have_background = false;
RTC_DATA_ATTR int saved_count = 0;

const int background_frames = 20; // how many frames of averaging are done
const int sleep_time = 5; //seconds until sleepy
const float wake_margin = 3.0; // degrees above the hottest background pixel that wakes us up
const gpio_num_t int_pin = GPIO_NUM_32;

uint32_t tick = 0;

void average_background() {
  Serial.println("Averaging the background temperature!!!");

  for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    background[i] = 0;
  }

  for (int frame = 0; frame < background_frames; frame++) {
    amg.readPixels(pixels);
    for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      background[i] += pixels[i];
    }
    delay(100);
  }

  float total = 0;
  for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    background[i] /= background_frames;
    total += background[i];
  }

  have_background = true;

  Serial.print("Average background temperature: ");
  Serial.print(total / AMG88xx_PIXEL_ARRAY_SIZE, 2);
  Serial.println(" C");
}

void go_to_sleep() {
  // the amg's INT pin fires when any pixel goes above the high level.
  // set it a bit above the hottest background pixel so only an actual warm thing wakes us
  float hottest = background[0];
  for (int i = 1; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    hottest = max(hottest, background[i]);
  }

  amg.setInterruptLevels(hottest + wake_margin, -20, 0.5);
  amg.setInterruptMode(AMG88xx_ABSOLUTE_VALUE);
  amg.enableInterrupt();

  delay(100);
  amg.clearInterrupt();

  saved_count = processor.get_count();

  rtc_gpio_pullup_en(int_pin);
  rtc_gpio_pulldown_dis(int_pin);
  esp_sleep_enable_ext0_wakeup(int_pin, 0);

  Serial.print("going to sleep zzz, wake threshold: ");
  Serial.print(hottest + wake_margin, 2);
  Serial.println(" C");
  Serial.flush();
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);

  bool status = amg.begin(0x69, &Wire);
  if (!status) {
    Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
    while (1);
  }

  Serial.println("AMG8833 ready");

  rtc_gpio_deinit(int_pin);

  // only average on a real power on. waking from sleep reuses the background saved in RTC memory
  // (redo it if we didn't wake from the INT pin, OR if there's no saved background yet)
  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_EXT0 || !have_background) {
    average_background();
    saved_count = 0;
  }

  processor.set_count(saved_count);
}

void loop() {
  amg.readPixels(pixels);

  // pixels are now degrees above the background instead of absolute temps
  for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    pixels[i] -= background[i];

    // clamp first: squaring a cold spot (negative) would flip it into a fake hot spot
    if (pixels[i] < 0) {
      pixels[i] = 0;
    }

    // lets make it quadratic!! should help differentiating blobs :D
    pixels[i] *= pixels[i];
  }

  // for (int row = 0; row < 8; row++) {
  //   for (int col = 0; col < 8; col++) {
  //     int index = row * 8 + col;

  //     Serial.print(pixels[index], 2);
  //     Serial.print(" ");


  //   }
  //   Serial.println();
  // }

  // Serial.println("--------------------");

  processor.read(pixels, millis());
  processor.find_centroids();
  Serial.print("Count: ");
  Serial.println(processor.get_count());
  Serial.print("Tracks: ");
  Serial.println(processor.get_blobcount());

  uint32_t now = millis();
  uint32_t dt = now - tick;
  Serial.print("FPS: ");
  Serial.println(dt > 0 ? 1000.0f / dt : 0.0f);

  tick = now;

  if ((now - processor.get_detection_time()) / 1000.0 > sleep_time) {
    //go_to_sleep();
  }
  // sensor runs at 10 FPS, so read every 100 ms
  // SIKE the code takes time to run too, duh! so an experimentally shorter delay.
  delay(85);

}





