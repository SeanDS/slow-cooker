// NOTE: board "Nano" CPU frequency manually changed (in ~/Downloads/arduino-1.8.9/hardware/arduino/avr/boards.txt)
// from 16 MHz to 8 MHz.
// TODO: Do this somehow without editing that file?

#ifndef POWER_RELAY_PIN
  #define POWER_RELAY_PIN       A0
#endif

#ifndef ONE_WIRE_BUS_PIN
  #define ONE_WIRE_BUS_PIN      A5 // DS18B20 data
#endif

// DS18B20 temperature precision: 9 (93.8ms), 10 (187.5ms), 11 (375ms) or 12 (750ms) bits
// equal to resolution of 0.5 C, 0.25 C, 0.125 C and 0.0625 C.
#ifndef DS18B20_PRECISION
  #define DS18B20_PRECISION     12
#endif

// Default temperature reading for when reading is out of range of sensor is not present.
#ifndef DS18B20_DEFAULT
  #define DS18B20_DEFAULT       -55
#endif

// Delay required to take temperature measurement (see above).
#ifndef ASYNC_DELAY
  #define ASYNC_DELAY           1000
#endif

#ifndef MEASUREMENT_TIME
  #define MEASUREMENT_TIME      1000
#endif

#ifndef MEASUREMENT_INTERVAL
  #define MEASUREMENT_INTERVAL  30000
#endif

#ifndef BUTTON_DEBOUNCE_DELAY
  #define BUTTON_DEBOUNCE_DELAY 500
#endif

#ifndef TFT_DC_PIN
  #define TFT_DC_PIN 7
#endif

#ifndef TFT_CS_PIN
  #define TFT_CS_PIN 9
#endif

#ifndef TFT_RESET_PIN
  #define TFT_RESET_PIN 8
#endif

#ifndef TFT_BACKLIGHT_PIN
  #define TFT_BACKLIGHT_PIN 6
#endif

#ifndef TOUCH_CS_PIN
  #define TOUCH_CS_PIN 5
#endif

#ifndef TOUCH_IRQ_PIN
  #define TOUCH_IRQ_PIN 2
#endif

#include <Arduino.h>
#include <SPI.h>
#include <Ucglib.h>
#include <XPT2046.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/*
 * TFT screen.
 */

Ucglib_ILI9341_18x240x320_HWSPI ucg(/*cd=*/ TFT_DC_PIN, /*cs=*/ TFT_CS_PIN);

// Flag for whether to redraw the screen in the loop.
bool redraw_screen;

// Screens.
bool screen_is_status;
bool screen_is_adjust;

/*
 * Touch screen.
 */

XPT2046 touch(/*cs=*/ TOUCH_CS_PIN, /*irq=*/ TOUCH_IRQ_PIN);

// Current x and y touch coordinates.
static uint16_t touch_x = 0xffff;
static uint16_t touch_y = 0xffff;

// Time of last touch (ms).
uint16_t touching_since = 0;

// Whether or not the screen is being touched.
bool touching = false;

/*
 * Temperature sensor.
 */

OneWire one_wire(ONE_WIRE_BUS_PIN);
DallasTemperature sensors(&one_wire);

// Temperature sensor address.
byte ds18b20_address[8] = {0x28, 0xff, 0xd5, 0xa9, 0x01, 0x15, 0x04, 0x27};

// Number of temperature sensors on the bus.
uint16_t ds18b20_count;

// Temperature (degrees).
float previous_temperature = DS18B20_DEFAULT;
float current_temperature = DS18B20_DEFAULT;

// Time of most recent measurement (ms).
uint32_t measurement_start_time;

// Whether measurement is in progress or not.
bool measurement_in_progress = false;

uint32_t last_measurement_time = 0;

/*
 * PID controller.
 */

bool pid_enabled;

// Desired temperature (degrees).
float pid_set_point;

// Coefficients.
float pid_kp;
float pid_ki;
float pid_kd;

// Internal variables.
float pid_error;
float pid_p_error;
float pid_i_error;
float pid_d_error;
float pid_integrator;
float pid_differentiator;

// Minimum/maximum integrator values.
float pid_imin;
float pid_imax;

/*
 * Power output.
 */

// Whether or not the output power is enabled.
bool power_on = false;

/*
 * Graphics.
 */

const ucg_fntpgm_uint8_t* font_small = ucg_font_9x18_tr;
const ucg_fntpgm_uint8_t* font_large = ucg_font_10x20_tr;

uint16_t temperature_box_x = 30;
uint16_t temperature_box_y = 10;
uint16_t temperature_box_w = 250;
uint16_t temperature_box_h = 100;
uint16_t temperature_box_tx = 135;
uint16_t temperature_box_ty = 70;

uint16_t temperature_box_triangle_x0 = 220;
uint16_t temperature_box_triangle_y0 = 73;
uint16_t temperature_box_triangle_base = 20;
uint16_t temperature_box_triangle_height = 20;

uint16_t output_status_box_x = 10;
uint16_t output_status_box_y = 120;
uint16_t output_status_box_w = 100;
uint16_t output_status_box_h = 50;
uint16_t output_status_box_tx = 35;
uint16_t output_status_box_ty = 150;

uint16_t pid_toggle_button_x = 30;
uint16_t pid_toggle_button_y = 180;
uint16_t pid_toggle_button_w = 150;
uint16_t pid_toggle_button_h = 50;
uint16_t pid_toggle_button_tx = 55;
uint16_t pid_toggle_button_ty = 210;

uint16_t pid_coefficient_box_x = 30;
uint16_t pid_coefficient_box_y = 10;
uint16_t pid_coefficient_box_w = 290;
uint16_t pid_coefficient_box_h = 50;
uint16_t pid_coefficient_box_tx = 35;
uint16_t pid_coefficient_box_ty = 40;

uint16_t adjust_screen_button_x = 200;
uint16_t adjust_screen_button_y = 180;
uint16_t adjust_screen_button_w = 100;
uint16_t adjust_screen_button_h = 50;
uint16_t adjust_screen_button_tx = 215;
uint16_t adjust_screen_button_ty = 210;

uint16_t temperature_adjust_setpoint_widget_value_x = 130;
uint16_t temperature_adjust_setpoint_widget_value_y = 100;
uint16_t temperature_adjust_setpoint_widget_value_w = 100;
uint16_t temperature_adjust_setpoint_widget_value_h = 50;
uint16_t temperature_adjust_setpoint_widget_value_tx = 150;
uint16_t temperature_adjust_setpoint_widget_value_ty = 130;

uint16_t temperature_adjust_minus_button_x = 70;
uint16_t temperature_adjust_minus_button_y = 100;
uint16_t temperature_adjust_minus_button_w = 50;
uint16_t temperature_adjust_minus_button_h = 50;
uint16_t temperature_adjust_minus_button_tx = 90;
uint16_t temperature_adjust_minus_button_ty = 130;

uint16_t temperature_adjust_plus_button_x = 240;
uint16_t temperature_adjust_plus_button_y = 100;
uint16_t temperature_adjust_plus_button_w = 50;
uint16_t temperature_adjust_plus_button_h = 50;
uint16_t temperature_adjust_plus_button_tx = 260;
uint16_t temperature_adjust_plus_button_ty = 130;

uint16_t back_to_status_screen_button_x = 30;
uint16_t back_to_status_screen_button_y = 180;
uint16_t back_to_status_screen_button_w = 100;
uint16_t back_to_status_screen_button_h = 50;
uint16_t back_to_status_screen_button_tx = 60;
uint16_t back_to_status_screen_button_ty = 210;

/**
 * Setup routine.
 */
void setup() {
  Serial.begin(9600);
  
  // I/O.
  pinMode(POWER_RELAY_PIN, OUTPUT);
  digitalWrite(POWER_RELAY_PIN, LOW);
  pinMode(TFT_RESET_PIN, OUTPUT);
  digitalWrite(TFT_RESET_PIN, HIGH); // active LOW
  pinMode(TFT_BACKLIGHT_PIN, OUTPUT); // TFT LED
  digitalWrite(TFT_BACKLIGHT_PIN, HIGH);

  // Initialise temperature sensor.
  ds18b20_init();

  // Initialise screen.
  ucg.begin(UCG_FONT_MODE_TRANSPARENT);
  ucg.setFont(font_small);
  ucg.setColor(255, 255, 255);

  // Initialise touch.
  touch.begin(ucg.getWidth(), ucg.getHeight());  // Must be done before setting rotation
  touch.setCalibration(248, 1728, 1776, 304); // Replace these for your screen module.

  // Set screen rotation.
  ucg.setRotate90();
  touch.setRotation(touch.ROT90);

  // Default PID terms.
  pid_kp = 1.0;
  pid_ki = 0.0;
  pid_kd = 12.0;
  pid_imin = -10.0;
  pid_imax = 10.0;
  pid_set_point = 42.0; // degrees (C)

  // Start first temperature read.
  ds18b20_start_read();

  // Set PID off by default.
  pid_enabled = false;

  // Show default screen.
  screen_is_status = true;

  draw_screen();
  redraw_screen = false;
}

/**
 * Loop routine.
 */
void loop() {
  uint32_t current_time = millis();

  if (!measurement_in_progress && (current_time - last_measurement_time) > (MEASUREMENT_INTERVAL - MEASUREMENT_TIME)) {
    ds18b20_start_read();
  }
  
  if (measurement_in_progress && (current_time - measurement_start_time) > MEASUREMENT_TIME) {
    ds18b20_finish_read();
    redraw_screen = true;

    if (pid_enabled) {
      // Update PID controller.
      pid_update();

      Serial.print(current_time);
      Serial.print(" ");
      Serial.print(current_temperature);
      Serial.print(" ");
      Serial.print(pid_p_error);
      Serial.print(" ");
      Serial.print(pid_i_error);
      Serial.print(" ");
      Serial.println(pid_d_error);
    }
  }
  
  if (touch.isTouching() && !touching) {    
    // Start of touch.
    touch.getPosition(touch_x, touch_y);
    touching = true;
    touching_since = millis();
  } else {
    if (touching) {
      // End of touch.
      touching = false;
      handle_touch();
    }

    touch_x = 0xffff;
    touch_y = 0xffff;
  }

  if (redraw_screen) {
    draw_screen();
    redraw_screen = false;
  }
}

bool button_bouncing() {
  return (touching_since - millis()) < BUTTON_DEBOUNCE_DELAY;
}

void set_power(bool on) {
  if (on != power_on) {
    redraw_screen = true;
  }
  
  digitalWrite(POWER_RELAY_PIN, on);
  power_on = on;
}

void pid_toggle() {  
  pid_enabled = !pid_enabled;

  if (!pid_enabled) {
    reset_pid_coefficients();
    // Switch off output.
    set_power(false);
  }

  redraw_screen = true;
}

void ds18b20_init() {  
  sensors.begin();
  // Set asynchronous conversion.
  sensors.setWaitForConversion(false);
  
  ds18b20_count = sensors.getDeviceCount();
  
  if (ds18b20_count > 0) {    
    // Get address.
    one_wire.search(ds18b20_address);
  }
}

/**
 * Start temperature reading. This starts an asynchronous temperature
 * measurement. The ds18b20_finish_read() function should be called to
 * retrieve the measurement after waiting ASYNC_DELAY.
 */
void ds18b20_start_read() {
  // Set the conversion resolution.
  sensors.setResolution(ds18b20_address, DS18B20_PRECISION);

  // Measure temperature.
  sensors.requestTemperatures();

  // Record time.
  measurement_start_time = millis();

  // Set flag.
  measurement_in_progress = true;
}

/**
 * Get temperature. Should be called after waiting ASYNC_DELAY after
 * calling ds18b20_start_read().
 */
void ds18b20_finish_read() {
  // Store previous temperature.
  previous_temperature = current_temperature;
  
  // Get temperature.
  current_temperature = sensors.getTempC(ds18b20_address);
  
  // Check if reading is within range.
  if ((current_temperature > 125.0) || (current_temperature < -55.0)) {
    // Out of range; set to default.
    current_temperature = DS18B20_DEFAULT;
  }

  // Record time.
  last_measurement_time = millis();

  // Set flag.
  measurement_in_progress = false;
}

float pid_update() {
  if (pid_correction(current_temperature) > 0) {
    // Below set point.
    set_power(true);
  } else {
    // Above set point.
    set_power(false);
  }
}

void reset_pid_coefficients() {
  pid_error = 0;
  pid_p_error = 0;
  pid_i_error = 0;
  pid_d_error = 0;
  pid_differentiator = 0;
  pid_integrator = 0;
}

float pid_correction(float value) {
  pid_error = pid_set_point - value;
  pid_p_error = pid_kp * pid_error;
  pid_d_error = pid_kd * (pid_error - pid_differentiator);

  // Update integrator and differentiator values.
  pid_differentiator = pid_error;
  pid_integrator += pid_error;

  if (pid_integrator > pid_imax) {
    pid_integrator = pid_imax;
  }
  
  if (pid_integrator < pid_imin) {
    pid_integrator = pid_imin;
  }

  pid_i_error = pid_ki * pid_integrator;

  // Calculate correction.
  return pid_p_error + pid_i_error + pid_d_error;
}

void show_status_screen() {
  screen_is_status = true;
  screen_is_adjust = false;
  redraw_screen = true;
}

void show_adjust_screen() {
  screen_is_status = false;
  screen_is_adjust = true;
  redraw_screen = true;
}

void draw_screen() {
  ucg.clearScreen();
  
  if (screen_is_status) {
    draw_status_screen();
  } else if (screen_is_adjust) {
    draw_adjust_screen();
  }
}

void draw_status_screen() {
  draw_temperature_box();
  draw_output_status_box();
  draw_pid_toggle_button();
  draw_adjust_screen_button();
}

void draw_adjust_screen() {
  draw_pid_coefficient_box();
  draw_temperature_adjust_widget();
  draw_back_to_status_screen_button();
}

void draw_temperature_box() {
  ucg.setColor(255, 255, 255);
  //ucg.drawBox(temperature_box_x, temperature_box_y, temperature_box_w, temperature_box_h);
  //ucg.setColor(0, 0, 0);
  ucg.setPrintPos(temperature_box_tx, temperature_box_ty);
  ucg.setFont(font_large);
  ucg.print(current_temperature);
  ucg.print(" C");

  if (current_temperature != previous_temperature) {
    // Draw temperature direction arrow.
    ucg.setColor(255, 0, 0);
    uint16_t x0, y0, x1, y1, x2, y2;
    if (current_temperature > previous_temperature) {
      // Temperature is increasing.
      x0 = temperature_box_triangle_x0;
      y0 = temperature_box_triangle_y0;
      x1 = temperature_box_triangle_x0 + temperature_box_triangle_base / 2.0;
      y1 = temperature_box_triangle_y0 - temperature_box_triangle_height;
      x2 = temperature_box_triangle_x0 + temperature_box_triangle_base;
      y2 = temperature_box_triangle_y0;
    } else {
      // Temperatue is decreasing.
      x0 = temperature_box_triangle_x0;
      y0 = temperature_box_triangle_y0 - temperature_box_triangle_height;
      x1 = temperature_box_triangle_x0 + temperature_box_triangle_base / 2.0;
      y1 = temperature_box_triangle_y0;
      x2 = temperature_box_triangle_x0 + temperature_box_triangle_base;
      y2 = temperature_box_triangle_y0 - temperature_box_triangle_height;
    }
    ucg.drawTriangle(x0, y0, x1, y1, x2, y2);
  }
}

void draw_output_status_box() {
  ucg.setColor(255, 255, 255);
  ucg.drawBox(output_status_box_x, output_status_box_y, output_status_box_w, output_status_box_h);
  ucg.setColor(0, 0, 0);
  ucg.setPrintPos(output_status_box_tx, output_status_box_ty);
  ucg.setFont(font_small);
  if (power_on) {
    ucg.print("On");
  } else {
    ucg.print("Off");
  }
}

void draw_pid_toggle_button() {
  ucg.setColor(255, 255, 255);
  ucg.drawBox(pid_toggle_button_x, pid_toggle_button_y, pid_toggle_button_w, pid_toggle_button_h);
  ucg.setColor(0, 0, 0);
  ucg.setPrintPos(pid_toggle_button_tx, pid_toggle_button_ty);
  ucg.setFont(font_small);
  if (pid_enabled) {
    ucg.print("Disable PID");
  } else {
    ucg.print("Enable PID");
  }
}

void check_pid_toggle_button_touched() {
  if (button_bouncing()) {
    return;
  }
  
  if (
    touch_x > pid_toggle_button_x &&
    touch_x < (pid_toggle_button_x + pid_toggle_button_w) &&
    touch_y > pid_toggle_button_y &&
    touch_y < (pid_toggle_button_y + pid_toggle_button_h)
  ) {
    // Touched.
    pid_toggle();
  }
}

void draw_pid_coefficient_box() {
  ucg.setColor(255, 255, 255);
  ucg.drawBox(pid_coefficient_box_x, pid_coefficient_box_y, pid_coefficient_box_w, pid_coefficient_box_h);
  ucg.setColor(0, 0, 0);
  ucg.setPrintPos(pid_coefficient_box_tx, pid_coefficient_box_ty);
  ucg.setFont(font_small);
  ucg.print("Ep: ");
  ucg.print(pid_p_error);
  ucg.print(" Ei: ");
  ucg.print(pid_i_error);
  ucg.print(" Ed: ");
  ucg.print(pid_d_error);
}

void draw_adjust_screen_button() {
  ucg.setColor(255, 255, 255);
  ucg.drawBox(adjust_screen_button_x,
              adjust_screen_button_y,
              adjust_screen_button_w,
              adjust_screen_button_h);
  ucg.setColor(0, 0, 0);
  ucg.setPrintPos(adjust_screen_button_tx, adjust_screen_button_ty);
  ucg.setFont(font_small);
  ucg.print("Settings");
}

void check_adjust_screen_button_touched() {
  if (button_bouncing()) {
    return;
  }
  
  if (
    touch_x > adjust_screen_button_x &&
    touch_x < (adjust_screen_button_x + adjust_screen_button_w) &&
    touch_y > adjust_screen_button_y &&
    touch_y < (adjust_screen_button_y + adjust_screen_button_h)
  ) {
    // Touched.
    show_adjust_screen();
  }
}

void draw_temperature_adjust_widget() {
  // Set point pane.
  ucg.setColor(255, 255, 255);
  ucg.drawBox(temperature_adjust_setpoint_widget_value_x,
              temperature_adjust_setpoint_widget_value_y,
              temperature_adjust_setpoint_widget_value_w,
              temperature_adjust_setpoint_widget_value_h);
  // Set point text.
  ucg.setColor(0, 0, 0);
  ucg.setPrintPos(temperature_adjust_setpoint_widget_value_tx,     
                  temperature_adjust_setpoint_widget_value_ty);
  ucg.setFont(font_small);
  ucg.print(pid_set_point);
  ucg.print(" C");
  // Minus button.
  ucg.setColor(255, 0, 0);
  ucg.drawBox(temperature_adjust_minus_button_x,
              temperature_adjust_minus_button_y,
              temperature_adjust_minus_button_w,
              temperature_adjust_minus_button_h);
  ucg.setPrintPos(temperature_adjust_minus_button_tx,
                  temperature_adjust_minus_button_ty);
  ucg.setFont(font_large);
  ucg.setColor(255, 255, 255);
  ucg.print("-");
  // Plus button.
  ucg.setColor(0, 255, 0);
  ucg.drawBox(temperature_adjust_plus_button_x,
              temperature_adjust_plus_button_y,
              temperature_adjust_plus_button_w,
              temperature_adjust_plus_button_h);
  ucg.setPrintPos(temperature_adjust_plus_button_tx,
                  temperature_adjust_plus_button_ty);
  ucg.setFont(font_large);
  ucg.setColor(255, 255, 255);
  ucg.print("+");
}

void draw_back_to_status_screen_button() {
  ucg.setColor(255, 255, 255);
  ucg.drawBox(back_to_status_screen_button_x,
              back_to_status_screen_button_y,
              back_to_status_screen_button_w,
              back_to_status_screen_button_h);
  ucg.setColor(0, 0, 0);
  ucg.setPrintPos(back_to_status_screen_button_tx, back_to_status_screen_button_ty);
  ucg.setFont(font_small);
  ucg.print("Back");
}

void check_plus_button_touched() {
  if (button_bouncing()) {
    return;
  }
  
  if (
    touch_x > temperature_adjust_plus_button_x &&
    touch_x < (temperature_adjust_plus_button_x + temperature_adjust_plus_button_w) &&
    touch_y > temperature_adjust_plus_button_y &&
    touch_y < (temperature_adjust_plus_button_y + temperature_adjust_plus_button_h)
  ) {
    // Touched.
    pid_set_point += 0.5;
    redraw_screen = true;
  }
}

void check_minus_button_touched() {
  if (button_bouncing()) {
    return;
  }
  
  if (
    touch_x > temperature_adjust_minus_button_x &&
    touch_x < (temperature_adjust_minus_button_x + temperature_adjust_minus_button_w) &&
    touch_y > temperature_adjust_minus_button_y &&
    touch_y < (temperature_adjust_minus_button_y + temperature_adjust_minus_button_h)
  ) {
    // Touched.
    pid_set_point -= 0.5;
    redraw_screen = true;
  }
}

void check_back_to_status_screen_button_touched() {
  if (button_bouncing()) {
    return;
  }
  
  if (
    touch_x > back_to_status_screen_button_x &&
    touch_x < (back_to_status_screen_button_x + back_to_status_screen_button_w) &&
    touch_y > back_to_status_screen_button_y &&
    touch_y < (back_to_status_screen_button_y + back_to_status_screen_button_h)
  ) {
    // Touched.
    show_status_screen();
  }
}

/**
 * Handle recent touch.
 */
void handle_touch() {
  if (screen_is_status) {
    handle_status_screen_touch();
  } else if (screen_is_adjust) {
    handle_adjust_screen_touch();
  }
}

void handle_status_screen_touch() {
  check_pid_toggle_button_touched();
  check_adjust_screen_button_touched();
}

void handle_adjust_screen_touch() {
  check_plus_button_touched();
  check_minus_button_touched();
  check_back_to_status_screen_button_touched();
}
