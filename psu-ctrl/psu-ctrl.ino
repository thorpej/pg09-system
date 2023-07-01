/*-
 * Copyright (c) 2023 Jason R. Thorpe.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * ATX power supply controller firmware for the 6809 Playground
 * computer.
 *
 * This handles the soft power button, and also implements software
 * reset and power-down control from the 6809 via an I/O port on one
 * of the system's 65C21s (the banked ROM PIA has port B free).
 *
 * 4 input pins are used:
 * - Power button (momentary on for power-on, hold for 3 seconds
 *   for power-off).
 * - A "host request" pin that notifies the microcontroller that
 *   the 6809 wants something.
 * - Reset request from 6809 (gated by host request).
 * - Power-off request from 6809 (gated by host request).
 *
 * 2 output pins are used:
 * - Drive #PSU_ON low on the ATX power supply.
 * - Drive #RESET low on the 6809.
 *
 * The output pins are configured as INPUT until they're ready to drive
 * their respective signals.
 *
 * This is targeted at the Adafruit Metro Mini, which is software-compatible
 * with Arduino UNO.  We're using AVR-specific libraries here, and assuming
 * ATMega328.
 */

#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>

#define POWER_BUTTON_PIN    2   /* uses interrupt */
#define HOST_REQ_PIN        3   /* uses interrupt */
#define RESET_REQ_PIN       4   /* paired with HOST_REQ_PIN */
#define POWEROFF_REQ_PIN    5   /* paired with HOST_REQ_PIN */

#define PSU_ON_PIN          8   /* not a PWM pin */
#define RST_OUT_PIN         12  /* not a PWM pin */
#define POWER_LED_PIN       13  /* system LED pin */

/*
 * We de-bounce input signals for this long, and consider
 * them asserted if they're asserted for 80% of that time.
 */
#define INPUT_DEBOUNCE_DURATION 100   /* milliseconds */
#define INPUT_DEBOUNCE_THRESH   (INPUT_DEBOUNCE_DURATION - ((INPUT_DEBOUNCE_DURATION / 10) * 2))
#define POWER_LED_TOGGLE_TICKS  ((1000 / INPUT_DEBOUNCE_DURATION) / 2)

#define BUTTON_DOWN_TIME        3000  /* milliseconds */
#define BUTTON_DOWN_STATES      (BUTTON_DOWN_TIME / INPUT_DEBOUNCE_DURATION)

#define STATE_OFF               0
#define STATE_ON                1
#define STATE_ON_BUTTON_DOWN(x) (2 + (x))
#define STATE_POWER_OFF         STATE_ON_BUTTON_DOWN(BUTTON_DOWN_STATES)

#define POWER_BUTTON_INPUT      0
#define HOST_REQ_INPUT          1
#define RESET_REQ_INPUT         2
#define POWEROFF_REQ_INPUT      3
#define INPUT_COUNT             4
int inputs[INPUT_COUNT];
int input_pins[] = {
  [POWER_BUTTON_INPUT]          = POWER_BUTTON_PIN,
  [HOST_REQ_INPUT]              = HOST_REQ_PIN,
  [RESET_REQ_INPUT]             = RESET_REQ_PIN,
  [POWEROFF_REQ_INPUT]          = POWEROFF_REQ_PIN,
};
int input_levels[] = {
  [POWER_BUTTON_INPUT]          = LOW,
  [HOST_REQ_INPUT]              = HIGH,
  [RESET_REQ_INPUT]             = HIGH,
  [POWEROFF_REQ_INPUT]          = HIGH,
};

#define power_button            inputs[POWER_BUTTON_INPUT]
#define reset_req               inputs[RESET_REQ_INPUT]
#define poweroff_req            inputs[POWEROFF_REQ_INPUT]

static volatile bool button_edge_detected;
static volatile bool host_req_edge_detected;
static volatile int state;

#define Info(x)   do { if (Serial) { Serial.println(x); } } while (0)
#define Debug(x)  do { if (Serial) { Serial.print("DEBUG: "); Serial.println(x); } } while (0)

/*
 * setup_pins --
 *
 * Set the configuraiton of the input and output pins.
 */
static void
setup_pins(void)
{
  /* Input pins. */
  pinMode(POWER_BUTTON_PIN, INPUT);
  pinMode(HOST_REQ_PIN, INPUT);
  pinMode(RESET_REQ_PIN, INPUT);
  pinMode(POWEROFF_REQ_PIN, INPUT);

  /* Output pins. Inputs until we drive them. */
  pinMode(PSU_ON_PIN, INPUT);
  pinMode(RST_OUT_PIN, INPUT);

  /* Regular output pins. */
  digitalWrite(POWER_LED_PIN, LOW);
  pinMode(POWER_LED_PIN, OUTPUT);
}

/*
 * set_power_led --
 *
 * Set the state of the power LED.
 */
static void
set_power_led(bool enable)
{
  digitalWrite(POWER_LED_PIN, enable);
}

/*
 * toggle_power_led --
 *
 * Toggle the state of the power LED.
 */
static void
toggle_power_led(void)
{
  digitalWrite(POWER_LED_PIN, !digitalRead(POWER_LED_PIN));
}

/*
 * set_output_signal --
 *
 * Set the state of an output signal.  enabled == true means assert
 * the signal, regardless of its output level.
 */
static void
set_output_signal(int pin, bool enable)
{
  if (enable) {
    digitalWrite(PSU_ON_PIN, LOW);
    pinMode(PSU_ON_PIN, OUTPUT);
  } else {
    pinMode(PSU_ON_PIN, INPUT);
  }
}

/*
 * set_psu_on --
 *
 * Set the #PSU_ON state.
 */
static int
set_psu_on(bool enable)
{
  set_output_signal(PSU_ON_PIN, enable);
  set_power_led(enable);
  return enable ? STATE_ON : STATE_OFF;
}

/*
 * set_rst_out --
 *
 * Set the #RESET state.
 */
static void
set_rst_out(bool enable)
{
  set_output_signal(RST_OUT_PIN, enable);
}

/*
 * sample_inputs --
 *
 * Sample the input signals.
 */
static bool
sample_inputs(bool require_button_edge)
{
  int tick, i;

  cli();
  bool button_edge = button_edge_detected;
  button_edge_detected = false;
  bool host_req_edge = host_req_edge_detected;
  host_req_edge_detected = false;
  sei();

  memset(inputs, 0, sizeof(inputs));

  for (tick = 0; tick < INPUT_DEBOUNCE_DURATION; tick++) {
    for (i = 0; i < INPUT_COUNT; i++) {
      inputs[i] += (digitalRead(input_pins[i]) == input_levels[i]);
    }
    delay(1);
  }

  for (i = 0; i < INPUT_COUNT; i++) {
    if (inputs[i] < INPUT_DEBOUNCE_THRESH) {
      inputs[i] = 0;
    }
  }

  if (require_button_edge && ! button_edge) {
    power_button = 0;
  }
  if (! inputs[HOST_REQ_INPUT]) {
    reset_req = poweroff_req = 0;
  }
  return power_button || reset_req || poweroff_req;
}

/*
 * reset_system --
 *
 * Reset the host system.
 */
static void
reset_system(void)
{
  /*
   * Assert #RESET for 250ms, then de-assert #RESET.
   */
  set_rst_out(true);
  delay(250);
  set_rst_out(false);
}

/*
 * button_isr / host_req_isr --
 *
 * Interrupt service routines that just say "Yup, interrupt fired!",
 * which is used to signal wait_for_request() to not go into power
 * saving mode.
 */
static void
button_isr(void)
{
  button_edge_detected = true;
}

static void
host_req_isr(void)
{
  host_req_edge_detected = true;
}

#ifdef SLEEP_MODE_PWR_SAVE
#define SNOOZE_MODE       SLEEP_MODE_PWR_SAVE
#else
#define SNOOZE_MODE       SLEEP_MODE_PWR_IDLE
#endif

/*
 * snooze --
 *
 * Snooze in reduced power state until an interrupt wakes us up.
 */
static void
snooze(void)
{
  set_sleep_mode(SNOOZE_MODE);
  cli();
  if (! (button_edge_detected || host_req_edge_detected)) {
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();
  }
  sei();
}

/*
 * wait_for_request --
 *
 * Wait for a request to be sent.
 */
 static void
wait_for_request(void)
{
 while (! sample_inputs(true)) {
   snooze();
 }
}

static int
state_OFF(void)
{
  /*
   * In the OFF state, the only input that makes sense is the
   * power button.  If we get that trigger at all, we transition
   * to ON state.
   */
  Debug("[OFF] Waiting for inputs...");
  wait_for_request();
  if (power_button) {
    Debug("[OFF] power_button -> ON");
    return set_psu_on(true);
  }
  Debug("[OFF] -> OFF");
  return STATE_OFF;
}

static int
state_ON(void)
{
  /*
   * In the ON state, we're waiting for requests from the host
   * or the power button.
   */
  Debug("[ON] Waiting for inputs...");
  wait_for_request();
  if (poweroff_req) {
    Debug("[ON] poweroff_req -> OFF");
    return set_psu_on(false);
  }
  if (reset_req) {
    Debug("[ON] reset_req");
    reset_system();
  }
  if (power_button) {
    Debug("[ON] power_button -> ON_BUTTON_DOWN");
    return STATE_ON_BUTTON_DOWN(0);
  }
  Debug("[ON] -> ON");
  return STATE_ON;
}

static int
state_ON_BUTTON_DOWN(void)
{
  /*
   * In the ON_BUTTON_DOWN state, we don't wait for requests,
   * we just sample them.  We're using the sampling time as
   * part of the 3 second countdown sequence.
   */
  /* Don't log here -- it'll be spammy. */
  sample_inputs(false);
  if (poweroff_req) {
    Debug("[ON_BUTTON_DOWN] poweroff_req -> OFF");
    return set_psu_on(false);
  }
  if (reset_req) {
    Debug("[ON_BUTTON_DOWN] reset_req");
    reset_system();
  }
  if (power_button) {
    /* Don't log here -- it'll be spammy. */
    int next_state = state + 1;
    if (next_state == STATE_POWER_OFF) {
      Debug("[ON_BUTTON_DOWN] power_button -> POWER_OFF");
    }
    if (((state - STATE_ON_BUTTON_DOWN(0)) % POWER_LED_TOGGLE_TICKS) == 0) {
      toggle_power_led();
    }
    return next_state;
  }
  Debug("[ON_BUTTON_DOWN] -> ON");
  set_power_led(true);
  return STATE_ON;
}

static int
state_POWER_OFF(void)
{
  Debug("[POWER_OFF] -> OFF");
  return set_psu_on(false);
}

static int
state_recover(void)
{
  /*
   * We lost our mind, so try to get back on track.
   */
  if (digitalRead(PSU_ON_PIN)) {
    Debug("[RECOVER] -> OFF");
    return state_OFF;
  }
  Debug("[RECOVER] -> ON");
  return state_ON;
}

void
setup(void)
{
  Serial.begin(115200);

  Info("");
  Info(">>> 6809 Playground PSU controller 0.1");
  Info(">>> Copyright (c) 2023 Jason R. Thorpe");
  Info("");

  Debug("Initializing pins.");
  setup_pins();

  Debug("Initializing interrupts.");
  attachInterrupt(digitalPinToInterrupt(POWER_BUTTON_PIN), button_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(HOST_REQ_PIN), host_req_isr, RISING);

  /* power down a bunch of microcontroller blocks we don't need. */
  Debug("Disabling unneeded functional blocks.");
  power_adc_disable();
  power_spi_disable();
  power_twi_disable();
}

void
loop(void)
{
  int next_state;

  if (state == STATE_OFF) {
    next_state = state_OFF();
  } else if (state == STATE_ON) {
    next_state = state_ON();
  } else if (state >= STATE_ON_BUTTON_DOWN(0) && state < STATE_ON_BUTTON_DOWN(BUTTON_DOWN_STATES)) {
    next_state = state_ON_BUTTON_DOWN();
  } else if (state == STATE_POWER_OFF) {
    next_state = state_POWER_OFF();
  } else {
    next_state = state_recover();
  }
  state = next_state;
}