/*-
 * Copyright (c) 2023, 2025 Jason R. Thorpe.
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
 * reset and power-down control from the 6809.
 *
 * 4 input pins are used:
 * - Power button (momentary on for power-on, hold for 3 seconds
 *   for power-off), triggered on the falling edge.
 * - A "host request" pin that notifies the microcontroller that
 *   the 6809 wants something, triggered on the rising edge.
 * - A 2-bit host request value, held in a latch between the controller
 *   and the 6809 data bus.
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

//#define CONFIG_ATTINY
#define DEBUG

#ifdef CONFIG_ATTINY
//
// From:
// https://github.com/damellis/attiny/blob/master/variants/tiny8/pins_arduino.h
//
// ATMEL ATTINY45/85 / ARDUINO
//
//                  +-\/-+
// Ain0 (D 5) PB5  1|    |8  Vcc
// Ain3 (D 3) PB3  2|    |7  PB2 (D 2)  Ain1
// Ain2 (D 4) PB4  3|    |6  PB1 (D 1) pwm1
//            GND  4|    |5  PB0 (D 0) pwm0
//                  +----+
//

// Inputs
#define POWER_BUTTON_PIN    2   /* uses interrupt */
#define HOST_REQ_PIN        1   /* uses interrupt */
#define HOST_REQ_BIT0_PIN   0

// Outputs
#define PSU_ON_PIN          3   /* not a PWM pin */
#define RST_OUT_PIN         5   /* not a PWM pin */
#define POWER_LED_PIN       4

//
// On an ATtiny85, the **physical** pins are thus:
//
// 1 /RESET out                 (RST_OUT_PIN)
// 2 /PSU_ON out                (PSU_ON_PIN)
// 3 POWERLED out (active-high) (POWER_LED_PIN)
// 4 GND
// 5 D0 in                      (HOST_REQ_BIT0_PIN)
// 6 /CS in                     (HOST_REQ)
// 7 /BUTTON in                 (POWER_BUTTON_PIN)
// 8 Vcc
//

#else /* ! CONFIG_ATTINY */

// Inputs
#define POWER_BUTTON_PIN    2   /* uses interrupt */
#define HOST_REQ_PIN        3   /* uses interrupt */
#define HOST_REQ_BIT0_PIN   4
#define HOST_REQ_BIT1_PIN   5

// Outputs
#define PSU_ON_PIN          8   /* not a PWM pin */
#define RST_OUT_PIN         12  /* not a PWM pin */
#define POWER_LED_PIN       LED_BUILTIN
#endif /* CONFIG_ATTINY */

/* Host commands. */
#define HOST_REQ_POWEROFF   0
#define HOST_REQ_RESET      1

/*
 * We de-bounce the power button for this long, and consider
 * it asserted if it's asserted for 80% of that time.
 */
#define INPUT_DEBOUNCE_DURATION 100   /* milliseconds */
#define INPUT_DEBOUNCE_THRESH   (INPUT_DEBOUNCE_DURATION - ((INPUT_DEBOUNCE_DURATION / 10) * 2))
#define POWER_LED_TOGGLE_TICKS  ((1000 / INPUT_DEBOUNCE_DURATION) / 2)

#define BUTTON_DOWN_TIME        3000  /* milliseconds */
#define BUTTON_DOWN_STATES      (BUTTON_DOWN_TIME / INPUT_DEBOUNCE_DURATION)

#define STATE_no_change             -1
#define STATE_INITIAL               0
#define STATE_OFF                   (STATE_INITIAL + 1)
#define STATE_ON                    (STATE_OFF + 1)
#define STATE_ON_BUTTON_DOWN(x)     (STATE_ON + 1 + (x))
#define STATE_ON_BUTTON_DOWN_FIRST  STATE_ON_BUTTON_DOWN(0)
#define STATE_ON_BUTTON_DOWN_LAST   STATE_ON_BUTTON_DOWN(BUTTON_DOWN_STATES - 1)
#define STATE_POWER_OFF             (STATE_ON_BUTTON_DOWN_LAST + 1)

#ifndef CONFIG_ATTINY
static const char *
state_name(int s)
{
  switch (s) {
    case STATE_OFF:
      return "OFF";

    case STATE_ON:
      return "ON";

    case STATE_ON_BUTTON_DOWN_FIRST ... STATE_ON_BUTTON_DOWN_LAST:
      return "ON_BUTTON_DOWN";

    case STATE_POWER_OFF:
      return "POWER_OFF";

    default:
      return "???";
  }
}
#endif /* ! CONFIG_ATTINY */

#ifndef CONFIG_ATTINY
static volatile bool button_edge_detected;
static volatile bool host_req_edge_detected;
#endif

static bool host_req_valid;
static bool host_req_command;
static int power_button;
static int state;

static void
print_state(int s, bool brackets)
{
#ifndef CONFIG_ATTINY
  if (Serial && s != STATE_INITIAL) {
    if (brackets) {
      Serial.print("[");
    }
    Serial.print(state_name(s));
    if (brackets) {
      Serial.print("] ");
    }
  }
#endif /* ! CONFIG_ATTINY */
}

static bool
print_current_state(void)
{
  print_state(state, true);
}

#ifndef CONFIG_ATTINY
static void
Info_internal(const char *str)
{
  if (Serial) {
    print_current_state();
    Serial.println(str);
  }
}

#define Info(x)   Info_internal(x)
#else
#define Info(x)   /* nothing */
#endif /* ! CONFIG_ATTINY */

#if defined(DEBUG) && !defined(CONFIG_ATTINY)
static void
Debug_internal(const char *str)
{
  if (Serial) {
    print_current_state();
    Serial.print("DEBUG: ");
    Serial.println(str);
  }
}

#define Debug(x)  Debug_internal(x)
#else
#define Debug(x)  /* nothing */
#endif /* DEBUG && ! CONFIG_ATTINY */

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
 * Set the state of an "open drain" output signal.  enabled == true means
 * assert the signal by pulling it low; false means let the output go back
 * to Vcc via external pull-up.
 */
static void
set_output_signal(int pin, bool enable)
{
  if (enable) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  } else {
    pinMode(pin, INPUT);
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
 * host_req --
 *
 * Check if the host requested the specified operation.
 */
static bool
host_req(int req)
{
  if (! host_req_valid) {
    return false;
  }
  return req == host_req_command;
}

/*
 * sample_inputs --
 *
 * Sample the input signals.
 */
static bool
sample_inputs(bool require_button_edge)
{
  int tick;

  cli();

#ifdef CONFIG_ATTINY
  /*
   * On ATtiny, we can't detect a specific edge, on a
   * specific pin, only that at least one of the pc-enalbed
   * pins has changed.
   *
   * If we had edge-detection, we'd be latching on the
   * rising edge.  But in this case, we just have to
   * hope that any set-up time has been met on the data
   * pins.
   */
  host_req_valid = digitalRead(HOST_REQ_PIN) == LOW;
#else /* ! CONFIG_ATTINY */
  bool button_edge = button_edge_detected;
  button_edge_detected = false;

  /*
   * The host request is held stable in a latch because
   * it'll be gone from the 6809's data bus by the time
   * we get to it.  We don't read the pins in the ISR
   * since the propagation delay through the latch might
   * mean the data isn't ready for us to sample in the
   * ISR.
   */
  host_req_valid = host_req_edge_detected;
  host_req_edge_detected = false;
#endif /* CONFIG_ATTINY */

  host_req_command =
      (digitalRead(HOST_REQ_BIT0_PIN) ? 0x01 : 0) |
      (
#if defined(HOST_REQ_BIT1_PIN)
      digitalRead(HOST_REQ_BIT1_PIN) ? 0x02 :
#endif
                                               0);
  sei();

  power_button = 0;

  for (tick = 0; tick < INPUT_DEBOUNCE_DURATION; tick++) {
    power_button += (digitalRead(POWER_BUTTON_PIN) == LOW);
    delay(1);
  }

  if (power_button < INPUT_DEBOUNCE_THRESH) {
    power_button = 0;
  }
#if !defined(CONFIG_ATTINY)
  if (require_button_edge && ! button_edge) {
    power_button = 0;
  }
#endif
  return power_button || host_req_valid;
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

#if defined(CONFIG_ATTINY)
/*
 * The ATtiny's pin change interrupts are limited
 * compared to the ATmega.  We have to do a little
 * extra work in software.
 */
ISR(PCINT0_vect)
{
  /*
   * We're now awake and disable pin-change interrupts
   * until we go back to sleep.
   */
  GIFR |= bit(PCIF);
  GIMSK &= ~bit(PCIE);
}
#else /* ! CONFIG_ATTINY */
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
#endif /* CONFIG_ATTINY */

#if defined(SLEEP_MODE_PWR_SAVE)
#define SNOOZE_MODE       SLEEP_MODE_PWR_SAVE
#elif defined(SLEEP_MODE_PWR_IDLE)
#define SNOOZE_MODE       SLEEP_MODE_PWR_IDLE
#elif defined(SLEEP_MODE_IDLE)
#define SNOOZE_MODE       SLEEP_MODE_IDLE
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
#if defined(CONFIG_ATTINY)
  bool have_input = digitalRead(POWER_BUTTON_PIN) == LOW ||
                    digitalRead(HOST_REQ_PIN) == LOW;
#else
  bool have_input = button_edge_detected || host_req_edge_detected;
#endif /* CONFIG_ATTINY */
  if (! have_input) {
#if defined(CONFIG_ATTINY)
    /* Re-enable pin change interrupts. */
    GIFR |= bit(PCIF);
    GIMSK &= ~bit(PCIE);
#endif
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
  Info("Waiting for inputs...");
  wait_for_request();
  if (power_button) {
    Debug("power_button");
    return set_psu_on(true);
  }
  return STATE_no_change;
}

static int
state_ON(void)
{
  /*
   * In the ON state, we're waiting for requests from the host
   * or the power button.
   */
  Info("Waiting for inputs...");
  wait_for_request();
  if (host_req(HOST_REQ_POWEROFF)) {
    Debug("HOST_REQ_POWEROFF");
    return set_psu_on(false);
  }
  if (host_req(HOST_REQ_RESET)) {
    Debug("HOST_REQ_RESET");
    reset_system();
  }
  if (power_button) {
    Debug("power_button");
    return STATE_ON_BUTTON_DOWN_FIRST;
  }
  return STATE_no_change;
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
  if (host_req(HOST_REQ_POWEROFF)) {
    Debug("HOST_REQ_POWEROFF");
    return set_psu_on(false);
  }
  if (host_req(HOST_REQ_RESET)) {
    Debug("HOST_REQ_RESET");
    reset_system();
  }
  if (power_button) {
    /* Don't log here -- it'll be spammy. */
    int next_state = state + 1;
    if (next_state == STATE_POWER_OFF) {
      Debug("power_button held");
    }
    if (((state - STATE_ON_BUTTON_DOWN_FIRST) % POWER_LED_TOGGLE_TICKS) == 0) {
      toggle_power_led();
    }
    return next_state;
  }
  Debug("power_button released");
  set_power_led(true);
  return STATE_ON;
}

static int
state_POWER_OFF(void)
{
  return set_psu_on(false);
}

static int
state_recover(void)
{
  /*
   * We lost our mind, so try to get back on track.
   */
  if (digitalRead(PSU_ON_PIN)) {
    Debug("RECOVERY: #PSU_ON is high");
    return state_OFF;
  }
  Debug("RECOVERY: #PSU_ON is low");
  return state_ON;
}

#define VERSION "1.2"

void
setup(void)
{
#if !defined(CONFIG_ATTINY)
  Serial.begin(115200);
#endif /* ! CONFIG_ATTINY */

  Info("");
  Info(">>> 6809 Playground PSU controller " VERSION);
  Info(">>> Copyright (c) 2023, 2025 Jason R. Thorpe");
  Info("");

  Info("Initializing pins.");
  /* Input pins. */
  pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(HOST_REQ_PIN, INPUT_PULLUP);
  pinMode(HOST_REQ_BIT0_PIN, INPUT_PULLUP);
#if defined(HOST_REQ_BIT1_PIN)
  pinMode(HOST_REQ_BIT1_PIN, INPUT_PULLUP);
#endif

  /*
   * Output pins. Inputs until we drive them.  They
   * are externally pulled-up.
   */
  pinMode(PSU_ON_PIN, INPUT);
  pinMode(RST_OUT_PIN, INPUT);

  /* Regular output pins. */
  digitalWrite(POWER_LED_PIN, LOW);
  pinMode(POWER_LED_PIN, OUTPUT);

  Info("Initializing interrupts.");
  cli();
#if defined(CONFIG_ATTINY)
  PCMSK = bit(POWER_BUTTON_PIN) | bit(HOST_REQ_PIN);
  GIFR |= bit(PCIF);
  GIMSK |= bit(PCIE);
#else
  /*
   * Falling-edge of power button because that's just a human.
   * Rising-edge of host-req pin because we're latching a value
   * That's been placed on the data bus and the host-req pin is
   * connected to the write-strobe output from the CPU.
   */
  attachInterrupt(digitalPinToInterrupt(POWER_BUTTON_PIN), button_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(HOST_REQ_PIN), host_req_isr, RISING);
#endif
  sei();

  /* power down a bunch of microcontroller blocks we don't need. */
  Info("Disabling unneeded functional blocks.");
  power_adc_disable();
#ifndef CONFIG_ATTINY
  power_spi_disable();
  power_twi_disable();
#endif

  /* And we're off! */
  state = STATE_OFF;
}

static bool
quiet_state_change(int next_state)
{
  if (next_state >= STATE_ON_BUTTON_DOWN_FIRST + 1 &&
      next_state <= STATE_ON_BUTTON_DOWN_LAST) {
    return true;
  }
  return false;
}

void
loop(void)
{
  int next_state;

  switch (state) {
    case STATE_OFF:
      next_state = state_OFF();
      break;

    case STATE_ON:
      next_state = state_ON();
      break;

    case STATE_ON_BUTTON_DOWN_FIRST ... STATE_ON_BUTTON_DOWN_LAST:
      next_state = state_ON_BUTTON_DOWN();
      break;

      case STATE_POWER_OFF:
      next_state = state_POWER_OFF();
      break;

    default:
      next_state = state_recover();
      break;
  }
  if (next_state != STATE_no_change) {
#if !defined(CONFIG_ATTINY)
    if (Serial && !quiet_state_change(next_state)) {
      print_current_state();
      Serial.print("New state -> ");
      print_state(next_state, false);
      Serial.println("");
    }
#endif /* CONFIG_ATTINY */
    state = next_state;
  }
}
