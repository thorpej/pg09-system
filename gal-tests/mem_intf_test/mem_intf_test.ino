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
 * Memory interface signal generator tester for 6809 Playground.
 *
 * This code is intended to run on an Arduino Mega 2560.
 */

/* Pins used for outputs. */
#define OUT_E         14
#define OUT_RW        15
#define OUT_BA        16
#define OUT_BS        17
#define OUT_FLASHWREN 18
#define OUT_A1        A1
#define OUT_A2        A2
#define OUT_A3        A3
#define OUT_A4        A4
#define ADDR_SHIFT    1

const int addr_pins[] = {
  OUT_A1, OUT_A2, OUT_A3, OUT_A4, -1,
};

const int out_pins[] = {
  OUT_E, OUT_RW, OUT_BA, OUT_BS, OUT_FLASHWREN,
  OUT_A1, OUT_A2, OUT_A3, OUT_A4, -1,
};

/* Pins used for inputs. */
#define IN_RD       22
#define IN_WR       23
#define IN_HALTED   24
#define IN_IRQF     25
#define IN_FIRQF    26
#define IN_FLASHWR  27
#define IN_RWO      28
#define IN_nA2      29
#define IN_nA3      30
#define IN_nA4      31

struct signal_desc {
  const char *name;
  int         pin;
};

#define RD        0
#define WR        1
#define HALTED    2
#define IRQF      3
#define FIRQF     4
#define FLASHWR   5
#define RWO       6
#define nA2       7
#define nA3       8
#define nA4       9

const struct signal_desc signal_map[] = {
[RD]      = { .name = "/RD",      .pin = IN_RD },
[WR]      = { .name = "/WR",      .pin = IN_WR },
[HALTED]  = { .name = "HALTED",   .pin = IN_HALTED },
[IRQF]    = { .name = "IRQF",     .pin = IN_IRQF },
[FIRQF]   = { .name = "FIRQF",    .pin = IN_FIRQF },
[FLASHWR] = { .name = "/FLASHWR", .pin = IN_FLASHWR },
[RWO]     = { .name = "RWO",      .pin = IN_RWO },
[nA2]     = { .name = "nA2",      .pin = IN_nA2 },
[nA3]     = { .name = "nA3",      .pin = IN_nA3 },
[nA4]     = { .name = "nA4",      .pin = IN_nA4 },
};
#define signal_map_count (sizeof(signal_map) / sizeof(signal_map[0]))

/* Lifted from my TeensyLogicAnalyzer project. */
extern "C" {
  __attribute__((__format__(__printf__, 1, 2)))
  int
  tla_printf(const char *fmt, ...) 
  {
    static char tla_printf_buf[256];
    va_list ap;
    int rv;

    va_start(ap, fmt);
    rv = vsnprintf(tla_printf_buf, sizeof(tla_printf_buf), fmt, ap);
    va_end(ap);

    Serial.print(tla_printf_buf);
        
    return rv;
  }
}

bool
signal_is_active(const struct signal_desc *s)
{
  bool active = digitalRead(s->pin);
  if (s->name[0] == '/') {
    /* Invert active-low signals. */
    active = !active;
  }
  return active;
}

void
init_pins(void)
{
  int i;

  for (i = 0; out_pins[i] != -1; i++) {
    pinMode(out_pins[i], OUTPUT);
    digitalWrite(out_pins[i], 0);
  }

  for (i = 0; i < signal_map_count; i++) {
    pinMode(signal_map[i].pin, INPUT);
  }
}

void
set_address(uint16_t address)
{
  int i;

  address >>= ADDR_SHIFT;

  for (i = 0; addr_pins[i] != -1; i++) {
    digitalWrite(addr_pins[i], address & 1);
    address >>= 1;
  }
}

int error_count;

bool
check_output(int sig, bool expected)
{
  int i;
  bool asserted;
  bool error = false;

  const struct signal_desc *s = &signal_map[sig];
  asserted = signal_is_active(s);
 
  if (expected != asserted) {
    if (!asserted) {
      tla_printf("   !!! %s -- NOT ASSERTED AS EXPECTED\r\n", s->name);
        error = true;
    }
  } else {
    if (asserted) {
      tla_printf("   !!! %s -- ASSERTED WHEN NOT EXPECTED\r\n", s->name);
      error = true;
    }
  }
  if (error) {
    error_count++;
  }
  return error;
}

void
setup()
{
  /* Initialize the pins. */
  init_pins();

  Serial.begin(115200);
  while (!Serial) {
    ; /* wait for serial port to connect. Needed for native USB. */
  }
  Serial.setTimeout(60000);
}

const char *
tf(bool val)
{
  return val ? "true" : "false";
}

void
loop()
{
  int i;

  tla_printf("6809 Playground MEMINTF signal tester.\r\n");
  tla_printf("Press <RETURN> to perform the test...\r\n");
  do {
    i = Serial.read();
  } while (i != '\r' && i != '\r\n');

  error_count = 0;

  /*
   * Test the /RD, /WR, and /FLASHWR strobes.
   */
  for (i = 0; i < 8; i++) {
    bool e = !!(i & 1);
    bool rw = !!(i & 2);
    bool flashwren = !!(i & 4);

    bool rd = e && rw;
    bool wr = e && !rw;
    bool flashwr = e && !rw && flashwren;

    tla_printf("E=%d R/W=%d FLASHWREN=%d -> /RD=%s /WR=%s /FLASHWR=%s\r\n",
               e, rw, flashwren,
               tf(rd), tf(wr), tf(flashwr));
    digitalWrite(OUT_E, e);
    digitalWrite(OUT_RW, rw);
    digitalWrite(OUT_FLASHWREN, flashwren);
    check_output(RD, rd);
    check_output(WR, wr);
    check_output(FLASHWR, flashwr);
    check_output(RWO, rw);
  }

  /*
   * Validate HALTED is only asserted when both BA=1 and
   * BS=1.
   */
  for (i = 0; i < 4; i++) {
    bool ba = !!(i & 1);
    bool bs = !!(i & 2);
    bool halted = ba && bs;

    tla_printf("BA=%d BS=%d -> HALTED=%s\r\n", ba, bs, tf(halted));
    digitalWrite(OUT_BA, ba);
    digitalWrite(OUT_BS, bs);
    check_output(HALTED, halted);
  }

  /*
   * Check IRQF and FIRQF signals.
   */
  for (i = 0; i < 4; i++) {
    bool ba = !!(i & 1);
    bool bs = !!(i & 2);
    bool intr_ack = !ba & bs;

    digitalWrite(OUT_BA, ba);
    digitalWrite(OUT_BS, bs);

    for (uint16_t address = 0xfff0; address != 0; address++) {
      bool firq_fetch = intr_ack && (address == 0xFFF6 || address == 0xFFF7);
      bool irq_fetch  = intr_ack && (address == 0xFFF8 || address == 0xFFF9);
      set_address(address);

      tla_printf("BA=%d BS=%d addr=$%04X -> IRQF=%s\r\n",
                 ba, bs, address, tf(irq_fetch));
      check_output(IRQF, irq_fetch);

      tla_printf("BA=%d BS=%d addr=$%04X -> FIRQF=%s\r\n",
                 ba, bs, address, tf(firq_fetch));
      check_output(FIRQF, firq_fetch);
    }
  }

  /*
   * Check the nAx signals.
   */
  for (uint16_t address = 0; address < 32; address++) {
    bool na2 = !(address & (1U << 2));
    bool a2 = !na2;
    bool na3 = !(address & (1U << 2));
    bool a3 = !na3;
    bool na4 = !(address & (1U << 2));
    bool a4 = !na4;

    set_address(address);

    tla_printf("A2=%s -> nA2=%s\r\n", tf(a2), tf(na2));
    check_output(nA2, na2);
    tla_printf("A3=%s -> nA3=%s\r\n", tf(a3), tf(na3));
    check_output(nA3, na3);
    tla_printf("A4=%s -> nA4=%s\r\n", tf(a4), tf(na4));
    check_output(nA4, na4);
  }

  if (error_count != 0) {
    tla_printf("%d ERRORS DETECTED!\r\n", error_count);
  } else {
    tla_printf("Passed!\r\n");
  }
}
