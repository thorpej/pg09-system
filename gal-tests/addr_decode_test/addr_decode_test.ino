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
 * Address decoder tester for 6809 Playground.
 *
 * This code is intended to run on an Arduino Mega 2560, and uses most
 * of the available I/O pins on that device.
 */

/* Pins used for address outputs. */
#define OUT_A5     A5
#define OUT_A6     A6
#define OUT_A7     A7
#define OUT_A8     A8
#define OUT_A9     A9
#define OUT_A10    A10
#define OUT_A11    A11
#define OUT_A12    A12
#define OUT_A13    A13
#define OUT_A14    A14
#define OUT_A15    A15
#define ADDR_SHIFT 5

const int addr_pins[] = {
  OUT_A5, OUT_A6, OUT_A7, OUT_A8, OUT_A9, OUT_A10,
  OUT_A11, OUT_A12, OUT_A13, OUT_A14, OUT_A15, -1,
};

/* All output pins are addr pins. */
const int *out_pins = addr_pins;

/* Pins used for select inputs. */
#define IN_IO0    22
#define IN_IO1    23
#define IN_IO2    24
#define IN_IO3    25
#define IN_IO4    26
#define IN_IO5    27
#define IN_IO6    28
#define IN_IO7    29
#define IN_BROM   30
#define IN_FROM   31

#define IN_IO8    32
#define IN_IO9    33
#define IN_IO10   34
#define IN_IO11   35
#define IN_IO12   36
#define IN_IO13   37
#define IN_IO14   38
#define IN_IO15   39
#define IN_FRAM   40
#define IN_HBRAM  41

#define IN_IO16   42
#define IN_IO17   43
#define IN_IO18   44
#define IN_IO19   45
#define IN_IO20   46
#define IN_IO21   47
#define IN_IO22   48
#define IN_IO23   49

#define IN_IO24   14
#define IN_IO25   15
#define IN_IO26   16
#define IN_IO27   17
#define IN_IO28   18
#define IN_IO29   19
#define IN_IO30   20
#define IN_IO31   21

struct signal_desc {
  const char *name;
  int         pin;
};

#define LBRAMSEL  0
#define FRAMSEL   1
#define IOSEL(x)  (FRAMSEL + 1 + (x))
#define HBRAMSEL  (IOSEL(31) + 1)
#define BROMSEL   (HBRAMSEL + 1)
#define FROMSEL   (BROMSEL + 1)

#define IOS(x)    ((x) * 32)            /* IO start */
#define IOE(x)    ((((x) + 1) * 32) - 1)/* IO end */

const struct signal_desc signal_map[] = {
[LBRAMSEL]  = { .name = "/LBRAM", .pin = -1 },    /* uses A15 directly */
[FRAMSEL]   = { .name = "/FRAM",  .pin = IN_FRAM },
[IOSEL(0)]  = { .name = "/IO0",   .pin = IN_IO0 },
[IOSEL(1)]  = { .name = "/IO1",   .pin = IN_IO1 },
[IOSEL(2)]  = { .name = "/IO2",   .pin = IN_IO2 },
[IOSEL(3)]  = { .name = "/IO3",   .pin = IN_IO3 },
[IOSEL(4)]  = { .name = "/IO4",   .pin = IN_IO4 },
[IOSEL(5)]  = { .name = "/IO5",   .pin = IN_IO5 },
[IOSEL(6)]  = { .name = "/IO6",   .pin = IN_IO6 },
[IOSEL(7)]  = { .name = "/IO7",   .pin = IN_IO7 },
[IOSEL(8)]  = { .name = "/IO8",   .pin = IN_IO8 },
[IOSEL(9)]  = { .name = "/IO9",   .pin = IN_IO9 },
[IOSEL(10)] = { .name = "/IO10",  .pin = IN_IO10 },
[IOSEL(11)] = { .name = "/IO11",  .pin = IN_IO11 },
[IOSEL(12)] = { .name = "/IO12",  .pin = IN_IO12 },
[IOSEL(13)] = { .name = "/IO13",  .pin = IN_IO13 },
[IOSEL(14)] = { .name = "/IO14",  .pin = IN_IO14 },
[IOSEL(15)] = { .name = "/IO15",  .pin = IN_IO15 },
[IOSEL(16)] = { .name = "/IO16",  .pin = IN_IO16 },
[IOSEL(17)] = { .name = "/IO17",  .pin = IN_IO17 },
[IOSEL(18)] = { .name = "/IO18",  .pin = IN_IO18 },
[IOSEL(19)] = { .name = "/IO19",  .pin = IN_IO19 },
[IOSEL(20)] = { .name = "/IO20",  .pin = IN_IO20 },
[IOSEL(21)] = { .name = "/IO21",  .pin = IN_IO21 },
[IOSEL(22)] = { .name = "/IO22",  .pin = IN_IO22 },
[IOSEL(23)] = { .name = "/IO23",  .pin = IN_IO23 },
[IOSEL(24)] = { .name = "/IO24",  .pin = IN_IO24 },
[IOSEL(25)] = { .name = "/IO25",  .pin = IN_IO25 },
[IOSEL(26)] = { .name = "/IO26",  .pin = IN_IO26 },
[IOSEL(27)] = { .name = "/IO27",  .pin = IN_IO27 },
[IOSEL(28)] = { .name = "/IO28",  .pin = IN_IO28 },
[IOSEL(29)] = { .name = "/IO29",  .pin = IN_IO29 },
[IOSEL(30)] = { .name = "/IO30",  .pin = IN_IO30 },
[IOSEL(31)] = { .name = "/IO31",  .pin = IN_IO31 },
[HBRAMSEL]  = { .name = "/HBRAM", .pin = IN_HBRAM },
[BROMSEL]   = { .name = "/BROM",  .pin = IN_BROM },
[FROMSEL]   = { .name = "/FROM",  .pin = IN_FROM },
};
#define signal_map_count (sizeof(signal_map) / sizeof(signal_map[0]))

struct addr_decode_entry {
  uint16_t    start;
  uint16_t    end;
  int         chip_select;
};

struct addr_decode_entry addr_decode_tab[] = {
#if 0
  { .start = 0x0000,           .end = 0x7fff,           .chip_select = LBRAMSEL },
  { .start = 0x8000,           .end = 0x9bff,           .chip_select = FRAMSEL },
#endif
  { .start = 0x9c00 + IOS(0),  .end = 0x9c00 + IOE(0),  .chip_select = IOSEL(0) },
#if 0
  { .start = 0x9c00 + IOS(1),  .end = 0x9c00 + IOE(1),  .chip_select = IOSEL(1) },
  { .start = 0x9c00 + IOS(2),  .end = 0x9c00 + IOE(2),  .chip_select = IOSEL(2) },
  { .start = 0x9c00 + IOS(3),  .end = 0x9c00 + IOE(3),  .chip_select = IOSEL(3) },
  { .start = 0x9c00 + IOS(4),  .end = 0x9c00 + IOE(4),  .chip_select = IOSEL(4) },
  { .start = 0x9c00 + IOS(5),  .end = 0x9c00 + IOE(5),  .chip_select = IOSEL(5) },
  { .start = 0x9c00 + IOS(6),  .end = 0x9c00 + IOE(6),  .chip_select = IOSEL(6) },
  { .start = 0x9c00 + IOS(7),  .end = 0x9c00 + IOE(7),  .chip_select = IOSEL(7) },
  { .start = 0x9c00 + IOS(8),  .end = 0x9c00 + IOE(8),  .chip_select = IOSEL(8) },
  { .start = 0x9c00 + IOS(9),  .end = 0x9c00 + IOE(9),  .chip_select = IOSEL(9) },
  { .start = 0x9c00 + IOS(10), .end = 0x9c00 + IOE(10), .chip_select = IOSEL(10) },
  { .start = 0x9c00 + IOS(11), .end = 0x9c00 + IOE(11), .chip_select = IOSEL(11) },
  { .start = 0x9c00 + IOS(12), .end = 0x9c00 + IOE(12), .chip_select = IOSEL(12) },
  { .start = 0x9c00 + IOS(13), .end = 0x9c00 + IOE(13), .chip_select = IOSEL(13) },
  { .start = 0x9c00 + IOS(14), .end = 0x9c00 + IOE(14), .chip_select = IOSEL(14) },
  { .start = 0x9c00 + IOS(15), .end = 0x9c00 + IOE(15), .chip_select = IOSEL(15) },
  { .start = 0x9c00 + IOS(16), .end = 0x9c00 + IOE(16), .chip_select = IOSEL(16) },
  { .start = 0x9c00 + IOS(17), .end = 0x9c00 + IOE(17), .chip_select = IOSEL(17) },
  { .start = 0x9c00 + IOS(18), .end = 0x9c00 + IOE(18), .chip_select = IOSEL(18) },
  { .start = 0x9c00 + IOS(19), .end = 0x9c00 + IOE(19), .chip_select = IOSEL(19) },
  { .start = 0x9c00 + IOS(20), .end = 0x9c00 + IOE(20), .chip_select = IOSEL(20) },
  { .start = 0x9c00 + IOS(21), .end = 0x9c00 + IOE(21), .chip_select = IOSEL(21) },
  { .start = 0x9c00 + IOS(22), .end = 0x9c00 + IOE(22), .chip_select = IOSEL(22) },
  { .start = 0x9c00 + IOS(23), .end = 0x9c00 + IOE(23), .chip_select = IOSEL(23) },
  { .start = 0x9c00 + IOS(24), .end = 0x9c00 + IOE(24), .chip_select = IOSEL(24) },
  { .start = 0x9c00 + IOS(25), .end = 0x9c00 + IOE(25), .chip_select = IOSEL(25) },
  { .start = 0x9c00 + IOS(26), .end = 0x9c00 + IOE(26), .chip_select = IOSEL(26) },
  { .start = 0x9c00 + IOS(27), .end = 0x9c00 + IOE(27), .chip_select = IOSEL(27) },
  { .start = 0x9c00 + IOS(28), .end = 0x9c00 + IOE(28), .chip_select = IOSEL(28) },
  { .start = 0x9c00 + IOS(29), .end = 0x9c00 + IOE(29), .chip_select = IOSEL(29) },
  { .start = 0x9c00 + IOS(30), .end = 0x9c00 + IOE(30), .chip_select = IOSEL(30) },
  { .start = 0x9c00 + IOS(31), .end = 0x9c00 + IOE(31), .chip_select = IOSEL(31) },
  { .start = 0xa000,           .end = 0xbfff,           .chip_select = HBRAMSEL },
  { .start = 0xc000,           .end = 0xdfff,           .chip_select = BROMSEL },
  { .start = 0xe000,           .end = 0xffff,           .chip_select = FROMSEL },
#endif
  { .start = 0,                .end = 0 },
};

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

static void
wait_return_key(void)
{
  int i;

  do {
    i = Serial.read();
  } while (i != '\r' && i != '\r\n');
}

bool
signal_is_active(const struct signal_desc *s)
{
  bool active = digitalRead(s->pin);
  Serial.print("signal_is_active: ");
  Serial.print(s->name);
  Serial.print(" -> ");
  Serial.println(active);
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
    pinMode(signal_map[i].pin, INPUT_PULLUP);
  }
}

void
set_address(uint16_t address)
{
  int i;

  address >>= ADDR_SHIFT;

  for (i = 0; out_pins[i] != -1; i++) {
    digitalWrite(out_pins[i], address & 1);
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
      tla_printf("  !!! %s -- NOT ASSERTED AS EXPECTED\r\n", s->name);
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

bool
check_all_selects(int cs_expected)
{
  int i;
  bool error = false;

  for (i = 0; i < signal_map_count; i++) {
    const struct signal_desc *s = &signal_map[i];

    if (s->pin != -1) {
      if (check_output(i, i == cs_expected)) {
        error = true;
      }
    }
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

  /*
   * We want to cover the entire address space of the system
   * (modulo the low-order address bits that we don't decode).
   * We will count the numnber of addresses that we test to
   * ensure we cover it all, and to make sure that we don't
   * mis-count, we will verify that the decode map does not
   * have any overlapping regions.
   */
  for (int i = 0; addr_decode_tab[i].start != addr_decode_tab[i].end; i++) {
    for (int j = 0; addr_decode_tab[j].start != addr_decode_tab[j].end; j++) {
      if (i == j) {
        continue;
      }
      if (addr_decode_tab[j].start >= addr_decode_tab[i].start &&
          addr_decode_tab[j].start <= addr_decode_tab[i].end) {
        goto bad_decode_table;
      }
      if (addr_decode_tab[j].end >= addr_decode_tab[i].start &&
          addr_decode_tab[j].end <= addr_decode_tab[i].end) {
 bad_decode_table:
        tla_printf("FATAL -- table entry %d overlaps with table entry %d.\r\n", i, j);
        for (;;) {
          /* Nothing */
        }
      }
    }
  }
}

void
loop()
{
  uint32_t address, addresses_tested;
  int i;

  tla_printf("6809 Playground address decoder tester.\r\n");
  tla_printf("Press <RETURN> to perform the test...\r\n");
  wait_return_key();

  addresses_tested = 0;
  error_count = 0;

  for (i = 0; addr_decode_tab[i].start != addr_decode_tab[i].end; i++) {
    tla_printf("Testing $%04X - $%04X -- %s\r\n",
               addr_decode_tab[i].start,
               addr_decode_tab[i].end,
               signal_map[addr_decode_tab[i].chip_select].name);
    for (address = addr_decode_tab[i].start;
         address <= addr_decode_tab[i].end;
         address += (1 << ADDR_SHIFT)) {
      set_address((uint16_t)address);
      tla_printf("XXXJRT: waiting...\r\n"); wait_return_key();
      if (check_all_selects(addr_decode_tab[i].chip_select)) {
        /* Break out of this section if an error occurs. */
        break;
      }
      addresses_tested++;
    }
  }
  if (error_count != 0) {
    tla_printf("%d ERRORS DETECTED!\r\n", error_count);
  } else if ((addresses_tested >> ADDR_SHIFT) != (65536 >> ADDR_SHIFT)) {
    tla_printf("ONLY %u OUT OF %u ADDRESSES TESTED!\r\n",
               addresses_tested >> ADDR_SHIFT,
               65536 >> ADDR_SHIFT);
  } else {
    tla_printf("Passed!\r\n");
  }
}
