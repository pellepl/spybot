/*
 * cli.c
 *
 *  Created on: Jul 24, 2012
 *      Author: petera
 */

#include "cli.h"
#include "uart_driver.h"
#include "taskq.h"
#include "miniutils.h"
#include "system.h"

#include "app.h"

#ifdef CONFIG_SPI
#include "spi_dev.h"
#include "spi_driver.h"
#endif

#include "comm_radio.h"
#include "nrf905_impl.h"

#ifdef CONFIG_SPYBOT_LSM
#include "i2c_driver.h"
#include "i2c_dev.h"
#include "lsm303_driver.h"
#endif

#ifdef CONFIG_M24M01
#include "m24m01_driver.h"
#endif

#ifdef CONFIG_SPYBOT_HCSR
#include "range_sens_hcsr04_driver.h"
#endif

#ifdef CONFIG_SPYBOT_VIDEO
#include "cvideo.h"
#include "hud.h"
#include "rover_3d.h"
#endif

#ifdef CONFIG_SPYBOT_JOYSTICK
#include "adc.h"
#endif

#ifdef CONFIG_SPYBOT_MOTOR
#include "motor.h"
#endif

#define RADIO_DBG

#define CLI_PROMPT "> "
#define IS_STRING(s) ((u8_t*)(s) >= (u8_t*)in && (u8_t*)(s) < (u8_t*)in + sizeof(in))

typedef int (*func)(int a, ...);

typedef struct cmd_s {
  const char* name;
  const func fn;
  const char* help;
} cmd;

struct {
  uart_rx_callback prev_uart_rx_f;
  void *prev_uart_arg;
} cli_state;

static u8_t in[256];

static int _argc;
static void *_args[16];

static int f_uwrite(int uart, char* data);
static int f_uread(int uart, int numchars);
static int f_uconf(int uart, int speed);

static int f_rand();

static int f_reset();
static int f_time(int d, int h, int m, int s, int ms);
static int f_help(char *s);
static int f_dump();
static int f_dump_trace();
static int f_assert();
static int f_dbg();

static int f_dbg_tx(char *s);

#ifdef CONFIG_I2C
static int f_i2c_read(int addr, int reg);
static int f_i2c_write(int addr, int reg, int data);
static int f_i2c_scan(void);

#ifdef CONFIG_M24M01
static int f_ee_open(void);
static int f_ee_read(int addr, int len);
static int f_ee_write(int addr, char *s);
#endif
#endif

#ifdef CONFIG_SPYBOT_LSM
static int f_lsm_open();
static int f_lsm_readacc();
static int f_lsm_readmag();
static int f_lsm_read();
static int f_lsm_calibrate();
static int f_lsm_close();
#endif

#ifdef CONFIG_SPYBOT_JOYSTICK
static int f_adc(void);
#endif

static int f_col(int col);
static int f_hardfault(int a);

#ifdef CONFIG_SPYBOT_HCSR
static int f_range_init(void);
static int f_range(void);
#endif
#ifdef CONFIG_SPYBOT_SERVO
static int f_servo(int p, int servo_d);
#endif
#ifdef CONFIG_SPYBOT_MOTOR
static int f_motor_init(void);
static int f_motor_go(int x);
#endif

#ifdef RADIO_DBG
static int f_radio_init(void);
static int f_radio_read_conf(void);
static int f_radio_set_conf(void);
static int f_radio_rx(void);
static int f_radio_tx_addr(void);
static int f_radio_tx(void);
static int f_radio_carrier(void);
static int f_radio_channel(u16_t channel);
static int f_radio_pa(u8_t pa);
#endif

#ifdef CONFIG_SPYBOT_VIDEO
static int f_cvideo_init(void);
static int f_cvideo_voffset(int i);
static int f_cvideo_vscroll(int i);
static int f_cvideo_effect(int i);

static int f_hud_dbg(char *s);
static int f_hud_state(int state);
static int f_hud_view(int x, int y, int z, int ax, int ay, int az);
static int f_hud_in(int i);
#endif

static int f_comrad_init(void);
static int f_comrad_tx(char *str, int ack);

static void cli_print_app_name(void);

#define HELP_UART_DEFS "uart - 0,1,2,3 - 0 is COMM, 1 is DBG, 2 is SPL, 3 is BT\n"

/////////////////////////////////////////////////////////////////////////////////////////////

static cmd c_tbl[] = {
    { .name = "dump", .fn = (func) f_dump,
        .help = "Dumps state of all system\n"
    },
    { .name = "dump_trace", .fn = (func) f_dump_trace, .help = "Dumps system trace\n"
    },
    { .name = "time", .fn = (func) f_time,
        .help = "Prints or sets time\n"
        "time or time <day> <hour> <minute> <second> <millisecond>\n"
    },
    { .name = "uwrite", .fn = (func) f_uwrite,
        .help = "Writes to uart\n"
        "uwrite <uart> <string>\n"
            HELP_UART_DEFS
            "ex: uwrite 2 \"foo\"\n"
    },
    { .name = "uread", .fn = (func) f_uread,
        .help = "Reads from uart\n"
        "uread <uart> (<numchars>)\n"
            HELP_UART_DEFS
            "numchars - number of chars to read, if omitted uart is drained\n"
            "ex: uread 2 10\n"
    },
    { .name = "uconf", .fn = (func) f_uconf,
        .help = "Configure uart\n"
        "uconf <uart> <speed>\n"
            HELP_UART_DEFS
            "ex: uconf 2 9600\n"
    },

    { .name = "tx", .fn = (func) f_dbg_tx,
        .help = "sends dbg string to other side\n"
    },

#ifdef CONFIG_SPYBOT_HCSR
    { .name = "range_init", .fn = (func) f_range_init,
        .help = "Initiates range sensor\n"
    },
    { .name = "range", .fn = (func) f_range,
        .help = "Range sample\n"
    },
#endif
#ifdef CONFIG_SPYBOT_SERVO
    { .name = "servo", .fn = (func) f_servo,
        .help = "Set PB9 servo, 0-99\n"
    },
#endif
#ifdef CONFIG_SPYBOT_MOTOR
    { .name = "motor_init", .fn = (func) f_motor_init,
        .help = "Motor driver init\n"
    },
    { .name = "motor_go", .fn = (func) f_motor_go,
        .help = "Start motors, -128..127\n"
    },
#endif

#ifdef RADIO_DBG
    { .name = "radio_init", .fn = (func)f_radio_init,
        .help = "Initialize radio\n"
    },
    { .name = "radio_rconf", .fn = (func)f_radio_read_conf,
        .help = "Read radio config\n"
    },
    { .name = "radio_conf", .fn = (func)f_radio_set_conf,
        .help = "Set radio config\n"
    },
    { .name = "radio_rx", .fn = (func)f_radio_rx,
        .help = "Put radio in rx\n"
    },
    { .name = "radio_set_addr", .fn = (func)f_radio_tx_addr,
        .help = "Radio set tx addr\n"
    },
    { .name = "radio_tx", .fn = (func)f_radio_tx,
        .help = "Radio tx a sequence\n"
    },
    { .name = "radio_carrier", .fn = (func)f_radio_carrier,
        .help = "Transmit carrier\n"
    },
    { .name = "radio_channel", .fn = (func)f_radio_channel,
        .help = "Set radio channel freq <0-511>\n"
    },
    { .name = "radio_pa", .fn = (func)f_radio_pa,
        .help = "Set radio PA <0-3>\n"
    },
#endif //RADIO_DBG
#ifdef CONFIG_SPYBOT_VIDEO
    { .name = "video_init", .fn = (func)f_cvideo_init,
        .help = "Initializes cvideo\n"
    },
    { .name = "video_offs", .fn = (func)f_cvideo_voffset,
        .help = "Sets vertical gram offset\n"
    },
    { .name = "video_scroll", .fn = (func)f_cvideo_vscroll,
        .help = "Sets vertical scroll\n"
    },
    { .name = "video_effect", .fn = (func)f_cvideo_effect,
        .help = "Sets effect\n"
    },

    { .name = "hud_dbg", .fn = (func) f_hud_dbg,
        .help = "Print string on hud screen\n"
    },
    { .name = "hud_state", .fn = (func) f_hud_state,
        .help = "Change hud state\n"
    },
    { .name = "hud_view", .fn = (func) f_hud_view,
        .help = "Change hud rover view (x,y,z,ax,ay,az)\n"
    },
    { .name = "hud_in", .fn = (func) f_hud_in,
        .help = "Emulate input (1=UP, 2=DOWN, 4=LEFT, 8=RIGHT, 16=PRESS)\n"
    },
#endif

#ifdef CONFIG_I2C
    { .name = "i2c_r", .fn = (func) f_i2c_read,
        .help = "i2c read reg\n"
    },
    { .name = "i2c_w", .fn = (func) f_i2c_write,
        .help = "i2c write reg\n"
    },
    { .name = "i2c_scan", .fn = (func) f_i2c_scan,
        .help = "scans i2c bus for all addresses\n" },

#ifdef CONFIG_M24M01
        { .name = "ee_open", .fn = (func) f_ee_open,
            .help = "ee open device\n"
        },
        { .name = "ee_r", .fn = (func) f_ee_read,
            .help = "read eeprom (address, length)\n"
        },
        { .name = "ee_w", .fn = (func) f_ee_write,
            .help = "write eeprom (address, string)\n" },
#endif
#endif

#ifdef CONFIG_SPYBOT_LSM
    { .name = "lsm_open", .fn = (func) f_lsm_open,
        .help = "setups and configures lsm303 device\n"
    },
    { .name = "lsm_calib", .fn = (func) f_lsm_calibrate,
        .help = "calibrate lsm303 device\n"
    },
    { .name = "lsm_acc", .fn = (func) f_lsm_readacc,
        .help = "reads out lsm303 acceleration values\n"
    },
    { .name = "lsm_mag", .fn = (func) f_lsm_readmag,
        .help = "reads out lsm303 magneto values\n"
    },
    { .name = "lsm_read", .fn = (func) f_lsm_read,
        .help = "reads out lsm303 values\n"
    },
    { .name = "lsm_close", .fn = (func) f_lsm_close,
        .help = "closes lsm303 device\n"
    },
#endif

#ifdef CONFIG_SPYBOT_JOYSTICK
    { .name = "adc", .fn = (func)f_adc,
        .help = "Sample adc\n"
    },
#endif
#if 0
    { .name = "test", .fn = (func)f_test,
        .help = "Test func\n"
    },
#endif

    { .name = "comrad_init", .fn = (func) f_comrad_init,
        .help = "Initiates comm radio stack\n"
    },
    { .name = "comrad_tx", .fn = (func) f_comrad_tx,
        .help = "Transmit packet\n"
        "comrad_tx <data> <ack>\n"
    },

    { .name = "dbg", .fn = (func) f_dbg,
        .help = "Set debug filter and level\n"
        "dbg (level <dbg|info|warn|fatal>) (enable [x]*) (disable [x]*)\n"
        "x - <task|heap|comm|cnc|cli|nvs|spi|all>\n"
        "ex: dbg level info disable all enable cnc comm\n"
    },
    { .name = "assert", .fn = (func) f_assert,
        .help = "Asserts system\n"
            "NOTE system will need to be rebooted\n"
    },
    { .name = "rand", .fn = (func) f_rand,
        .help = "Generates pseudo random sequence\n"
    },
    { .name = "col", .fn = (func) f_col,
        .help = "Set text color\n"
    },
    { .name = "reset", .fn = (func) f_reset,
        .help = "Resets system\n"
    },
    { .name = "hardfault", .fn = (func) f_hardfault,
        .help = "Generate hardfault, div by zero\n"
    },
    { .name = "help", .fn = (func) f_help,
        .help = "Prints help\n"
        "help or help <command>\n"
    },
    { .name = "?", .fn = (func) f_help,
        .help = "Prints help\n"
        "help or help <command>\n" },

    // menu end marker
    { .name = NULL, .fn = (func) 0, .help = NULL },
  };

/////////////////////////////////////////////////////////////////////////////////////////////

static int f_rand() {
  print("%08x\n", rand_next());
  return 0;
}

static int f_reset() {
  SYS_reboot(REBOOT_USER);
  return 0;
}

static int f_time(int ad, int ah, int am, int as, int ams) {
  if (_argc == 0) {
    u16_t d, ms;
    u8_t h, m, s;
    SYS_get_time(&d, &h, &m, &s, &ms);
    print("day:%i time:%02i:%02i:%02i.%03i\n", d, h, m, s, ms);
  } else if (_argc == 5) {
    SYS_set_time(ad, ah, am, as, ams);
  } else {
    return -1;
  }
  return 0;
}

#ifdef DBG_OFF
static int f_dbg() {
  print("Debug disabled compile-time\n");
  return 0;
}
#else
const char* DBG_BIT_NAME[] = _DBG_BIT_NAMES;

static void print_debug_setting() {
  print("DBG level: %i\n", SYS_dbg_get_level());
  int d;
  for (d = 0; d < sizeof(DBG_BIT_NAME) / sizeof(const char*); d++) {
    print("DBG mask %s: %s\n", DBG_BIT_NAME[d],
        SYS_dbg_get_mask() & (1 << d) ? "ON" : "OFF");
  }
}

static int f_dbg() {
  enum state {
    NONE, LEVEL, ENABLE, DISABLE
  } st = NONE;
  int a;
  if (_argc == 0) {
    print_debug_setting();
    return 0;
  }
  for (a = 0; a < _argc; a++) {
    u32_t f = 0;
    char *s = (char*) _args[a];
    if (!IS_STRING(s)) {
      return -1;
    }
    if (strcmp("level", s) == 0) {
      st = LEVEL;
    } else if (strcmp("enable", s) == 0) {
      st = ENABLE;
    } else if (strcmp("disable", s) == 0) {
      st = DISABLE;
    } else {
      switch (st) {
      case LEVEL:
        if (strcmp("dbg", s) == 0) {
          SYS_dbg_level(D_DEBUG);
        } else if (strcmp("info", s) == 0) {
          SYS_dbg_level(D_INFO);
        } else if (strcmp("warn", s) == 0) {
          SYS_dbg_level(D_WARN);
        } else if (strcmp("fatal", s) == 0) {
          SYS_dbg_level(D_FATAL);
        } else {
          return -1;
        }
        break;
      case ENABLE:
      case DISABLE: {
        int d;
        for (d = 0; f == 0 && d < sizeof(DBG_BIT_NAME) / sizeof(const char*);
            d++) {
          if (strcmp(DBG_BIT_NAME[d], s) == 0) {
            f = (1 << d);
          }
        }
        if (strcmp("all", s) == 0) {
          f = D_ANY;
        }
        if (f == 0) {
          return -1;
        }
        if (st == ENABLE) {
          SYS_dbg_mask_enable(f);
        } else {
          SYS_dbg_mask_disable(f);
        }
        break;
      }
      default:
        return -1;
      }
    }
  }
  print_debug_setting();
  return 0;
}
#endif

static int f_assert() {
  ASSERT(FALSE);
  return 0;
}

static int f_uwrite(int uart, char* data) {
  if (_argc != 2 || !IS_STRING(data)) {
    return -1;
  }
  if (uart < 0 || uart >= CONFIG_UART_CNT) {
    return -1;
  }
  char c;
  while ((c = *data++) != 0) {
    UART_put_char(_UART(uart), c);
  }
  return 0;
}

static int f_uread(int uart, int numchars) {
  if (_argc < 1 || _argc > 2) {
    return -1;
  }
  if (uart < 0 || uart >= CONFIG_UART_CNT) {
    return -1;
  }
  if (_argc == 1) {
    numchars = 0x7fffffff;
  }
  int l = UART_rx_available(_UART(uart));
  l = MIN(l, numchars);
  int ix = 0;
  while (ix++ < l) {
    print("%c", UART_get_char(_UART(uart)));
  }
  print("\n%i bytes read\n", l);
  return 0;
}

static int f_uconf(int uart, int speed) {
  if (_argc != 2) {
    return -1;
  }
  if (IS_STRING(uart) || IS_STRING(speed) || uart < 0 || uart >= CONFIG_UART_CNT) {
    return -1;
  }
  USART_TypeDef *uart_hw = _UART(uart)->hw;

  USART_Cmd(uart_hw, DISABLE);

  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = speed;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure the USART */
  USART_Init(uart_hw, &USART_InitStructure);

  USART_Cmd(uart_hw, ENABLE);

  return 0;
}

static int f_help(char *s) {
  if (IS_STRING(s)) {
    int i = 0;
    while (c_tbl[i].name != NULL ) {
      if (strcmp(s, c_tbl[i].name) == 0) {
        print("%s\t%s", c_tbl[i].name, c_tbl[i].help);
        return 0;
      }
      i++;
    }
    print("%s\tno such command\n", s);
  } else {
    print ("  ");
    cli_print_app_name();
    print("\n");
    int i = 0;
    while (c_tbl[i].name != NULL ) {
      int len = strpbrk(c_tbl[i].help, "\n") - c_tbl[i].help;
      char tmp[64];
      strncpy(tmp, c_tbl[i].help, len + 1);
      tmp[len + 1] = 0;
      char fill[24];
      int fill_len = sizeof(fill) - strlen(c_tbl[i].name);
      memset(fill, ' ', sizeof(fill));
      fill[fill_len] = 0;
      print("  %s%s%s", c_tbl[i].name, fill, tmp);
      i++;
    }
  }
  return 0;
}

static int f_dump() {
  print("FULL DUMP\n=========\n");
  TASK_dump(IOSTD);
  print("\n");
  print("=========\n");
  return 0;
}

static int f_dump_trace() {
#ifdef DBG_TRACE_MON
  SYS_dump_trace(IOSTD);
#else
  print("trace not enabled\n");
#endif
  return 0;
}

#ifdef CONFIG_SPYBOT_LSM

extern lsm303_dev lsm_dev;
static int lsm_op = 0;
static int lsm_still = 0;
static int lsm_readings = 0;
static s16_t lsm_last_mag[3];
static s16_t lsm_mag_min[3];
static s16_t lsm_mag_max[3];

static void lsm_cb(lsm303_dev *dev, int res) {
  s16_t *mag = lsm_get_mag_reading(&lsm_dev);
  s16_t *acc = lsm_get_acc_reading(&lsm_dev);
  switch (lsm_op) {
  case 0: // open
    if (res != I2C_OK) {
      print("error %i\n", res);
    } else {
      print("lsm opened\n");
    }
    break;
  case 1: // readacc
    if (res != I2C_OK) {
      print("error %i\n", res);
    } else {
      print("lsm acc %04x %04x %04x\n", acc[0], acc[1], acc[2]);
    }
    break;
  case 2: // readmag
    if (res != I2C_OK) {
      print("error %i\n", res);
    } else {
      print("lsm mag %04x %04x %04x\n", mag[0], mag[1], mag[2]);
    }
    break;
  case 3: // read both
    if (res != I2C_OK) {
      print("error %i\n", res);
    } else {
      print("lsm acc %04x %04x %04x, mag %04x %04x %04x\n", acc[0], acc[1],
          acc[2], mag[0], mag[1], mag[2]);
      u16_t heading = lsm_get_heading(&lsm_dev);
      print("heading: %04x, %i\n", heading, (heading * 360) >> 16);
    }
    break;
  case 4: // calibrate
    lsm_readings++;
    if (res == I2C_OK) {
      if (ABS(lsm_last_mag[0] - mag[0]) < 32
          && ABS(lsm_last_mag[1] - mag[1]) < 32
          && ABS(lsm_last_mag[2] - mag[2]) < 32) {
        lsm_still++;
        if (lsm_still > 100) {
          print("Calibration finished.\n");
          print("%i < x < %i\n", lsm_mag_min[0], lsm_mag_max[0]);
          print("%i < y < %i\n", lsm_mag_min[1], lsm_mag_max[1]);
          print("%i < z < %i\n", lsm_mag_min[2], lsm_mag_max[2]);
          lsm_op = -1;
        }
      } else {
        lsm_still = 0;
        int i;
        for (i = 0; i < 3; i++) {
          lsm_mag_min[i] = MIN(lsm_mag_min[i], mag[i]);
          lsm_mag_max[i] = MAX(lsm_mag_max[i], mag[i]);
        }
      }
      memcpy(lsm_last_mag, mag, sizeof(lsm_last_mag));
    } else if (res == I2C_ERR_DEV_TIMEOUT) {
      print("lsm calib error %i\n", res);
      break;
    }
    if (lsm_op == 4) {
      if ((lsm_readings & 0x3f) == 0 && lsm_readings > 0) {
        print("%i, still:%i, ", lsm_readings, lsm_still);
        print("%i < x < %i, ", lsm_mag_min[0], lsm_mag_max[0]);
        print("%i < y < %i, ", lsm_mag_min[1], lsm_mag_max[1]);
        print("%i < z < %i\n", lsm_mag_min[2], lsm_mag_max[2]);
      }
      SYS_hardsleep_ms(50);
      (void) lsm_read_mag(&lsm_dev);
    }
    break;
  }
}

static int f_lsm_open() {
  lsm_open(&lsm_dev, _I2C_BUS(0), FALSE, lsm_cb);
  lsm_op = 0;
  int res = lsm_config_default(&lsm_dev);
  if (res != I2C_OK) {
    print("err: %i\n", res);
  }
  return 0;
}

static int f_lsm_readacc() {
  lsm_op = 1;
  int res = lsm_read_acc(&lsm_dev);
  if (res != I2C_OK) {
    print("err: %i\n", res);
  }
  return 0;
}
static int f_lsm_readmag() {
  lsm_op = 2;
  int res = lsm_read_mag(&lsm_dev);
  if (res != I2C_OK) {
    print("err: %i\n", res);
  }
  return 0;
}
static int f_lsm_read() {
  lsm_op = 3;
  int res = lsm_read_both(&lsm_dev);
  if (res != I2C_OK) {
    print("err: %i\n", res);
  }
  return 0;
}
static int f_lsm_calibrate() {
  print("Move device around all axes slowly, put it to rest when finished\n");
  lsm_op = 4;
  lsm_still = 0;
  lsm_readings = 0;
  lsm_mag_min[0] = 0x7fff;
  lsm_mag_min[1] = 0x7fff;
  lsm_mag_min[2] = 0x7fff;
  lsm_mag_max[0] = -0x7fff;
  lsm_mag_max[1] = -0x7fff;
  lsm_mag_max[2] = -0x7fff;
  int res = lsm_read_mag(&lsm_dev);
  if (res != I2C_OK) {
    print("err: %i\n", res);
  }
  return 0;

}
static int f_lsm_close() {
  lsm_close(&lsm_dev);
  return 0;
}
#endif // CONFIG_SPYBOT_LSM

#ifdef CONFIG_I2C

static u8_t i2c_dev_reg = 0;
static u8_t i2c_dev_val = 0;
static i2c_dev i2c_device;
static u8_t i2c_wr_data[2];
static i2c_dev_sequence i2c_r_seqs[] = { I2C_SEQ_TX(&i2c_dev_reg, 1),
    I2C_SEQ_RX_STOP(&i2c_dev_val, 1) };
static i2c_dev_sequence i2c_w_seqs[] = { I2C_SEQ_TX_STOP(i2c_wr_data, 2) , };

static void i2c_test_cb(i2c_dev *dev, int result) {
  print("i2c_dev_cb: reg %02x val %02x res:%i\n", i2c_dev_reg, i2c_dev_val,
      result);
  I2C_DEV_close(&i2c_device);
}

static int f_i2c_read(int addr, int reg) {
  I2C_DEV_init(&i2c_device, 100000, _I2C_BUS(0), addr);
  I2C_DEV_set_callback(&i2c_device, i2c_test_cb);
  I2C_DEV_open(&i2c_device);
  i2c_dev_reg = reg;
  I2C_DEV_sequence(&i2c_device, i2c_r_seqs, 2);
  return 0;
}

static int f_i2c_write(int addr, int reg, int data) {
  I2C_DEV_init(&i2c_device, 100000, _I2C_BUS(0), addr);
  I2C_DEV_set_callback(&i2c_device, i2c_test_cb);
  I2C_DEV_open(&i2c_device);
  i2c_wr_data[0] = reg;
  i2c_wr_data[1] = data;
  i2c_dev_reg = reg;
  i2c_dev_val = data;
  I2C_DEV_sequence(&i2c_device, i2c_w_seqs, 1);
  return 0;
}

static u8_t i2c_scan_addr;
extern task_mutex i2c_mutex;

void i2c_scan_report_task(u32_t addr, void *res) {
  if (addr == 0) {
    print("\n    0  2  4  6  8  a  c  e");
  }
  if ((addr & 0x0f) == 0) {
    print("\n%02x ", addr & 0xf0);
  }

  print("%s", (char *) res);

  if (i2c_scan_addr < 254) {
    i2c_scan_addr += 2;
    I2C_query(_I2C_BUS(0), i2c_scan_addr);
  } else {
    print("\n");
    TASK_mutex_unlock(&i2c_mutex);
  }
}

static void i2c_scan_cb_irq(i2c_bus *bus, int res) {
  task *report_scan_task = TASK_create(i2c_scan_report_task, 0);

  TASK_run(report_scan_task, bus->addr & 0xfe, res == I2C_OK ? "UP " : ".. ");
}

static int f_i2c_scan(void) {
  if (!TASK_mutex_try_lock(&i2c_mutex)) {
    print("i2c busy\n");
    return 0;
  }
  i2c_scan_addr = 0;
  int res = I2C_config(_I2C_BUS(0), 10000);
  if (res != I2C_OK) print("i2c config err %i\n", res);
  res = I2C_set_callback(_I2C_BUS(0), i2c_scan_cb_irq);
  if (res != I2C_OK) print("i2c cb err %i\n", res);
  res = I2C_query(_I2C_BUS(0), i2c_scan_addr);
  if (res != I2C_OK) print("i2c query err %i\n", res);
  return 0;
}

#ifdef CONFIG_M24M01
m24m01_dev ee_dev;
static u8_t ee_data[128];
bool ee_bufprint;
int ee_buflen;

static void ee_cb(m24m01_dev *dev, int res) {
  print("ee_dev cb, res %i\n", res);
  if (ee_bufprint) {
    printbuf(IOSTD, &ee_data[0],ee_buflen);
  }
  TASK_mutex_unlock(&i2c_mutex);
}

static int f_ee_open(void) {
  m24m01_open(&ee_dev, _I2C_BUS(0), FALSE, FALSE, ee_cb);
  return 0;
}

static int f_ee_read(int addr, int len) {
  if (_argc < 2) {
    return -1;
  }
  if (!TASK_mutex_try_lock(&i2c_mutex)) {
    print("i2c busy\n");
    return 0;
  }
  ee_bufprint = TRUE;
  ee_buflen = len;
  int res = m24m01_read(&ee_dev, addr, (u8_t *)ee_data, len);
  if (res != I2C_OK) {
    print("err:%i\n", res);
  }
  return 0;
}

static int f_ee_write(int addr, char *s) {
  if (_argc < 2 || !IS_STRING(s)) {
    return -1;
  }
  if (!TASK_mutex_try_lock(&i2c_mutex)) {
    print("i2c busy\n");
    return 0;
  }
  ee_bufprint = FALSE;
  int res = m24m01_write(&ee_dev, addr, (u8_t*)s, (u32_t)strlen(s));
  if (res != I2C_OK) {
    print("err:%i\n", res);
  }
  return 0;
}
#endif // CONFIG_M24M01

#endif // CONFIG_I2C

#ifdef CONFIG_ADC
static int f_adc(void) {
  ADC_sample(NULL);
  return 0;
}
#endif // CONFIG_ADC



static int f_comrad_init(void) {
  COMRAD_init();
  return 0;
}

static int f_comrad_tx(char *str, int ack) {
  if (_argc < 2)
    return -1;
  int res = COMRAD_send((u8_t*) str, strlen(str), ack);
  if (res < R_COMM_OK) {
    print("err:%i\n", res);
  } else {
    print("seq:%03x\n", res);

  }
  return 0;
}

static int f_col(int col) {
  print("\033[1;3%im", col & 7);
  return 0;
}

static int f_hardfault(int a) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdiv-by-zero"
  SCB ->CCR |= SCB_CCR_DIV_0_TRP_Msk;
  volatile int q = 3;
  volatile int x = 0;
  return q / x;
#pragma GCC diagnostic pop
}


static int f_dbg_tx(char *s) {
  if (_argc < 1 || !IS_STRING(s)) {
    return -1;
  }
  APP_tx_dbg(s);
  return 0;
}

#ifdef CONFIG_SPYBOT_HCSR
static void cli_range_cb(u32_t t) {
  print("range cb:%i\n", t);
}

static int f_range_init(void) {
  RANGE_SENS_init(cli_range_cb);
  return 0;
}

static int f_range(void) {
  s32_t res;
  res = RANGE_SENS_trigger();
  if (res != RANGE_SENS_OK) {
    print("err: %i\n", res);
  }
  return 0;
}

#endif // CONFIG_SPYBOT_HCSR


#ifdef CONFIG_SPYBOT_SERVO

static task *servo_task;
static bool servo_timer_started = FALSE;
static task_timer servo_timer;
static int servo_delta = 1;
static void servo_task_f(u32_t a, void *b) {
  static bool s_dir = FALSE;
  static int s_val = 0;
  if (s_dir) {
    s_val += servo_delta;
    if (s_val >= 200) {
      s_val = 199;
      s_dir = FALSE;
    }
  } else {
    s_val -= servo_delta;
    if (s_val <= 0) {
      s_val = 0;
      s_dir = TRUE;
    }
  }
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  //TIM_OCInitStructure.TIM_Pulse = 3277 + ((6554 - 3277) * s_val) / 200;
  TIM_OCInitStructure.TIM_Pulse = 3277 + (5000 * s_val) / 200;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
}
static int f_servo(int p, int servo_d) {
  if (p == 9999) {
    servo_delta = 1+(servo_d%8);
    servo_task = TASK_create(servo_task_f, TASK_STATIC);
    TASK_start_timer(servo_task, &servo_timer, 0, 0, 0, 50, "servo_tim");
    servo_timer_started = TRUE;
    return 0;
  }
  if (servo_timer_started) {
    TASK_free(servo_task);
    TASK_stop_timer(&servo_timer);
    servo_timer_started = FALSE;
  }
  p %= 100;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_Pulse = 3277 + ((6554 - 3277) * p) / 99;
  // 3277 ~= 1ms
  // 4915 ~= 1.5ms
  // 6554 ~= 2ms
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  return 0;
}

#endif // CONFIG_SPYBOT_SERVO

#ifdef CONFIG_SPYBOT_MOTOR
static int f_motor_init(void) {
  MOTOR_init();
  return 0;
}
static int f_motor_go(int x) {
  if (_argc < 1) return -1;
  MOTOR_go((s8_t)(x & 0xff));
  return 0;
}

#endif // CONFIG_SPYBOT_MOTOR

#ifdef RADIO_DBG

static int f_radio_init(void) {
  NRF905_IMPL_init(NULL, NULL, NULL, NULL);
  return 0;
}

static int f_radio_carrier(void) {
  int res = NRF905_IMPL_carrier();
  if (res < NRF905_OK) {
    print("err:%i\n",res);
  }
  return 0;
}

static int f_radio_channel(u16_t channel) {
  if (_argc < 1) return -1;
  int res = NRF905_IMPL_conf_channel(channel, TRUE);
  if (res < NRF905_OK) {
    print("err:%i\n",res);
  }
  return 0;
}

static int f_radio_pa(u8_t pa) {
  if (_argc < 1) return -1;
  int res = NRF905_IMPL_conf_pa(pa, TRUE);
  if (res < NRF905_OK) {
    print("err:%i\n",res);
  }
  return 0;
}

static int f_radio_read_conf(void) {
  NRF905_IMPL_read_conf();
  return 0;
}

static int f_radio_set_conf(void) {
  NRF905_IMPL_conf(NULL, TRUE);
  return 0;
}

static int f_radio_rx(void) {
  NRF905_IMPL_rx();
  return 0;
}

static int f_radio_tx_addr(void) {
  NRF905_IMPL_conf_tx_addr((u8_t[4]) {0x63, 0x1c, 0x51, 0x2d}, TRUE);
  return 0;
}

static int f_radio_tx(void) {
  NRF905_IMPL_tx((u8_t*)"hello world", 11);
  return 0;
}

#endif //RADIO_DBG

#ifdef CONFIG_SPYBOT_VIDEO

extern gcontext gctx;

static int f_cvideo_init(void) {
  CVIDEO_init(HUD_vbl);
  CVIDEO_init_gcontext(&gctx);
  return 0;
}

static int f_cvideo_voffset(int i) {
  if (_argc < 1) return -1;
  CVIDEO_set_v_offset(i);
  return 0;
}

static int f_cvideo_vscroll(int i) {
  if (_argc < 1) return -1;
  CVIDEO_set_v_scroll(i);
  return 0;
}

static int f_cvideo_effect(int i) {
  if (_argc < 1) return -1;
  CVIDEO_set_effect(i);
  return 0;
}


static int f_hud_dbg(char *s) {
  if (_argc < 1 || !IS_STRING(s)) {
    return -1;
  }
  HUD_dbg_print(&gctx, s);
  return 0;
}

static int f_hud_state(int state) {
  if (_argc < 1) {
    return -1;
  }
  HUD_state(state);
  return 0;
}

static int f_hud_view(int x, int y, int z, int ax, int ay, int az) {
  if (_argc < 6) {
    return -1;
  }
  ROVER_view(x, y, z, ax, ay, az, FALSE);
  return 0;
}

static int f_hud_in(int i) {
  if (_argc < 1) {
    return -1;
  }
  HUD_input(i, TRUE);
  return 0;
}

#endif // CONFIG_SPYBOT_VIDEO

/////////////////////////////////////////////////////////////////////////////////////////////

void CLI_TASK_on_input(u32_t len, void *p) {
  if (len > sizeof(in)) {
    DBG(D_CLI, D_WARN, "CONS input overflow\n");
    print(CLI_PROMPT);
    return;
  }
  u32_t rlen = UART_get_buf(_UART(UARTSTDIN), in, MIN(len, sizeof(in)));
  if (rlen != len) {
    DBG(D_CLI, D_WARN, "CONS length mismatch\n");
    print(CLI_PROMPT);
    return;
  }
  cursor cursor;
  strarg_init(&cursor, (char*) in, rlen);
  strarg arg;
  _argc = 0;
  func fn = NULL;
  int ix = 0;

  // parse command and args
  while (strarg_next(&cursor, &arg)) {
    if (arg.type == INT) {
      //DBG(D_CLI, D_DEBUG, "CONS arg %i:\tlen:%i\tint:%i\n",arg_c, arg.len, arg.val);
    } else if (arg.type == STR) {
      //DBG(D_CLI, D_DEBUG, "CONS arg %i:\tlen:%i\tstr:\"%s\"\n", arg_c, arg.len, arg.str);
    }
    if (_argc == 0) {
      // first argument, look for command function
      if (arg.type != STR) {
        break;
      } else {
        while (c_tbl[ix].name != NULL ) {
          if (strcmp(arg.str, c_tbl[ix].name) == 0) {
            fn = c_tbl[ix].fn;
            break;
          }
          ix++;
        }
        if (fn == NULL ) {
          break;
        }
      }
    } else {
      // succeeding arguments¸ store them in global vector
      if (_argc - 1 >= 16) {
        DBG(D_CLI, D_WARN, "CONS too many args\n");
        fn = NULL;
        break;
      }
      _args[_argc - 1] = (void*) arg.val;
    }
    _argc++;
  }

  // execute command
  if (fn) {
    _argc--;
    DBG(D_CLI, D_DEBUG, "CONS calling [%p] with %i args\n", fn, _argc);
    int res = (int) _variadic_call(fn, _argc, _args);
    if (res == -1) {
      print("%s", c_tbl[ix].help);
    } else {
      print("OK\n");
    }
  } else {
    print("unknown command - try help\n");
  }
  print(CLI_PROMPT);
}

void CLI_timer() {
}

void CLI_uart_check_char(void *a, u8_t c) {
  if (c == '\n') {
    task *t = TASK_create(CLI_TASK_on_input, 0);
    TASK_run(t, UART_rx_available(_UART(UARTSTDIN)), NULL);
  }
}

void CLI_init() {
  memset(&cli_state, 0, sizeof(cli_state));
  DBG(D_CLI, D_DEBUG, "CLI init\n");
  SYS_dbg_mask_set(0);
  UART_set_callback(_UART(UARTSTDIN), CLI_uart_check_char, NULL);
  print ("\n");
  cli_print_app_name();
  print("\n\n");
  print("build     : %i\n", SYS_build_number());
  print("build date: %i\n", SYS_build_date());
  print("\ntype '?' or 'help' for list of commands\n\n");
  print(CLI_PROMPT);
}

static void cli_print_app_name(void) {
  print (APP_NAME);
#ifdef SECONDARY
    print(" SECONDARY: ");
#else
    print(" PRIMARY: ");
#endif
#ifdef CONFIG_SPYBOT_TEST
    print ("test config");
#endif
#ifdef CONFIG_SPYBOT_ROVER
    print ("rover config");
#endif
#ifdef CONFIG_SPYBOT_CONTROLLER
    print ("controller config");
#endif
}
