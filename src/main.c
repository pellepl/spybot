#include "stm32f10x.h"
#include "system.h"
#include "uart_driver.h"
#include "io.h"
#include "timer.h"
#include "miniutils.h"
#include "taskq.h"
#include "cli.h"
#include "processor.h"
#include "linker_symaccess.h"
#include "i2c_driver.h"
#include "spi_driver.h"

#ifdef DBG_KERNEL_TASK_BLINKY
task_timer dbg_blinky_task_timer;
task *dbg_blinky_task;
static bool _dbg_bl_state = TRUE;
static void dbg_blinky_task_func(u32_t i, void *p) {
  if (_dbg_bl_state) {
    GPIO_disable(GPIOC, GPIO_Pin_6);
    TASK_set_timer_recurrence(&dbg_blinky_task_timer, 10);
    _dbg_bl_state = FALSE;
  } else {
    GPIO_enable(GPIOC, GPIO_Pin_6);
    TASK_set_timer_recurrence(&dbg_blinky_task_timer, 990);
    _dbg_bl_state = TRUE;
  }
}
#endif

#ifdef DBG_RADIO
#include "nrf905_impl.h"
task_timer dbg_radio_timer;
task *dbg_radio_task;
static u32_t _dbg_radio_state = 0;
static void dbg_radio_task_func(u32_t i, void *p) {
  switch (_dbg_radio_state) {
  case 0:
    NRF905_IMPL_init();
    break;
  case 1:
    NRF905_IMPL_set_conf(NULL);
    break;
  case 2:
    NRF905_IMPL_set_addr();
    break;
  default:
    NRF905_IMPL_tx();
    break;
  }
  _dbg_radio_state++;
}
#endif

// main entry from bootstrap

int main(void) {
  s32_t res;
  enter_critical();
  PROC_base_init();
  SYS_init();
  UART_init();
  UART_assure_tx(_UART(0), TRUE);
  PROC_periph_init();

  exit_critical();

  IO_define(IOSTD, io_uart, UARTSTDIN);

  print("\n\n\nHardware initialization done\n");

  print("Stack 0x%08x -- 0x%08x\n", STACK_START, STACK_END);

  print("Subsystem initialization done\n");

  TASK_init();

  SPI_init();

  CLI_init();

#ifdef CONFIG_ADC
  {
    int i = 16;
    uint32_t s = 0;
    while (i--) {
      s ^= (ADC_sample() << (i*2));
    }
    rand_seed(s);
  }
#else
  rand_seed(0xd0decaed ^ SYS_get_tick());
#endif
#ifdef DBG_KERNEL_TASK_BLINKY
  dbg_blinky_task = TASK_create(dbg_blinky_task_func, TASK_STATIC);
  TASK_start_timer(dbg_blinky_task, &dbg_blinky_task_timer, 0,0,0,950,"dbg_blink");
#endif

#ifdef DBG_RADIO
  dbg_radio_task = TASK_create(dbg_radio_task_func, TASK_STATIC);
  TASK_start_timer(dbg_radio_task, &dbg_radio_timer, 0,0,0,1000,"radio");
#endif

  while (1) {
    while (TASK_tick());
    TASK_wait();
  }

  return 0;
}

// assert failed handler from stmlib? TODO

void assert_failed(uint8_t* file, uint32_t line) {
  SYS_assert((char*)file, (s32_t)line);
}

// user hardfault handler

#if USER_HARDFAULT

void **HARDFAULT_PSP;
//register void *stack_pointer asm("sp");
volatile void *stack_pointer;
volatile unsigned int stacked_r0;
volatile unsigned int stacked_r1;
volatile unsigned int stacked_r2;
volatile unsigned int stacked_r3;
volatile unsigned int stacked_r12;
volatile unsigned int stacked_lr;
volatile unsigned int stacked_pc;
volatile unsigned int stacked_psr;

void hard_fault_handler_c (unsigned int * hardfault_args)
{
  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);

  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr = ((unsigned long) hardfault_args[5]);
  stacked_pc = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);

  u32_t bfar = SCB->BFAR;
  u32_t cfsr = SCB->CFSR;
  u32_t hfsr = SCB->HFSR;
  u32_t dfsr = SCB->DFSR;
  u32_t afsr = SCB->AFSR;

  // Hijack the process stack pointer to make backtrace work
  asm("mrs %0, psp" : "=r"(HARDFAULT_PSP) : :);
  stack_pointer = HARDFAULT_PSP;

  u8_t io = IODBG;

  IO_blocking_tx(io, TRUE);

  IO_tx_flush(io);

  ioprint(io, TEXT_BAD("\n!!! HARDFAULT !!!\n\n"));
  ioprint(io, "Stacked registers:\n");
  ioprint(io, "  pc:   0x%08x\n", stacked_pc);
  ioprint(io, "  lr:   0x%08x\n", stacked_lr);
  ioprint(io, "  psr:  0x%08x\n", stacked_psr);
  ioprint(io, "  sp:   0x%08x\n", stack_pointer);
  ioprint(io, "  r0:   0x%08x\n", stacked_r0);
  ioprint(io, "  r1:   0x%08x\n", stacked_r1);
  ioprint(io, "  r2:   0x%08x\n", stacked_r2);
  ioprint(io, "  r3:   0x%08x\n", stacked_r3);
  ioprint(io, "  r12:  0x%08x\n", stacked_r12);
  ioprint(io, "\nFault status registers:\n");
  ioprint(io, "  BFAR: 0x%08x\n", bfar);
  ioprint(io, "  CFSR: 0x%08x\n", cfsr);
  ioprint(io, "  HFSR: 0x%08x\n", hfsr);
  ioprint(io, "  DFSR: 0x%08x\n", dfsr);
  ioprint(io, "  AFSR: 0x%08x\n", afsr);
  ioprint(io, "\n");
  if (cfsr & (1<<(7+0))) {
    ioprint(io, "MMARVALID: MemMan 0x%08x\n", SCB->MMFAR);
  }
  if (cfsr & (1<<(4+0))) {
    ioprint(io, "MSTKERR: MemMan error during stacking\n");
  }
  if (cfsr & (1<<(3+0))) {
    ioprint(io, "MUNSTKERR: MemMan error during unstacking\n");
  }
  if (cfsr & (1<<(1+0))) {
    ioprint(io, "DACCVIOL: MemMan memory access violation, data\n");
  }
  if (cfsr & (1<<(0+0))) {
    ioprint(io, "IACCVIOL: MemMan memory access violation, instr\n");
  }

  if (cfsr & (1<<(7+8))) {
    ioprint(io, "BFARVALID: BusFlt 0x%08x\n", SCB->BFAR);
  }
  if (cfsr & (1<<(4+8))) {
    ioprint(io, "STKERR: BusFlt error during stacking\n");
  }
  if (cfsr & (1<<(3+8))) {
    ioprint(io, "UNSTKERR: BusFlt error during unstacking\n");
  }
  if (cfsr & (1<<(2+8))) {
    ioprint(io, "IMPRECISERR: BusFlt error during data access\n");
  }
  if (cfsr & (1<<(1+8))) {
    ioprint(io, "PRECISERR: BusFlt error during data access\n");
  }
  if (cfsr & (1<<(0+8))) {
    ioprint(io, "IBUSERR: BusFlt bus error\n");
  }

  if (cfsr & (1<<(9+16))) {
    ioprint(io, "DIVBYZERO: UsaFlt division by zero\n");
  }
  if (cfsr & (1<<(8+16))) {
    ioprint(io, "UNALIGNED: UsaFlt unaligned access\n");
  }
  if (cfsr & (1<<(3+16))) {
    ioprint(io, "NOCP: UsaFlt execute coprocessor instr\n");
  }
  if (cfsr & (1<<(2+16))) {
    ioprint(io, "INVPC: UsaFlt general\n");
  }
  if (cfsr & (1<<(1+16))) {
    ioprint(io, "INVSTATE: UsaFlt execute ARM instr\n");
  }
  if (cfsr & (1<<(0+16))) {
    ioprint(io, "UNDEFINSTR: UsaFlt execute bad instr\n");
  }

  if (hfsr & (1<<31)) {
    ioprint(io, "DEBUGEVF: HardFlt debug event\n");
  }
  if (hfsr & (1<<30)) {
    ioprint(io, "FORCED: HardFlt SVC/BKPT within SVC\n");
  }
  if (hfsr & (1<<1)) {
    ioprint(io, "VECTBL: HardFlt vector fetch failed\n");
  }

  SYS_dump_trace(IODBG);

  while(1);
}
#endif
