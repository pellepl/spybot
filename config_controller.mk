CONFIG_SPYBOT_CONTROLLER = 1

ARCH_STM32 = 1
PROC_STM32F1 = 1

CONFIG_SPYBOT_APP_MASTER = 1
CONFIG_SPYBOT_APP_CLIENT = 0
CONFIG_SPYBOT_LSM = 0
CONFIG_SPYBOT_HCSR = 0
CONFIG_SPYBOT_SERVO = 0
CONFIG_SPYBOT_MOTOR = 0
CONFIG_SPYBOT_VIDEO = 1
CONFIG_SPYBOT_JOYSTICK = 1
CONFIG_SPYBOT_XBUTTONS = 1
CONFIG_SPYBOT_XLEDS = 1

# just to let eclipse get the picture
STM32F10X_MD = 1
USE_STDPERIPH_DRIVER = 1
USER_HARDFAULT=1

# generic system configs
CONFIG_MEMOPS = 1
CONFIG_VARCALL = 1
CONFIG_MATH = 1
CONFIG_MINIUTILS = 1
CONFIG_TASK_QUEUE = 1
CONFIG_RINGBUFFER = 1
CONFIG_GPIO = 1
CONFIG_UART = 1
CONFIG_SPI = 1
CONFIG_SPI_DEVICE = 1
CONFIG_NRF905 = 1

CONFIG_I2C = 1
CONFIG_I2C_DEVICE = 1
CONFIG_LSM303 = 0
CONFIG_M24M01 = 1
CONFIG_STMPE811 = 1

CONFIG_WIFI232 = 0
CONFIG_SPI_FLASH = 0
CONFIG_SPI_FLASH_M25P16 = 0
CONFIG_SPI_DEVICE_OS = 0
CONFIG_SPI_FLASH_OS = 0
CONFIG_USB_VCD = 0
CONFIG_OS = 0
CONFIG_SHARED_MEM = 0
CONFIG_BOOTLOADER = 0

include ${gensysdir}/include.mk
