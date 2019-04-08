/*
 * Author: downey <linux-downey@sina.com>
 * Copyright (C) 2019  Seeed Technology Co.,Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <glob.h>
#include <dirent.h>
#include <mraa/common.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#include "arm/jetson_nano.h"
#include "common.h"

#define PLATFORM_NAME_JETSON_NANO_REV_BETA "Jetson Nano Beta"
#define PLATFORM_JETSON_NANO_REV_BETA 1
#define MMAP_PATH         "/dev/mem"
#define PLATFORM_SYS_PATH "/sys/devices/platform/"
#define SYSFS_CLASS_PWM   "/sys/class/pwm/"
#define MAX_SIZE          64

// TODO, gpio maping
#define _GPIO_MAP         0

#if _GPIO_MAP
#define BCM2835_PERI_BASE 0x20000000
#define BCM2835_BLOCK_SIZE (4 * 1024)

#define BCM2836_PERI_BASE 0x3f000000
#define BCM2836_BLOCK_SIZE (4 * 1024)

#define BCM2837_PERI_BASE (0x3F000000)
#define BCM2837_BLOCK_SIZE (4 * 1024)
#define BCM283X_GPSET0 0x001c
#define BCM283X_GPCLR0 0x0028
#define BCM2835_GPLEV0 0x0034

#define GPIO_OFFSET (0x200000)
#define CLOCK_OFFSET (0x101000)

#define JETSON_NANO_BASE  0x6000d000
#define JETSON_NANO_BLOCK_SIZE (4 * 1024)

#endif


#define DEFAULT_PERIOD_US 500

#define MIN_PERIOD_US 1
#define MAX_PERIOD_US 1000000

#if _GPIO_MAP
static volatile unsigned* clk_reg = NULL;
static volatile unsigned* gpio_reg = NULL;

// GPIO MMAP
static uint8_t* mmap_reg = NULL;
static int mmap_fd = 0;
static int mmap_size;
static unsigned int mmap_count = 0;

static uint32_t peripheral_base = JETSON_NANO_BASE;
static uint32_t block_size = JETSON_NANO_BLOCK_SIZE;
#endif

static int platform_detected = 0;

typedef struct {
    unsigned int index;
    unsigned int pid;
    const char* chip;
} pwm_t;
static pwm_t pwm_table[] = {
    { 0, 32,  "30660000" }, // "PIN32", PWM1
    { 0, 33,  "30670000" }, // "PIN33", PWM2
    { 0, 15,  "30680000" }, // "PIN15", PWM3
};

static const pwm_t*
pwm_from_pin(int pin)
{
    size_t num_chips = sizeof(pwm_table) / sizeof(pwm_table[0]);
    int i = 0;

    for (i = 0; i < num_chips; i++) {
        if (pwm_table[i].pid == pin) {
            return &pwm_table[i];
        }
    }

    return NULL;
}

static mraa_result_t
build_path(const char* partial_path, const char* prefix, char* full_path, size_t full_path_len)
{
    glob_t results;
    size_t len = strlen(partial_path) + strlen(prefix) + 5;
    char* pattern = malloc(len);
    snprintf(pattern, len, "%s/%s*", partial_path, prefix);

    int err = glob(pattern, 0, NULL, &results);
    free(pattern);
    if (err != 0) {
        globfree(&results);
        if (err == GLOB_NOSPACE)
            return MRAA_ERROR_UNSPECIFIED;
        else
            return MRAA_ERROR_UNSPECIFIED;
    }

    // We will return the first match
    strncpy(full_path, results.gl_pathv[0], full_path_len);

    // Free memory
    globfree(&results);

    return MRAA_SUCCESS;
}

// Given a pin, find the chip id (eg. pwmchipX)
static mraa_result_t
mraa_get_pwm_chip_id(int pin, int* chip_id)
{
    // Get device path
    mraa_result_t ret = MRAA_ERROR_UNSPECIFIED;

    const pwm_t* pwm_chip = pwm_from_pin(pin);

    if (pwm_chip) {
        // /sys/devices/platform/30660000.*
        char sys_path[128];
        ret = build_path(PLATFORM_SYS_PATH, pwm_chip->chip, sys_path, sizeof(sys_path));

        if (ret != MRAA_SUCCESS)
            return ret;

        // sys path should now be something like
        // /sys/devices/platform/30660000.pwm/
        char addr_path[128];
        ret = build_path(sys_path, "pwm/pwmchip", addr_path, sizeof(addr_path));

        if (ret != MRAA_SUCCESS)
            return ret;

        // Grab the integer chip id
        char* chip_id_pos = strstr(addr_path, "pwmchip");

        if (chip_id_pos) {
            chip_id_pos += strlen("pwmchip");
            *chip_id = strtol(chip_id_pos, NULL, 10);
            ret = MRAA_SUCCESS;
        }
    }

    return ret;
}

#if _GPIO_MAP
/**
* Memory map an arbitrary address
*/
static volatile unsigned*
mmap_reg_addr(unsigned long base_addr)
{
    int mem_fd = 0;
    void* reg_addr_map = MAP_FAILED;

    /* open /dev/mem.....need to run program as root i.e. use sudo or su */
    if (!mem_fd) {
        if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
            perror("can't open /dev/mem");
            return NULL;
        }
    }

    /* mmap IO */
    reg_addr_map =
    mmap(NULL,                               // Any adddress in our space will do
         block_size,                         // Map length
         PROT_READ | PROT_WRITE | PROT_EXEC, // Enable reading & writting to mapped memory
         MAP_SHARED | MAP_LOCKED,            // Shared with other processes
         mem_fd,                             // File to map
         base_addr                           // Offset to base address
         );

    if (reg_addr_map == MAP_FAILED) {
        perror("mmap error");
        close(mem_fd);
        return NULL;
    }

    if (close(mem_fd) < 0) { // No need to keep mem_fd open after mmap
        // i.e. we can close /dev/mem
        perror("couldn't close /dev/mem file descriptor");
        return NULL;
    }
    return (volatile unsigned*) reg_addr_map;
}

/**
* Memory map gpio registers
*/
static int8_t
mmap_regs()
{
    // Already mapped!
    if (clk_reg || gpio_reg) {
        return 0;
    }

    clk_reg = mmap_reg_addr(peripheral_base + CLOCK_OFFSET);
    gpio_reg = mmap_reg_addr(peripheral_base + GPIO_OFFSET);

    // If all 3 weren't mapped correctly
    if (!(clk_reg && gpio_reg)) {
        return -1;
    }

    return 0;
}

#endif

mraa_pwm_context
mraa_jetson_nano_pwm_init_replace(int pin)
{
    char devpath[MAX_SIZE];
    char chippath[MAX_SIZE];
    // char pinmode[5];

    if (plat == NULL) {
        syslog(LOG_ERR, "pwm: Platform Not Initialised");
        return NULL;
    }
    if (plat->pins[pin].capabilities.pwm != 1) {
        syslog(LOG_ERR, "pwm: pin %d not capable of pwm", pin);
        return NULL;
    }

    /*
    // TODO
    if (pin == (BUILD_PIN(P9, 28))) {
        sprintf(pinmode, "pwm2");
    } else {
        sprintf(pinmode, "pwm");
    }

    if (set_pin_mode(pin, pinmode) != MRAA_SUCCESS) {
        syslog(LOG_ERR, "pwm: pin %d not capable of pwm", pin);
        return NULL;
    }
    */

    int chip_id = 0;
    if (mraa_get_pwm_chip_id(pin, &chip_id) != MRAA_SUCCESS) {
        syslog(LOG_ERR, "pwm: Unable to find chip ID for pin %d", pin);
        return NULL;
    }

    sprintf(chippath, SYSFS_CLASS_PWM "pwmchip%u", chip_id);

    if (!mraa_file_exist(chippath)) {
        syslog(LOG_ERR, "pwm: Chip path %s does not exist for pin %d", chippath, pin);
        return NULL;
    }

    const pwm_t* pwm_chip = pwm_from_pin(pin);

    if (!pwm_chip) {
        syslog(LOG_ERR, "pwm: No pwm definition for pin %d", pin);
        return NULL;
    }

    sprintf(devpath, SYSFS_CLASS_PWM "pwmchip%u/pwm%u", chip_id, pwm_chip->index);

    if (!mraa_file_exist(devpath)) {
        FILE* fh;
        char exportpath[MAX_SIZE];
        sprintf(exportpath, "%s/export", chippath);
        fh = fopen(exportpath, "w");
        if (fh == NULL) {
            syslog(LOG_ERR, "pwm: Failed to open %s for writing, check access "
                            "rights for user",
                   exportpath);
            return NULL;
        }
        if (fprintf(fh, "%d", pwm_chip->index) < 0) {
            syslog(LOG_ERR, "pwm: Failed to write to CapeManager for pin %d", pin);
        }
        fclose(fh);
    }

    if (mraa_file_exist(devpath)) {
        mraa_pwm_context dev = (mraa_pwm_context) calloc(1, sizeof(struct _pwm));
        if (dev == NULL)
            return NULL;
        dev->duty_fp = -1;
        dev->chipid = chip_id;
        dev->pin = pwm_chip->index;
        dev->period = -1;
        return dev;
    } else
        syslog(LOG_ERR, "pwm: pin %d not initialized", pin);
    return NULL;
}

/*No spidev under /dev dir.*/
mraa_result_t
mraa_jetson_nano_spi_init_pre(int index)
{
    char devpath[MAX_SIZE];
    sprintf(devpath, "/dev/spidev%u.0", plat->spi_bus[index].bus_id);
    if (!mraa_file_exist(devpath)) {
        syslog(LOG_ERR, "spi: Device not initialized");
        if (!mraa_file_exist(devpath)) {
            return MRAA_ERROR_NO_RESOURCES;
        }
    }
    return MRAA_SUCCESS;
}

/*I2C1 connect to connector.*/
mraa_result_t
mraa_jetson_nano_i2c_init_pre(unsigned int bus)
{
    char devpath[MAX_SIZE];
    sprintf(devpath, "/dev/i2c-%u", bus);
    if (!mraa_file_exist(devpath)) {
        syslog(LOG_INFO, "i2c: trying modprobe for i2c-dev");
        system("modprobe i2c_dev >/dev/null 2>&1");
    }
    if (!mraa_file_exist(devpath)) {
        syslog(LOG_ERR, "i2c: Device not initialized");
        return MRAA_ERROR_NO_RESOURCES;
    }
    return MRAA_SUCCESS;
}

#if _GPIO_MAP
int
mraa_jetson_nano_mmap_read(mraa_gpio_context dev)
{
    uint32_t value = *(volatile uint32_t*) (mmap_reg + BCM2835_GPLEV0 + (dev->pin / 32) * 4);
    if (value & (uint32_t)(1 << (dev->pin % 32))) {
        return 1;
    }
    return 0;
}

mraa_result_t
mraa_jetson_nano_mmap_write(mraa_gpio_context dev, int value)
{
    if (value) {
        *(volatile uint32_t*) (mmap_reg + BCM283X_GPSET0 + (dev->pin / 32) * 4) =
        (uint32_t)(1 << (dev->pin % 32));
    } else {
        *(volatile uint32_t*) (mmap_reg + BCM283X_GPCLR0 + (dev->pin / 32) * 4) =
        (uint32_t)(1 << (dev->pin % 32));
    }
    return MRAA_SUCCESS;
}

static mraa_result_t
mraa_jetson_nano_mmap_unsetup()
{
    if (mmap_reg == NULL) {
        syslog(LOG_ERR, "Jetson nano mmap: null register cant unsetup");
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    munmap(mmap_reg, mmap_size);
    mmap_reg = NULL;
    if (close(mmap_fd) != 0) {
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    return MRAA_SUCCESS;
}

mraa_result_t
mraa_jetson_nano_mmap_setup(mraa_gpio_context dev, mraa_boolean_t en)
{
    if (dev == NULL) {
        syslog(LOG_ERR, "Jetson nano mmap: context not valid");
        return MRAA_ERROR_INVALID_HANDLE;
    }

    if (en == 0) {
        if (dev->mmap_write == NULL && dev->mmap_read == NULL) {
            syslog(LOG_ERR, "Jetson nano mmap: can't disable disabled mmap gpio");
            return MRAA_ERROR_INVALID_PARAMETER;
        }
        dev->mmap_write = NULL;
        dev->mmap_read = NULL;
        mmap_count--;
        if (mmap_count == 0) {
            return mraa_jetson_nano_mmap_unsetup();
        }
        return MRAA_SUCCESS;
    }

    if (dev->mmap_write != NULL && dev->mmap_read != NULL) {
        syslog(LOG_ERR, "Jetson_nano mmap: can't enable enabled mmap gpio");
        return MRAA_ERROR_INVALID_PARAMETER;
    }

    // Might need to make some elements of this thread safe.
    // For example only allow one thread to enter the following block
    // to prevent mmap'ing twice.
    if (mmap_reg == NULL) {
        if ((mmap_fd = open(MMAP_PATH, O_RDWR)) < 0) {
            syslog(LOG_ERR, "Jetson nano map: unable to open resource0 file");
            return MRAA_ERROR_INVALID_HANDLE;
        }

        mmap_reg = (uint8_t*) mmap(NULL, block_size, PROT_READ | PROT_WRITE, MAP_FILE | MAP_SHARED,
                                   mmap_fd, peripheral_base + GPIO_OFFSET);
        if (mmap_reg == MAP_FAILED) {
            syslog(LOG_ERR, "Jetson nano mmap: failed to mmap");
            mmap_reg = NULL;
            close(mmap_fd);
            return MRAA_ERROR_NO_RESOURCES;
        }
    }
    dev->mmap_write = &mraa_jetson_nano_mmap_write;
    dev->mmap_read = &mraa_jetson_nano_mmap_read;
    mmap_count++;

    return MRAA_SUCCESS;
}

mraa_result_t
mraa_jetson_nano_spi_frequency_replace(mraa_spi_context dev, int hz)
{
    dev->clock = hz;
    return MRAA_SUCCESS;
}
#endif

mraa_board_t*
mraa_jetson_nano()
{
    mraa_board_t* b = (mraa_board_t*) calloc(1, sizeof(mraa_board_t));
    int pin_base = 1000;

    if (b == NULL) {
        return NULL;
    }

    b->platform_name  = PLATFORM_NAME_JETSON_NANO_REV_BETA;
    platform_detected = PLATFORM_JETSON_NANO_REV_BETA;
    b->phy_pin_count  = MRAA_JETSON_NANO_REV_BETA_PINCOUNT;

    b->aio_count = 0;
    b->adc_raw = 0;
    b->adc_supported = 0;
    b->pwm_default_period = DEFAULT_PERIOD_US;
    b->pwm_max_period = MAX_PERIOD_US;
    b->pwm_min_period = MIN_PERIOD_US;

    if (b->phy_pin_count == 0) {
        free(b);
        syslog(LOG_ERR, "Jetson nano Dev Board: Failed to detect platform revision");
        return NULL;
    }

    b->adv_func = (mraa_adv_func_t*) calloc(1, sizeof(mraa_adv_func_t));
    if (b->adv_func == NULL) {
        free(b);
        return NULL;
    }

    b->pins = (mraa_pininfo_t*) calloc(b->phy_pin_count, sizeof(mraa_pininfo_t));
    if (b->pins == NULL) {
        free(b->adv_func);
        free(b);
        return NULL;
    }

    // Detect the base of the gpiochip
    DIR* gpio_dir = opendir("/sys/class/gpio");
    if (gpio_dir == NULL) {
        free(b->adv_func);
        free(b);
        return NULL;
    }

    struct dirent* child;
    while ((child = readdir(gpio_dir)) != NULL) {
        if (strstr(child->d_name, "gpiochip")) {
            char chip_path[MAX_SIZE];
            int base = 0;

            sprintf(chip_path, "/sys/class/gpio/%s/label", child->d_name);
            if (mraa_file_contains(chip_path, "gpio")) {
                if (mraa_atoi(child->d_name + 8, &base) != MRAA_SUCCESS) {
                    free(b->adv_func);
                    free(b);
                    return NULL;
                }
                if (base < pin_base) {
                    pin_base = base;
                }
            }
        }
    }

    b->adv_func->spi_init_pre = &mraa_jetson_nano_spi_init_pre;
    b->adv_func->i2c_init_pre = &mraa_jetson_nano_i2c_init_pre;
    #if _GPIO_MAP
    b->adv_func->gpio_mmap_setup = &mraa_jetson_nano_mmap_setup;
    b->adv_func->spi_frequency_replace = &mraa_jetson_nano_spi_frequency_replace;
    #endif
    b->adv_func->pwm_init_replace = &mraa_jetson_nano_pwm_init_replace;

    strncpy(b->pins[0].name, "INVALID", MRAA_PIN_NAME_SIZE);
    b->pins[0].capabilities = (mraa_pincapabilities_t){ 0, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[1].name, "3V3", MRAA_PIN_NAME_SIZE);
    b->pins[1].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[2].name, "5V", MRAA_PIN_NAME_SIZE);
    b->pins[2].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    // IMX8M I2C2_SDA, GPIO5_IO17
    strncpy(b->pins[3].name, "SDA0", MRAA_PIN_NAME_SIZE);
    b->pins[3].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 1, 0, 0 };
    b->pins[3].gpio.pinmap = pin_base + 72;
    b->pins[3].gpio.mux_total = 0;
    b->pins[3].i2c.pinmap = 0;
    b->pins[3].i2c.mux_total = 0;

    strncpy(b->pins[4].name, "5V", MRAA_PIN_NAME_SIZE);
    b->pins[4].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    // IMX8M I2C2_SCL, GPIO5_IO16
    strncpy(b->pins[5].name, "SCL0", MRAA_PIN_NAME_SIZE);
    b->pins[5].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 1, 0, 0 };
    b->pins[5].gpio.pinmap = pin_base + 73;
    b->pins[5].gpio.mux_total = 0;
    b->pins[5].i2c.pinmap = 0;
    b->pins[5].i2c.mux_total = 0;

    strncpy(b->pins[6].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[6].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    // IMX8M UART3_TXd, GPIO5_IO27
    strncpy(b->pins[7].name, "GPIO4", MRAA_PIN_NAME_SIZE);
    b->pins[7].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[7].gpio.pinmap = pin_base + 216;
    b->pins[7].gpio.mux_total = 0;

    // IMX8M UART1_TXD, GPIO5_IO23
    strncpy(b->pins[8].name, "UART_TX", MRAA_PIN_NAME_SIZE);
    b->pins[8].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 1 };
    b->pins[8].gpio.pinmap = pin_base + 0;
    b->pins[8].gpio.mux_total = 0;
    b->pins[8].uart.parent_id = 0;
    b->pins[8].uart.mux_total = 0;

    strncpy(b->pins[9].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[9].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    // IMX8M UART1_RXD, GPIO5_IO22
    strncpy(b->pins[10].name, "UART_RX", MRAA_PIN_NAME_SIZE);
    b->pins[10].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 1 };
    b->pins[10].gpio.pinmap = pin_base + 0;
    b->pins[10].gpio.mux_total = 0;
    b->pins[10].uart.parent_id = 0;
    b->pins[10].uart.mux_total = 0;

    // IMX8M UART3_RXD, GPIO5_IO26
    strncpy(b->pins[11].name, "GPIO17", MRAA_PIN_NAME_SIZE);
    b->pins[11].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[11].gpio.pinmap = pin_base + 50;
    b->pins[11].gpio.mux_total = 0;

    // IMX8M SAI1_TXC,  GPIO4_IO11
    strncpy(b->pins[12].name, "GPIO18", MRAA_PIN_NAME_SIZE);
    b->pins[12].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[12].gpio.pinmap = pin_base + 79;
    b->pins[12].gpio.mux_total = 0;
    b->pins[12].pwm.pinmap = 0;

    // IMX8M GPIO6, GPIO6
    strncpy(b->pins[13].name, "GPIO27", MRAA_PIN_NAME_SIZE);
    b->pins[13].gpio.pinmap = pin_base + 14;
    b->pins[13].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[13].gpio.mux_total = 0;

    strncpy(b->pins[14].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[14].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    // IMX8M PWM3, GPIO1_IO14
    strncpy(b->pins[15].name, "GPIO22", MRAA_PIN_NAME_SIZE);
    b->pins[15].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[15].gpio.pinmap = pin_base + 194;
    b->pins[15].gpio.mux_total = 0;

    // IMX8M NAND_DATA03, GPIO3_IO09
    strncpy(b->pins[16].name, "GPIO23", MRAA_PIN_NAME_SIZE);
    b->pins[16].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[16].gpio.pinmap = pin_base + 232;
    b->pins[16].gpio.mux_total = 0;

    strncpy(b->pins[17].name, "3V3", MRAA_PIN_NAME_SIZE);
    b->pins[17].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    // IMX8M ECSPI2_SCLK, GPIO5_IO10
    strncpy(b->pins[18].name, "GPIO24", MRAA_PIN_NAME_SIZE);
    b->pins[18].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[18].gpio.pinmap = pin_base + 15;
    b->pins[18].gpio.mux_total = 0;

    // IMX8M ECSPI1_MOSI, GPIO5_IO07
    strncpy(b->pins[19].name, "SPI_MOSI", MRAA_PIN_NAME_SIZE);
    b->pins[19].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 1, 0, 0, 0 };
    b->pins[19].gpio.pinmap = pin_base + 16;
    b->pins[19].gpio.mux_total = 0;
    b->pins[19].spi.pinmap = 0;
    b->pins[19].spi.mux_total = 0;

    strncpy(b->pins[20].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[20].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    // IMX8M ECSPI1_MISO, GPIO5_IO08
    strncpy(b->pins[21].name, "SPI_MISO", MRAA_PIN_NAME_SIZE);
    b->pins[21].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 1, 0, 0, 0 };
    b->pins[21].gpio.pinmap = pin_base + 17;
    b->pins[21].gpio.mux_total = 0;
    b->pins[21].spi.pinmap = 0;
    b->pins[21].spi.mux_total = 0;

    // IMX8M ECSPI2_MISO, GPIO5_IO12
    strncpy(b->pins[22].name, "GPIO25", MRAA_PIN_NAME_SIZE);
    b->pins[22].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[22].gpio.pinmap = pin_base + 13;
    b->pins[22].gpio.mux_total = 0;

    // IMX8M ECSPI1_SCLK, GPIO5_IO06
    strncpy(b->pins[23].name, "SPI_CLK", MRAA_PIN_NAME_SIZE);
    b->pins[23].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 1, 0, 0, 0 };
    b->pins[23].gpio.pinmap = pin_base + 18;
    b->pins[23].gpio.mux_total = 0;
    b->pins[23].spi.pinmap = 0;
    b->pins[23].spi.mux_total = 0;

    // IMX8M ECSPI1_SS0, GPIO5_IO09
    strncpy(b->pins[24].name, "SPI_CS0", MRAA_PIN_NAME_SIZE);
    b->pins[24].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 1, 0, 0, 0 };
    b->pins[24].gpio.pinmap = pin_base + 19;
    b->pins[24].gpio.mux_total = 0;
    b->pins[24].spi.pinmap = 0;
    b->pins[24].spi.mux_total = 0;

    strncpy(b->pins[25].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[25].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    // IMX8M ECSPI1_SS1, GPIO???
    strncpy(b->pins[26].name, "SPI_CS1", MRAA_PIN_NAME_SIZE);
    b->pins[26].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 1, 0, 0, 0 };
    b->pins[26].gpio.pinmap = pin_base + 20;
    b->pins[26].gpio.mux_total = 0;
    b->pins[26].spi.pinmap = 0;
    b->pins[26].spi.mux_total = 0;

    // IMX8M I2C3_SDA, GPIO5_IO19
    strncpy(b->pins[27].name, "ID_SD", MRAA_PIN_NAME_SIZE);
    b->pins[27].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[27].gpio.pinmap = pin_base + 0;
    b->pins[27].gpio.mux_total = 0;

    // IMX8M I2C3_SCL, GPIO5_IO18
    strncpy(b->pins[28].name, "ID_SC", MRAA_PIN_NAME_SIZE);
    b->pins[28].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[28].gpio.pinmap = pin_base + 0;
    b->pins[28].gpio.mux_total = 0;

    // IMX8M GPIO7, GPIO7
    strncpy(b->pins[29].name, "GPIO05", MRAA_PIN_NAME_SIZE);
    b->pins[29].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[29].gpio.pinmap = pin_base + 149;
    b->pins[29].gpio.mux_total = 0;

    strncpy(b->pins[30].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[30].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    // IMX8M GPIO8, GPIO8
    strncpy(b->pins[31].name, "GPIO06", MRAA_PIN_NAME_SIZE);
    b->pins[31].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[31].gpio.pinmap = pin_base + 200;
    b->pins[31].gpio.mux_total = 0;

    // IMX8M PWM1, GPIO1_IO01
    strncpy(b->pins[32].name, "GPIO12", MRAA_PIN_NAME_SIZE);
    b->pins[32].capabilities = (mraa_pincapabilities_t){ 1, 1, 1, 0, 0, 0, 0, 0 };
    b->pins[32].gpio.pinmap = pin_base + 168;
    b->pins[32].gpio.mux_total = 0;
    b->pins[32].pwm.pinmap = 1;

    // IMX8M PWM2, GPIO1_IO13
    strncpy(b->pins[33].name, "GPIO13", MRAA_PIN_NAME_SIZE);
    b->pins[33].capabilities = (mraa_pincapabilities_t){ 1, 1, 1, 0, 0, 0, 0, 0 };
    b->pins[33].gpio.pinmap = pin_base + 38;
    b->pins[33].gpio.mux_total = 0;

    strncpy(b->pins[34].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[34].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    // IMX8M SAI1_TXFS, GPIO4_IO10
    strncpy(b->pins[35].name, "GPIO19", MRAA_PIN_NAME_SIZE);
    b->pins[35].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[35].gpio.pinmap = pin_base + 76;
    b->pins[35].gpio.mux_total = 0;

    // IMX8M ECSPI2_SS0, GPIO5_IO13
    strncpy(b->pins[36].name, "GPIO16", MRAA_PIN_NAME_SIZE);
    b->pins[36].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[36].gpio.pinmap = pin_base + 51;
    b->pins[36].gpio.mux_total = 0;

    // IMX8M NAND_DATA07, GPIO3_IO13
    strncpy(b->pins[37].name, "GPIO26", MRAA_PIN_NAME_SIZE);
    b->pins[37].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[37].gpio.pinmap = pin_base + 12;
    b->pins[37].gpio.mux_total = 0;

    // IMX8M SAI1_RXD0, GPIO4_IO02
    strncpy(b->pins[38].name, "GPIO20", MRAA_PIN_NAME_SIZE);
    b->pins[38].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[38].gpio.pinmap = pin_base + 77;
    b->pins[38].gpio.mux_total = 0;

    strncpy(b->pins[39].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[39].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    // IMX8M SAI1_TXD0, GPIO4_IO12
    strncpy(b->pins[40].name, "GPIO21", MRAA_PIN_NAME_SIZE);
    b->pins[40].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[40].gpio.pinmap = pin_base + 78;
    b->pins[40].gpio.mux_total = 0;

    b->gpio_count = 0;
    int i;
    for (i = 0; i < b->phy_pin_count; i++) {
        if (b->pins[i].capabilities.gpio) {
            b->gpio_count++;
        }
    }

    // BUS DEFINITIONS
    b->i2c_bus_count = 1;
    b->def_i2c_bus = 0;
    b->i2c_bus[0].bus_id = 1;
    b->i2c_bus[0].sda = 3;
    b->i2c_bus[0].scl = 5;

    b->spi_bus_count = 1;
    b->def_spi_bus = 0;
    b->spi_bus[0].bus_id = 32766;
    b->spi_bus[0].slave_s = 0;
    b->spi_bus[0].cs = 24;
    b->spi_bus[0].mosi = 19;
    b->spi_bus[0].miso = 21;
    b->spi_bus[0].sclk = 23;

    b->uart_dev_count = 1;
    b->def_uart_dev = 0;
    b->uart_dev[0].rx = 10;
    b->uart_dev[0].tx = 8;

    return b;
}
