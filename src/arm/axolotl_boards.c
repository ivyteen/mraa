/*
 * Author: Jack Shao (jacky.shaoxg@gmail.com)
 * Copyright (c) 2017 Seeed Technology Co., Ltd.
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

#include <mraa/common.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#include "arm/axolotl_boards.h"
#include "common.h"

#define DT_BASE "/proc/device-tree"
#define PLATFORM_NAME_MAX_SIZE 64

// ReSpeaker Core v2.0
#define PLATFORM_NAME_RESPEAKVER2 "ReSpeaker Core v2.0"
#define MRAA_RESPEAKER2_PHY_PIN_COUNT 13 

/*
Pin index definition for headers :

**8 pin header**
0 <- + + -> 1
2 <- + + -> 3
4 <- + + -> 5
6 <- + + -> 7
(The square pad is 0)

**Grove socket**
-+ -> 8(SCL)
-+ -> 9(SDA)
-+ -> 10(VCC)
-+ -> 11(GND)
(The silk "I2C" at the botto)

**GPIO**

| MRAA | HEADER PIN INDEX | SYSFS PIN | RK3229 PIN |
| :--- | :--------------- | :-------- | :--------- |
| 0    | 0                | 1091        | GPIO2_D3   |
| 1    | 1                | --        | VCC        |
| 2    | 2                | 1043        | GPIO1_B3   |
| 3    | 3                | 1127       | GPIO3_D7   |
| 4    | 4                | 1017        | GPIO0_C1   |
| 5    | 5                | 1067        | GPIO2_A3   |
| 6    | 6                | --        | GND        |
| 7    | 7                | 1013        | GPIO0_B5   |
| 8    | 8                | 1085        | GPIO2_C5   |
| 9    | 9                | 1084        | GPIO2_C4   |
| 10   | 10               | --        | VCC        |
| 11   | 11               | --        | GND        |

**Pixel Ring Enable Pin**
| MRAA | HEADER PIN INDEX | SYSFS PIN | RK3229 PIN |
| :--- | :--------------- | :-------- | :--------- |
| 12   |                  | 1066      |            |

**I2C**

| MRAA | HEADER PIN INDEX | SYSFS PIN | RK3229 PIN |
| :--- | :--------------- | :-------- | :--------- |
| 0    | 8                | --        | I2C2_SCL   |
| 0    | 9                | --        | I2C2_SDA   |

*/
int
mraa_add_board_respeaker2(mraa_board_t* b)
{
    unsigned int i2c2_enabled = 1;

    if (mraa_file_exist("/sys/class/i2c-dev/i2c-3")) {
        i2c2_enabled = 1;
    } else {
        i2c2_enabled = 0;
    }

    b->platform_name = PLATFORM_NAME_RESPEAKVER2;
    b->phy_pin_count = MRAA_RESPEAKER2_PHY_PIN_COUNT;
    b->gpio_count = 0;
    b->i2c_bus_count = 0;
    b->spi_bus_count = 0;
    b->aio_count = 0;
    b->uart_dev_count = 0;
    b->pwm_dev_count = 0;

    b->adv_func = (mraa_adv_func_t*) calloc(1, sizeof(mraa_adv_func_t));
    if (b->adv_func == NULL) {
        return -1;
    }

    b->pins = (mraa_pininfo_t*) calloc(b->phy_pin_count, sizeof(mraa_pininfo_t));
    if (b->pins == NULL) {
        return -1;
    }

    strncpy(b->pins[0].name, "GPIO91", MRAA_PIN_NAME_SIZE);
    b->pins[0].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[0].gpio.pinmap = 1091;
    b->pins[0].gpio.mux_total = 0;

    strncpy(b->pins[1].name, "VCC", MRAA_PIN_NAME_SIZE);
    b->pins[1].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[2].name, "GPIO43", MRAA_PIN_NAME_SIZE);
    b->pins[2].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[2].gpio.pinmap = 1043;
    b->pins[2].gpio.mux_total = 0;

    strncpy(b->pins[3].name, "GPIO127", MRAA_PIN_NAME_SIZE);
    b->pins[3].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[3].gpio.pinmap = 1127;
    b->pins[3].gpio.mux_total = 0;

    strncpy(b->pins[4].name, "GPIO17", MRAA_PIN_NAME_SIZE);
    b->pins[4].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[4].gpio.pinmap = 1017;
    b->pins[4].gpio.mux_total = 0;

    strncpy(b->pins[5].name, "GPIO67", MRAA_PIN_NAME_SIZE);
    b->pins[5].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[5].gpio.pinmap = 1067;
    b->pins[5].gpio.mux_total = 0;

    strncpy(b->pins[6].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[6].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[7].name, "GPIO13", MRAA_PIN_NAME_SIZE);
    b->pins[7].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[7].gpio.pinmap = 1013;
    b->pins[7].gpio.mux_total = 0;

    if (i2c2_enabled){
        strncpy(b->pins[8].name, "I2C2_SCL", MRAA_PIN_NAME_SIZE);
        b->pins[8].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 1, 0, 0 };
    }else{
        strncpy(b->pins[8].name, "GPIO85", MRAA_PIN_NAME_SIZE);
        b->pins[8].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 1, 0, 0 };
    }
    b->pins[8].gpio.pinmap = 1085;
    b->pins[8].gpio.mux_total = 0;
    b->pins[8].i2c.mux_total = 0;

    if (i2c2_enabled){
        strncpy(b->pins[9].name, "I2C2_SDA", MRAA_PIN_NAME_SIZE);
        b->pins[9].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 1, 0, 0 };
    }else{
        strncpy(b->pins[9].name, "GPIO84", MRAA_PIN_NAME_SIZE);
        b->pins[9].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 1, 0, 0 };
    }
    b->pins[9].gpio.pinmap = 1084;
    b->pins[9].gpio.mux_total = 0;
    b->pins[9].i2c.mux_total = 0;

    strncpy(b->pins[10].name, "VCC", MRAA_PIN_NAME_SIZE);
    b->pins[10].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[11].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[11].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };
    
    strncpy(b->pins[12].name, "GPIO66", MRAA_PIN_NAME_SIZE);
    b->pins[12].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[12].gpio.pinmap = 1066;
    b->pins[12].gpio.mux_total = 0;

    // BUS DEFINITIONS

    // I2C
    if (i2c2_enabled){
        b->i2c_bus_count += 1;
        b->i2c_bus[0].bus_id = 3;
        b->def_i2c_bus = 3;
        b->i2c_bus[0].scl = 8;  // here should be the b->pins index
        b->i2c_bus[0].sda = 9;
    }else{
        b->i2c_bus_count = 0;
    }
    
    // SPI

    // AIO
    b->adc_raw = 0;
    b->adc_supported = 0;

    // UART

    // PWM
    
    int i;
    for (i = 0; i < b->phy_pin_count; i++) {
        if (b->pins[i].capabilities.gpio) {
            b->gpio_count++;
        }
    }

    return 0;
}

mraa_board_t*
mraa_axolotl_boards()
{
    int ret = -1;

    mraa_board_t* b = (mraa_board_t*) calloc(1, sizeof(mraa_board_t));
    if (b == NULL) {
        goto failed;
    }

    // test the board model
    if (mraa_file_exist(DT_BASE "/model")) {
        if (mraa_file_contains(DT_BASE "/model", "RK3229 ReSpeaker Board V1.0")) {
            ret = mraa_add_board_respeaker2(b);
        } 
    }else{
        syslog(LOG_ERR, "mraa: Failed to detect platform model");
    }

    if (ret < 0) {
        goto failed;
    }

    return b;

failed:
    if (b) {
        if (b->pins) {
            free(b->pins);
        }
        if (b->adv_func) {
            free(b->adv_func);
        }
        free(b);
    }
    return NULL;
}
