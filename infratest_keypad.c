/**
 * keypad driver for GPPIO extender PCA 9555A
 */

/*********************
 *      INCLUDES
 *********************/
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include "infratest_keypad.h"

/*********************
 *      DEFINES
 *********************/
#define I2C_BUS         0x2
#define I2C_ADDR        0x20

/**********************
 *      TYPEDEFS
 **********************/
typedef enum {
    I2C_REG_INPUT    = 0x0,
    I2C_REG_OUTPUT   = 0x2,
    I2C_REG_POLARITY = 0x4,
    I2C_REG_CONFIG   = 0x6,
} I2C_REG;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static bool keypad_init(void);
static bool keypad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);

/**********************
 *  STATIC VARIABLES
 **********************/
static int i2c_fd;


/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void infratest_keypad_init(uint16_t btn_code, lv_point_t * point)
{
    lv_indev_drv_t indev_drv;
    if (!keypad_init())
            return;
    lv_indev_t * indev_btn;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_BUTTON;
    indev_drv.read_cb = keypad_read;
    indev_drv.user_data = &(btn_code);
    indev_btn = lv_indev_drv_register(&indev_drv);
    lv_indev_set_button_points(indev_btn,point);
    printf("keypad: registered\n");s
}


/**********************
 *   STATIC FUNCTIONS
 **********************/

/* Initialize your keypad */
static bool keypad_init(void)
{
    char filename[20];
    char polarity_buf[2];
    sprintf(filename, "/dev/i2c-%d", I2C_BUS);
    i2c_fd = open(filename, O_RDWR);
    if (i2c_fd < 0) {
        printf("keypad: Failed to open i2c-%d device\n", I2C_BUS);
        return false;
    }

    if (ioctl(i2c_fd, I2C_SLAVE, I2C_ADDR) < 0) {
        printf("keypad: Failed to set device address 0x%X\n", I2C_ADDR);
        close(i2c_fd);
        return false;
    }

    polarity_buf[0] = I2C_REG_POLARITY;
    polarity_buf[1] = 0xFF; // invert all inputs
    if (write(i2c_fd, polarity_buf, 2) != 2) {
        printf("keypad: Failed to set polarity of device 0x%X\n", I2C_ADDR);
        close(i2c_fd);
        return false;
    }

    /* to read from I2C_REG_INPUT reg */
    polarity_buf[0] = I2C_REG_INPUT;
    write(i2c_fd, polarity_buf, 2);

    printf("keypad: inited\n");
    return true;
}

/* Will be called by the library to read the mouse */
static bool keypad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    uint16_t new_byte = keypad_get_key();
    uint16_t * drv_byte = (uint16_t *)indev_drv->user_data;
    if (new_byte & *(drv_byte)){
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
    return false;
}



/*Get the currently being pressed key.  0 if no key is pressed*/
static uint16_t keypad_get_key(void)
{
    uint16_t buf = I2C_REG_INPUT;
    uint16_t res;

    // in second read PCA 9555A reports input port 1
    res = read(i2c_fd, &buf, 2);

    if (res != 2) {
        return HW_KEY_CODE_NONE;
    }

    return buf >> 8;
}
