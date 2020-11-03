#ifndef INFRATEST_KEYPAD_H
#define INFRATEST_KEYPAD_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "lvgl/lvgl.h"

/*********************
 *      DEFINES
 *********************/
enum {
	HW_KEY_CODE_NONE    = 0x00,

	HW_KEY_CODE_PWR     = 0x01,
	HW_KEY_CODE_UP      = 0x10,
	HW_KEY_CODE_ZOOM    = 0x20,
	HW_KEY_CODE_DOWN    = 0x40,
	HW_KEY_CODE_MENU    = 0x80,

	HW_KEY_CODE_INVAL   = 0xFF,
};

#define HW_EVENT_KEY_PRESSED	0xFF

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
 lv_indev_t * infratest_keypad(void);
 void infratest_keypad_init(void);
 void init_indev_btns(void);
 //void set_group_to_hardware_btn(lv_indev_t ** keypad_btns,lv_group_t * group);

/**********************
 *      MACROS
 **********************/


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*INFRATEST_KEYPAD_H*/
