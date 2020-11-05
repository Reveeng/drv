#include "lvgl/lvgl.h"
/**
infratest_keypad_init(HW_KEY_CODE_UP,LED_POINTS[0]);
infratest_keypad_init(HW_KEY_CODE_DOWN,LED_POINTS[1]);
infratest_keypad_init(HW_KEY_CODE_ZOOM,LED_POINTS[2]);
infratest_keypad_init(HW_KEY_CODE_MENU,LED_POINTS[3]);
*/


static lv_point_t LED_POINTS[] = {{25,2},{95,2},{175,2},{255,2}};

void led_pressed(lv_obj_t * led,lv_event_t event){
    if ( event == LV_EVENT_PRESSED){
        lv_led_on(led);
    }
    if (event == LV_EVENT_RELEASED){
        lv_led_off(led);
    }
}

void btn_led_test(void){
    /**********************
     *  CREATE OBJECTS
     **********************/
    lv_obj_t * led_btn1 = lv_led_create(lv_scr_act(),NULL);
    lv_obj_t * led_btn2 = lv_led_create(lv_scr_act(),NULL);
    lv_obj_t * led_btn3 = lv_led_create(lv_scr_act(),NULL);
    lv_obj_t * led_btn4 = lv_led_create(lv_scr_act(),NULL);
    lv_obj_t * label1 = lv_label_create(lv_scr_act(),NULL);
    lv_obj_t * label2 = lv_label_create(lv_scr_act(),NULL);
    lv_obj_t * label3 = lv_label_create(lv_scr_act(),NULL);
    lv_obj_t * label4 = lv_label_create(lv_scr_act(),NULL);
    /**********************
     *  SET LED LABELS
     **********************/
    lv_label_set_text(label1,"up btn");
    lv_label_set_text(label2,"down btn");
    lv_label_set_text(label3,"zoom btn");
    lv_label_set_text(label4,"menu btn");
    /**********************
     *  SET LED POSITION
     **********************/
    lv_obj_set_pos(led_btn1,20,0);
    lv_obj_set_pos(led_btn2,90,0);
    lv_obj_set_pos(led_btn3,170,0);
    lv_obj_set_pos(led_btn4,250,0);

    lv_obj_set_size(led_btn1,15,15);
    lv_obj_set_size(led_btn2,15,15);
    lv_obj_set_size(led_btn3,15,15);
    lv_obj_set_size(led_btn4,15,15);

    lv_obj_align_mid(label1,led_btn1,LV_ALIGN_OUT_BOTTOM_MID,0,10);
    lv_obj_align_mid(label2,led_btn2,LV_ALIGN_OUT_BOTTOM_MID,0,10);
    lv_obj_align_mid(label3,led_btn3,LV_ALIGN_OUT_BOTTOM_MID,0,10);
    lv_obj_align_mid(label4,led_btn4,LV_ALIGN_OUT_BOTTOM_MID,0,10);

    lv_led_off(led_btn1); lv_led_off(led_btn2); lv_led_off(led_btn3); lv_led_off(led_btn4);

    lv_obj_set_event_cb(led_btn1,led_pressed);
    lv_obj_set_event_cb(led_btn2,led_pressed);
    lv_obj_set_event_cb(led_btn3,led_pressed);
    lv_obj_set_event_cb(led_btn4,led_pressed);
}
