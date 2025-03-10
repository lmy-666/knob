#ifndef LV_PORT_DISP_TEMPL_H
#define LV_PORT_DISP_TEMPL_H

#ifdef __cplusplus //这是一个预处理指令，用于检查当前是否使用 C++ 编译器进行编译。__cplusplus 是 C++ 编译器定义的一个预定义宏，在 C++ 编译环境中会被定义，而在 C 编译环境中不会被定义。
extern "C" {//如果当前是 C++ 编译环境，这个代码块会被执行。它告诉 C++ 编译器，接下来的代码块中的函数和变量要按照 C 语言的规则进行处理。
#endif

/*********************
 *      INCLUDES
 *********************/
#if 1
#include "lvgl.h"


/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
/* Initialize low level display driver */
void lv_port_disp_init(void);

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void);

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_PORT_DISP_TEMPL_H*/

#endif /*Disable/Enable content*/
