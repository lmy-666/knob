#include <lvgl.h>
#include <TFT_eSPI.h>
#include <SimpleFOC.h>
#include "demos\lv_demos.h"
#include "gui_guider.h"
#include "events_init.h" //用引号括起来的，这说明编译器会先在当前文件所在的目录里找这个头文件
#include "OneButton.h"
#include "lv_port_indev.h"
#include "lv_port_disp.h"
#include "button.h"
#include "motor.h"


lv_ui guider_ui;

void setup()
{
    Serial.begin(115200);
    
    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();

    setup_ui( &guider_ui );
    events_init( &guider_ui );

    motor_init();

}

void loop()
{

     Serial.print("tft & lvgl");
     Serial.print("    loop() running on core ");
     Serial.println(xPortGetCoreID());
    // button.handle();
    // lv_timer_handler(); /* let the GUI do its work */
    delay( 1000 );
}


/**
FreeRTOS作为ESP32的操作系统，提供了多任务支持，可以使得这两个核心同时工作，
双核，包含核心0（CPU0）和核心1（CPU1），在不使用freeRTOS情况下程序是跑在核心1上，而核心0主要运行WIFI和bluetooth
Core 1（默认）通常用来运行系统的主任务和控制逻辑，也通常是应用程序的入口点（即 app_main() 函数）

两种方式创建任务
xTaskCreate()： 这个函数不指定任务运行在哪个核心上，FreeRTOS调度器将根据系统负载和可用资源自动分配任务到不同的核心。
xTaskCreatePinnedToCore()： 允许你明确指定任务应该被调度到哪个核心上运行



使用示例
TaskHandle_t myTaskHandle;
void myTask(void* parameter) {
  // 任务代码
}
 
void setup() {
  xTaskCreatePinnedToCore(myTask, "My Task", 2048, NULL, 1, &myTaskHandle, 1);
}
 
 */