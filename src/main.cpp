#include <lvgl.h>
#include <TFT_eSPI.h>
#include <SimpleFOC.h>
#include "OneButton.h"

#include "gui_guider.h"
#include "events_init.h" //用引号括起来的，这说明编译器会先在当前文件所在的目录里找这个头文件

#include "lv_port_indev.h"
#include "lv_port_disp.h"
#include "button.h"
#include "motor.h"
#include "power.h"
#include "RGB.h"
#include "display.h"

lv_ui guider_ui;

void setup()
{
  Serial.begin(115200);

  power_init();
  RGB_init();
  motor_init();

  display_init();

  setup_ui(&guider_ui);
  events_init(&guider_ui);
}

/*
总共两个核心，三个任务
电机单独一个0核心
更新硬件状态和lvgl显示共用核心1
 */

void loop()
{

  // Serial.print("tft & lvgl");
  // Serial.print("    loop() running on core ");
  // Serial.println(xPortGetCoreID());
  unsigned long currentMillis = millis(); // millis()是 Arduino 中获取开机以来毫秒数的函数
  RGB_start(currentMillis);               // 用于函数内部的时间基准计算，根据当前时间规划灯带亮灭、颜色变化的时序逻辑,避免传统 delay() 导致的程序卡顿
  // myrainbow(255);
  colorWipe(strip.Color(255,   0,   0), 20);    // Red
  strip.show();

  button.handle();

  delay(10);
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