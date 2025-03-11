#include "display.h"

TaskHandle_t handleTaskLvgl;

void TaskLvglUpdate(void *parameter)
{
    delay(1500);//等待系统初始化完毕
    // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);//FreeRTOS 操作系统中任务通知机制的关键函数，用于使任务进入阻塞状态，等待其他任务或中断发送通知
    for (;;)
    {
        lv_timer_handler(); /* let the GUI do its work */

        delay(5);
    }
}

void display_init(void)
{

    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();
    // Update display in parallel thread.
    xTaskCreatePinnedToCore(
        TaskLvglUpdate,
        "LvglThread",
        20000,
        nullptr,
        configMAX_PRIORITIES - 1, // 最高优先级
        &handleTaskLvgl,
        1);
}