#include "power.h"
#include "RGB.h"
void power_init(void) 
{
    pinMode(ON_OFF_PIN, OUTPUT);
    digitalWrite(ON_OFF_PIN, HIGH);  //电池始终供电
}

void power_off(void) 
{
    //关掉led
    strip.setBrightness(0);
    strip.clear();
    strip.show();//一定要show，才能冲洗led的ram


    // keep PUSH_PIN HIGH level in deep sleep mode  
    rtc_gpio_init((gpio_num_t)PUSH_BUTTON_PIN);//进入深度睡眠模式后，为了最大程度降低功耗，大部分普通 GPIO 的时钟和供电会被切断，普通 GPIO 的状态无法被保持和检测。
                                    //而 RTC GPIO 由专门的低功耗电路供电，在深度睡眠甚至芯片掉电（部分情况）时，依然能保持工作状态
    rtc_gpio_pullup_en((gpio_num_t)PUSH_BUTTON_PIN);
    rtc_gpio_pulldown_dis((gpio_num_t)PUSH_BUTTON_PIN);
    gpio_deep_sleep_hold_en();//强制芯片在深度睡眠期间保留所有 GPIO 的配置（包括方向、上下拉、中断触发类型等）。该功能对 所有 GPIO（包括普通 GPIO 和 RTC GPIO）均有效
                            //虽然配置会被保留，深度睡眠时，普通 GPIO 的数字电路会断电，仅保留寄存器配置。
    // low level will trigger wakeup
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PUSH_BUTTON_PIN, 0);//该函数只支持rtc gpio唤醒
    //RTC GPIO 可在这种低功耗状态下持续监测引脚电平，当检测到满足唤醒条件低电平时，触发唤醒机制
    esp_deep_sleep_start();

    /**
     * ESP32 从深度睡眠唤醒后会重新执行 setup 函数。因为在深度睡眠模式下，ESP32 的 CPU、大部分由 APB_CLK 驱动的外设和大部分 RAM 都会掉电 
     * 相当于完全重启。所以唤醒后代码会重新从程序起始位置运行，即会再次执行 setup 函数。
     */

}