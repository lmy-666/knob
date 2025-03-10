
/**
 * FunctionalButton.ino - Example for the OneButtonLibrary library.
 * This is a sample sketch to show how to use OneClick library functionally on ESP32,ESP8266... 
 * 
 */
#include <Arduino.h>
#include <OneButton.h>

int key_state = -1;

class Button{
private:
  OneButton button;
  int value;
public:
  explicit Button(uint8_t pin):button(pin) {
    button.attachClick([](void *scope) { ((Button *) scope)->Clicked();}, this);
    button.attachDoubleClick([](void *scope) { ((Button *) scope)->DoubleClicked();}, this);
    button.attachLongPressStart([](void *scope) { ((Button *) scope)->LongPressed();}, this);
    
    // [ ] 捕捉列表 外部变量, ( ) 参数列表 定义一个变量 给后面的函数体使用,   ->returntype 返回值类型 函数运行结束返回的结果类型, { } 函数体 lambda表达式的逻辑实现
    //f = [x] (int y) -> int { return x + y; };
  }

  void Clicked(){
    key_state = 1;
    Serial.println("Click then value++");
    value++;
  }

  void DoubleClicked(){
    key_state = 2;
    Serial.println("DoubleClick");
  }

  void LongPressed(){
    key_state = 0;
    Serial.println("LongPress and the value is");
  }
    
  void handle(){
    button.tick();
  }
};

#define PIN_INPUT 5

Button button(PIN_INPUT);//pin : 按钮的pin角 activeLow : true:按下为低电平 false : 按下为高电平 pullupActive : 如果有上拉电阻就激活上拉电阻