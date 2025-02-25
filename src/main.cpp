#include <lvgl.h>
#include <TFT_eSPI.h>
#include "demos\lv_demos.h"
#include "gui_guider.h"
#include "events_init.h" //用引号括起来的，这说明编译器会先在当前文件所在的目录里找这个头文件
#include "OneButton.h"

lv_ui guider_ui;

#define PIN_INPUT 5

OneButton button(PIN_INPUT, true);//pin : 按钮的pin角 activeLow : true:按下为低电平 false : 按下为高电平 pullupActive : 如果有上拉电阻就激活上拉电阻

static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */


void doubleclick()
{
  Serial.println("doubleclick");
}
void click()
{
  Serial.println("click");
}
void longclick()
{
  Serial.println("longclick");
}


/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp_drv );
}


void setup()
{
    Serial.begin(115200);
    
  button.reset();//清除一下按钮状态机的状态
  button.attachClick(click);
  button.attachDoubleClick(doubleclick);
  button.attachLongPressStart(longclick);


    lv_init();

    tft.begin();          /* TFT init */
    tft.setRotation( 3 ); /* Landscape orientation, flipped */

    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );
  
    setup_ui( &guider_ui );
    events_init( &guider_ui );
 
}

void loop()
{
    // Serial.println("tft & lvgl");
    button.tick();
    lv_timer_handler(); /* let the GUI do its work */
    delay( 5 );
}
