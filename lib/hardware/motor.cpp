#include "motor.h"

//  BLDCMotor( pole_pairs )
BLDCMotor motor = BLDCMotor(7);
//  BLDCDriver( pin_pwmA, pin_pwmB, pin_pwmC, enable (optional) )
BLDCDriver3PWM driver = BLDCDriver3PWM(15, 16, 17);

// angle set point variable
float target_angle = 1;
// instantiate the commander
Commander command = Commander(Serial);



// XKnobConfig motor_config = {
//     .num_positions = 0,
//     .position = 0,
//     .position_width_radians = 8.225806452 * _PI / 180,
//     .detent_strength_unit = 2.3,
//     .endstop_strength_unit = 1,
//     .snap_point = 1.1,
// };

XKnobConfig motor_config = {
    .num_positions = 2,
    .position = 0,
    .position_width_radians = 60 * PI / 180,
    .detent_strength_unit = 1,
    .endstop_strength_unit = 1,
    .snap_point = 0.55,
};

/* 
“死区” 是指一个角度范围，当旋钮的位置与目标中心点的偏差在这个范围内时，电机不会施加扭矩（或施加极小的扭矩）。
 “死区” 就模拟了这种机械阻尼感，确保旋钮在目标位置附近时保持稳定。 
 如果没有死区，电机可能会因为微小的角度偏差不断震动，而有了死区后，电机只会在偏差较大时响应（类似机械旋钮的 “段落感”）
 */
// 死区制动百分率，死区占每个档位宽度的百分比（例如档位宽度为 60 度时，死区占 12 度）。
static const float DEAD_ZONE_DETENT_PERCENT = 0.2;  //但是一般不起作用，除非档位宽度小于5度 5*0.2=1
// 死区RAD，死区的绝对最大弧度值（这里固定为 1 度，防止百分比计算时死区过小）。
static const float DEAD_ZONE_RAD = 1 * _PI / 180;


/*
怠速：无主动操作时的基础状态。      我这里理解的是，电机内部实际上是一直在转动的，因为有电压，只不过力没有能够驱动旋钮跟着转动
怠速状态指的是电机在没有人为干预的情况下处于静止或低速状态
怠速检查速度：存储经过平滑后的电机速度值，idle_check_velocity_ewma = 新速度 * 0.001 + 旧值 * 0.999
*/
float idle_check_velocity_ewma = 0;
/*
怠速速度平滑参数，用于指数加权移动平均（EWMA）算法，平滑电机速度数据。
EWMA 是一种滤波方法，通过赋予新数据较小权重（0.001）、旧数据较大权重（0.999）
防止电机因瞬时速度波动误判为 “非静止状态”。
*/
static const float IDLE_VELOCITY_EWMA_ALPHA = 0.001;
/* 
怠速速度每秒钟弧度，判断旋钮是否处于静止状态
当电机速度低于此阈值时，认为旋钮处于 “静止状态”，触发后续的怠速校正
*/
static const float IDLE_VELOCITY_RAD_PER_SEC = 0.05;
/*
怠速修正延迟millis
旋钮进入静止状态后，等待一段时间才开始校正位置
防止用户轻微触碰导致的误触发
*/
static const uint32_t IDLE_CORRECTION_DELAY_MILLIS = 500;
/* 
怠速校正最大角度rad
若旋钮位置与目标位置的偏差超过 5°，则认为需要用户干预，不进行自动校正
*/
static const float IDLE_CORRECTION_MAX_ANGLE_RAD = 5 * PI / 180;
/* 
怠速修正速率：控制中心位置（current_detent_center）向当前角度缓慢调整的速率
每次调整时，新角度占比 0.05%，旧中心位置占比 99.95%
缓慢修正漂移，避免因快速调整导致旋钮抖动
*/
static const float IDLE_CORRECTION_RATE_ALPHA = 0.0005;

/*
 当前控制中心位置：记录旋钮的 “目标中心位置”，用于计算偏差
 在怠速校正时，此值会逐渐接近当前实际角度（是改变中心位置而不是改变当前位置）
 */
float current_detent_center = 0;
// 上次空闲开始状态：记录旋钮进入静止状态的起始时间
uint32_t last_idle_start = 0;

// 当前角度与目标中心位置的差值
float angle_to_detent_center = 0;

/*
判断静止：通过 IDLE_VELOCITY_EWMA_ALPHA 和 IDLE_VELOCITY_RAD_PER_SEC 检测旋钮是否静止。
延迟触发：静止持续 500ms（IDLE_CORRECTION_DELAY_MILLIS）后，开始校正。
缓慢修正：以 0.0005 的速率（IDLE_CORRECTION_RATE_ALPHA）调整目标中心位置，最大修正角度为 5°（IDLE_CORRECTION_MAX_ANGLE_RAD）。
消除漂移：最终使 current_detent_center 逐渐接近实际角度，确保旋钮长期稳定。
*/

// -------------monitor--------------------

SPIClass *hspi = NULL;
static const int spiClk = 1000000; // 400KHz
static float readMySensorCallback(void)
{
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));//MSBFIRST 表示数据传输时先发送最高有效位。SPI_MODE0 定义了 SPI 的工作模式。
    /**
SPI（Serial Peripheral Interface）有四种工作模式，分别由时钟极性（CPOL）和时钟相位（CPHA）这两个参数决定。
SPI_MODE0 到 SPI_MODE3 的具体定义如下：
模式	空闲时 SCK 电平	数据采样时刻
SPI_MODE0	低电平	    时钟上升沿
SPI_MODE1	低电平	    时钟下降沿
SPI_MODE2	高电平   	时钟下降沿
SPI_MODE3	高电平	    时钟上升沿
     */
    
    digitalWrite(hspi->pinSS(), LOW); // pull SS slow to prep other end for transfer
    uint16_t ag = hspi->transfer16(0);//在一次操作中发送 16 位数据给从机，同时接收从机返回的 16 位数据
    digitalWrite(hspi->pinSS(), HIGH); // pull ss high to signify end of data transfer
    hspi->endTransaction();
    ag = ag >> 2;
    float rad = (float)ag * 2 * PI / 16384;
    //若计算得到的弧度值小于 0，则加上 2 * PI，将其调整到 [0, 2 * PI) 的范围内。
    if (rad < 0)
    {
        rad += 2 * PI;
    }
    return rad;
}
static void initMySensorCallback(void)
{
    hspi = new SPIClass(HSPI);  //HSPIClass和 VSPIClass，代表不同的 SPI 总线实例。HSPI是高速 SPI 总线
    hspi->begin(2, 1, -1, 42);      // SCLK, MISO, MOSI, SS   //miso,Mast In Slave Out，主机这边是输入，在从机那边是输出，即从机传给主机
    pinMode(hspi->pinSS(), OUTPUT); // HSPI SS
}

GenericSensor sensor = GenericSensor(readMySensorCallback, initMySensorCallback);

// ---------------------------------


/* 
 * 将随机变化的值限制在一个给定的区间[min,max]内
*/ 
static float CLAMP(const float value, const float low, const float high)
{
    return value < low ? low : (value > high ? high : value);
}

TaskHandle_t handleTaskMotor;

int i = 0;

void TaskMotorUpdate(void *pvParameters)
{

    while (1)
    {
 
        motor.loopFOC();// the faster you run this function the better// Arduino UNO loop  ~1kHz

        // function intended to be used with serial plotter to monitor motor variables
        // significantly slowing the execution down!!!!
        if (i == 1000)
        {
            Serial.print(motor.target);
            Serial.print("    ");
            Serial.print(motor.shaft_angle);
            Serial.print("    ");
            Serial.print(motor.shaft_velocity);
            Serial.print("    loop() running on core ");
            Serial.println(xPortGetCoreID());
            i = 0;
        }
//////////////////////////旋钮主逻辑/////////////////////////////////////////////
/*
转动主要就分为四个部分：（以顺时针转动为例）
死区1内：怠速校正，慢慢校正目标角度到当前角度                     （死区小于等于1度）
死区1到临界区：靠pid纠正，逆时针反作用，力逐渐加大
临界区到死区2：靠pid纠正，顺时针正作用，力逐渐减小
死区2内：怠速校正
*/


    //怠速检查：检测电机速度是否低于阈值，如果是的话，调整控制中心位置，防止电机在无操作时漂移。这可能是为了保持旋钮的位置稳定，避免误操作或位置偏移。
        idle_check_velocity_ewma = motor.shaft_velocity * IDLE_VELOCITY_EWMA_ALPHA + idle_check_velocity_ewma * (1 - IDLE_VELOCITY_EWMA_ALPHA);
        if (fabsf(idle_check_velocity_ewma) > IDLE_VELOCITY_RAD_PER_SEC)
        {
            last_idle_start = 0;
        }
        else
        {
            if (last_idle_start == 0)
            {
                last_idle_start = millis();
            }
        }

    //怠速校正
        // 如果电机没有人为干预，并且电机接近目标中心(但不是完全在那里)，慢慢调整中心点以匹配当前位置
        if (last_idle_start > 0 && millis() - last_idle_start > IDLE_CORRECTION_DELAY_MILLIS 
                && fabsf(motor.shaft_angle - current_detent_center) < IDLE_CORRECTION_MAX_ANGLE_RAD)
        {
            current_detent_center = motor.shaft_angle * IDLE_CORRECTION_RATE_ALPHA + current_detent_center * (1 - IDLE_CORRECTION_RATE_ALPHA);
        }

    //人为触发挡位切换
        //到控制中心的角度 差值
            angle_to_detent_center = motor.shaft_angle - current_detent_center;
        // 每一步都乘以了 snap_point 的值
        if (angle_to_detent_center > motor_config.position_width_radians * motor_config.snap_point 
                && (motor_config.num_positions <= 0 || motor_config.position > 0)) 
        //正转：条件 1：角度偏差超过正向触发阈值。需转动超过档位宽度的 x% 才切换档位
        //      条件 2：允许正转的边界检查：表示旋钮无位置限制（自由旋转）或表示当前逻辑位置未达到下限（如最低档为 0）
        {
            current_detent_center += motor_config.position_width_radians;
            angle_to_detent_center -= motor_config.position_width_radians;
            motor_config.position--;   
        }
        else if (angle_to_detent_center < -motor_config.position_width_radians * motor_config.snap_point 
                    && (motor_config.num_positions <= 0 || motor_config.position < motor_config.num_positions - 1))
        //反转
        {
            current_detent_center -= motor_config.position_width_radians;
            angle_to_detent_center += motor_config.position_width_radians;//挡位触发时只过了一半左右，所以还是有偏差，这时就需要电机自己矫正到中心点了
            motor_config.position++;
        }

    //死区调整，不同电机模式下死区不同（强阻尼/开关），但是最小死区为1度
        float dead_zone_adjustment = CLAMP(
            angle_to_detent_center,
            fmaxf(-motor_config.position_width_radians * DEAD_ZONE_DETENT_PERCENT, -DEAD_ZONE_RAD),
            fminf(motor_config.position_width_radians * DEAD_ZONE_DETENT_PERCENT, DEAD_ZONE_RAD)
            );
          
    //边界检查
        bool out_of_bounds = motor_config.num_positions > 0 && 
                    (   (angle_to_detent_center > 0 && motor_config.position == 0)  ||
                     (angle_to_detent_center < 0 && motor_config.position == motor_config.num_positions - 1)  ); 
        motor.PID_velocity.limit = out_of_bounds ? 10 : 3;
        motor.PID_velocity.P = out_of_bounds ? motor_config.endstop_strength_unit * 4 : motor_config.detent_strength_unit * 4;

    //输出扭矩
        if (fabsf(motor.shaft_velocity) > 60) // 处理float类型的取绝对值
        {
            // 如果速度太高 则不增加扭矩，避免正反馈失控
            motor.move(0);
        }
        else
        {
            // 输入偏差计算 PID 输出值
            float torque = motor.PID_velocity(-angle_to_detent_center + dead_zone_adjustment);//负号可以理解为反作用力，加上 死区的缓冲。在死区内时，dead_zone_adjustment=angle_to_detent_center
            motor.move(torque);
        }
/////////////////////////////////////////////////////////////////////////

        // user communication
        command.run();

        i++;
        vTaskDelay(1);
    }
}

void motor_init()
{
    
    sensor.init();// initialise magnetic sensor hardware
    motor.linkSensor(&sensor); // link the motor to the sensor

    // driver config
    driver.voltage_power_supply = 5;// power supply voltage [V]
    driver.init();
    
    motor.linkDriver(&driver);// link the motor and the driver

    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;// choose FOC modulation (optional)
    motor.controller = MotionControlType::torque;// set motion control loop to be used

    // contoller configuration
    // default parameters in defaults.h
    // 速度PI环设置
    motor.PID_velocity.P = 1;
    motor.PID_velocity.I = 0;
    motor.PID_velocity.D = 0.01;
    motor.voltage_limit = 5;// maximal voltage to be set to the motor
    motor.LPF_velocity.Tf = 0.01;// 速度低通滤波时间常数    越低过滤越少
    motor.P_angle.P = 1;// angle P controller
    motor.velocity_limit = 10;// maximal velocity of the position control
    motor.useMonitoring(Serial);// comment out if not needed

    // initialize motor
    motor.init();
    // align sensor and start FOC
    motor.initFOC();

    // add target command T
    // command.add('T', doTarget, "target torque");
    Serial.println(F("Motor ready."));
    Serial.println(F("Set the target angle using serial terminal:"));
    _delay(1000);

    xTaskCreatePinnedToCore(//xTaskCreatePinnedToCore()是一个FreeRTOS库函数，用于创建一个新的任务，并将其固定到ESP32的指定核心上运行。
        TaskMotorUpdate,
        "MotorThread",
        4096,
        nullptr,
        2,//任务的优先级。数字越高，优先级越高。
        &handleTaskMotor,
        0);//ESP32有两个核心

}

