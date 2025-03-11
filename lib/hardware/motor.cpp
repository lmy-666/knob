#include "motor.h"

//  BLDCMotor( pole_pairs )
BLDCMotor motor = BLDCMotor(7);
//  BLDCDriver( pin_pwmA, pin_pwmB, pin_pwmC, enable (optional) )
BLDCDriver3PWM driver = BLDCDriver3PWM(15, 16, 17);

// angle set point variable
float target_angle = 5;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char *cmd) { command.scalar(&target_angle, cmd); }

// -------------monitor--------------------

SPIClass *hspi = NULL;
static const int spiClk = 1000000; // 400KHz
static float readMySensorCallback(void)
{
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(hspi->pinSS(), LOW); // pull SS slow to prep other end for transfer
    uint16_t ag = hspi->transfer16(0);
    digitalWrite(hspi->pinSS(), HIGH); // pull ss high to signify end of data transfer
    hspi->endTransaction();
    ag = ag >> 2;
    float rad = (float)ag * 2 * PI / 16384;
    if (rad < 0)
    {
        rad += 2 * PI;
    }
    return rad;
}
static void initMySensorCallback(void)
{
    hspi = new SPIClass(HSPI);
    hspi->begin(2, 1, -1, 42);      // SCLK, MISO, MOSI, SS
    pinMode(hspi->pinSS(), OUTPUT); // HSPI SS
}

GenericSensor sensor = GenericSensor(readMySensorCallback, initMySensorCallback);

TaskHandle_t handleTaskMotor;

int i = 0;

void TaskMotorUpdate(void *pvParameters)
{

    while (1)
    {
        // main FOC algorithm function
        // the faster you run this function the better
        // Arduino UNO loop  ~1kHz
        // Bluepill loop ~10kHz
        motor.loopFOC();

        // Motion control function
        // velocity, position or voltage (defined in motor.controller)
        // this function can be run at much lower frequency than loopFOC() function
        // You can also use motor.move() and set the motor.target in the code
//        motor.move(target_angle);

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
        // user communication
        command.run();

        i++;
        vTaskDelay(1);
    }
}

void motor_init()
{
    // initialise magnetic sensor hardware
    sensor.init();
    // link the motor to the sensor
    motor.linkSensor(&sensor);

    // driver config
    // power supply voltage [V]
    driver.voltage_power_supply = 5;
    driver.init();
    // link the motor and the driver
    motor.linkDriver(&driver);

    // choose FOC modulation (optional)
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // set motion control loop to be used
    motor.controller = MotionControlType::velocity;

    // contoller configuration
    // default parameters in defaults.h

    // 速度PI环设置
    motor.PID_velocity.P = 0.25;
    motor.PID_velocity.I = 0.01;
    motor.PID_velocity.D = 0;
    // maximal voltage to be set to the motor
    motor.voltage_limit = 5;

    // velocity low pass filtering time constant
    // the lower the less filtered
    motor.LPF_velocity.Tf = 0.01f;

    // angle P controller
    motor.P_angle.P = 1;
    // maximal velocity of the position control
    motor.velocity_limit = 10;

    // comment out if not needed
    motor.useMonitoring(Serial);

    // initialize motor
    motor.init();
    // align sensor and start FOC
    motor.initFOC();

    // add target command T
    command.add('T', doTarget, "target torque");

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

