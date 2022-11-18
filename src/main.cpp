#include <Arduino.h>
#include "SimpleFOC/SimpleFOC.h"
#include "TLE5012b/TLE5012b_FOC.h"
#include "CANCommunication.h"
#include "config.h"

/// CAN Bus
CANCommunication can(CAN_DEVICE_ID);
static void CAN_task(void *arg);
CAN_device_t CAN_cfg;
bool CAN_CONNECTED = false;

// magnetic sensor instance - SPI
TLE5012b_FOC ch1_sensor = TLE5012b_FOC();
TLE5012b_FOC ch2_sensor = TLE5012b_FOC();

// BLDC motor & driver instance
BLDCMotor ch1_motor = BLDCMotor(11);
BLDCMotor ch2_motor = BLDCMotor(11);

BLDCDriver3PWM ch1_driver = BLDCDriver3PWM(32, 33, 25);
BLDCDriver3PWM ch2_driver = BLDCDriver3PWM(14, 27, 26);

SPIClass *spiClass = nullptr;

/// FOC task
static void focTask(void *arg);
static void focTask(void *arg) {
    while(1) {
        ch1_motor.loopFOC();
        ch2_motor.loopFOC();
        ch1_motor.move();
        ch2_motor.move();
    }
}

/// CAN report task
static TimerHandle_t reportPositionTimer = nullptr;
static void reportPositionTimerCallback(TimerHandle_t xTimer);
static void reportPositionTimerCallback(TimerHandle_t xTimer) {
    if(CAN_CONNECTED) {
        can.reportSensorPosition();
        can.reportSensorVelocity();
    }
}

void setup() {
    spiClass = new SPIClass(VSPI);
    spiClass->begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
    ch1_sensor.init(22, spiClass);
    ch1_motor.linkSensor(&ch1_sensor);

    ch2_sensor.init(5, spiClass);
    ch2_motor.linkSensor(&ch2_sensor);

    ch1_driver.voltage_power_supply = 16.0;
    ch2_driver.voltage_power_supply = 16.0;

    ch1_driver.init();
    ch2_driver.init();

    ch1_motor.linkDriver(&ch1_driver);
    ch2_motor.linkDriver(&ch2_driver);

    // set motion control loop to be used
    ch1_motor.torque_controller = TorqueControlType::voltage;
    ch1_motor.controller = MotionControlType::velocity;
    ch2_motor.torque_controller = TorqueControlType::voltage;
    ch2_motor.controller = MotionControlType::velocity;

    ch1_motor.PID_velocity.P = 1.6;
    ch1_motor.PID_velocity.I = 4.0;
    ch1_motor.PID_velocity.D = 0.01;
    ch1_motor.PID_velocity.output_ramp = 300;
    ch1_motor.P_angle.P = 0.5;
    ch1_motor.phase_resistance = 0.11; // [Ohm]
    ch1_motor.current_limit = 22;   // [Amps] - 如果相电阻有被定义

    ch2_motor.PID_velocity.P = 1.6;
    ch2_motor.PID_velocity.I = 4.0;
    ch2_motor.PID_velocity.D = 0.01;
    ch2_motor.PID_velocity.output_ramp = 300;
    ch2_motor.P_angle.P = 0.5;
    ch2_motor.phase_resistance = 0.11; // [Ohm]
    ch2_motor.current_limit = 22;   // [Amps] - 如果相电阻有被定义

    // default voltage_power_supply
    ch1_motor.voltage_limit = 12;
    ch2_motor.voltage_limit = 12;

    ch1_motor.LPF_velocity.Tf = 0.01f;
    ch1_motor.velocity_limit = 500;
    ch2_motor.LPF_velocity.Tf = 0.01f;
    ch2_motor.velocity_limit = 500;

    ch1_motor.zero_electric_angle = 5.46;
    ch1_motor.sensor_direction = 1;

    ch2_motor.zero_electric_angle = 0.42;
    ch2_motor.sensor_direction = -1;

    ch1_motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    ch2_motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // use monitoring with serial
    Serial.begin(115200);

    // initialize motor
    ch1_motor.init();
    ch2_motor.init();
    // align sensor and start FOC
    ch1_motor.initFOC();
    ch2_motor.initFOC();
    //Serial.print("ch1_zero_electric_angle: "); Serial.println(ch1_motor.zero_electric_angle);
    //Serial.print("ch1_sensor_direction: "); Serial.println(ch1_motor.sensor_direction);
    //Serial.print("ch2_zero_electric_angle: "); Serial.println(ch2_motor.zero_electric_angle);
    //Serial.print("ch2_sensor_direction: "); Serial.println(ch2_motor.sensor_direction);
    ch1_motor.target = 0.0;
    ch2_motor.target = 0.0;

    _delay(200);

    xTaskCreatePinnedToCore(
            focTask,                   // Function to be called
            "focTask",         // Name of task
            8192,            // Stack size in byte
            NULL,            // Parameter to pass to function
            1,                  // Task priority
            NULL,           // Task handle
            1                   // Core id
    );

    xTaskCreatePinnedToCore(
            CAN_task,                   // Function to be called
            "CAN_task",         // Name of task
            4096,            // Stack size in byte
            NULL,            // Parameter to pass to function
            1,                  // Task priority
            NULL,            // Task handle
            0                   // Core id
    );

}

void loop() {
    vTaskDelay(2000/portTICK_PERIOD_MS);
}

static void CAN_task(void *arg) {
    can.registerDriver(&ch1_motor, &ch2_motor);
    CAN_cfg.speed = CAN_SPEED_500KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_2;
    CAN_cfg.rx_pin_id = GPIO_NUM_15;
    CAN_cfg.rx_queue = xQueueCreate(CAN_RX_QUEUE_SIZE, sizeof(CAN_frame_t));

    ESP32Can.CANInit();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = CAN_DEVICE_ID;
    tx_frame.FIR.B.RTR =CAN_RTR;
    tx_frame.FIR.B.DLC = 0;

    reportPositionTimer = xTimerCreate(
            "reportPositionTimer",
            5 / portTICK_PERIOD_MS,
            pdTRUE,
            (void *) 0,
            reportPositionTimerCallback
    );
    if(reportPositionTimer == nullptr) {
        printf("Could not create the reportPositionTimer!\n");
        while(1);
    } else {
        xTimerStart(reportPositionTimer, portMAX_DELAY);
    }

    ESP32Can.CANWriteFrame(&tx_frame);

    while(1) {
        if (xQueueReceive(CAN_cfg.rx_queue, &can.rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
            //printf("xQueueReceive! ID: %d, length: %d\n", can.rx_frame.MsgID, can.rx_frame.FIR.B.DLC);
            if (can.rx_frame.FIR.B.RTR == CAN_no_RTR) {
                can.dataUnpack();
            }
            if (can.rx_frame.FIR.B.RTR == CAN_RTR) {
                if(can.rx_frame.MsgID == MASTER_RESPONSE) {
                    printf("CAN_CONNECTED!\n");
                    CAN_CONNECTED = true;
                }
            }
        }
    }
}