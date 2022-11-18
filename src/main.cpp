#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "CAN/CAN_config.h"
#include "CAN/ESP32CAN.h"
#include "MotorControl.h"
#include "IMU.h"
#include "Kalman.h"
#include "soc/timer_group_struct.h"
#include "WebPage.h"

#include <WiFi.h>
#include <WebServer.h>
#include <sstream>

// #define WIFI_DEBUG

#ifdef WIFI_DEBUG
const char* ssid = "VM2866187";
const char* password = "tk8vyPczhfhr";
#else
const char* ap_ssid = "MengMengRobot";
const char* ap_password = "12345678";
#endif


WebServer server(80);

IPAddress PageIP(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
float target_position = 0;
float target_angle = 0;
float target_velocity = 0;
float height = 150.0;
bool ifJump = false;

CAN_device_t CAN_cfg;                // CAN Config
int rx_queue_size = 10;       // Receive Queue size

MotorControl motorControl;

IMU imu;
Kalman kalmanRoll;
Kalman kalmanPitch;
SCSCL sc(0);


static SIX_CHANNEL_BWLPF *bwlpf;
static float pitch, roll;
static double gyroXangle, gyroYangle; // Angle calculate using the gyro only
static double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
static double gyroY, gyroZ;
static double dt;
static ulong timer;

static TimerHandle_t motorControlTimer = nullptr;
void motorControlTimerCallback(TimerHandle_t xTimer);

void IMUDataProcessing(void *parameters);
void printIMUData(void *parameters);
void ledTask(void *parameters);
static void controlInit();

static void CAN_task(void *arg) {
    CAN_cfg.speed = CAN_SPEED_500KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_2;
    CAN_cfg.rx_pin_id = GPIO_NUM_15;
    CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
    // Init CAN Module
    ESP32Can.CANInit();

    CAN_frame_t rx_frame;
    while(true) {
        //Serial.println("CAN while loop");
        if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
            //Serial.println("xQueueReceive");
            if (rx_frame.FIR.B.RTR == CAN_no_RTR) {
                motorControl.canDataDecode(rx_frame);
            }
            if (rx_frame.FIR.B.RTR == CAN_RTR) {
                //Serial.println("CAN_RTR");
                if(rx_frame.MsgID == BOARD_ID_1) {
                    //Serial.println("BOARD_01_IS_READY received");
                    CAN_frame_t tx_frame;
                    tx_frame.FIR.B.FF = CAN_frame_std;
                    tx_frame.MsgID = MASTER_RESPONSE;
                    tx_frame.FIR.B.RTR =CAN_RTR;
                    tx_frame.FIR.B.DLC = 0;
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    ESP32Can.CANWriteFrame(&tx_frame);

                    motorControl.BOARD_01_IS_READY = true;
                    double temp_Y = 0;
                    double temp_gyroY = 0;
                    for(uint16_t i = 0; i < 1000; i++) {
                        temp_Y += kalAngleY;
                        temp_gyroY += gyroY;
                        vTaskDelay(3 / portTICK_PERIOD_MS);
                    }

                    motorControl.mechanical_zero_angle = (temp_Y / 1000.0);
                    motorControl.zero_gyroY = (temp_gyroY / 1000.0);
                    motorControl.ch1_zero_position = motorControl.ch1_motor_position;
                    motorControl.ch2_zero_position = motorControl.ch2_motor_position;

                    motorControlTimer = xTimerCreate(
                            "motorControlTimer",
                            5 / portTICK_PERIOD_MS,
                            pdTRUE,
                            (void *) 0,
                            motorControlTimerCallback
                    );
                    if(motorControlTimer == nullptr) {
                        Serial.println("Could not create the motorControlTimer!");
                        while(1);
                    } else {
                        xTimerStart(motorControlTimer, portMAX_DELAY);
                    }
                }
            }
        }
    }
}


static void controlInit() {
    Serial2.begin(SERVO_BAUD_RATE); // TX: GPIO17, RX: GPIO16
    motorControl.initMotors(&sc);

    // pure angle loop: kp:200.2, ki:0.4, kd:5.5
    // add velocity loop: both kp and ki * 0.7
    motorControl.angleLoopPID.kp = 200.2;
    motorControl.angleLoopPID.ki = 0.41;
    motorControl.angleLoopPID.kd = 5.25;
    motorControl.angleLoopPID.error_sum_constrain = 20;
    motorControl.angleLoopPID.output_constrain = 100;

    motorControl.velocityLoopPID.kp = 0.0002;
    motorControl.velocityLoopPID.ki = 0.00006;
    motorControl.velocityLoopPID.kd = 0.00033;
    motorControl.velocityLoopPID.error_sum_constrain = 0.5;
    motorControl.velocityLoopPID.output_constrain = 0.5;

    motorControl.standing_loop_output = 0;
    motorControl.target_velocity = 0;
}

//Matrix<5,5> A_hat = {1.0, 0.005, 0, 0, 0,
//                     0.4083, 1.0, 0, 0, -0.4083,
//                     0, 0, 1.0, 0.005, 0,
//                     -0.0123, 0, 0, 1.0, 0.0123,
//                     0, 0, 0, 0, 1.0};  //I+ts*A_hat
//Matrix<5,1> B_hat = {0,
//                     -0.0278,
//                     0,
//                     0.0042,
//                     0}; //ts*B_hat
//Matrix<2,5> C_hat = {1,0,0,0,0,
//                     0,0,1,0,0};
//Matrix<5,5> A_LC = {-4.46, 0.005, 2.861, 0, 0,
//                    -298.5586, 1.0, 434.6414, 0, -0.4083,
//                    2.2012, 0, -36.3375, 0.005, 0,
//                    36.5914, 0, -220.8225, 1.0, 0.0123,
//                    29.6981, 0, -55.059, 0, 1.0};
//Matrix<5,2> L = {0.0650,   -0.0011,
//        0.6369 , -0.0062,
//        -0.0011,   0.0275,
//        -0.0184,   0.0227,
//        -0.0021,   0.0001};
//Matrix<1,5> K = {12.8, 28.5, 2.0, 8.0, 20.0};
//Matrix<5,5> E = {0.9350,   -0.0050,    0.0011,         0,         0,
//                 -1.4008,    0.2083,   -0.0494,   -0.2222,   -0.1472,
//                 0.0011 ,        0 ,   0.9725 ,  -0.0050 ,        0,
//                 0.0840 ,   0.1187 ,  -0.0144 ,   1.0333 ,   0.0711,
//                 0.0021 ,        0 ,  -0.0001 ,        0 ,   1.0000};
//Matrix<5,1> last_z, current_z;
//Matrix<2,1> y_m;

// LQR
//float lqr[4] = {0.0,0.1,-158.416857,-285.082587};
//float lqr[4] = {0.0,-2.0,-1238.4,-285.0};
float lqr[4] = {0.0,-2.0,-2538.4,-245.0};

float m1, m2;
float last_vel = 0;
float last_last_vel = 0;
float vel = 0;
float last_target;
float last_last_target;

float last_yaw = 0;
float alpha = 0.1;

float last_df = 0;
float last_phase_lead = 0;
void motorControlTimerCallback(TimerHandle_t xTimer) {
    //y_m(0,0) = kalAngleY - motorControl.mechanical_zero_angle;
    //y_m(1,0) = motorControl.ch1_motor_position - motorControl.ch1_zero_position;
    ////current_z = A_hat * last_z + B_hat * K * last_z + L * (y_m - C_hat * last_z);
    //current_z = E * last_z + L * y_m;
    //last_z = current_z;
    //Serial << '\t' << "d(4):" << current_z(4) << '\n' ;
    vel = 1.91122623f * last_vel + (-0.91500257f) * last_last_vel + (0.00094408f) * target_angle + (0.00188817f) * last_target + (0.00094408f) * last_last_target;
    last_last_vel = last_vel;
    last_vel = vel;
    last_last_target = last_target;
    last_target = target_angle;
    //if(target_angle == 0.0) {
    //    vel = 0; last_last_vel = 0; last_vel = 0;
    //    last_last_target = 0; last_target = 0;
    //}

    float yaw = target_velocity * alpha + (1.0f - alpha) * last_yaw;
    last_yaw = yaw;

    float df = (float)( ((motorControl.ch1_motor_position + motorControl.ch2_motor_position)/2.0f) * lqr[0]
                       + ((motorControl.ch1_motor_velocity + motorControl.ch2_motor_velocity)/2.0f - vel) * lqr[1]
                       + (kalAngleY - motorControl.mechanical_zero_angle) *lqr[2]
                       + (gyroY - motorControl.zero_gyroY) * lqr[3])*0.005f;

    //float phase_lead = 0.995 * df - 0.985 * last_df +0.98 * last_phase_lead;
    //last_df = df;
    //last_phase_lead = phase_lead;
    //float dv = 0.01 * (((motorControl.ch1_motor_velocity+motorControl.ch2_motor_velocity)*0.024/0.15) - target_velocity);
    m1 += (df);
    m2 += (df);

    /// Output limitation
    m1 = fast_constrain(m1, -100.0, 100.0);
    m2 = fast_constrain(m2, -100.0, 100.0);
    if(kalAngleY > 20 * (M_PI / 180.0) || kalAngleY < -20 * (M_PI / 180.0)) {
        m1 = 0;
        m2 = 0;
    }

    /// Send desired torque to motors
    if(motorControl.BOARD_01_IS_READY) {
        motorControl._tx_frame = motorControl.create2ChannelMotorTargetFrame(
                SET_TARGET,
                m1 + yaw,
                m2 - yaw);
        ESP32Can.CANWriteFrame(&motorControl._tx_frame);
    }

}

void RGB_Color(int i, int j, int k)
{
    ledcWrite(R_channel, i);
    ledcWrite(G_channel, j);
    ledcWrite(B_channel, k);
}

void ledTask(void *parameters){
    ledcAttachPin(LEDR, R_channel);
    ledcAttachPin(LEDG, G_channel);
    ledcAttachPin(LEDB, B_channel);
    ledcSetup(R_channel, pwm_Frequency, pwm_resolution);
    ledcSetup(G_channel, pwm_Frequency, pwm_resolution);
    ledcSetup(B_channel, pwm_Frequency, pwm_resolution);

    int i = 0;
    while(1) {
        for(i = 0; i < 200; ++i) {
            RGB_Color(i, 0, 0);
            delay(3);
        }
        for(i = 0; i < 200; ++i) {
            RGB_Color(200, i, 0);
            delay(3);
        }
        for(i = 0; i < 200; ++i) {
            RGB_Color(200, 200, i);
            delay(3);
        }
        for(i = 200; i > 0; --i) {
            RGB_Color(i, 200, 200);
            delay(3);
        }
        for(i = 200; i > 0; --i) {
            RGB_Color(0, i, 200);
            delay(3);
        }
        for(i = 200; i > 0; --i) {
            RGB_Color(0, 0, i);
            delay(3);
        }
    }
}

void printIMUData(void *parameters) {
    while(true) {
        //Serial.print("IMU_Pitch:");
        //Serial.print(kalAngleY);
        //Serial.print(",motorControl.target_torque:");
        //Serial.println(motorControl.target_torque);
        //Serial.print("IMU_Pitch:");Serial.println(kalAngleY - motorControl.mechanical_zero_angle);
        Serial.print("mechanical_zero_angle:");Serial.println(motorControl.mechanical_zero_angle);

        //Serial.print("Pos:");
        //Serial.print(((motorControl.ch1_motor_position - motorControl.ch1_zero_position) + target_position) * lqr[0]);
        //Serial.print(",Vel:");
        //Serial.print(-(motorControl.ch1_motor_velocity) * lqr[1]);
        //Serial.print(",Ang:");
        //Serial.print((kalAngleY - motorControl.mechanical_zero_angle + target_angle) *lqr[2]);
        //Serial.print(",Gyro:");
        //Serial.println((gyroY - motorControl.zero_gyroY) * lqr[3]);

        //Serial.print("position:");
        //Serial.print(motorControl.ch1_motor_position - motorControl.ch1_zero_position);
        //Serial.print(",velocity:");
        //Serial.println(motorControl.ch1_motor_velocity);

        //Serial.print(",CH1_Velocity:");
        //Serial.print(motorControl.ch1_motor_velocity);
        //Serial.print(",CH2_Velocity:");
        //Serial.print(motorControl.ch2_motor_velocity);
        //Serial.print(",CH1_Pos:");
        //Serial.print(motorControl.ch1_motor_position);
        //Serial.print(",CH2_Pos:");
        //Serial.println(motorControl.ch2_motor_position);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void handleRoot(){
    server.send(200, "text/html", webPage);
}

void rockerHandler() {
    String x_value_string = server.arg("x");
    String y_value_string = server.arg("y");
    float x_value = x_value_string.toFloat();
    float y_value = y_value_string.toFloat();
    target_angle = x_value;
    target_velocity = y_value;
    server.send(200, "text/plain", "");
}

void heightHandler() {
    String heightStr = server.arg("h");
    height = heightStr.toFloat();
    motorControl.setRobotHeight(0, height);
    delay(100);
    server.send(200, "text/plain", "");
}


void angleTuneHandler() {
    String direction = server.arg("angle");
    if(direction == "inc") {
        motorControl.mechanical_zero_angle += 0.005;
    } else if(direction == "dec") {
        motorControl.mechanical_zero_angle -= 0.005;
    }
    server.send(200, "text/plain", String(motorControl.mechanical_zero_angle, 5));
}

void lqrHandler() {
    String lqrpar = server.arg("lqr");
    int lqr_index = 0;
    int de_index = 0;
    for (int i = 0; i < lqrpar.length(); i++) {
        if (lqrpar.substring(i, i+1) == ",") {
            lqr[lqr_index] = lqrpar.substring(de_index, i).toFloat();
            de_index = i+1;
            lqr_index += 1;
        }
    }
    lqr[3] = lqrpar.substring(de_index, lqrpar.length()-1).toFloat();
    for (int i = 0; i < 4; ++i) {
        Serial.print(lqr[i]); Serial.print(",");
    }
    Serial.println();
    server.send(200, "text/plain", "");
}


//void jumpHandler() {
//    ifJump = true;
//    server.send(200, "text/plain", "");
//}



void handleNotFound(){
    server.send(404, "text/plain", "404: Not found");
}

static void WiFiTask(void *arg) {
    while(1) {
        server.handleClient();
    }
}

void setup() {
    delay(200);
    disableCore1WDT();

    Wire.setClock(100000);
    Wire.begin();

    Serial.begin(115200);
    imu.init(&Serial);
    bwlpf = imu.initializeBWLPF(2, 200, 50);
    roll = imu.getRoll();
    pitch = imu.getPitch();
    kalmanRoll.setQbias(0.005f);
    kalmanPitch.setQbias(0.005f);
    kalmanRoll.setAngle(roll);
    kalmanPitch.setAngle(pitch);
    gyroXangle = roll;
    gyroYangle = pitch;
    timer = micros();

    controlInit();

    //last_z.Fill(0);
    //current_z.Fill(0);
    //y_m.Fill(0);

    /// WiFi
#ifdef WIFI_DEBUG
    WiFi.mode(WIFI_MODE_STA);
    WiFi.begin(ssid, password);
    while(WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
#else
    WiFi.softAP(ap_ssid, ap_password);
    delay(100);
    WiFi.softAPConfig(PageIP, gateway, subnet);
    delay(100);
    Serial.print("IP Address:"); Serial.println(WiFi.softAPIP());
#endif

    server.on("/", handleRoot);      //当访问服务器首页时，采用handleRoot函数处理
    server.on("/rocker", rockerHandler);
    server.on("/angle-tune", angleTuneHandler);
    //server.on("/jump", jumpHandler);
    server.on("/height", heightHandler);
    server.on("/lqr", lqrHandler);
    server.onNotFound(handleNotFound);  //如果访问对象不存在时，采用handleNotFound函数处理
    server.begin();

    Serial.print("HTTP server started");


    xTaskCreatePinnedToCore(WiFiTask,
                            "WiFiTask",
                            8192,
                            NULL,
                            1,
                            NULL,
                            1);

    xTaskCreatePinnedToCore(IMUDataProcessing,
                            "IMU Data Processing",
                            4096,
                            NULL,
                            1,
                            NULL,
                            0);

    xTaskCreatePinnedToCore(ledTask,
                            "ledTask",
                            800,
                            NULL,
                            1,
                            NULL,
                            0);

    //xTaskCreatePinnedToCore(printIMUData,
    //                        "Print IMU Data",
    //                        1024,
    //                        NULL,
    //                        1,
    //                        NULL,
    //                        0);

    xTaskCreatePinnedToCore(
            CAN_task,                   // Function to be called
            "CAN_task",         // Name of task
            4096,            // Stack size in byte
            NULL,            // Parameter to pass to function
            1,                  // Task priority
            NULL,            // Task handle
            0                   // Core id
    );

    vTaskDelete(nullptr);
}


void IMUDataProcessing(void *parameters) {
    double gyroXrate, gyroYrate;
    double temp_kalAngleX, temp_kalAngleY;
    float temp_roll, temp_pitch;
    while(true) {
        dt = (double)(micros() - timer) / 1000000; // Calculate delta time
        timer = micros();
        gyroXrate = imu.getGyroX();
        gyroYrate = imu.getGyroY();
        imu.ButterworthFilter(bwlpf);

        temp_pitch = imu.getPitch();
        temp_roll = imu.getRoll();
        if ((temp_roll < -90 && temp_kalAngleX > 90) || (temp_roll > 90 && temp_kalAngleX < -90)) {
            kalmanPitch.setAngle(temp_roll);
            temp_kalAngleX = temp_roll;
            gyroXangle = temp_roll;
        } else {
            temp_kalAngleX = kalmanPitch.getAngle(temp_roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
        }

        if (abs(temp_kalAngleX) > 90) {
            gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
        }
        //temp_kalAngleY = kalmanRoll.getAngle(temp_pitch, gyroYrate, dt);

        gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
        gyroYangle += gyroYrate * dt;

        // Reset the gyro angle when it has drifted too much
        if (gyroXangle < -180 || gyroXangle > 180)
            gyroXangle = kalAngleX;
        if (gyroYangle < -180 || gyroYangle > 180)
            gyroYangle = kalAngleY;

        roll = temp_roll;
        pitch = temp_pitch;
        kalAngleX = temp_kalAngleX * (M_PI / 180.0);
        kalAngleY = temp_pitch * (M_PI / 180.0);
        //gyroZ = imu.getGyroZ() * (M_PI / 180.0);
        gyroY = imu.getGyroY() * (M_PI / 180.0);
    }
}


void loop() {
    vTaskDelay(2000/portTICK_PERIOD_MS);
}