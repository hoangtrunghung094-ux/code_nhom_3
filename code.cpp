#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <MKE_I2C_MotorDriver.h>
#include <ESP32Servo.h>
#include "MKE_I2C_5C_LineTracking.h"

//                   K210 UART
// ESP32 RX = GPIO3, TX = GPIO1
// K210 TX -> ESP32 RX3
// K210 RX <- ESP32 TX1
HardwareSerial K210Serial(0);
HardwareSerial DebugSerial(2);
#define K210_RX_PIN 3
#define K210_TX_PIN 1
#define K210_BAUD 115200
#define DEBUG_RX_PIN 16
#define DEBUG_TX_PIN 17

String k210RxLine = "";
String k210Object = "NOOBJ";
int k210Confidence = 0;
int k210X = 0, k210Y = 0, k210W = 0, k210H = 0;
bool k210Detected = false;
unsigned long k210LastPacket = 0;
unsigned long k210LastObjectSeen = 0;
#define K210_DETECTION_HOLD_MS 1200

void readK210UART();
void parseK210Line(const String &line);

#define TRIG_PIN   5
#define ECHO_PIN   18
#define SERVO_PIN  15

Servo myServo;

//                   MKE MOTOR DRIVER

MKE_I2C_MotorDriver driverFront;
MKE_I2C_MotorDriver driverRear;

// I2C phía trước
#define SDA_FRONT 21
#define SCL_FRONT 22

// I2C phía sau
#define SDA_REAR 32
#define SCL_REAR 33

//                        WIFI

const char* AP_SSID = "AGV_MECANUM";
const char* AP_PASS = "12345678";

WebServer server(80);

//                   CHẾ ĐỘ HOẠT ĐỘNG

bool isAutoMode = false;
char current_cmd = 'S';

//               TRẠNG THÁI HIỂN THỊ TRÊN WEB
volatile int webSpeed = 0;
volatile float webDistance = 0;
String webState = "DUNG";
unsigned long lastAutoAction = 0;

// Khi AUTO, người dùng có thể can thiệp bằng web.
// Sau khi nhả nút, xe quay lại AUTO.
bool manualOverride = false;
char overrideCmd = 'S';

//                   THÔNG SỐ SERVO

#define LEFT_ANGLE    150
#define CENTER_ANGLE  90
#define RIGHT_ANGLE   30

//                   THÔNG SỐ VẬT CẢN

// Bắt đầu giảm tốc khi vật cản trong 1 mét
#define DETECT_DISTANCE 100

// Khoảng cách dừng trước vật cản
#define STOP_DISTANCE 30

// Khoảng cách tối thiểu để coi bên trái/phải là đủ rộng
#define SAFE_SIDE_DISTANCE 60

//                        TỐC ĐỘ

#define NORMAL_SPEED 180
#define MIN_SPEED 50

// Tốc độ ĐI NGANG khi tránh vật cản
#define SIDE_SPEED 100

// Thời gian ĐI NGANG khi tránh vật cản (ra khỏi line)
#define SIDE_TIME 700

// Thời gian TIẾN THẲNG để vượt qua thân vật cản
#define AVOID_FORWARD_TIME 1000

// Thời gian ĐI NGANG NGƯỢC LẠI để quay về vùng line
#define SIDE_RETURN_TIME 650

// Khoảng cách an toàn phía trước khi đang tiến vượt vật cản
#define AVOID_MIN_FRONT 25

//       THÔNG SỐ LÙI + QUAY ĐẦU KHI KHÔNG THỂ NÉ

// Tốc độ lùi khi cả hai bên đều chật
#define BACK_SPEED 100

// Thời gian lùi để tạo khoảng trống trước khi quay đầu
#define BACK_TIME 600

// Tốc độ quay đầu 180 độ
#define TURN_SPEED 100

// Thời gian quay đầu 180 độ
// Cần hiệu chỉnh thực tế theo khung xe, pin và mặt sàn
#define TURN_180_TIME 900

//               HÀM ĐIỀU KHIỂN 4 BÁNH MKE

void selectFrontI2C();

void set4Wheels(int fl, int fr, int rl, int rr)
{
    
    selectFrontI2C();

    // BÁNH TRƯỚC TRÁI

    if (fl >= 0)
        driverFront.motorA_CW(fl);
    else
        driverFront.motorA_CCW(-fl);

    // BÁNH TRƯỚC PHẢI

    if (fr >= 0)
        driverFront.motorB_CW(fr);
    else
        driverFront.motorB_CCW(-fr);

    
    // BÁNH SAU TRÁI

    if (rl >= 0)
        driverRear.motorA_CW(rl);
    else
        driverRear.motorA_CCW(-rl);

    // BÁNH SAU PHẢI

    if (rr >= 0)
        driverRear.motorB_CW(rr);
    else
        driverRear.motorB_CCW(-rr);
}

//                     CHUYỂN ĐỘNG

// TIẾN

void moveForward(int speed)
{
    set4Wheels(
         speed,
         speed,
         speed,
         speed
    );
}

// LÙI

void moveBackward(int speed)
{
    set4Wheels(
        -speed,
        -speed,
        -speed,
        -speed
    );
}

// NGANG TRÁI
void moveLeft(int speed)
{
    
    set4Wheels(
        -speed,
         speed,
        -speed,
         speed
    );
}

// NGANG PHẢI
void moveRight(int speed)
{
    
    set4Wheels(
         speed,
        -speed,
         speed,
        -speed
    );
}

// XOAY TRÁI

void rotateLeft(int speed)
{
    // XOAY TRAI 
    set4Wheels(
        -speed,
         speed,
         speed,
        -speed
    );
}

// XOAY PHẢI

void rotateRight(int speed)
{
    // XOAY PHAI 
    set4Wheels(
         speed,
        -speed,
        -speed,
         speed
    );
}

// DỪNG

void stopCar()
{
    set4Wheels(
        0,
        0,
        0,
        0
    );
}

//                     ĐỌC CẢM BIẾN SIÊU ÂM

float readDistance()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);

    digitalWrite(TRIG_PIN, LOW);

    long duration =
        pulseIn(ECHO_PIN, HIGH, 20000);

    // Không nhận được Echo
    if (duration == 0)
        return 400;

    float distance =
        duration * 0.0343 / 2.0;

    if (distance <= 0 || distance > 400)
        return 400;

    return distance;
}

//              TÍNH TỐC ĐỘ GIẢM DẦN

int calculateSpeed(float distance)
{
    // Xa hơn 1 mét
    if (distance > DETECT_DISTANCE)
        return NORMAL_SPEED;

    // Đã tới khoảng dừng
    if (distance <= STOP_DISTANCE)
        return 0;

    int speed = map(
        (int)distance,
        STOP_DISTANCE,
        DETECT_DISTANCE,
        MIN_SPEED,
        NORMAL_SPEED
    );

    speed = constrain(
        speed,
        MIN_SPEED,
        NORMAL_SPEED
    );

    return speed;
}

//               ĐO KHOẢNG CÁCH TRÁI / PHẢI

float measureAngle(int angle)
{
    myServo.write(angle);

    // Chờ servo quay
    delay(300);

    // Đo 3 lần để giảm nhiễu
    float d1 = readDistance();
    delay(30);

    float d2 = readDistance();
    delay(30);

    float d3 = readDistance();

    return (d1 + d2 + d3) / 3.0;
}

//         LÙI LẠI + QUAY ĐẦU KHI KHÔNG THỂ NÉ

void backAndTurnAround()
{
    DebugSerial.println();
    DebugSerial.println("================================");
    DebugSerial.println("   KHONG THE NE TRAI / PHAI");
    DebugSerial.println("   -> LUI LAI -> QUAY DAU");
    DebugSerial.println("================================");

    // BƯỚC 1: LÙI LẠI MỘT KHOẢNG
    stopCar();
    webSpeed = 0;
    webState = "KHONG NE DUOC - LUI LAI";

    delay(200);

    DebugSerial.println(">>> XE LUI LAI...");
    moveBackward(BACK_SPEED);
    webSpeed = BACK_SPEED;
    webState = "LUI LAI DE QUAY DAU";

    delay(BACK_TIME);

    // BƯỚC 2: DỪNG LẠI TRƯỚC KHI QUAY
    stopCar();
    webSpeed = 0;
    delay(200);

    // BƯỚC 3: QUAY ĐẦU 180 ĐỘ
    DebugSerial.println(">>> XE QUAY DAU 180 DO...");

    rotateLeft(TURN_SPEED);
    webSpeed = TURN_SPEED;
    webState = "QUAY DAU 180 DO";

    delay(TURN_180_TIME);

    // BƯỚC 4: DỪNG, ĐƯA SERVO VỀ GIỮA
    stopCar();
    webSpeed = 0;

    myServo.write(CENTER_ANGLE);
    delay(300);

    DebugSerial.println(">>> DA QUAY DAU");
    DebugSerial.println(">>> TIEP TUC DI");

    webState = "DA QUAY DAU - TIEP TUC";
    delay(200);
}

//               DÒ LẠI LINE SAU KHI NÉ

bool reacquireLineAfterAvoid();

//         QUÉT VÀ TRÁNH VẬT CẢN BẰNG ĐI NGANG
// Trình tự: DỪNG -> QUÉT -> ĐI NGANG RA -> TIẾN VƯỢT VẬT CẢN -> ĐI NGANG VÀO -> DÒ LINE

void scanAndAvoid()
{
    DebugSerial.println();
    DebugSerial.println("================================");
    DebugSerial.println("   NE VAT CAN - DI NGANG MECANUM");
    DebugSerial.println("================================");

    stopCar();
    webSpeed = 0;
    webState = "DUNG - QUET TRAI PHAI";
    delay(250);

    float leftDistance = measureAngle(LEFT_ANGLE);
    float rightDistance = measureAngle(RIGHT_ANGLE);

    myServo.write(CENTER_ANGLE);
    delay(150);

    DebugSerial.print("TRAI = ");
    DebugSerial.print(leftDistance, 1);
    DebugSerial.println(" cm");
    DebugSerial.print("PHAI = ");
    DebugSerial.print(rightDistance, 1);
    DebugSerial.println(" cm");

    // Không đủ khoảng trống cả 2 bên -> DỪNG
    if (leftDistance < SAFE_SIDE_DISTANCE && rightDistance < SAFE_SIDE_DISTANCE)
    {
        stopCar();
        webSpeed = 0;
        webState = "HAI BEN CHAT - DUNG";
        DebugSerial.println(">>> HAI BEN DEU CHAT -> DUNG");
        return;
    }

    bool goLeft = (leftDistance >= SAFE_SIDE_DISTANCE &&
                   (leftDistance >= rightDistance || rightDistance < SAFE_SIDE_DISTANCE));

    if (goLeft)
    {
        DebugSerial.println(">>> CHON TRAI");
        DebugSerial.println(">>> DI NGANG SANG TRAI NE VAT CAN");
        webState = "DI NGANG TRAI NE VAT CAN";
        webSpeed = SIDE_SPEED;

        // 1. Đi ngang sang trái để lệch khỏi vật cản
        moveLeft(SIDE_SPEED);
        delay(SIDE_TIME);
        stopCar();
        delay(150);

        // 2. Tiến thẳng để vượt qua thân vật cản
        DebugSerial.println(">>> TIEN VUOT VAT CAN");
        webState = "TIEN VUOT VAT CAN";
        webSpeed = SIDE_SPEED;
        moveForward(SIDE_SPEED);

        unsigned long t0 = millis();
        while (millis() - t0 < AVOID_FORWARD_TIME)
        {
            float d = readDistance();
            webDistance = d;
            if (d <= AVOID_MIN_FRONT)
            {
                stopCar();
                webSpeed = 0;
                webState = "VAT CAN PHIA TRUOC - DUNG";
                DebugSerial.println(">>> VAT CAN MOI PHIA TRUOC -> DUNG");
                return;
            }
            delay(20);
        }
        stopCar();
        delay(150);

        // 3. Đi ngang sang phải để quay lại vùng line
        DebugSerial.println(">>> DI NGANG PHAI VAO LINE");
        webState = "DI NGANG VAO LINE";
        webSpeed = SIDE_SPEED;
        moveRight(SIDE_SPEED);
        delay(SIDE_RETURN_TIME);
        stopCar();
    }
    else
    {
        DebugSerial.println(">>> CHON PHAI");
        DebugSerial.println(">>> DI NGANG SANG PHAI NE VAT CAN");
        webState = "DI NGANG PHAI NE VAT CAN";
        webSpeed = SIDE_SPEED;

        // 1. Đi ngang sang phải để lệch khỏi vật cản
        moveRight(SIDE_SPEED);
        delay(SIDE_TIME);
        stopCar();
        delay(150);

        // 2. Tiến thẳng để vượt qua thân vật cản
        DebugSerial.println(">>> TIEN VUOT VAT CAN");
        webState = "TIEN VUOT VAT CAN";
        webSpeed = SIDE_SPEED;
        moveForward(SIDE_SPEED);

        unsigned long t0 = millis();
        while (millis() - t0 < AVOID_FORWARD_TIME)
        {
            float d = readDistance();
            webDistance = d;
            if (d <= AVOID_MIN_FRONT)
            {
                stopCar();
                webSpeed = 0;
                webState = "VAT CAN PHIA TRUOC - DUNG";
                DebugSerial.println(">>> VAT CAN MOI PHIA TRUOC -> DUNG");
                return;
            }
            delay(20);
        }
        stopCar();
        delay(150);

        // 3. Đi ngang sang trái để quay lại vùng line
        DebugSerial.println(">>> DI NGANG TRAI VAO LINE");
        webState = "DI NGANG VAO LINE";
        webSpeed = SIDE_SPEED;
        moveLeft(SIDE_SPEED);
        delay(SIDE_RETURN_TIME);
        stopCar();
    }

    stopCar();
    webSpeed = 0;
    myServo.write(CENTER_ANGLE);
    delay(100);

    // Sau khi vào lại vùng line, dùng cơ chế dò line hiện có.
    reacquireLineAfterAvoid();
}

//                  DÒ LINE 

MKE_I2C_5C_LineTracking LineFollowSensor;

#define SDA_LINE 25
#define SCL_LINE 26
#define LINE_SENSOR_ADDRESS 0x2A

#define MAX_SPEED 55
#define MIN_LINE_SPEED 45

// Biên độ chỉnh ngang của mecanum khi dò line
#define MAX_LATERAL 50

// PID 
#define PID_KP 0.038
#define PID_KI 0.0
#define PID_KD 0.076

#define PID_INTEGRAL_LIMIT 100.0

#define SEARCH_SPEED 60
#define SEARCH_BACKUP_SPEED 60

#define LINE_SEARCH_TIMEOUT 5000

#define SEARCH_SPIN_INITIAL 500
#define SEARCH_SPIN_STEP 200
#define SEARCH_SPIN_MAX 1200

#define SEARCH_BACKUP_DURATION 500

int lineP1 = 0;
int lineP2 = 0;
int lineP3 = 0;
int lineP4 = 0;
int lineP5 = 0;

int lineError = 0;
int lastLineError = 0;
bool lineDetectedNow = false;

double pidIntegral = 0.0;
double pidLastError = 0.0;
unsigned long pidLastTime = 0;

unsigned long lineLostSince = 0;

enum SearchState
{
    SEARCH_SPIN,
    SEARCH_BACKUP
};

SearchState searchState = SEARCH_SPIN;
unsigned long searchStateStart = 0;
int searchSpinDuration = SEARCH_SPIN_INITIAL;

void selectFrontI2C()
{
    Wire.end();
    Wire.setPins(SDA_FRONT, SCL_FRONT);
    Wire.begin(SDA_FRONT, SCL_FRONT, 100000);
}

void selectLineI2C()
{
    Wire.end();

    Wire.setPins(
        SDA_LINE,
        SCL_LINE
    );

    Wire.begin(
        SDA_LINE,
        SCL_LINE,
        100000
    );
}

const char* searchStateName()
{
    if (searchState == SEARCH_SPIN)
        return "XOAY";
    else
        return "LUI";
}

void readLineSensor()
{
    selectLineI2C();

    LineFollowSensor.getData();

    lineP1 = LineFollowSensor.getValue(0);
    lineP2 = LineFollowSensor.getValue(1);
    lineP3 = LineFollowSensor.getValue(2);
    lineP4 = LineFollowSensor.getValue(3);
    lineP5 = LineFollowSensor.getValue(4);

    int sum =
        lineP1 +
        lineP2 +
        lineP3 +
        lineP4 +
        lineP5;

    lineDetectedNow = (sum > 0);

    if (sum > 0)
    {
        int weighted =
            (-2 * lineP1) +
            (-1 * lineP2) +
            (0 * lineP3) +
            (1 * lineP4) +
            (2 * lineP5);

        lineError =
            (weighted * 100) / sum;

        lineError =
            constrain(
                lineError,
                -200,
                200
            );

        lastLineError = lineError;
    }
    else
    {
        lineError = lastLineError;
    }
}

void printLineSensor()
{
    DebugSerial.print("P1=");
    DebugSerial.print(lineP1);

    DebugSerial.print(" P2=");
    DebugSerial.print(lineP2);

    DebugSerial.print(" P3=");
    DebugSerial.print(lineP3);

    DebugSerial.print(" P4=");
    DebugSerial.print(lineP4);

    DebugSerial.print(" P5=");
    DebugSerial.print(lineP5);

    DebugSerial.print(" | ERROR=");
    DebugSerial.print(lineError);

    DebugSerial.print(" | ");

    if (lineDetectedNow)
        DebugSerial.println("CO LINE");
    else
    {
        DebugSerial.print("MAT LINE | ");
        DebugSerial.println(searchStateName());
    }
}

int computePID(int error)
{
    unsigned long now = millis();

    double dt =
        (now - pidLastTime) / 1000.0;

    if (
        pidLastTime == 0 ||
        dt <= 0 ||
        dt > 0.5
    )
    {
        dt = 0.02;
    }

    double pTerm =
        PID_KP * error;

    pidIntegral +=
        error * dt;

    pidIntegral =
        constrain(
            pidIntegral,
            -PID_INTEGRAL_LIMIT,
            PID_INTEGRAL_LIMIT
        );

    double iTerm =
        PID_KI * pidIntegral;

    double derivative =
        (error - pidLastError) / dt;

    double dTerm =
        PID_KD * derivative;

    double output =
        pTerm +
        iTerm +
        dTerm;

    pidLastError = error;
    pidLastTime = now;

    int result = (int)output;

    result =
        constrain(
            result,
            -MAX_LATERAL,
            MAX_LATERAL
        );

    return result;
}

void resetPID()
{
    pidIntegral = 0.0;
    pidLastError = 0.0;
    pidLastTime = 0;
}

int calculateAdaptiveLineSpeed(int error)
{
    int absError = abs(error);

    absError =
        constrain(
            absError,
            0,
            200
        );

    int speed =
        map(
            absError,
            0,
            200,
            MAX_SPEED,
            MIN_LINE_SPEED
        );

    speed =
        constrain(
            speed,
            MIN_LINE_SPEED,
            MAX_SPEED
        );

    return speed;
}

void followLine()
{
    readLineSensor();

    int sum =
        lineP1 +
        lineP2 +
        lineP3 +
        lineP4 +
        lineP5;

    if (sum == 0)
    {
        unsigned long now = millis();

        if (lineLostSince == 0)
        {
            lineLostSince = now;

            searchState = SEARCH_SPIN;
            searchStateStart = now;
            searchSpinDuration = SEARCH_SPIN_INITIAL;

            DebugSerial.println(
                ">>> MAT LINE - BAT DAU TIM"
            );
        }

        if (
            now - lineLostSince >
            LINE_SEARCH_TIMEOUT
        )
        {
            stopCar();

            webSpeed = 0;
            webState = "MAT LINE - DUNG";

            DebugSerial.println(
                ">>> MAT LINE QUA LAU - DUNG"
            );

            return;
        }

        bool searchLeft =
            (lastLineError < 0);

        if (searchState == SEARCH_SPIN)
        {
            if (searchLeft)
                rotateLeft(SEARCH_SPEED);
            else
                rotateRight(SEARCH_SPEED);

            webSpeed = SEARCH_SPEED;
            webState = "TIM LINE - XOAY";

            if (
                now - searchStateStart >=
                (unsigned long)searchSpinDuration
            )
            {
                searchState = SEARCH_BACKUP;
                searchStateStart = now;
            }

            return;
        }

        if (searchState == SEARCH_BACKUP)
        {
            moveBackward(SEARCH_BACKUP_SPEED);

            webSpeed = SEARCH_BACKUP_SPEED;
            webState = "TIM LINE - LUI";

            if (
                now - searchStateStart >=
                SEARCH_BACKUP_DURATION
            )
            {
                searchState = SEARCH_SPIN;
                searchStateStart = now;

                searchSpinDuration +=
                    SEARCH_SPIN_STEP;

                if (
                    searchSpinDuration >
                    SEARCH_SPIN_MAX
                )
                {
                    searchSpinDuration =
                        SEARCH_SPIN_MAX;
                }
            }

            return;
        }
    }

    if (lineLostSince != 0)
    {
        resetPID();
    }

    lineLostSince = 0;

    searchState = SEARCH_SPIN;
    searchSpinDuration = SEARCH_SPIN_INITIAL;

    int output = computePID(lineError);

    int adaptiveSpeed =
        calculateAdaptiveLineSpeed(lineError);

    // Tốc độ tiến luôn dương
    int forwardSpeed = adaptiveSpeed;

    // Thành phần chỉnh ngang
    int lateralSpeed = output;

    lateralSpeed =
        constrain(
            lateralSpeed,
            -MAX_LATERAL,
            MAX_LATERAL
        );

    int fl = forwardSpeed + lateralSpeed;
    int fr = forwardSpeed - lateralSpeed;
    int rl = forwardSpeed + lateralSpeed;
    int rr = forwardSpeed - lateralSpeed;

    fl = constrain(fl, -255, 255);
    fr = constrain(fr, -255, 255);
    rl = constrain(rl, -255, 255);
    rr = constrain(rr, -255, 255);

    set4Wheels(
        fl,
        fr,
        rl,
        rr
    );

    webSpeed = forwardSpeed;
    webState = "DO LINE";
}

//           DÒ LẠI LINE NGAY SAU KHI NÉ VẬT CẢN

bool reacquireLineAfterAvoid()
{
    DebugSerial.println();
    DebugSerial.println("================================");
    DebugSerial.println("     DO LAI LINE SAU KHI NE");
    DebugSerial.println("================================");

    stopCar();
    webSpeed = 0;
    webState = "DO LAI LINE";

    // Reset trạng thái tìm line để bắt đầu một chu kỳ tìm mới.
    lineLostSince = 0;
    searchState = SEARCH_SPIN;
    searchStateStart = millis();
    searchSpinDuration = SEARCH_SPIN_INITIAL;
    resetPID();

    unsigned long startTime = millis();

    while (millis() - startTime < LINE_SEARCH_TIMEOUT)
    {
        followLine();

        // Đọc lại để xác nhận đã bắt được line.
        readLineSensor();

        if (lineDetectedNow)
        {
            stopCar();
            webSpeed = 0;
            webState = "DA BAT LAI LINE";

            lineLostSince = 0;
            searchState = SEARCH_SPIN;
            searchStateStart = millis();
            searchSpinDuration = SEARCH_SPIN_INITIAL;
            resetPID();

            DebugSerial.println(">>> DA BAT LAI LINE");
            delay(50);
            return true;
        }

        delay(20);
    }

    // Hết thời gian mà vẫn chưa có line -> dừng xe an toàn.
    stopCar();
    webSpeed = 0;
    webState = "KHONG TIM THAY LINE - DUNG";

    lineLostSince = 0;
    resetPID();

    DebugSerial.println(">>> KHONG TIM THAY LINE SAU KHI NE");
    DebugSerial.println(">>> XE DUNG");

    return false;
}

//                 GIAO DIỆN WEB

const char* html_page = R"rawliteral(

<!DOCTYPE html>

<html>

<head>

<meta charset='UTF-8'>

<meta name='viewport'
content='width=device-width,
initial-scale=1,
maximum-scale=1,
user-scalable=0'>

<title>AGV MECANUM</title>

<style>

body {
    text-align:center;
    font-family:Arial;
    background:#111;
    color:white;
    margin-top:15px;
    user-select:none;
}

h2 {
    margin-bottom:10px;
}

.status {
    font-size:18px;
    font-weight:bold;
    color:#00ff00;
    margin:10px;
}

button {
    border:none;
    color:white;
    touch-action:none;
    user-select:none;
    -webkit-user-select:none;
}

.control-groups {
    margin-top:20px;
    display:flex;
    flex-direction:row;
    justify-content:center;
    align-items:flex-start;
    gap:16px;
    flex-wrap:nowrap;
    padding:0 8px;
}

.group {
    background:#1b1b1b;
    border:1px solid #2a2a2a;
    border-radius:18px;
    padding:14px 12px 18px;
    box-shadow:0 4px 10px rgba(0,0,0,0.4);
    flex:0 1 auto;
}

.group-title {
    font-size:14px;
    font-weight:bold;
    letter-spacing:1px;
    color:#8a8a8a;
    margin-bottom:12px;
}

.control-panel {
    display:flex;
    flex-direction:column;
    align-items:center;
}

.row {
    display:flex;
    justify-content:center;
    align-items:center;
}

.arrow {
    width:75px;
    height:75px;
    margin:6px;
    border-radius:18px;
    background:#007bff;
    color:white;
    font-size:36px;
    font-weight:bold;
    box-shadow:0 4px 8px rgba(0,0,0,0.4);
    touch-action:none;
    user-select:none;
    -webkit-user-select:none;
}

.arrow:active {
    transform:scale(0.92);
    background:#0056b3;
}

.stop {
    background:#dc3545;
    font-size:30px;
}

.stop:active {
    background:#a71d2a;
}

@media (max-width: 380px) {
    .arrow {
        width:58px;
        height:58px;
        margin:4px;
        font-size:26px;
    }
    .stop {
        font-size:22px;
    }
    .control-groups {
        gap:10px;
    }
    .group {
        padding:10px 8px 14px;
    }
}

.rotate-panel {
    display:flex;
    flex-direction:column;
    align-items:center;
    justify-content:center;
    gap:18px;
    height:100%;
    padding:20px 0;
}

.rotate {
    background:#6f42c1;
}

.rotate:active {
    background:#4b2882;
}

.mode {
    width:250px;
    height:55px;
    margin:5px;
    font-size:17px;
    font-weight:bold;
    border-radius:12px;
    background:#28a745;
}

.info {
    margin-top:15px;
    color:#ffc107;
}

</style>

</head>

<body>

<h2>AGV MECANUM</h2>

<div id='status' class='status'>CHE DO: THU CONG</div>

<div style='font-size:20px;margin:8px'>
  TOC DO: <b id='speed'>0 / 255</b>
</div>

<div style='font-size:20px;margin:8px'>
  KHOANG CACH: <b id='distance'>-- cm</b>
</div>

<div style='font-size:20px;margin:8px'>
  TRANG THAI: <b id='state'>DUNG</b>
</div>

<div style='font-size:20px;margin:8px'>
  K210: <b id='k210'>NOOBJ</b>
</div>

<button id='mode'
class='mode'>
CHUYEN SANG TU HANH
</button>

<br>

<div class='control-groups'>

    <div class='group'>
        <div class='group-title'>DI CHUYEN</div>

        <div class='control-panel'>

            <!-- TIEN -->
            <div class='row'>
                <button class='arrow up' data-cmd='F' aria-label='Tien'>&#9650;</button>
            </div>

            <!-- NGANG TRAI - DUNG - NGANG PHAI -->
            <div class='row'>
                <button class='arrow' data-cmd='Q' aria-label='Ngang trai'>&#9664;</button>

                <button class='arrow stop' data-cmd='S' aria-label='Dung'>&#9679;</button>

                <button class='arrow' data-cmd='E' aria-label='Ngang phai'>&#9654;</button>
            </div>

            <!-- LUI -->
            <div class='row'>
                <button class='arrow down' data-cmd='B' aria-label='Lui'>&#9660;</button>
            </div>

        </div>
    </div>

    <div class='group'>
        <div class='group-title'>XOAY</div>

        <div class='rotate-panel'>
            <button class='arrow rotate' data-cmd='L' aria-label='Xoay trai'>&#8634;</button>
            <button class='arrow rotate' data-cmd='R' aria-label='Xoay phai'>&#8635;</button>
        </div>
    </div>

</div>

<div class='info' id='info'>
Ket noi WiFi AGV_MECANUM
</div>

<script>

let autoMode = false;
let lastCmd = 'S';

function updateStatus()
{
    fetch('/status')
      .then(r => r.json())
      .then(d => {
          document.getElementById('speed').innerText =
              d.speed + ' / 255';

          document.getElementById('distance').innerText =
              d.distance.toFixed(1) + ' cm';

          document.getElementById('state').innerText =
              d.state;

          document.getElementById('status').innerText =
              d.auto ? 'CHE DO: TU HANH' : 'CHE DO: THU CONG';

          document.getElementById('k210').innerText =
              d.k210Detected ?
              (d.k210Object + ' ' + d.k210Confidence + '%') :
              'KHONG PHAT HIEN';
      })
      .catch(() => {});
}

setInterval(updateStatus, 200);
updateStatus();

function sendCommand(cmd)
{
    if (cmd !== lastCmd || cmd === 'S')
    {
        fetch('/action?dir=' + cmd);
        lastCmd = cmd;
    }
}

document.getElementById('mode')
.onclick = function()
{
    if (!autoMode)
    {
        autoMode = true;

        this.innerText = 'DANG TU HANH';
        this.style.background = '#ffc107';
        this.style.color = '#000';

        document.getElementById('status').innerText =
            'CHE DO: TU HANH';

        sendCommand('A');
    }
    else
    {
        sendCommand('A');

        this.innerText = 'DANG TU HANH';
        this.style.background = '#ffc107';
        this.style.color = '#000';

        document.getElementById('status').innerText =
            'CHE DO: TU HANH - TIEP TUC';
    }
};

document
.querySelectorAll('[data-cmd]')
.forEach(function(button)
{
    let cmd =
        button.getAttribute('data-cmd');

    button.onmousedown =
        function()
        {
            sendCommand(cmd);
        };

    button.onmouseup =
        function()
        {
            sendCommand('S');
        };

    button.onmouseleave =
        function()
        {
            sendCommand('S');
        };

    button.ontouchstart =
        function(e)
        {
            e.preventDefault();

            sendCommand(cmd);
        };

    button.ontouchend =
        function(e)
        {
            e.preventDefault();

            sendCommand('S');
        };
});

window.addEventListener(
'gamepadconnected',
function()
{
    document.getElementById('info')
        .innerText =
        'GAMEPAD DA KET NOI';
});

setInterval(function()
{
    if (autoMode)
        return;

    let pads =
        navigator.getGamepads();

    if (!pads[0])
        return;

    let gp = pads[0];

    let cmd = 'S';

    let lx = gp.axes[0];
    let ly = gp.axes[1];
    let rx = gp.axes[2];

    let deadzone = 0.3;

    if (Math.abs(rx) > deadzone)
    {
        if (rx > 0)
            cmd = 'R';
        else
            cmd = 'L';
    }

    else if (
        Math.abs(ly) > deadzone ||
        Math.abs(lx) > deadzone
    )
    {
        if (Math.abs(ly) >
            Math.abs(lx))
        {
            if (ly > 0)
                cmd = 'B';
            else
                cmd = 'F';
        }
        else
        {
            if (lx > 0)
                cmd = 'E';
            else
                cmd = 'Q';
        }
    }

    sendCommand(cmd);

}, 50);

</script>

</body>

</html>

)rawliteral";

//                 TRẠNG THÁI WEB

void handleStatus()
{
    String json = "{";
    json += "\"auto\":" + String(isAutoMode ? "true" : "false") + ",";
    json += "\"speed\":" + String(webSpeed) + ",";
    json += "\"distance\":" + String(webDistance, 1) + ",";
    json += "\"state\":\"" + webState + "\",";
    json += "\"k210Detected\":" + String(k210Detected ? "true" : "false") + ",";
    json += "\"k210Object\":\"" + k210Object + "\",";
    json += "\"k210Confidence\":" + String(k210Confidence) + ",";
    json += "\"k210X\":" + String(k210X) + ",";
    json += "\"k210Y\":" + String(k210Y) + ",";
    json += "\"k210W\":" + String(k210W) + ",";
    json += "\"k210H\":" + String(k210H);
    json += "}";

    server.send(200, "application/json", json);
}

//                 XỬ LÝ TRANG WEB

void handleRoot()
{
    server.send(
        200,
        "text/html; charset=UTF-8",
        html_page
    );
}

//                 XỬ LÝ LỆNH

void handleAction()
{
    if (server.hasArg("dir"))
    {
        String cmd = server.arg("dir");

        if (cmd.length() > 0)
        {
            current_cmd = cmd[0];

            // A = TRỞ LẠI / TIẾP TỤC TỰ HÀNH
            if (current_cmd == 'A')
            {
                isAutoMode = true;
                manualOverride = false;
                overrideCmd = 'S';

                stopCar();
                webSpeed = 0;
                webState = "TIEP TUC TU HANH";

                DebugSerial.println();
                DebugSerial.println(">>> TIEP TUC CHE DO TU HANH");
            }

            // CÁC LỆNH KHÁC = ĐIỀU KHIỂN THỦ CÔNG
            else
            {
                // Không tắt AUTO.
                // Chỉ tạm giao quyền điều khiển cho người dùng.
                manualOverride = true;
                overrideCmd = current_cmd;

                DebugSerial.print(">>> CAN THIEP THU CONG: ");
                DebugSerial.println(current_cmd);

                if (current_cmd == 'S')
                {
                    stopCar();
                    webSpeed = 0;
                    webState = isAutoMode ?
                               "AUTO: DUNG BOI NGUOI DUNG" :
                               "DUNG";
                }
            }
        }
    }

    server.send(200, "text/plain", "OK");
}

//                   K210 UART PARSER
// Protocol from K210:
// OBJ,<label>,<confidence>,<x>,<y>,<w>,<h>
// NOOBJ

void parseK210Line(const String &line)
{
    String s = line;
    s.trim();
    if (s.length() == 0) return;

    if (s == "NOOBJ")
    {
        // Giữ kết quả nhận diện gần nhất trong một khoảng ngắn để tránh
        // mất detection do khung hình YOLO chập chờn.
        k210LastPacket = millis();
        return;
    }

    if (!s.startsWith("OBJ,")) return;

    int p1 = s.indexOf(',', 4);
    int p2 = s.indexOf(',', p1 + 1);
    int p3 = s.indexOf(',', p2 + 1);
    int p4 = s.indexOf(',', p3 + 1);
    int p5 = s.indexOf(',', p4 + 1);
    if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0 || p5 < 0) return;

    k210Object = s.substring(4, p1);
    k210Confidence = s.substring(p1 + 1, p2).toInt();
    k210X = s.substring(p2 + 1, p3).toInt();
    k210Y = s.substring(p3 + 1, p4).toInt();
    k210W = s.substring(p4 + 1, p5).toInt();
    k210H = s.substring(p5 + 1).toInt();
    k210Detected = true;
    k210LastPacket = millis();
    k210LastObjectSeen = k210LastPacket;
}

void readK210UART()
{
    while (K210Serial.available())
    {
        char c = (char)K210Serial.read();

        if (c == '\n')
        {
            parseK210Line(k210RxLine);
            k210RxLine = "";
        }
        else if (c != '\r')
        {
            if (k210RxLine.length() < 180)
                k210RxLine += c;
            else
                k210RxLine = "";
        }
    }

    // Chỉ giữ detection khi vừa nhận được một vật thể trong thời gian gần.
    // NOOBJ đơn lẻ không xóa ngay kết quả để tránh bỏ lỡ khung hình nhận diện.
    if (k210LastObjectSeen == 0 ||
        millis() - k210LastObjectSeen > K210_DETECTION_HOLD_MS)
    {
        k210Detected = false;
        k210Object = "NOOBJ";
        k210Confidence = 0;
        k210X = k210Y = k210W = k210H = 0;
    }
}

//                     SETUP

void setup()
{
    // UART0: K210 <-> ESP32 GPIO1/3
    K210Serial.begin(K210_BAUD, SERIAL_8N1, K210_RX_PIN, K210_TX_PIN);

    // UART2: debug output, optional USB-TTL on GPIO16/17
    DebugSerial.begin(115200, SERIAL_8N1, DEBUG_RX_PIN, DEBUG_TX_PIN);

    // HC-SR04

    pinMode(
        TRIG_PIN,
        OUTPUT
    );

    pinMode(
        ECHO_PIN,
        INPUT
    );

    digitalWrite(
        TRIG_PIN,
        LOW
    );

    // SERVO

    myServo.setPeriodHertz(50);

    myServo.attach(
        SERVO_PIN,
        500,
        2400
    );

    myServo.write(
        CENTER_ANGLE
    );

    // I2C MKE

    Wire.begin(
        SDA_FRONT,
        SCL_FRONT
    );

    Wire1.begin(
        SDA_REAR,
        SCL_REAR
    );

    // Driver phía trước
    driverFront.begin(
        0x40,
        Wire
    );

    // Driver phía sau
    driverRear.begin(
        0x40,
        Wire1
    );

    // MKE-S16
    selectLineI2C();

    LineFollowSensor.setup(
        LINE_SENSOR_ADDRESS
    );

    DebugSerial.println(
        "MKE-S16: OK"
    );

    // DỪNG MOTOR KHI KHỞI ĐỘNG

    stopCar();

    // TẠO WIFI AP

    WiFi.softAP(
        AP_SSID,
        AP_PASS
    );

    IPAddress IP =
        WiFi.softAPIP();

    // WEB SERVER

    server.on(
        "/",
        handleRoot
    );

    server.on(
        "/action",
        handleAction
    );

    server.on(
        "/status",
        handleStatus
    );

    server.begin();

    // SERIAL

    DebugSerial.println();
    DebugSerial.println(
        "======================================"
    );

    DebugSerial.println(
        "       AGV MECANUM ESP32"
    );

    DebugSerial.println(
        "======================================"
    );

    DebugSerial.println();

    DebugSerial.print(
        "WiFi: "
    );

    DebugSerial.println(
        AP_SSID
    );

    DebugSerial.print(
        "IP: "
    );

    DebugSerial.println(
        IP
    );

    DebugSerial.println();

    DebugSerial.println(
        "Mo trinh duyet va truy cap:"
    );

    DebugSerial.println(
        "http://192.168.4.1"
    );

    DebugSerial.println();

    DebugSerial.println(
        "CHE DO BAN DAU: THU CONG"
    );

    DebugSerial.println();
}

//                       LOOP

void loop()
{
    // Nhận dữ liệu YOLOv2 từ K210 qua GPIO3/1
    readK210UART();

    // XỬ LÝ WEB
    server.handleClient();

    // LỆNH CAN THIỆP THỦ CÔNG
    if (isAutoMode && manualOverride)
    {
        float d = readDistance();
        webDistance = d;

        int speed = 180;

        switch (overrideCmd)
        {
            case 'F':
                if (d <= STOP_DISTANCE)
                {
                    stopCar();
                    webSpeed = 0;
                    webState = "CAN THIEP: DUNG - VAT CAN";
                }
                else
                {
                    moveForward(speed);
                    webSpeed = speed;
                    webState = "CAN THIEP: TIEN";
                }
                break;

            case 'B':
                moveBackward(speed);
                webSpeed = speed;
                webState = "CAN THIEP: LUI";
                break;

            case 'Q':
                moveLeft(speed);
                webSpeed = speed;
                webState = "CAN THIEP: NGANG TRAI";
                break;

            case 'E':
                moveRight(speed);
                webSpeed = speed;
                webState = "CAN THIEP: NGANG PHAI";
                break;

            case 'L':
                rotateLeft(speed);
                webSpeed = speed;
                webState = "CAN THIEP: XOAY TRAI";
                break;

            case 'R':
                rotateRight(speed);
                webSpeed = speed;
                webState = "CAN THIEP: XOAY PHAI";
                break;

            case 'S':
            default:
                stopCar();
                webSpeed = 0;
                webState = "CAN THIEP: DUNG";
                break;
        }

        delay(20);
        return;
    }

    // CHẾ ĐỘ TỰ HÀNH
    if (isAutoMode)
    {
        myServo.write(CENTER_ANGLE);
        delay(20);

        float distanceFront = readDistance();
        webDistance = distanceFront;

        DebugSerial.print("AUTO | Khoang cach: ");
        DebugSerial.print(distanceFront, 1);
        DebugSerial.print(" cm");

        // CHƯA TỚI NGƯỠNG 30 CM -> TIẾP TỤC DÒ LINE
        if (distanceFront > STOP_DISTANCE)
        {
            followLine();

            DebugSerial.print(" | DO LINE");
            DebugSerial.println();
        }
        else
        {
            // Đã tới 30 cm: dừng hẳn và chờ K210 xác nhận vật cản.
            stopCar();
            webSpeed = 0;
            webState = "DUNG 25CM - CHO K210";
            DebugSerial.println(" | DUNG 30CM - CHO K210 NHAN DIEN");

            // Trong lúc chờ, vẫn phục vụ web và đọc UART K210.
            // Nếu K210 chưa nhận diện được thì xe tiếp tục đứng yên.
            while (isAutoMode && !manualOverride && !k210Detected)
            {
                stopCar();
                webSpeed = 0;
                webState = "DUNG 30CM - CHO K210";
                server.handleClient();
                readK210UART();

                if (!isAutoMode || manualOverride)
                    break;

                delay(10);
            }

            // Chỉ né khi K210 nhận diện được vật thể.
            if (isAutoMode && !manualOverride && k210Detected)
            {
                DebugSerial.print(">>> K210 NHAN DIEN: ");
                DebugSerial.print(k210Object);
                DebugSerial.print(" ");
                DebugSerial.print(k210Confidence);
                DebugSerial.println("% - BAT DAU NE");

                webState = "K210 NHAN DIEN - BAT DAU NE";
                delay(150);
                scanAndAvoid();

                myServo.write(CENTER_ANGLE);
                delay(100);
            }
            else
            {
                stopCar();
                webSpeed = 0;
                webState = "DUNG - CHO LENH / K210";
            }
        }

        delay(80);
    }

    // CHẾ ĐỘ THỦ CÔNG
    else
    {
        int speed = 180;

        switch(current_cmd)
        {
            case 'F':
                moveForward(speed);
                webSpeed = speed;
                webState = "THU CONG: TIEN";
                break;

            case 'B':
                moveBackward(speed);
                webSpeed = speed;
                webState = "THU CONG: LUI";
                break;

            case 'Q':
                moveLeft(speed);
                webSpeed = speed;
                webState = "THU CONG: NGANG TRAI";
                break;

            case 'E':
                moveRight(speed);
                webSpeed = speed;
                webState = "THU CONG: NGANG PHAI";
                break;

            case 'L':
                rotateLeft(speed);
                webSpeed = speed;
                webState = "THU CONG: XOAY TRAI";
                break;

            case 'R':
                rotateRight(speed);
                webSpeed = speed;
                webState = "THU CONG: XOAY PHAI";
                break;

            case 'S':
            default:
                stopCar();
                webSpeed = 0;
                webState = "DUNG";
                break;
        }

        static unsigned long lastPrint = 0;

        if (millis() - lastPrint >= 100)
        {
            lastPrint = millis();
            printLineSensor();
        }

        delay(20);
    }
}


