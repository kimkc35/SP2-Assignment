#include <Servo.h>
#include <math.h>

#define PIN_SERVO 10
#define PIN_TRIG 12
#define PIN_ECHO 13

Servo myServo;

const int ANGLE_DOWN = 0;
const int ANGLE_UP = 80;

const unsigned long MOVING_TIME = 3000;
const unsigned long WAIT_TIME = 1000;

const int DISTANCE_MAX_DETECT = 35;
const int DISTANCE_PASS_THRESHOLD = 20;

enum MotionCurve {
    CURVE_SIGMOID,
    CURVE_ATAN
};

const MotionCurve CURVE_MODE = CURVE_ATAN;

const float SIGMOID_K_FACTOR = 6.0;
const float ATAN_K_FACTOR = 6.0;

unsigned long moveStartTime = 0;
unsigned long waitStartTime = 0;

bool targetIsUp = false;
bool isWaiting = false;
bool isMoving = false;

long readDistance() {
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    unsigned long duration = pulseIn(PIN_ECHO, HIGH, 25000UL);

    if (duration == 0) return 999;

    return (long)(duration * 0.034 / 2.0);
}

float sigmoid(float x, float k) {
    float z = (x * 2.0 * k) - k;
    return 1.0 / (1.0 + exp(-z));
}

float motionProgress(float x) {
    if (x <= 0.0) return 0.0;
    if (x >= 1.0) return 1.0;

    if (CURVE_MODE == CURVE_SIGMOID) {
        float k = SIGMOID_K_FACTOR;

        if (k <= 0.0) return x;

        float minValue = sigmoid(0.0, k);
        float maxValue = sigmoid(1.0, k);
        float value = sigmoid(x, k);

        return (value - minValue) / (maxValue - minValue);
    } else {
        float k = ATAN_K_FACTOR;

        if (k <= 0.0) return x;

        float value = atan((2.0 * x - 1.0) * k);

        return (value + atan(k)) / (2.0 * atan(k));
    }
}

void setup() {
    Serial.begin(9600);

    myServo.attach(PIN_SERVO);
    myServo.write(ANGLE_DOWN);

    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    digitalWrite(PIN_TRIG, LOW);
}

void loop() {
    long distance = readDistance();
    unsigned long currentTime = millis();

    if (!isMoving && !isWaiting) {
        if (distance <= DISTANCE_MAX_DETECT &&
            distance > DISTANCE_PASS_THRESHOLD &&
            !targetIsUp) {

            moveStartTime = currentTime;
            targetIsUp = true;
            isMoving = true;

        } else if (distance <= DISTANCE_PASS_THRESHOLD &&
                   targetIsUp) {

            waitStartTime = currentTime;
            isWaiting = true;
        }
    }

    if (isWaiting) {
        myServo.write(ANGLE_UP);

        if (currentTime - waitStartTime >= WAIT_TIME) {
            moveStartTime = currentTime;
            isWaiting = false;
            targetIsUp = false;
            isMoving = true;
        }
    }

    if (isMoving) {
        unsigned long timeElapsed = currentTime - moveStartTime;

        if (timeElapsed >= MOVING_TIME) {
            myServo.write(targetIsUp ? ANGLE_UP : ANGLE_DOWN);
            isMoving = false;
        } else {
            float progress = (float)timeElapsed / MOVING_TIME;
            float curveValue = motionProgress(progress);

            int startAngle = targetIsUp ? ANGLE_DOWN : ANGLE_UP;
            int endAngle = targetIsUp ? ANGLE_UP : ANGLE_DOWN;

            int currentAngle = (int)(
                startAngle + curveValue * (endAngle - startAngle) + 0.5
            );

            myServo.write(currentAngle);
        }
    }

    delay(50);
}
