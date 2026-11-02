#include <Servo.h>

#define PIN_IR    A0
#define PIN_LED   9
#define PIN_SERVO 10

#define _DUTY_MIN 542
#define _DUTY_NEU 1475
#define _DUTY_MAX 2408

#define _DIST_MIN 100.0  // mm
#define _DIST_MAX 250.0  // mm
#define EMA_ALPHA 0.2
#define LOOP_INTERVAL 20  // ms

Servo myservo;
unsigned long last_loop_time;
float dist_prev = _DIST_MIN;
float dist_ema = _DIST_MIN;
bool has_valid_distance = false;

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);
  Serial.begin(1000000);
  last_loop_time = millis();
}

void loop()
{
  unsigned long time_curr = millis();
  if (time_curr - last_loop_time < LOOP_INTERVAL)
    return;
  last_loop_time = time_curr;

  int a_value = analogRead(PIN_IR);
  // Reject ADC <= 9 before division; -1 denotes an invalid reading.
  float dist_raw = -1.0;
  if (a_value > 9)
    dist_raw = (6762.0 / (a_value - 9) - 4.0) * 10.0 - 60.0;

  bool in_range = dist_raw >= _DIST_MIN && dist_raw <= _DIST_MAX;
  digitalWrite(PIN_LED, in_range ? LOW : HIGH);

  // Range filter: retain the last valid distance for rejected readings.
  if (in_range)
  {
    dist_prev = dist_raw;
    if (!has_valid_distance)
    {
      dist_ema = dist_raw;
      has_valid_distance = true;
    }
  }

  int duty = _DUTY_NEU;
  if (has_valid_distance)
  {
    dist_ema = EMA_ALPHA * dist_prev + (1.0 - EMA_ALPHA) * dist_ema;
    // Linear interpolation: 100 mm -> 542 us, 250 mm -> 2408 us.
    duty = (int)(_DUTY_MIN + (dist_ema - _DIST_MIN)
           * (_DUTY_MAX - _DUTY_MIN) / (_DIST_MAX - _DIST_MIN) + 0.5);
    duty = constrain(duty, _DUTY_MIN, _DUTY_MAX);
    myservo.writeMicroseconds(duty);
  }

  // One sample per line, comma-separated labelled values for Serial Plotter.
  Serial.print("_DUTY_MIN:"); Serial.print(_DUTY_MIN);
  Serial.print(",_DIST_MIN:"); Serial.print(_DIST_MIN);
  Serial.print(",IR:"); Serial.print(a_value);
  Serial.print(",dist_raw:"); Serial.print(dist_raw);
  Serial.print(",ema:"); Serial.print(dist_ema);
  Serial.print(",servo:"); Serial.print(duty);
  Serial.print(",_DIST_MAX:"); Serial.print(_DIST_MAX);
  Serial.print(",_DUTY_MAX:"); Serial.print(_DUTY_MAX);
  Serial.println();
}
