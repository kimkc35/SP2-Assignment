#define PIN_LED   9
#define PIN_TRIG 12
#define PIN_ECHO 13

#define SND_VEL 346.0
#define INTERVAL 25
#define PULSE_DURATION 10
#define _DIST_MIN 100
#define _DIST_MAX 300

#define TIMEOUT ((INTERVAL / 2) * 1000.0)
#define SCALE (0.001 * 0.5 * SND_VEL)

#define _EMA_ALPHA 0.5

#define N_SAMPLES 30

unsigned long last_sampling_time;
float dist_prev = _DIST_MAX;
float dist_ema = 0.0;

float dist_samples[N_SAMPLES];
int sample_index = 0;
float dist_median;

float median_filter(float *data, int size) {
  float temp[size];
  for (int i = 0; i < size; i++) {
    temp[i] = data[i];
  }

  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (temp[j] > temp[j + 1]) {
        float swap = temp[j];
        temp[j] = temp[j + 1];
        temp[j + 1] = swap;
      }
    }
  }
  
  return temp[size / 2]; 
}


void setup() {
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  pinMode(PIN_ECHO,INPUT);
  digitalWrite(PIN_TRIG, LOW);

  Serial.begin(57600);

  for (int i = 0; i < N_SAMPLES; i++) {
    dist_samples[i] = 0.0;
  }
  
  dist_ema = 0.0; 
}

void loop() {
  float dist_raw;
  
  if (millis() < last_sampling_time + INTERVAL)
    return;

  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);

  dist_samples[sample_index] = dist_raw;
  sample_index = (sample_index + 1) % N_SAMPLES;

  dist_median = median_filter(dist_samples, N_SAMPLES);
  
  Serial.print("Min:");   Serial.print(_DIST_MIN);
  Serial.print(",raw:"); Serial.print(dist_raw); 
  Serial.print(",median:"); Serial.print(dist_median); 
  Serial.print(",ema:"); Serial.print(dist_ema);     
  Serial.print(",Max:");  Serial.print(_DIST_MAX);
  Serial.println("");

  if ((dist_raw < _DIST_MIN) || (dist_raw > _DIST_MAX) || (dist_raw == 0.0))
    digitalWrite(PIN_LED, 1);       
  else
    digitalWrite(PIN_LED, 0);       

  last_sampling_time += INTERVAL;
}

float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE;
}
