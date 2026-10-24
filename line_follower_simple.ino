// =====================================================
// 5-Channel IR Line Follower - SIMPLE working baseline
// Mount sensor array at ~5cm above the track (needed for
// this sensor type's emitter/receiver angle to work at all)
// =====================================================

#define S1 A0
#define S2 A1
#define S3 A2
#define S4 A3
#define S5 A4

#define ENA 5
#define IN1 7
#define IN2 8
#define ENB 6
#define IN3 9
#define IN4 10

// ---------- SET THIS FIRST ----------
// Run with the debug Serial.print lines (near the bottom) uncommented,
// open Serial Monitor, hold the bot at its real 5cm mounting height,
// and note the values over white vs over black. Set threshold to
// roughly the middle of those two numbers.
int threshold = 500;

// ---------- Speed/smoothness ----------
int baseSpeed = 90;   // cruising speed, keep low while testing
int maxSpeed  = 100;  // hard cap
float Kp = 18.0;      // how hard it corrects
float Kd = 8.0;       // damps the correction so it doesn't overshoot
int maxDelta = 8;     // limits how fast motor speed can change per loop (smoothness)

float lastError = 0;
int lastLeft = 0;
int lastRight = 0;

int sensorPins[5] = {S1, S2, S3, S4, S5};
int weights[5]    = {-2, -1, 0, 1, 2}; // left to right

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  Serial.begin(9600);
}

void loop() {
  int values[5];
  long weightedSum = 0;
  long sum = 0;
  bool lineDetected = false;

  for (int i = 0; i < 5; i++) {
    values[i] = analogRead(sensorPins[i]);
    int onLine = (values[i] > threshold) ? 1 : 0; // flip to "<" if it's backwards on your sensor
    if (onLine) lineDetected = true;
    weightedSum += (long)weights[i] * onLine;
    sum += onLine;
  }

  float error;
  if (lineDetected && sum != 0) {
    error = (float)weightedSum / (float)sum;
  } else {
    // line lost - keep turning toward whichever side it last saw the line
    error = (lastError > 0) ? 2.5 : -2.5;
  }

  float output = Kp * error + Kd * (error - lastError);
  lastError = error;

  int leftSpeed  = constrain(baseSpeed - output, 0, maxSpeed);
  int rightSpeed = constrain(baseSpeed + output, 0, maxSpeed);

  leftSpeed  = slew(leftSpeed, lastLeft, maxDelta);
  rightSpeed = slew(rightSpeed, lastRight, maxDelta);
  lastLeft  = leftSpeed;
  lastRight = rightSpeed;

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);

  // ---- DEBUG: uncomment to see raw sensor values in Serial Monitor ----
  // Serial.print(values[0]); Serial.print(" ");
  // Serial.print(values[1]); Serial.print(" ");
  // Serial.print(values[2]); Serial.print(" ");
  // Serial.print(values[3]); Serial.print(" ");
  // Serial.println(values[4]);

  delay(10);
}

int slew(int target, int current, int maxStep) {
  int diff = target - current;
  if (diff > maxStep) diff = maxStep;
  if (diff < -maxStep) diff = -maxStep;
  return current + diff;
}
