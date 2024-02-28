// GPIO
#define LOCK_PIN    12
#define CLOSED_PIN  14
#define PWM_PIN     4

// PWM
#define PWM_CHANNEL 0
#define PWM_RES     10
#define PWM_STEPS   1024
#define PWM_FREQ    5000

// CONFIG
#define OPEN_DELAY  1000
#define LOW_BRIGHT  700

// SERIAL COMMANDS
#define CMD_OPEN    'O'
#define CMD_STATUS  'S'

// vars
int opening = 0;
int closed = 1;
uint32_t last_millis;
float r;                     // The R value in the graph equation
int brightness;              // actual pwm value
int interval = LOW_BRIGHT;   // animation position


void setup() {
  // configure GPIO
  pinMode(LOCK_PIN, OUTPUT);
  pinMode(CLOSED_PIN, INPUT_PULLUP);
  digitalWrite(LOCK_PIN, LOW);

  // serial
  Serial.begin(115200);
  
  // configure ESP32 PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);


  // Calculate the R variable for more linear brightness
  // (only needs to be done once at setup)
  r = (PWM_STEPS * log10(2))/(log10(PWM_STEPS));
  // initial brightness
  brightness = pow (2, (interval / r)) - 1;
  ledcWrite(PWM_CHANNEL, brightness);
  
  Serial.println("Started!");
}

void loop() {
  // animation delay
  delay(5);

  // check serial commands
  if (Serial.available() > 0) {
    uint8_t c = Serial.read();

    if (c == CMD_OPEN) {
      Serial.println("Opening...");
      digitalWrite(LOCK_PIN, HIGH);
      opening = 1;
      last_millis = millis();
    }
    if (c == CMD_STATUS) {
      Serial.println(closed ? 'C' : 'O');
    }
  }

  // check if we need to release the lock
  if (opening && (millis() - last_millis > OPEN_DELAY)) {
    Serial.println("Releasing lock...");
    digitalWrite(LOCK_PIN, LOW);
    opening = 0;
  }

  // animate lights?
  if (opening == 1) {
    if (interval < PWM_STEPS) {
      interval++;
      brightness = pow (2, (interval / r)) - 1;
      ledcWrite(PWM_CHANNEL, brightness);
    }
  }

  // check for lock sensor and dim the lights
  // when closed  
  closed = !digitalRead(CLOSED_PIN);
  if (closed && !opening) {
    interval = LOW_BRIGHT;
    brightness = pow (2, (interval / r)) - 1;
    ledcWrite(PWM_CHANNEL, brightness);
  }
}
