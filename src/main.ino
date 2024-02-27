#define LOCK_PIN    12
#define PWM_PIN     4
#define PWM_STEPS   1024

#define PWM_CHANNEL 0
#define PWM_RES     10
#define PWM_FREQ    5000

#define OPEN_DELAY  3000

int opened = 0;
uint32_t last_millis;
// The R value in the graph equation
float r;
int brightness = 700;

void setup() {

  // configure GPIO
  pinMode(LOCK_PIN, OUTPUT);
  digitalWrite(LOCK_PIN, LOW);


  // serial
  Serial.begin(115200);
  
  // configure ESP32 PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  

  // Calculate the R variable (only needs to be done once at setup)
  r = (PWM_STEPS * log10(2))/(log10(PWM_STEPS));

  Serial.println("Started!");
}

void loop() {
  // animation delay
  delay(5);

  // run animation
  // TODO


  // check serial commands
  if (Serial.available() > 0) {
    uint8_t c = Serial.read();

    if (c == 'O') {
      Serial.println("Opening...");
      digitalWrite(LOCK_PIN, HIGH);
      opened = 1;
      last_millis = millis();
    }
  }

  // check if we need to release the lock
  if (opened && (millis() - last_millis > OPEN_DELAY)) {
    Serial.println("Releasing lock...");
    digitalWrite(LOCK_PIN, LOW);
    opened = 0;
  }

/* for (int interval = 700; interval <= PWM_STEPS; interval++) { */
  /*     // Calculate the required PWM value for this interval step */
  /*     brightness = pow (2, (interval / r)) - 1; */
  /*     // Set the LED output to the calculated brightness */
  /*     ledcWrite(PWM_CHANNEL, brightness); */
  /*     delay(5); */
  /* } */
  /* for (int interval = PWM_STEPS; interval >= 700; interval--) { */
  /*     // Calculate the required PWM value for this interval step */
  /*     brightness = pow (2, (interval / r)) - 1; */
  /*     // Set the LED output to the calculated brightness */
  /*     ledcWrite(PWM_CHANNEL, brightness); */
  /*     delay(5); */
  /* } */
  /* delay(3000); */
  
  
}
