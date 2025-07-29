/*
 * Arduino Dual Encoder Publisher
 * Reads two quadrature encoders and publishes via serial
 * ENC1: pins 2&3 (interrupt-based)
 * ENC2: pins 4&5 (polling-based)
 */

// Encoder 1 variables (interrupt-based)
volatile long encoder1_count = 0;
int encoder1_pin_a = 2;
int encoder1_pin_b = 3;

// Encoder 2 variables (polling-based)
long encoder2_count = 0;
int encoder2_pin_a = 4;
int encoder2_pin_b = 5;
int encoder2_last_a = 0;
int encoder2_last_b = 0;

void setup() {
  Serial.begin(38400);
  
  // Setup encoder 1 pins (interrupt-based)
  pinMode(encoder1_pin_a, INPUT_PULLUP);
  pinMode(encoder1_pin_b, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1_pin_a), encoder1_isr, CHANGE);
  
  // Setup encoder 2 pins (polling-based)
  pinMode(encoder2_pin_a, INPUT_PULLUP);
  pinMode(encoder2_pin_b, INPUT_PULLUP);
  encoder2_last_a = digitalRead(encoder2_pin_a);
  encoder2_last_b = digitalRead(encoder2_pin_b);
  
  delay(100);
}

void loop() {
  // Read encoder 2 (polling)
  read_encoder2();
  
  // Send encoder values
  Serial.print("ENC1:");
  Serial.print(encoder1_count);
  Serial.print(",ENC2:");
  Serial.println(encoder2_count);
  
  delay(50); // 20Hz update rate
}

// Encoder 1 interrupt service routine
void encoder1_isr() {
  int a = digitalRead(encoder1_pin_a);
  int b = digitalRead(encoder1_pin_b);
  
  if (a == HIGH) {
    if (b == LOW) {
      encoder1_count++;
    } else {
      encoder1_count--;
    }
  } else {
    if (b == HIGH) {
      encoder1_count++;
    } else {
      encoder1_count--;
    }
  }
}

// Encoder 2 polling function
void read_encoder2() {
  int a = digitalRead(encoder2_pin_a);
  int b = digitalRead(encoder2_pin_b);
  
  if (a != encoder2_last_a) {
    if (a == HIGH) {
      if (b == LOW) {
        encoder2_count++;
      } else {
        encoder2_count--;
      }
    } else {
      if (b == HIGH) {
        encoder2_count++;
      } else {
        encoder2_count--;
      }
    }
    encoder2_last_a = a;
  }
  
  if (b != encoder2_last_b) {
    encoder2_last_b = b;
  }
}
