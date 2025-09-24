/*
 * Simple Arduino Encoder Bridge - Clean Version
 * Sends only ENC1:value,ENC2:value format
 * Encoder 1: Pins 4,5 (manual polling)
 * Encoder 2: Pins 2,3 (interrupt-based)
 */

// Encoder 1 variables (manual polling on pins 4,5)
volatile long encoder1_count = 0;
int encoder1_pin_a = 4;
int encoder1_pin_b = 5;

// Encoder 2 variables (interrupt-based on pins 2,3)
volatile long encoder2_count = 0;
int encoder2_pin_a = 2;  // Interrupt 0
int encoder2_pin_b = 3;  // Interrupt 1

// Manual polling state for encoder 1
int last_enc1_a = HIGH;
int last_enc1_b = HIGH;

void setup() {
  Serial.begin(38400);
  
  // Setup encoder 1 pins (manual polling)
  pinMode(encoder1_pin_a, INPUT_PULLUP);
  pinMode(encoder1_pin_b, INPUT_PULLUP);
  
  // Setup encoder 2 pins (interrupt-based)
  pinMode(encoder2_pin_a, INPUT_PULLUP);
  pinMode(encoder2_pin_b, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder2_pin_a), encoder2_isr_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2_pin_b), encoder2_isr_b, CHANGE);
  
  // Initialize encoder 1 state
  last_enc1_a = digitalRead(encoder1_pin_a);
  last_enc1_b = digitalRead(encoder1_pin_b);
  
  delay(100);
}

void loop() {
  // Check for reset command
  if(Serial.available() > 0) {
    String command = Serial.readString();
    command.trim();
    if(command == "RESET") {
      encoder1_count = 0;
      encoder2_count = 0;
      Serial.println("ENCODERS_RESET");
      return;
    }
  }
  
  // Manual polling for encoder 1
  int enc1_a_now = digitalRead(encoder1_pin_a);
  int enc1_b_now = digitalRead(encoder1_pin_b);
  
  // Detect changes on encoder 1 pin A
  if(enc1_a_now != last_enc1_a) {
    if(enc1_a_now == HIGH) {
      if(enc1_b_now == LOW) {
        encoder1_count++;
      } else {
        encoder1_count--;
      }
    } else {
      if(enc1_b_now == HIGH) {
        encoder1_count++;
      } else {
        encoder1_count--;
      }
    }
  }
  
  // Update state
  last_enc1_a = enc1_a_now;
  last_enc1_b = enc1_b_now;
  
  // Send ONLY the encoder data in clean format
  Serial.print("ENC1:");
  Serial.print(encoder1_count);
  Serial.print(",ENC2:");
  Serial.println(encoder2_count);
  
  delay(50); // 20Hz update rate
}

// Encoder 2 interrupt handlers
void encoder2_isr_a() {
  int a = digitalRead(encoder2_pin_a);
  int b = digitalRead(encoder2_pin_b);
  
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
}

void encoder2_isr_b() {
  int a = digitalRead(encoder2_pin_a);
  int b = digitalRead(encoder2_pin_b);
  
  if (b == HIGH) {
    if (a == HIGH) {
      encoder2_count++;
    } else {
      encoder2_count--;
    }
  } else {
    if (a == LOW) {
      encoder2_count++;
    } else {
      encoder2_count--;
    }
  }
}
