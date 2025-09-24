/*
 * Fixed Arduino-RoboClaw Encoder Bridge
 * SOLUTION: Arduino Uno only has interrupts on pins 2,3
 * SWAPPED: Encoder 1 on pins 4,5 (manual polling) ✓
 * SWAPPED: Encoder 2 on pins 2,3 (interrupt-based) ✓
 */

// Encoder 1 variables (manual polling) - SWAPPED TO 4,5
volatile long encoder1_count = 0;
int encoder1_pin_a = 4;  // Manual polling
int encoder1_pin_b = 5;  // Manual polling

// Encoder 2 variables (interrupt-based) - SWAPPED TO 2,3
volatile long encoder2_count = 0;
volatile long interrupt_count_a = 0;  // Debug counter
volatile long interrupt_count_b = 0;  // Debug counter
volatile long last_valid_enc2_count = 0;  // For noise filtering
int encoder2_pin_a = 2;  // Interrupt 0
int encoder2_pin_b = 3;  // Interrupt 1

// Debouncing variables for Encoder 2
volatile unsigned long last_interrupt_time_a = 0;
volatile unsigned long last_interrupt_time_b = 0;
const unsigned long debounce_delay = 0;  // DISABLED - Allow all interrupts for testing

// Noise filtering variables
long last_sent_enc1 = 0;
long last_sent_enc2 = 0;
const long max_change_per_cycle = 1000;  // DISABLED - Allow large changes for testing

// Manual polling state tracking for encoder 1
int last_enc1_a = HIGH;
int last_enc1_b = HIGH;

void setup() {
  Serial.begin(38400);
  
  // Setup encoder 1 pins (manual polling) - SWAPPED TO 4,5
  pinMode(encoder1_pin_a, INPUT_PULLUP);
  pinMode(encoder1_pin_b, INPUT_PULLUP);
  
  // Setup encoder 2 pins (interrupt-based) - SWAPPED TO 2,3
  pinMode(encoder2_pin_a, INPUT_PULLUP);
  pinMode(encoder2_pin_b, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder2_pin_a), encoder2_isr_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2_pin_b), encoder2_isr_b, CHANGE);
  
  // Initialize encoder 1 manual polling state
  last_enc1_a = digitalRead(encoder1_pin_a);
  last_enc1_b = digitalRead(encoder1_pin_b);
  
  delay(100);
}

void loop() {
  // Check for reset command from serial
  if(Serial.available() > 0) {
    String command = Serial.readString();
    command.trim();
    if(command == "RESET") {
      encoder1_count = 0;
      encoder2_count = 0;
      last_valid_enc2_count = 0;
      last_sent_enc1 = 0;
      last_sent_enc2 = 0;
      Serial.println("ENCODERS_RESET");
      return;
    }
  }
  
  // Manual polling for encoder 1 (SWAPPED!)
  int enc1_a_now = digitalRead(encoder1_pin_a);
  int enc1_b_now = digitalRead(encoder1_pin_b);
  
  // Detect changes on encoder 1 pin A (edge detection)
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
  
  // Update state for next iteration
  last_enc1_a = enc1_a_now;
  last_enc1_b = enc1_b_now;
  
  // Apply noise filtering to encoder readings
  long current_enc1 = encoder1_count;
  long current_enc2 = encoder2_count;
  
  // Check if changes are reasonable (noise filtering)
  if (abs(current_enc1 - last_sent_enc1) > max_change_per_cycle) {
    current_enc1 = last_sent_enc1;  // Reject noisy reading
  }
  if (abs(current_enc2 - last_sent_enc2) > max_change_per_cycle) {
    current_enc2 = last_sent_enc2;  // Reject noisy reading
  }
  
  // Update last sent values
  last_sent_enc1 = current_enc1;
  last_sent_enc2 = current_enc2;
  
  // Send encoder data at 20Hz with pin state debugging
  Serial.print("ENC1:");
  Serial.print(current_enc1);
  Serial.print(",ENC2:");
  Serial.print(current_enc2);
  Serial.print(",PIN4:");
  Serial.print(digitalRead(encoder1_pin_a));
  Serial.print(",PIN5:");
  Serial.print(digitalRead(encoder1_pin_b));
  Serial.print(",PIN2:");
  Serial.print(digitalRead(encoder2_pin_a));
  Serial.print(",PIN3:");
  Serial.print(digitalRead(encoder2_pin_b));
  Serial.print(",INTA:");
  Serial.print(interrupt_count_a);
  Serial.print(",INTB:");
  Serial.println(interrupt_count_b);
  
  delay(50); // 20Hz update rate
}

// Encoder 2 interrupt service routines (both pins A and B) - SWAPPED TO 2,3
void encoder2_isr_a() {
  unsigned long current_time = millis();
  if (current_time - last_interrupt_time_a < debounce_delay) {
    return;  // Ignore noise - too soon after last interrupt
  }
  last_interrupt_time_a = current_time;
  
  interrupt_count_a++;  // Debug counter
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
  unsigned long current_time = millis();
  if (current_time - last_interrupt_time_b < debounce_delay) {
    return;  // Ignore noise - too soon after last interrupt
  }
  last_interrupt_time_b = current_time;
  
  interrupt_count_b++;  // Debug counter
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
