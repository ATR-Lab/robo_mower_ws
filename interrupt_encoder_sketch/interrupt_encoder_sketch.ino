/*
 * Arduino Encoder Bridge - INTERRUPT VERSION
 * Both encoders using interrupt-based detection
 * Encoder 1: Pins 4,5 (Pin Change Interrupts)
 * Encoder 2: Pins 2,3 (Hardware Interrupts)
 */

// Encoder 1 variables (Pin Change Interrupts on pins 4,5)
volatile long encoder1_count = 0;
int encoder1_pin_a = 4;  // PCINT20 (Pin Change Interrupt)
int encoder1_pin_b = 5;  // PCINT21 (Pin Change Interrupt)

// Encoder 2 variables (Hardware Interrupts on pins 2,3)
volatile long encoder2_count = 0;
int encoder2_pin_a = 2;  // INT0 (Hardware Interrupt)
int encoder2_pin_b = 3;  // INT1 (Hardware Interrupt)

// State tracking for Pin Change Interrupts (Encoder 1)
volatile int last_enc1_a = HIGH;
volatile int last_enc1_b = HIGH;

void setup() {
  Serial.begin(38400);
  
  // Setup encoder 1 pins (Pin Change Interrupts)
  pinMode(encoder1_pin_a, INPUT_PULLUP);
  pinMode(encoder1_pin_b, INPUT_PULLUP);
  
  // Setup encoder 2 pins (Hardware Interrupts)
  pinMode(encoder2_pin_a, INPUT_PULLUP);
  pinMode(encoder2_pin_b, INPUT_PULLUP);
  
  // Initialize encoder 1 state
  last_enc1_a = digitalRead(encoder1_pin_a);
  last_enc1_b = digitalRead(encoder1_pin_b);
  
  // Enable Pin Change Interrupts for encoder 1 (pins 4,5)
  // PCICR: Pin Change Interrupt Control Register
  // PCIE2: Pin Change Interrupt Enable 2 (for pins D0-D7)
  PCICR |= (1 << PCIE2);
  
  // PCMSK2: Pin Change Mask Register 2
  // Enable interrupts on pins 4 and 5
  PCMSK2 |= (1 << PCINT20) | (1 << PCINT21);  // Pins 4,5
  
  // Enable Hardware Interrupts for encoder 2 (pins 2,3)
  attachInterrupt(digitalPinToInterrupt(encoder2_pin_a), encoder2_isr_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2_pin_b), encoder2_isr_b, CHANGE);
  
  delay(100);
}

void loop() {
  // Check for reset command
  if(Serial.available() > 0) {
    String command = Serial.readString();
    command.trim();
    if(command == "RESET") {
      noInterrupts();  // Disable interrupts during reset
      encoder1_count = 0;
      encoder2_count = 0;
      interrupts();    // Re-enable interrupts
      Serial.println("ENCODERS_RESET");
      return;
    }
  }
  
  // Send encoder data in clean format
  // Disable interrupts briefly while reading volatile variables
  noInterrupts();
  long enc1 = encoder1_count;
  long enc2 = encoder2_count;
  interrupts();
  
  Serial.print("ENC1:");
  Serial.print(enc1);
  Serial.print(",ENC2:");
  Serial.println(enc2);
  
  delay(50); // 20Hz update rate
}

// Pin Change Interrupt Service Routine for Encoder 1 (pins 4,5)
ISR(PCINT2_vect) {
  int enc1_a_now = digitalRead(encoder1_pin_a);
  int enc1_b_now = digitalRead(encoder1_pin_b);
  
  // Check if pin 4 (encoder1_pin_a) changed
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
    last_enc1_a = enc1_a_now;
  }
  
  // Check if pin 5 (encoder1_pin_b) changed
  if(enc1_b_now != last_enc1_b) {
    if(enc1_b_now == HIGH) {
      if(enc1_a_now == HIGH) {
        encoder1_count++;
      } else {
        encoder1_count--;
      }
    } else {
      if(enc1_a_now == LOW) {
        encoder1_count++;
      } else {
        encoder1_count--;
      }
    }
    last_enc1_b = enc1_b_now;
  }
}

// Hardware Interrupt Service Routines for Encoder 2 (pins 2,3)
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
