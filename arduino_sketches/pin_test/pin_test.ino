/*
 * Pin Test - Check if encoders are connected
 * Shows raw pin states for encoder pins
 */

void setup() {
  Serial.begin(38400);
  
  // Setup encoder pins
  pinMode(2, INPUT_PULLUP);  // Encoder 2 A
  pinMode(3, INPUT_PULLUP);  // Encoder 2 B  
  pinMode(4, INPUT_PULLUP);  // Encoder 1 A
  pinMode(5, INPUT_PULLUP);  // Encoder 1 B
  
  delay(100);
  Serial.println("Pin Test Started - Checking encoder connections");
}

void loop() {
  int pin2 = digitalRead(2);
  int pin3 = digitalRead(3);
  int pin4 = digitalRead(4);
  int pin5 = digitalRead(5);
  
  Serial.print("PIN2:");
  Serial.print(pin2);
  Serial.print(",PIN3:");
  Serial.print(pin3);
  Serial.print(",PIN4:");
  Serial.print(pin4);
  Serial.print(",PIN5:");
  Serial.println(pin5);
  
  delay(100);
}
