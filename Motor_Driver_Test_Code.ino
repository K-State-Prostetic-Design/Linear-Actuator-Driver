const int N1 = 2;  // IN1 MotorDriver1
const int N2 = 3;  // IN2 MotorDriver1
const int N3 = 4;  // IN3 MotorDriver1
const int N4 = 5;  // IN4 MotorDriver1

const int N5 = 6;  // IN1 MotorDriver2
const int N6 = 7;  // IN2 MotorDriver2
const int N7 = 8;  // IN3 MotorDriver2
const int N8 = 9;  // IN4 MotorDriver2

void setup() {
  pinMode(N1, OUTPUT);
  pinMode(N2, OUTPUT);
  pinMode(N3, OUTPUT);
  pinMode(N4, OUTPUT);
  pinMode(N5, OUTPUT);
  pinMode(N6, OUTPUT);
  pinMode(N7, OUTPUT);
  pinMode(N8, OUTPUT);

  Serial.begin(9600);
  Serial.println("Type 1, 2, or 0:");
}

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();

    if (input == '1') {
      // Direction 1
      digitalWrite(N1, HIGH);
      digitalWrite(N2, LOW);
      digitalWrite(N3, HIGH);
      digitalWrite(N4, LOW);
      digitalWrite(N5, HIGH);
      digitalWrite(N6, LOW);
      digitalWrite(N7, HIGH);
      digitalWrite(N8, LOW);
      Serial.println("Direction 1");
    }
    else if (input == '2') {
      // Direction 2
      digitalWrite(N1, LOW);
      digitalWrite(N2, HIGH);
      digitalWrite(N3, LOW);
      digitalWrite(N4, HIGH);
      digitalWrite(N5, LOW);
      digitalWrite(N6, HIGH);
      digitalWrite(N7, LOW);
      digitalWrite(N8, HIGH);
      Serial.println("Direction 2");
    }
    else if (input == '0') {
      // Stop (brake)
      digitalWrite(N1, LOW);
      digitalWrite(N2, LOW);
      digitalWrite(N3, LOW);
      digitalWrite(N4, LOW);
      digitalWrite(N5, LOW);
      digitalWrite(N6, LOW);
      digitalWrite(N7, LOW);
      digitalWrite(N8, LOW);
      Serial.println("Stopped");
    }
  }
}