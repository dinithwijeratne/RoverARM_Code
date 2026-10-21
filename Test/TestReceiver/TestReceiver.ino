//Embedded Code for PICO to test if it's readign from the serial bridge package. 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) {
    delay(15);
  }
  pinMode(25, OUTPUT);
  digitalWrite(25, LOW); 
  delay(150); 
  digitalWrite(25, HIGH); 
  delay(500); 
  digitalWrite(25, LOW); 
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    if (data == "A,0,0,0,0,0,0") {  // What we want
      digitalWrite(25, HIGH);
    }else{ //IK will change thats how we know it works 
      digitalWrite(25, LOW);
    }
  }
}
