void setup(){
  Serial.begin(9600);
}

void loop(){
  int s0 = analogRead(A0);
  int s1 = analogRead(A1);
  int s2 = analogRead(A2);
  int s3 = analogRead(A3);
  int s4 = analogRead(A4);
  
  
  Serial.print(s0);
  Serial.print("---");
  
  Serial.print(s1);
  Serial.print("---");
  
  Serial.print(s2);
  Serial.print("---");
  
  Serial.print(s3);
  Serial.print("---");
  
  Serial.print(s4);
  Serial.print("---");
  
  Serial.println("");
  Serial.println("");
  
  delay(1000);
}
