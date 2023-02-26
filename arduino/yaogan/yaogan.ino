int value = 0; 
 
void setup() { 
pinMode(2, INPUT_PULLUP); //上拉电阻保持电平稳定
pinMode(A0,INPUT);
pinMode(A1,INPUT);
Serial.begin(9600); 
} 
void loop() { 
value = analogRead(A0); 
Serial.print("X:"); 
Serial.print(value); 
value = analogRead(A1); 
Serial.print(" | Y:"); 
Serial.print(value); 
value = digitalRead(2); 
Serial.print(" | Z: "); 
Serial.println(value); 
delay(100); 
}