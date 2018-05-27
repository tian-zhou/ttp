#define fadePin 3

int FSR_Pin_Up = A2; 
int FSR_Pin_Left = A1; 
int FSR_Pin_Down = A0; 
int FSR_Pin_Right = A3; 

void setup(){
  Serial.begin(9600);
  pinMode(fadePin, OUTPUT);  
}

void loop(){
  // for controlling the magnet
  int Out = 0; // 0 - 255
  analogWrite(fadePin, Out);
  
  // for reading force sensors
  int FSRReading_Up = analogRead(FSR_Pin_Up); 
  int FSRReading_Left = analogRead(FSR_Pin_Left); 
  int FSRReading_Down = analogRead(FSR_Pin_Down); 
  int FSRReading_Right = analogRead(FSR_Pin_Right); 
  
  Serial.print("Up: "); Serial.print(FSRReading_Up);
  Serial.print(" Left: "); Serial.print(FSRReading_Left);
  Serial.print(" Down: "); Serial.print(FSRReading_Down);
  Serial.print(" Right: "); Serial.println(FSRReading_Right);
  delay(250); //just here to slow down the output for easier reading
}
