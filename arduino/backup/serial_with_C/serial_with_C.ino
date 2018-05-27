// Pin for controlling magnet
#define fadePin 3

// Pin to read force sensor
int FSR_Pin_Up = A2; //analog pin 0 for up
int FSR_Pin_Left = A1; //analog pin 0 for left
int FSR_Pin_Down = A0; //analog pin 0 for down
int FSR_Pin_Right = A3; //analog pin 0 for right

// Vars to comm. with PC through Serial
String incomingString;

// Vars to store sensor reading
int FSRReading_Up;
int FSRReading_Left; 
int FSRReading_Down; 
int FSRReading_Right; 
char SensorReadingString[60];

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete


void setup(){
  Serial.begin(9600);
  pinMode(fadePin, OUTPUT);  
  //establishContact();  // send a byte to establish contact until receiver responds
  inputString.reserve(200);
}

void loop()
{
  if (stringComplete) 
  {   
    if (inputString[0] == 'a')
    {  
      char *cstr = new char[inputString.length() + 1];
      strcpy(cstr, inputString.c_str());
      //Serial.print( cstr);
      char * pch;
      pch = strtok (cstr, " ");
      pch = strtok (NULL, " ");
      /*for controlling the magnet*/
      //Serial.print( "control value: ");
      //Serial.println(atoi(pch)); // 0-255 to control 0-5v
      analogWrite(fadePin, atoi(pch)); // 0-255 to control 0-5v  
    }
    
    else if (inputString[0] == 'b')
    {
      /* 
      for reading force sensors
      It will map input voltages between 0 and 5 volts 
      into integer values between 0 and 1023. 
      This yields a resolution between readings of: 
      5 volts / 1024 units or, .0049 volts (4.9 mV) per unit
      */
      FSRReading_Up = analogRead(FSR_Pin_Up); 
      FSRReading_Left = analogRead(FSR_Pin_Left); 
      FSRReading_Down = analogRead(FSR_Pin_Down); 
      FSRReading_Right = analogRead(FSR_Pin_Right); 
      
      sprintf(SensorReadingString, "Up: %i Left: %i Down: %i Right: %i abcd1234", FSRReading_Up, FSRReading_Left, FSRReading_Down, FSRReading_Right);
      Serial.println(SensorReadingString);
      // delay(250); //just here to slow down the output for easier reading
   } 
   
   // re-init the buffer
   inputString = "";
   stringComplete = false;  
   //Serial.flush();
   //delay(100);
  }
}

void serialEvent() {
  while (Serial.available() > 0) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is "*", set a flag
    // so the main loop can do something about it:
    if (inChar == '*') {
       stringComplete = true;
       break;
       //Serial.println ("stringComplete = true");
       //Serial.println(inputString);
    }
  }
}




//////////////////////////////////////////////////////////
/////// Previous version, no ending with "*" /////////////
//////////////////////////////////////////////////////////

//// Pin for controlling magnet
//#define fadePin 9
//
//// Pin to read force sensor
//int FSR_Pin_Up = A0; //analog pin 0 for up
//int FSR_Pin_Left = A1; //analog pin 0 for left
//int FSR_Pin_Down = A2; //analog pin 0 for down
//int FSR_Pin_Right = A3; //analog pin 0 for right
//
//// Vars to comm. with PC through Serial
//String incomingString;
//
//// Vars to store sensor reading
//int FSRReading_Up;
//int FSRReading_Left; 
//int FSRReading_Down; 
//int FSRReading_Right; 
//char SensorReadingString[60];
//String inputString = "";         // a string to hold incoming data
//boolean stringComplete = false;  // whether the string is complete
//
//
//void setup(){
//  Serial.begin(9600);
//  pinMode(fadePin, OUTPUT);  
//  establishContact();  // send a byte to establish contact until receiver responds
//  inputString.reserve(200);
//}
//
//void loop(){
//  if (stringComplete) 
//  { 
//    // init the buffer
//    inputString = "";
//    stringComplete = false;
//    
//    /*read the incoming string command*/
//    //incomingString = Serial.readString();
//    //Serial.print("The available incoming string is: ");
//    //Serial.println(incomingString);
//    
//    
//    
//    if (incomingString[0] == 'a')
//    { 
//      char *cstr = new char[incomingString.length() + 1];
//      strcpy(cstr, incomingString.c_str());
//      char * pch;
//      pch = strtok (cstr, " ");
//      pch = strtok (NULL, " ");
//      /*for controlling the magnet*/
//      Serial.print( "control value: ");
//      Serial.println(atoi(pch)); // 0-255 to control 0-5v
//      analogWrite(fadePin, atoi(pch)); // 0-255 to control 0-5v  
//    }
//    
//    else if (incomingString == "b")
//    {
//      /* 
//      for reading force sensors
//      It will map input voltages between 0 and 5 volts 
//      into integer values between 0 and 1023. 
//      This yields a resolution between readings of: 
//      5 volts / 1024 units or, .0049 volts (4.9 mV) per unit
//      */
//      FSRReading_Up = analogRead(FSR_Pin_Up); 
//      FSRReading_Left = analogRead(FSR_Pin_Left); 
//      FSRReading_Down = analogRead(FSR_Pin_Down); 
//      FSRReading_Right = analogRead(FSR_Pin_Right); 
//      
//      sprintf(SensorReadingString, "Up: %i Left: %i Down: %i Right: %i abcd1234", FSRReading_Up, FSRReading_Left, FSRReading_Down, FSRReading_Right);
//      Serial.println(SensorReadingString);
//      // delay(250); //just here to slow down the output for easier reading
//     }   
//  }
//    
//  // say what you got:
//  // Serial.print and Serial.println will send back the actual ASCII code, 
//  // whereas Serial.write will send back the actual text
//  // Serial.print("I received: ");
//  // Serial.print(incomingByte, DEC);
//  // Serial.write(incomingByte);
//  
//}
//
//void serialEvent() {
//  while (Serial.available() > 0) {
//    // get the new byte:
//    char inChar = (char)Serial.read();
//    // add it to the inputString:
//    inputString += inChar;
//    // if the incoming character is a newline, set a flag
//    // so the main loop can do something about it:
//    if (inChar == '*') {
//       stringComplete = true;
//       Serial.println ("stringComplete = true");
//       Serial.println(inputString);
//    }
//  }
//}
//
//void establishContact() {
//  while (Serial.available() <= 0) {
//    Serial.println("serial connection request");   // send an initial string
//    delay(300);
//  }
//}

