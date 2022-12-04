#include "Mouse.h"
#include "HID.h"

String inputString = "";      // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

double dX,dY,dZ;

void setup() {
  //Giao tiếp với Arduino IO MCU
  Serial1.begin(115200);
  Mouse.begin();
  // Bật serial monitor (giao tiếp với máy tính) ở mức baudrate 1152000 để bạn debug thôi mà :3
  Serial.begin(115200);
 
}

void serialEvent1(){
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  }
 
void loop() {
  // đọc rồi xuất ra
//  if (Serial1.available()) {
//    char c = Serial1.read();
//    Serial.print("USB: ");
//    Serial.print(c);
//    Serial.println();
//    Keyboard.print(c);
//  }
  if (stringComplete) {
//    Serial.print(inputString);
    dX=inputString.substring(0,inputString.indexOf(",")).toFloat();
    dY=inputString.substring(inputString.indexOf(",")+1,inputString.lastIndexOf(",")).toFloat();
    dZ=inputString.substring(inputString.lastIndexOf(",")+1,inputString.length()-1).toFloat();
    Mouse.move(dZ*20,-dY*20);
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
}
