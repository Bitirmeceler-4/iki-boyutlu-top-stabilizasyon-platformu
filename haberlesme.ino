// yorumlar eklenecek
#include <NeoSWSerial.h>
#include <LiquidCrystal_I2C.h>

String xypos = "";

String xstr = "";
String ystr = "";

char everyChar;

const byte rxPin = 8;
const byte txPin = 9;
NeoSWSerial mySerial (rxPin, txPin);
LiquidCrystal_I2C lcd(0x27,16,2);
void setup() {
Serial.begin(9600);
mySerial.begin(9600);
lcd.begin();
}

void loop() {
    if(mySerial.available()>0)
    lcd.clear();

    while(mySerial.available() > 0) {
        everyChar = mySerial.read();
        xypos.concat(everyChar);
    }

    if(xypos != ""){

      xstr = xypos.substring(0, xypos.indexOf('x'));
      ystr = xypos.substring(xypos.indexOf('x')+1, xypos.length());
    }

    xypos = "";

    lcd.setCursor(0,0);
    lcd.print("x: " + xstr);
    lcd.setCursor(0,1);
    lcd.print("y: " + ystr);
}

