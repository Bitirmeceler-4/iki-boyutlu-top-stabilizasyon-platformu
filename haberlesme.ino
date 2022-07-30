// TOBB ETU Elektrik-Elektronik Muhendisligi
// 2021-2022 Yaz Donemi
// ELE 495 Bitirme Projesi - Iki Boyutlu Top Stabilizasyon Platformu
// Grup 4
// Haberlesme Tarafi Bluetooth ve LCD Ekran Kodu
// Arduino Uno

// Bluetooth'tan okuma yapmayi saglayan kutuphane
#include <NeoSWSerial.h>
// 2x16 LCD ekran kutuphanesi
#include <LiquidCrystal_I2C.h>

// tek seferde gelen x ve y konumlarini iceren metin icin
String xypos = "";

// metnin ayristirilmasi icin
String xstr = "";
String ystr = "";

// gelen her karakteri okumak icin
char everyChar;

// HC-05 RX-TX pinleri
const byte rxPin = 8;
const byte txPin = 9;
NeoSWSerial bluSerial (rxPin, txPin);

LiquidCrystal_I2C lcd(0x27,16,2);

void setup() {
// bluetooth haberlesmesini 9600 baudrate'te baslat
bluSerial.begin(9600);
// LCD'yi baslat
lcd.begin();
}

void loop() {
    // Yeni veri geldiginde LCD'yi temizle
    if(bluSerial.available()>0)
        lcd.clear();
    
    // Yeni veri gelirken donguye gir ve karakter karakter oku
    while(bluSerial.available() > 0) {
        everyChar = bluSerial.read();
        xypos.concat(everyChar);
    }

    // Gelen metni 'x' karakterine gore farkli metinlere x ve y konumlari olarak ayristir
    if(xypos != ""){
      xstr = xypos.substring(0, xypos.indexOf('x'));
      ystr = xypos.substring(xypos.indexOf('x')+1, xypos.length());
    }

    xypos = "";

    // LCD'ye x ve y konumlarini bas
    lcd.setCursor(0,0);
    lcd.print("x: " + xstr);
    lcd.setCursor(0,1);
    lcd.print("y: " + ystr);
}

