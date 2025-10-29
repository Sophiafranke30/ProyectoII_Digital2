#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LiquidCrystal.h>

// ==== CONFIGURACIÓN LCD ====
const int rs = 33, en = 25, d4 = 26, d5 = 27, d6 = 14, d7 = 13;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// ==== POTENCIÓMETRO ====
const int potPin = 32;

// ==== LED RGB ====
const int ledR = 15;
const int ledG = 2;
const int ledB = 4;
char lastLED = '-';

// ==== VARIABLES ====
String spiCommand = "";
bool newCommand = false;

// ==== FUNCIONES ====
void executeSPICommand(String cmd) {
  int comma = cmd.indexOf(',');
  if (comma == -1) return;

  int ledNum = cmd.substring(0, comma).toInt();
  int timeMs = cmd.substring(comma + 1).toInt();

  // Apagar todos
  digitalWrite(ledR, LOW);
  digitalWrite(ledG, LOW);
  digitalWrite(ledB, LOW);

  switch (ledNum) {
    case 1: digitalWrite(ledR, HIGH); lastLED = 'R'; break;
    case 2: digitalWrite(ledG, HIGH); lastLED = 'G'; break;
    case 3: digitalWrite(ledB, HIGH); lastLED = 'B'; break;
    default: return; // Si no es 1,2,3 no hace nada
  }

  delay(timeMs);

  // Apagar todos después del tiempo
  digitalWrite(ledR, LOW);
  digitalWrite(ledG, LOW);
  digitalWrite(ledB, LOW);
}

// ==== I2C configuración ====
#define I2C_ADDRESS 0x28

void requestEvent() {
  int potValue = analogRead(potPin);
  Wire.write((uint8_t)potValue); // Enviando solo 0-255
}

// ==== SETUP ====
void setup() {
  Serial.begin(115200);

  // LCD
  lcd.begin(16, 2);
  lcd.print("Inicializando...");

  // LEDs
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);

  // SPI (simulación)
  SPI.begin();

  // I2C esclavo
  //Wire.begin(I2C_ADDRESS);
  //Wire.onRequest(requestEvent);

  delay(1000);
  lcd.clear();
}

// ==== LOOP ====
void loop() {
  // Recepción de comando Serial
  if (Serial.available()) {
    spiCommand = Serial.readStringUntil('\n');
    newCommand = true;
  }

  // Ejecutar comando si llegó
  if (newCommand) {
    executeSPICommand(spiCommand);
    spiCommand = "";
    newCommand = false;
  }

  // Leer potenciómetro y mapear 0-255
  int potValue = analogRead(potPin);
  potValue = map(potValue, 0, 4095, 0, 255);
  float voltage = (potValue / 255.0) * 3.3;

  // Mostrar en LCD
  lcd.setCursor(0, 0);
  lcd.print("Pot1:");
  lcd.setCursor(5, 0);
  lcd.print(voltage, 2);
  lcd.print("v   ");

  lcd.setCursor(0, 1);
  lcd.print("Val:");
  lcd.setCursor(5, 1);
  lcd.print(potValue);
  lcd.print("   ");

  lcd.setCursor(11, 1);
  lcd.print("LED:");
  lcd.setCursor(15, 1);
  lcd.print(lastLED);

  delay(300);
}
