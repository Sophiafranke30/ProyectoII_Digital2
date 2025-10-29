#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LiquidCrystal.h>

// ==== CONFIGURACIÓN LCD ====
const int rs = 25, en = 26, d4 = 27, d5 = 14, d6 = 12, d7 = 13;
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
  }

  delay(timeMs);

  digitalWrite(ledR, LOW);
  digitalWrite(ledG, LOW);
  digitalWrite(ledB, LOW);
}

// ==== I2C configuración ====
#define I2C_ADDRESS 0x28  // Puedes cambiarlo según tu configuración

void requestEvent() {
  int potValue = analogRead(potPin);
  Wire.write((uint8_t)(potValue >> 8));
  Wire.write((uint8_t)(potValue & 0xFF));
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

  // SPI (modo maestro simulado)
  SPI.begin(); // SPI como master (ESP32 no soporta esclavo por defecto)

  // I2C esclavo
  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(requestEvent);

  delay(1000);
  lcd.clear();
}

// ==== LOOP ====
void loop() {
  // Simulación de recepción SPI por Serial (para pruebas)
  if (Serial.available()) {
    spiCommand = Serial.readStringUntil('\n');
    newCommand = true;
  }

  // Si llega comando SPI, ejecutarlo
  if (newCommand) {
    executeSPICommand(spiCommand);
    spiCommand = "";
    newCommand = false;
  }

  // Leer potenciómetro
  int potValue = analogRead(potPin);
  float voltage = (potValue / 4095.0) * 3.3;

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
  lcd.print("    ");

  lcd.setCursor(11, 1);
  lcd.print("LED:");
  lcd.setCursor(15, 1);
  lcd.print(lastLED);

  delay(300);
}
