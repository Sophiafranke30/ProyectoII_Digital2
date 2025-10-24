#include <LiquidCrystal.h>

// === Definición de pines LCD ===
#define RS 13
#define E  12
#define D4 33
#define D5 32
#define D6 35
#define D7 15
#define LED_R 25
#define LED_G 26
#define LED_B 27
#define POT_PIN 34

LiquidCrystal lcd(RS, E, D4, D5, D6, D7);

void setup() {
  Serial.begin(115200);
  
  // Configurar LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("LCD Inicializando...");
  delay(1500);
  lcd.clear();

  // Configurar LEDs
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  // Apagar LEDs al inicio
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);

}

void loop() {
  int adcValue = analogRead(POT_PIN);   // Leer el potenciómetro
  float voltage = (adcValue * 3.3) / 4095;

  // Mostrar valores en Serial
  Serial.print("ADC: ");
  Serial.print(adcValue);
  Serial.print("\tVoltaje: ");
  Serial.print(voltage, 2);
  Serial.println(" V");

  // Mostrar en LCD
  lcd.setCursor(0, 0);
  lcd.print("Bits: ");
  lcd.print(adcValue);
  lcd.print("    ");

  lcd.setCursor(0, 1);
  lcd.print("Volt: ");
  lcd.print(voltage, 2);
  lcd.print("V   ");

  // Cambia el color del LED según el valor
  if (voltage < 1.1) {
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
  } else if (voltage < 2.2) {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, LOW);
  } else {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, HIGH);
  }

  delay(500);
}
