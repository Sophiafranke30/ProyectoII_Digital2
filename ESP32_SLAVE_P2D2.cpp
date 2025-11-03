//Sophia Franke (23030)
//Dulce Ovandi (23441)
// Proyecto 2 - Comunicación SPI e I2C entre ESP32 y STM32
//------------------------------------------------------------

#include <Arduino.h>
#include <ESP32SPISlave.h>
#include <Wire.h>
#include <LiquidCrystal.h>

// =============================
// CONFIGURACIÓN GENERAL
// =============================

LiquidCrystal lcd(33, 25, 26, 27, 14, 13);

#define LED_R 15
#define LED_G 2
#define LED_B 4
#define POT_PIN 32
#define I2C_ADDRESS 0x64
#define PIN_CS 5

// =============================
// CONFIGURACION SPI ESCLAVO
// =============================
ESP32SPISlave slave;
static constexpr uint32_t BUFFER_SIZE {32}; // Tamaño del buffer SPI. Se usa un tamaño mayor para evitar pérdida de datos.
uint8_t spi_slave_tx_buf[BUFFER_SIZE]; // Buffer de transmisión SPI. Funciona como respuesta a los comandos recibidos.
uint8_t spi_slave_rx_buf[BUFFER_SIZE]; // Buffer de recepción SPI. Aquí se almacenan los comandos recibidos.

// =============================
// VARIABLES DE ESTADO
// =============================
// Variables para el potenciómetro y la gestión de LEDs.
uint16_t potValue = 0;
uint8_t potMapped = 0;
float voltage = 0.0;
char lastLED = 'N';

// Variables para la gestión de comandos SPI.
String spiCommandBuffer = "";
bool newSPIDataAvailable = false;
unsigned long lastSPIProcessTime = 0;

// =============================
// PROTOTIPOS
// =============================
void processSPICommand(String comando); // Procesa un comando SPI recibido
void updateLCD();                       // Actualiza la pantalla LCD con la información actual
void onI2CRequest();                    // Maneja las solicitudes I2C del STM a través del UART
void readAndProcessSPI();               // Lee y procesa datos SPI para evitar pérdida de datos
void executeLEDCommand(int led, int tiempo); // Ejecuta el comando para encender un LED


void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Inicializando sistema...");

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);

  // Configuración I2C como esclavo
  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(onI2CRequest);   // Se registra la función callback para solicitudes I2C
  Serial.println("I2C esclavo configurado correctamente");

  // Configuración SPI como esclavo
  slave.setDataMode(SPI_MODE0);
  slave.setQueueSize(2); // Buffer más grande para mejor recepción de datos
  slave.begin(VSPI, 18, 19, 23, PIN_CS); // Pines SCK, MISO, MOSI, SS en la configuración VSPI
  Serial.println("SPI esclavo configurado correctamente");

  // Configuración LCD
  lcd.begin(16, 2);
  lcd.print("Inicializando...");
  delay(1000);
  lcd.clear();

  // Buffer inicial SPI. Se establece aquí el mensaje de "READY" para indicar que el sistema está listo.
  memset(spi_slave_tx_buf, 0, BUFFER_SIZE);
  memset(spi_slave_rx_buf, 0, BUFFER_SIZE);
  strcpy((char*)spi_slave_tx_buf, "READY"); //El mensaje READY indica que el sistema está listo para recibir comandos SPI. Este se envía automáticamente al maestro SPI cuando se establece la conexión.
//strcpy se usa para copiar la cadena "READY" al buffer de transmisión SPI. y la función viene de la biblioteca estándar de C. El *char se usa para convertir el buffer de bytes a una cadena de caracteres.

  slave.queue(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE); //slave.queue(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE); se usa para preparar los buffers de transmisión y recepción SPI. Aquí se configura el buffer de transmisión con el mensaje "READY" y el buffer de recepción para almacenar los comandos entrantes.
  slave.trigger(); // Inicia la operación SPI esclavo. Esto permite que el dispositivo comience a escuchar y responder a las solicitudes del maestro SPI.

  Serial.println("Sistema completamente inicializado");
  Serial.println("Esperando comandos SPI...");
  Serial.println("========================================");
}


void loop() {
  // Leer y actualizar valores del potenciómetro constantemente. Se mapea el valor para uso general y se calcula el voltaje.
  potValue = analogRead(POT_PIN);
  potMapped = map(potValue, 0, 4095, 0, 255);
  voltage = (potValue * 3.3) / 4095.0; //Aquí se divide por 4095.0 para asegurar que la división se realice en punto flotante, evitando ya que si se divide por 255.0 se perdería precisión en el cálculo del voltaje real (El LCD me parpadeaba demasiado por la medida inexacta)

  readAndProcessSPI(); // Leer y procesar datos SPI nomas lleguen para evitar pérdida de datos

  updateLCD();

  delay(10); 
}

// =============================
// FUNCIONES ADICIONALES
// =============================

void readAndProcessSPI() {
  // Verificar si hay transacciones SPI completadas. Si las hay, se procesan inmediatamente para evitar pérdida de datos.
  if (slave.hasTransactionsCompletedAndAllResultsReady(1)) { //slave.hasTransactionsCompletedAndAllResultsReady(1) verifica si hay al menos una transacción SPI completada y lista para ser procesada. El parámetro '1' indica que se está verificando una transacción.
    
    size_t bytesRecibidos = slave.numBytesReceived(); // Obtiene el número de bytes recibidos en la última transacción SPI.
    
    if (bytesRecibidos > 0 && bytesRecibidos < BUFFER_SIZE) { // Validar que se hayan recibido datos y que no excedan el tamaño del buffer (por esto el buffer es más grande).
      spi_slave_rx_buf[bytesRecibidos] = '\0';  // Asegurar que el buffer de recepción esté correctamente terminado como cadena C.
      
      String comandoRecibido = String((char*)spi_slave_rx_buf); // Convertir el buffer de recepción a un String de Arduino para facilitar su manejo e impresión.
      comandoRecibido.trim(); // Limpiar espacios en blanco al inicio y final del comando recibido.
     
      //Cuando se recibe un comando SPI, se imprime en el monitor serie para control y luego se procesa inmediatamente. 
      Serial.println("----------------------------------------");
      Serial.println("COMANDO SPI RECIBIDO:");
      Serial.print("  Bytes: ");
      Serial.println(bytesRecibidos);
      Serial.print("  Raw: '");
      Serial.print(comandoRecibido);
      Serial.println("'");
      
      // Procesar el comando inmediatamente para evitar pérdida de datos en caso de múltiples comandos rápidos.
      processSPICommand(comandoRecibido);
    }

    // Re-armar buffers para siguiente transacción
    memset(spi_slave_rx_buf, 0, BUFFER_SIZE); // Limpiar buffer de recepción
    strcpy((char*)spi_slave_tx_buf, "READY"); // Preparar respuesta
    
    slave.queue(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE); //slave.queue(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE); se usa para preparar los buffers de transmisión y recepción SPI nuevamente después de procesar un comando. Aquí se vuelve a configurar el buffer de transmisión con el mensaje "READY" y se limpia el buffer de recepción para la próxima transacción.
    slave.trigger();
  }
}


void processSPICommand(String comando) {
  // Limpia y valida el comando recibido asegurándose de que no esté vacío y que siga el formato esperado de LED,TIME.
  comando.trim(); //comando.trim(); elimina espacios en blanco al inicio y final del comando recibido para asegurar que el procesamiento sea correcto.
  
  if (comando.length() == 0) { // Validar que el comando no esté vacío y que tenga contenido.
    Serial.println("ERROR: Comando vacío");
    strcpy((char*)spi_slave_tx_buf, "ERR:EMPTY"); // Respuesta de error por comando vacío que se envía al maestro SPI.
    return; // Salir si el comando está vacío
  }

  Serial.print("  Procesando: '"); //Imprime el comando que se está procesando actualmente en el monitor serie para control.
  Serial.print(comando);
  Serial.println("'");

  // Buscar la coma que separa LED y tiempo. Esta se busca para dividir el comando en dos partes y la función substring se usa para extraerlas.
  int commaIndex = comando.indexOf(','); //comando.indexOf(',') busca la posición de la primera coma en el comando recibido. Esta posición se usa para dividir el comando en las partes de LED y tiempo.
  
  if (commaIndex == -1) { // Validar que exista la coma separadora en el comando y si no está, se envía un error.
    Serial.println("ERROR: No se encontró coma separadora");
    strcpy((char*)spi_slave_tx_buf, "ERR:FORMAT");
    return;
  }

  // Extraer partes del comando que se obtuvieron usando la posición de la coma. La primera parte es el LED y la segunda es el tiempo.
  String ledStr = comando.substring(0, commaIndex); //comoando.substring(0, commaIndex) extrae la parte del comando que corresponde al LED, desde el inicio hasta la posición de la coma.
  String tiempoStr = comando.substring(commaIndex + 1); //comando.substring(commaIndex + 1) extrae la parte del comando que corresponde al tiempo, desde justo después de la coma hasta el final del comando.
  

  // Limpiar espacios en blanco de ambas partes
  ledStr.trim(); 
  tiempoStr.trim();

  // Validar que no estén vacías las nuevas cadenas obtenidas
  if (ledStr.length() == 0 || tiempoStr.length() == 0) {
    Serial.println("ERROR: LED o tiempo vacíos");
    strcpy((char*)spi_slave_tx_buf, "ERR:PARAMS");
    return;
  }

  // Convertir a números ambas partes para poder trabajar con ellas en las funciones siguientes.
  int led = ledStr.toInt();
  int tiempo = tiempoStr.toInt();

  // Imprimir valores convertidos para control y asegurar que la conversión fue correcta y que los datos si estén llegando bien.
  Serial.print("  LED: ");
  Serial.print(led);
  Serial.print(", Tiempo: ");
  Serial.print(tiempo);
  Serial.println(" ms");

  // Validar rangos para LED (1-3) y tiempo (1-30000 ms. Se puso un límite para evitar bloqueos largos ya que me daba error cuando se usaban tiempos muy largos).
  if (led < 1 || led > 3) {
    Serial.println("ERROR: LED debe ser 1, 2 o 3");
    strcpy((char*)spi_slave_tx_buf, "ERR:LED");
    return;
  }

  if (tiempo <= 0 || tiempo > 30000) { // Máximo 30 segundos como se mencionó antes.
    Serial.println("ERROR: Tiempo fuera de rango (1-30000 ms)");
    strcpy((char*)spi_slave_tx_buf, "ERR:TIME");
    return;
  }

  // Ejecutar comando
  executeLEDCommand(led, tiempo);
}

// Enciende el LED especificado por un tiempo determinado, actualizando el LCD y el potenciómetro durante la espera. 
// El encendido de los LEDS funciona ya que se llama dentro de loop() que actualiza constantemente. Este toma el valor mandado por SPI y lo ejectuta tomando en cuenta el tiempo y actualizando el LCD.
void executeLEDCommand(int led, int tiempo) {
  int pinLed = 0;
  char ledChar = 'N';
  
  // Determinar pin y número de LED basado en el parámetro recibido en la función anterior.
  switch(led) {
    case 1: 
      pinLed = LED_R; // Asignar pin del LED rojo y encenderlo
      ledChar = 'R';  // Asignar carácter 'R' para indicar LED rojo y se usa en el LCD
      Serial.println("Encendiendo LED ROJO");
      break;
    case 2:  //lo mismo para verde
      pinLed = LED_G; 
      ledChar = 'G';
      Serial.println("Encendiendo LED VERDE");
      break;
    case 3:  //y para azul
      pinLed = LED_B; 
      ledChar = 'B';
      Serial.println("Encendiendo LED AZUL");
      break;
  }

  // Encender LED que se seleccionó e imprimir en Serial el tiempo que estará encendido
  digitalWrite(pinLed, HIGH);
  Serial.print("LED encendido por ");
  Serial.print(tiempo);
  Serial.println(" ms");
  
  // Actualizar LCD inmediatamente
  lastLED = ledChar;
  updateLCD();
  
  // Esperar el tiempo especificado (sin bloquear completamente)
  unsigned long startTime = millis();
  while (millis() - startTime < tiempo) {
    // Actualizar potenciómetro y LCD durante la espera para que la pantalla no titiritee y se siga viendo el valor del potenciómetro
    potValue = analogRead(POT_PIN);
    potMapped = map(potValue, 0, 4095, 0, 255);
    voltage = (potValue * 3.3) / 4095.0; //misma explicación que arriba, si no se perdía exactitud en el voltaje
    updateLCD();
    delay(50);
  }
  
  // Apagar LED
  digitalWrite(pinLed, LOW);
  Serial.println("LED apagado"); //Se indica en Serial que el LED se apagó (se puso principalmente para debug por si hubieran problemas de software)
  
  // Respuesta exitosa
  strcpy((char*)spi_slave_tx_buf, "OK"); //strcpy((char*)spi_slave_tx_buf, "OK"); prepara la respuesta "OK" para indicar al maestro SPI que el comando se ejecutó correctamente.
  lastLED = 'N'; // Resetear a ninguno después de apagar
  
  Serial.println("COMANDO EJECUTADO EXITOSAMENTE");
  Serial.println("----------------------------------------");
}


// Esta función se llama cuando el maestro I2C solicita datos. Los datos enviados son el valor del potenciómetro en formato de 2 bytes (12-bit) y se muestra información detallada en el monitor serie.
void onI2CRequest() {
  // Usar los valores actuales del potenciómetro (ya calculados en loop). Los imprime en Serial ya que la conexión tarda a veces en actualizarse en el STM32.
  Serial.println("----------------------------------------");
  Serial.println("SOLICITUD I2C RECIBIDA:");
  Serial.print("  Valor en Bits: ");
  Serial.println(potMapped);
  Serial.print("  Voltaje: ");
  Serial.print(voltage, 2);
  Serial.println(" V");
  
  // Preparar datos para envío. Se realiza de esta forma para mantener consistencia y asegurar que se envían los valores correctos.
  uint8_t data[2];
  data[0] = potValue & 0xFF;         // Byte bajo. Se usa 0xFF para asegurar que solo se toma el byte menos significativo
  data[1] = (potValue >> 8) & 0xFF;  // Byte alto. Se usa potvalue desplazado 8 bits a la derecha y 0xFF para el byte menos significativo y así obtener el byte alto correctamente
  
  Serial.print("  Enviando por I2C: ["); // Imprime los bytes que se van a enviar por I2C para control Serial ya que el STM guarda los datos anteriores
  Serial.print(data[0]);
  Serial.print(", ");
  Serial.print(data[1]);
  Serial.println("]");
  
  Wire.write(data, 2);
}

// Actualiza la pantalla LCD con el voltaje y el valor del potenciómetro. También muestra el último LED encendido mientras este esté activo.
void updateLCD() {
  lcd.setCursor(0, 0);
  lcd.print("Volt:");
  lcd.print(voltage, 2);
  lcd.print("V   ");

  lcd.setCursor(0, 1);
  lcd.print("Bits:");
  lcd.print(potMapped);
  lcd.print("   ");

  lcd.setCursor(11, 1);
  lcd.print("LED:");
  lcd.print(lastLED);
}
