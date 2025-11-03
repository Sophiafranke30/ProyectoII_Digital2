/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

//Sophia Franke y Dulce Ovando :))
//Proyecto Digital II | I2C, SPI y UART

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Se usó un typedef ya que era más snecillo el poder guardar los estados en lo que se encontraba para poder después cambair entre estados depende de que función se esté usando actualmente.
typedef enum {
    MENU_PRINCIPAL,
    MENU_SPI,
    MENU_I2C
} estado_menu_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 100 //buffer para comunicación uART
#define SPI_COMMAND_SIZE 50 //tamaño máximo para SPI (número grande así no se pierden datos)
#define I2C_DEVICE_ADDRESS 0x64  // Dirección del I2C
#define SPI_TIMEOUT 2000 //Timeout para transacciones de SPI en ms para que no se trabe la función

//Sophia Franke y Dulce Ovando :))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char rxData; //datos recibidos de UART
char buffer_uart[BUFFER_SIZE]; 
estado_menu_t estado_actual = MENU_PRINCIPAL;
char comando_spi[SPI_COMMAND_SIZE];
uint8_t indice_comando = 0;
uint8_t tx_buf[32];  // Buffer transmisión SPI (M --> S)
uint8_t rx_buf[32];  // Buffer recepción SPI (S --> M)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void transmit_uart(char* texto);
void menu_principal(void);
void submenu_spi(void);
void submenu_i2c(void);
void procesar_comando_spi(void);
void enviar_comando_spi(char* comando);
bool validar_comando_spi(char* comando);
void leer_sensor_i2c(void);
void procesar_entrada_spi(char caracter);
void spi_check_connection(void); 

//Sophia Franke y Dulce Ovando :))

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  // Inicialización del sistema
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // CS en alto para que no haya comunicaci[on a[un
  HAL_Delay(1000);

  transmit_uart("Inicializando...\r\n");
  HAL_Delay(1500);

  // Verificar conexión SPI para que se puedan enviar los datos bien desp[ues
  spi_check_connection();

  menu_principal(); //se muestra el men[u principal para iniciar todo.
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Recepción de caracteres por UART
    if (HAL_UART_Receive(&huart2, (uint8_t*)&rxData, 1, 100) == HAL_OK) {

        HAL_UART_Transmit(&huart2, (uint8_t*)&rxData, 1, HAL_MAX_DELAY);

//Switch case de menús 
        switch (estado_actual) {
        	case MENU_PRINCIPAL:
        		if (rxData == '1') submenu_spi();
                else if (rxData == '2') submenu_i2c();
                else if (rxData != '\r' && rxData != '\n') {
                	("\r\nOpcion no valida. Use 1 o 2.\r\n");
                    menu_principal();
                }
                break;
                
        	case MENU_SPI:
                procesar_entrada_spi(rxData);
                break;

            case MENU_I2C:
                if (rxData == '\r' || rxData == '\n') {
                    leer_sensor_i2c();
                    HAL_Delay(2000);
                    menu_principal();
                }
                break;
        }
    }
    HAL_Delay(10);
  }
  //Sophia Franke y Dulce Ovando :))

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;           // PLL habilitado
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;   // Fuente HSI
  RCC_OscInitStruct.PLL.PLLM = 16;                       // Divider M
  RCC_OscInitStruct.PLL.PLLN = 336;                      // Multiplier N
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;            // Divider P
  RCC_OscInitStruct.PLL.PLLQ = 2;                        // Divider Q
  RCC_OscInitStruct.PLL.PLLR = 2;                        // Divider R
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin : PB6 (SPI CS) */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

//Envia el texto al monitor serial mediante UART
void transmit_uart(char* texto){
    HAL_UART_Transmit(&huart2, (uint8_t*)texto, strlen(texto), HAL_MAX_DELAY);
}

//Muestra el menú prinicpal y da el estado para el switch case de MENU_PRINCIPL
void menu_principal(void){
    transmit_uart("\r\n====================\r\n");
    transmit_uart("    MENU PRINCIPAL\r\n");
    transmit_uart("====================\r\n");
    transmit_uart("Seleccione opcion (1-2): \r\n");
    transmit_uart("1. Controlar dispositivo SPI\r\n");
    transmit_uart("2. Obtener medicion de Sensor I2C\r\n");
    estado_actual = MENU_PRINCIPAL;
}

//Submenu del SPI en donde se indica que se escriba el LED,TIEMPO para que el ESP32 (SLAVE) lo pueda interpretar y da el estado para el switch case de MENU_SPI
void submenu_spi(void){
    transmit_uart("====================\r\n");
    transmit_uart("\r\n--- CONTROL SPI ---\r\n");
    transmit_uart("Formato: LED,tiempo\r\n");
    transmit_uart("Ejemplo: 1,500 (LED1 Rojo por 500ms)\r\n");
    transmit_uart("====================\r\n");
    transmit_uart("LEDs: 1(Rojo), 2(Verde), 3(Azul)\r\n");
    transmit_uart("Ingrese comando: ");
    estado_actual = MENU_SPI;
    indice_comando = 0;
    memset(comando_spi, 0, sizeof(comando_spi));
}

//Submenú del I2C en donde se solicita la lectura del Sensor I2C (potenciómetro) y da el estado para el switch case de MENU_I2C
void submenu_i2c(void){
    transmit_uart("====================\r\n");
    transmit_uart("\r\n--- LECTURA SENSOR I2C ---\r\n");
    transmit_uart("====================\r\n");
    transmit_uart("Presione ENTER para leer el potenciometro...\r\n");
    estado_actual = MENU_I2C;
}

//Construye el comando SPI carácter por carácter y lo procesa cuando se presiona ENTER.
void procesar_entrada_spi(char caracter){
    if (caracter == '\r') return;
    if (caracter == '\n') {
        if (indice_comando > 0) {
            comando_spi[indice_comando] = '\0';
            procesar_comando_spi();
        }
    }
    else if (indice_comando < sizeof(comando_spi) - 1) {
        comando_spi[indice_comando++] = caracter;
    }
}

//Valida el comando recibido, lo envía si es correcto y retorna al menú principal.
void procesar_comando_spi(void) {
    transmit_uart("\r\nComando recibido: ");
    transmit_uart(comando_spi);
    transmit_uart("\r\n");

    //Comunicación UART en el Serial para poder ir revisando todas las cosas en tiempo real.
    if(validar_comando_spi(comando_spi)) {
        transmit_uart("Comando valido. Enviando dispositivo...\r\n");
        enviar_comando_spi(comando_spi); 
    } else {
        transmit_uart("ERROR: Formato incorrecto.\r\n");
        transmit_uart("Use: LED,tiempo (Ej: 2,1000)\r\n");
    }

    HAL_Delay(1000);
    menu_principal();
}

//Transmite el comando por SPI al ESP32 y recibe la respuesta.

void enviar_comando_spi(char* comando) {
    HAL_StatusTypeDef status;

    // Preparar buffers
    memset(tx_buf, 0, sizeof(tx_buf));
    memset(rx_buf, 0, sizeof(rx_buf));
    strcpy((char*)tx_buf, comando);

    // Comunicación SPI
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // CS en LOW para  inicie la comunicación SPI
    HAL_Delay(1);

    // Transmitir y recibir con tx y rx
    status = HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, strlen((char*)tx_buf), SPI_TIMEOUT);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);    // CS alto para parar la comunicacipopn SPI
    HAL_Delay(5);

    // Procesar respuesta  y lo imprime en UART en el Serial.
    if (status == HAL_OK) {
        sprintf(buffer_uart, "Respuesta SPI: '%s'\r\n", rx_buf);
        transmit_uart(buffer_uart);
    } else {
        sprintf(buffer_uart, "Error SPI: %d\r\n", status);
        transmit_uart(buffer_uart);
    }
}

//Revisa que el formato del comando cumpla con LED,tiempo y que los valores sean válidos.
bool validar_comando_spi(char* comando) {
    char led_str[10], tiempo_str[10];
    int led, tiempo;

    int elementos = sscanf(comando, "%[^,],%s", led_str, tiempo_str);

    if(elementos != 2) {
        return false;
    }

    led = atoi(led_str);
    tiempo = atoi(tiempo_str);

    if(led < 1 || led > 3) {
        transmit_uart("ERROR: LED debe ser 1, 2 o 3\r\n");
        return false;
    }

    if(tiempo <= 0) {
        transmit_uart("ERROR: Tiempo debe ser mayor a 1");
        return false;
    }

    return true;
}

//Lee los datos del sensor I2C, intenta hasta 3 veces si hay fallos, y muestra el resultado en voltaje y valor analógico.
void leer_sensor_i2c(void) {
    uint8_t data[2] = {0};
    HAL_StatusTypeDef status;
    uint8_t intento;

    transmit_uart("Leyendo sensor I2C...\r\n");

    for(intento = 0; intento < 3; intento++) {
        status = HAL_I2C_Master_Receive(&hi2c1, I2C_DEVICE_ADDRESS << 1, data, 2, 1000);

        if (status == HAL_OK && data[0] != 0xFF && data[1] != 0xFF &&
            data[0] != 0x00 && data[1] != 0x00) {
            break; // Lectura válida
        }
        HAL_Delay(5);
    }

    if (status == HAL_OK) {
        uint16_t pot_value = (data[1] << 8) | data[0];        // Lectura deol potenciómetro mediante I2C
        uint16_t voltage_mv = (pot_value * 3300) / 4095;      // Convertit a mV dividiendo por 4095 para que no hayan errores de datos.
        uint8_t pot_8bit = (pot_value * 255) / 4095;          // Convertir a 0-255

        
        sprintf(buffer_uart, "  - Valor en Bits: %d\r\n", pot_8bit);
        transmit_uart(buffer_uart);
        sprintf(buffer_uart, "  - Voltaje: %d.%02d V\r\n", voltage_mv / 1000, voltage_mv % 1000 / 10);
        transmit_uart(buffer_uart);
    } else {
        transmit_uart("Error: No se pudo leer el sensor\r\n");
    }

    transmit_uart("Medicion completada correctamente.\r\n");
}

//Verifica que exista comunicación física con el ESP32 antes de iniciar el programa para que todo lo que vaya después si funciones bien.
void spi_check_connection(void) {
    memset(tx_buf, 0, sizeof(tx_buf));
    memset(rx_buf, 0, sizeof(rx_buf));
    strcpy((char*)tx_buf, "PING");

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_Delay(2);

    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 5, 500);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(5);
}

//Sophia Franke y Dulce Ovando :))
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
