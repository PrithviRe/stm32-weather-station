/**
 ******************************************************************************
 * @file    main.c
 * @author  PrithviRe
 * @brief   This file provides main program functions
 ******************************************************************************
 **/
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l475e_iot01.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_tsensor.h"
/* Private defines -----------------------------------------------------------*/

#define TERMINAL_USE

/* Update SSID and PASSWORD with own Access point settings */
#define SSID "YourSSID"
#define PASSWORD "YourPassword"
#define API_KEY "YourAPIKey"
#define SERVER_NAME "api.thingspeak.com"
#define SERVER_PORT 80

#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT 10000

#define CONNECTION_TRIAL_MAX 10

#if defined(TERMINAL_USE)
#define TERMOUT(...) printf(__VA_ARGS__)
#else
#define TERMOUT(...)
#endif

/* Private variables ---------------------------------------------------------*/
#if defined(TERMINAL_USE)
extern UART_HandleTypeDef hDiscoUart;
#endif /* TERMINAL_USE */
volatile uint8_t userButtonPressed = 0;

/* Private function prototypes -----------------------------------------------*/
#if defined(TERMINAL_USE)
#ifdef __GNUC__
/* With GCC, small TERMOUT (option LD Linker->Libraries->Small TERMOUT
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#endif /* TERMINAL_USE */

static void SystemClock_Config(void);

extern SPI_HandleTypeDef hspi;

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */

void Execute(void)
{
    uint8_t MAC_Addr[6] = {0};
    uint8_t IP_Addr[4] = {0};
    int32_t Socket = -1;
    uint16_t Datalen;

    /*Initialize  WIFI module */
    if (WIFI_Init() == WIFI_STATUS_OK)
    {
        TERMOUT("> WIFI Module Initialized.\n");
        if (WIFI_GetMAC_Address(MAC_Addr, sizeof(MAC_Addr)) == WIFI_STATUS_OK)
        {
            TERMOUT("> es-wifi module MAC Address : %X:%X:%X:%X:%X:%X\n",
                    MAC_Addr[0],
                    MAC_Addr[1],
                    MAC_Addr[2],
                    MAC_Addr[3],
                    MAC_Addr[4],
                    MAC_Addr[5]);
        }
        else
        {
            TERMOUT("> ERROR : CANNOT get MAC address\n");
            BSP_LED_On(LED2);
        }

        if (WIFI_Connect(SSID, PASSWORD, WIFI_ECN_WPA2_PSK) == WIFI_STATUS_OK)
        {
            TERMOUT("> es-wifi module connected \n");
            if (WIFI_GetHostAddress("api.thingspeak.com", IP_Addr, sizeof(IP_Addr)) == WIFI_STATUS_OK)
            {
                TERMOUT("> GOT IP OF SERVER: %d.%d.%d.%d\n", IP_Addr[0], IP_Addr[1], IP_Addr[2], IP_Addr[3]);
            }
            else
            {
                TERMOUT("> DNS RESOLUTION FAILED! Trying manual IP...\n");
                uint8_t fallbackIP[4] = {216, 24, 57, 251}; // replace with actual IP
                memcpy(IP_Addr, fallbackIP, 4);
            }

            // Now try to connect
            if (WIFI_OpenClientConnection(0, WIFI_TCP_PROTOCOL, "conn", IP_Addr, SERVER_PORT, 0) == WIFI_STATUS_OK)
            {
                TERMOUT("> CONNECTED TO SERVER!!\n");
                Socket = 0;
            }
            else
            {
                TERMOUT("> FAILED TO CONNECT TO SERVER!!\n");
            }
        }
        else
        {
            TERMOUT("> ERROR : es-wifi module NOT connected\n");
            BSP_LED_On(LED2);
        }
    }
    else
    {
        TERMOUT("> ERROR : WIFI Module cannot be initialized.\n");
        BSP_LED_On(LED2);
    }
    float temperature = BSP_TSENSOR_ReadTemp();
    float humidity = BSP_HSENSOR_ReadHumidity();

    char payload[128];
    snprintf(payload, sizeof(payload),
             "api_key=%s&field1=%.2f&field2=%.2f",
             API_KEY, temperature, humidity);
    char http_request[512];
    snprintf(http_request, sizeof(http_request),
             "POST /update HTTP/1.1\r\n"
             "Host: %s\r\n"
             "Connection: close\r\n"
             "Content-Type: application/x-www-form-urlencoded\r\n"
             "Content-Length: %d\r\n\r\n"
             "%s",
             SERVER_NAME, strlen(payload), payload);

    if (Socket != -1)
    {
        if (WIFI_SendData(0, (uint8_t *)http_request, strlen(http_request), &Datalen, WIFI_WRITE_TIMEOUT) == WIFI_STATUS_OK)
        {
            TERMOUT("DATA SENT!!\n");

            uint8_t RxData[512] = {0};
            if (WIFI_ReceiveData(0, RxData, sizeof(RxData) - 1, &Datalen, WIFI_READ_TIMEOUT) == WIFI_STATUS_OK)
            {
                RxData[Datalen] = 0;
                TERMOUT("Response: %s\n", RxData);
            }
        }
        else
        {
            TERMOUT("Send failed, try again later\n");
        }
    }
    else
    {
        TERMOUT("CONNECTION FAILED!\n");
    }
}

int main(void)
{

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    /* Configure the system clock */
    SystemClock_Config();
    BSP_TSENSOR_Init();
    BSP_HSENSOR_Init();

#if defined(TERMINAL_USE)
    /* Initialize all configured peripherals */
    hDiscoUart.Instance = DISCOVERY_COM1;
    hDiscoUart.Init.BaudRate = 115200;
    hDiscoUart.Init.WordLength = UART_WORDLENGTH_8B;
    hDiscoUart.Init.StopBits = UART_STOPBITS_1;
    hDiscoUart.Init.Parity = UART_PARITY_NONE;
    hDiscoUart.Init.Mode = UART_MODE_TX_RX;
    hDiscoUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hDiscoUart.Init.OverSampling = UART_OVERSAMPLING_16;
    hDiscoUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    hDiscoUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    BSP_COM_Init(COM1, &hDiscoUart);
#endif /* TERMINAL_USE */
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
    BSP_LED_Init(LED2);
    Execute();
    while (1)
    {
        if (userButtonPressed)
        {
            userButtonPressed = 0;
            HAL_Delay(100);
            BSP_LED_Toggle(LED2);
            Execute();
        }
    }
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (MSI)
 *            SYSCLK(Hz)                     = 80000000
 *            HCLK(Hz)                       = 80000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            APB2 Prescaler                 = 1
 *            MSI Frequency(Hz)              = 4000000
 *            PLL_M                          = 1
 *            PLL_N                          = 40
 *            PLL_R                          = 2
 *            PLL_P                          = 7
 *            PLL_Q                          = 4
 *            Flash Latency(WS)              = 4
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* MSI is enabled after System reset, activate PLL with MSI as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 40;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLP = 7;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        /* Initialization Error */
        while (1)
            ;
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
       clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        /* Initialization Error */
        while (1)
            ;
    }
}

#if defined(TERMINAL_USE)
/**
 * @brief  Retargets the C library TERMOUT function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART1 and Loop until the end of transmission */
    HAL_UART_Transmit(&hDiscoUart, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
}
#endif /* TERMINAL_USE */

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
      ex: TERMOUT("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}

#endif

/**
 * @brief  EXTI line detection callback.
 * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
    case (GPIO_PIN_1):
    {
        SPI_WIFI_ISR();
        break;
    }
    case (GPIO_PIN_13):
    {
        TERMOUT("Executing task...\n");
        userButtonPressed = 1;
        break;
    }
    default:
    {
        break;
    }
    }
}

void SPI3_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi);
}
