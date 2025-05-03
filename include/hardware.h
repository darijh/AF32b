// PINOUT
#define HW_VOUT_A ADC_CHANNEL_0
#define HW_IOUT_A ADC_CHANNEL_1
#define HW_VOUT_B ADC_CHANNEL_4
#define HW_IOUT_B ADC_CHANNEL_8
#define HW_ID_B0 PB3
#define HW_ID_B1 PA10
#define HW_ID_B2 PA2
#define HW_ID_B3 PA3
#define HW_CTRL PB5
#define HW_SPV_A PA8
#define HW_SPI_A PA9
#define HW_SPV_B PA7
#define HW_SPI_B PA6
#define HW_STS PC13
#define HW_OK PB9
#define HW_FAIL PB8

#define READS 20          // Número de lecturas para promediar
#define READ_INTERVAL 100 // Intervalo de lectura de ADC

#define VKp 0.9
#define VKi 12
#define VKd 0
#define IKp 0.9
#define Iki 12
#define Ikd 0

// HW CONFIG
#define DBG_PORT Serial
#define COM_PORT Serial3
#define BAUDRATE 19200