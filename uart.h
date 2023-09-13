
#ifndef UART_H_
#define UART_H_



/* Private includes ----------------------------------------------------------*/
#include "configboard.h"
#include "LPC17xx.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
enum {UART_0=0,UART_1,UART_2,UART_3};
enum {ACCESS_DISABLE=0,ACCESS_ENABLE};
enum {FIFO_DISABLE=0,FIFO_ENABLE=1};
enum {RBR_INTERRUPT=0,THRE_INTERRUPT,RX_LINE_STATUS_INTERRUPT,ABEOINT,ABTOINT};
enum {RDA_INT_DISABLE=0,RDA_INT_ENABLE};
enum {THRE_INT_DISABLE=0,THRE_INT_ENABLE};
enum {RX_LINE_INT_DISABLE=0,RX_LINE_INT_ENABLE};
enum {AUTOBAUD_INT_DISABLE=0,AUTOBAUD_INT_ENABLE};
enum {TRIGGER_LEVEL0=0,TRIGGER_LEVEL1,TRIGGER_LEVEL2,TRIGGER_LEVEL3};
enum {RXFIFO=1,TXFIFO=2};
enum {THRE_INT=1,RDA=2,RLS=3,CTI=6};

enum {BINARY=2,OCTAL=8,DECIMAL=10,HEXADECIMAL=16};

/* Buffer size for the software Tx and Rx queue */
#define UART0TXDATABUFFERSIZE   127
#define UART0RXDATABUFFERSIZE    63

/* Indicates the UART data is one byte */
#define UARTDATASIZE              1

/* UART Hardware FIFO SIZE */
#define MAXFIFOSIZE 16


/* Private macro -------------------------------------------------------------*/
#define UART0 LPC_UART0
#define UART1 LPC_UART1
#define UART2 LPC_UART2
#define UART3 LPC_UART3

/* Baudrate values determined when UART peripheral frequency is 25MHz */
#define BAUDRATE_9600   		 0
#define BAUDRATE_9600_DLM        0
#define BAUDRATE_9600_DLL       92
#define BAUDRATE_9600_DIVVAL    10
#define BAUDRATE_9600_MULVAL    13

#define BAUDRATE_38400  		 1
#define BAUDRATE_38400_DLM       0
#define BAUDRATE_38400_DLL      23
#define BAUDRATE_38400_DIVVAL   10
#define BAUDRATE_38400_MULVAL   13

#define BAUDRATE_57600  		 2
#define BAUDRATE_57600_DLM       0
#define BAUDRATE_57600_DLL      19
#define BAUDRATE_57600_DIVVAL    3
#define BAUDRATE_57600_MULVAL    7

#define BAUDRATE_115200 		 3
#define BAUDRATE_115200_DLM      0
#define BAUDRATE_115200_DLL     10
#define BAUDRATE_115200_DIVVAL   5
#define BAUDRATE_115200_MULVAL  14


/* Peripheral Register bits */

/* Bits in UARTn Receiver Buffer Register */
#define BIT_RBR_RBR 0

/* Bits in UARTn Transmit Holding Register */
#define BIT_THR_THR 0

/* Bits in UARTn Divisor Latch LSB register */
#define BIT_DLL_DLLSB 0

/*Bits in UARTn Divisor Latch MSB register */
#define BIT_DLM_DLMSB 0

/*Bits in UARTn Interrupt Enable Register */
#define BIT_IER_RBR_IE                          0
#define BIT_IER_THRE_IE                         1
#define BIT_IER_RXLine_Status                   2
#define BIT_IER_MSIE                            3
#define BIT_IER_CTS_IE                          7
#define BIT_IER_ABEOIntEn                       8
#define BIT_IER_ABTOIntEn                       9

/*Bits in UARTn Interrupt Identification Register */
#define BIT_IIR_IntStatus   0
#define BIT_IIR_IntId       1
#define BIT_IIR_FIFO        6
#define BIT_IIR_ABEOInt     8
#define BIT_IIR_ABTOInt     9

/*Bits in UARTn FIFO Control Register */
#define BIT_FCR_FIFO             0
#define BIT_FCR_RX_FIFO          1
#define BIT_FCR_TX_FIFO          2
#define BIT_FCR_DMAMode          3
#define BIT_FCR_RX_TriggerLevel  6

/*Bits in UARTn Line Control Register */
#define BIT_LCR_WLS                    0
#define BIT_LCR_SBC                    2
#define BIT_LCR_ParityEnable           3
#define BIT_LCR_ParitySelect           4
#define BIT_LCR_BreakControl           6
#define BIT_LCR_DLAB                   7

/*Bits in UARTn Line Status Register */
#define BIT_LSR_RDR      0
#define BIT_LSR_OE       1
#define BIT_LSR_PE       2
#define BIT_LSR_BI       4
#define BIT_LSR_THRE     5
#define BIT_LSR_TEMT     6
#define BIT_LSR_RXFE     7

/*Bits in UARTn Scratch Pad Register */
#define BIT_SCR_PAD      0

/*Bits in UARTn Auto-baud Control Register */
#define BIT_ACR_START        0
#define BIT_ACR_MODE         1
#define BIT_ACR_AUTORESTART  2
#define BIT_ACR_ABEOIntClr   8
#define BIT_ACR_ABTOIntClr   9

/*Bits in UARTn IrDA Control Register */
#define BIT_ICR_IrDAEn        0
#define BIT_ICR_IrDAInv       1
#define BIT_ICR_FixPulseEn    2
#define BIT_ICR_PulseDiv      3

/*Bits in UARTn Fractional Divider Register */
#define BIT_FDR_DIVADDVAL        0
#define BIT_FDR_MULVAL           4

/*Bits in UARTn Transmit Enable Register */
#define BIT_TER_TXEN       0


/* Function pointer declaration */
typedef void (*uartisrfuncptr)(void);


/* Private function prototypes -----------------------------------------------*/

/**
  * @brief     Initialize the selected UART Peripheral
  * @param[in] UARTx base address of the selected uart value can be UART0,UART1,UART2,UART3
  * @param[in] ucUARTNum value can be UART_0,UART_1,UART_2,UART_3
  * @param[in] baudrate value can BAUDRATE_9600,BAUDRATE_38400,BAUDRATE_57600,BAUDRATE_115200
  * @return    No return value
  **/
void vUARTInitialize(LPC_UART_TypeDef* UARTx,uint8_t ucUARTNum,uint8_t baudrate);

/**
  * @brief     Function to transmit characters in polling method
  * @param[in] UARTx base address of the selected uart value can be UART0,UART1,UART2,UART3
  * @param[in] bytetosend value 8bit value to transmit over UART
  * @return    No return value
  **/
void vUARTPutCharBlocking(LPC_UART_TypeDef* UARTx,char bytetosend);

/**
  * @brief     Function to transmit characters in polling method
  * @param[in] UARTx base address of the selected uart value can be UART0,UART1,UART2,UART3
  * @param[in] ptrtoarrayofchars address of the array of characters to be send
  * @return    No return value
  **/
void vUARTPutStringBlocking(LPC_UART_TypeDef* UARTx,char* ptrtoarrayofchars);

/**
 * @brief     UART put number Function
 * @param[in] UARTx base address of the selected uart value can be UART0,UART1,UART2,UART3
 * @param[in] ucNum number to be written to the UART
 * @param[in] base base of the number to be written to the selected UART - {BINARY,OCTAL,DECIMAL,HEXADECIMAL};
 * @return    No return value
 **/
void vUARTPutNumBlocking(LPC_UART_TypeDef* UARTx, uint8_t ucNum,uint8_t base);

/**
 * @brief     Printf implementation on UART
 * @param[in] UARTx base address of the selected uart value can be UART0,UART1,UART2,UART3
 * @param[in] formatstring string which contains format specifiers
 * @return    No return value
 **/

void vUARTPrintfBlocking(LPC_UART_TypeDef* UARTx,char *formatstring,...);

/**
  * @brief     Function to receive character in polling method
  * @param[in] UARTx base address of the selected uart value can be UART0,UART1,UART2,UART3
  * @param[in] ptrtoreceivebyte address of the buffer to read the received value
  * @return    No return value
  **/
void vUARTGetCharBlocking(LPC_UART_TypeDef* UARTx,char* ptrtoreceivebyte);

/**
  * @brief     Enabling Interrupt for the UART and the source for interrupt
  * @param[in] UARTx base address of the selected uart value can be UART0,UART1,UART2,UART3
  * @param[in] interruptsource value can be RBR_INTERRUPT_ENABLE,THRE_INTERRUPT_ENABLE,RX_LINE_STATUS_INTERRUPT_ENABLE,ABEOINTEN,ABTOINTEN
  * @return    No return value
  **/
void vUARTInterruptEnable(LPC_UART_TypeDef* UARTx,uint8_t interruptsource);

/**
  * @brief     Disabling Interrupt for the UART and the source for interrupt
  * @param[in] UARTx base address of the selected uart value can be UART0,UART1,UART2,UART3
  * @param[in] interruptsource value can be RBR_INTERRUPT_ENABLE,THRE_INTERRUPT_ENABLE,RX_LINE_STATUS_INTERRUPT_ENABLE,ABEOINTEN,ABTOINTEN
  * @return    No return value
  **/
void vUARTInterruptDisable(LPC_UART_TypeDef* UARTx,uint8_t interruptsource);

/**
 * @brief     UART interrupt call back registration
 * @param[in] ucUARTNum value can be UART_0,UART_1,UART_2,UART_3
 * @param[in] vUARTintisrobject address of the function which should be called when the respective interrupt occurs
 * @return    No return value
 **/

void vUARTIntAttachCallback(uint8_t ucUARTNum, uartisrfuncptr vUARTintisrobject);

/**
 * @brief     Application UART interrupt call back handler
 * @return    No return value
 **/
void vUARTCallbackHandler(void);

/**
  * @brief     Function to transmit characters in interrupt method
  * @param[in] UARTx base address of the selected uart value can be UART0,UART1,UART2,UART3
  * @param[in] ptrbuffertosend pointer to the buffer for sending the bytes to UART
  * @param[in] numofbytes number of bytes to send over selected UART
  * @return    No return value
  **/
uint8_t ucUARTSendNonBlocking(LPC_UART_TypeDef* UARTx,uint8_t* ptrbuffertosend, uint32_t numofbytes);

#endif /* UART_H_ */
