#include "radio.h"
#include "efm32_cmu.h"
#include "hal_nrf.h"

#define SPI_BAUDRATE 500000

volatile uint8_t progPayload[32];
uint8_t volatile procPayload = false;

/* Buffer pointers and indexes */
char* slaveTxBuffer;
int slaveTxBufferSize;
volatile int slaveTxBufferIndex;
char* slaveRxBuffer;
int slaveRxBufferSize;
volatile int slaveRxBufferIndex;
char* masterRxBuffer;
int masterRxBufferSize;
volatile int masterRxBufferIndex;
uint8_t payloadAddr[30];
bool radio_busy = false;
bool is_reuse = false;
/* Prototypes */
void SPI_setup(uint8_t spiNumber, uint8_t location, bool master);

void GPIO_EVEN_IRQHandler(void)
{ 
  uint8_t irq_flags;
  DEBUGPIN_SET;
  /* clear flag for RF_IRQ interrupt */
  GPIO_IntClear(RFIRQ);
  
  // Read and clear IRQ flags from radio
  irq_flags = hal_nrf_get_clear_irq_flags(); 
  
  switch(irq_flags)
  {
    // Transmission success
	case ( (1 << HAL_NRF_RX_DR) | (1 << HAL_NRF_TX_DS) ):	  // We rx payload packet
	   	procPayload = true;
	   	// Read payload
		while(!hal_nrf_rx_fifo_empty())
  	 	{
	    	  hal_nrf_read_rx_payload(progPayload);
		}
	  	radio_busy = false;
    case (1 << HAL_NRF_TX_DS):
      radio_busy = false;
      // Data has been sent
      break;
    // Transmission failed (maximum re-transmits)
    case (1 << HAL_NRF_MAX_RT):
      // When a MAX_RT interrupt occurs the TX payload will not be removed from the TX FIFO. 
      // If the packet is to be discarded this must be done manually by flushing the TX FIFO.
      // Alternatively, CE_PULSE() can be called re-starting transmission of the payload.
      // (Will only be possible after the radio irq flags are cleared) 
      hal_nrf_flush_tx();
      radio_busy = false;
      is_reuse = false;
      break;
  }
  DEBUGPIN_CLEAR;
  // Wake up MCU to do some processing
}

void RADIO_TX(uint8_t* payload, uint8_t length)
{
  DEBUGPIN_SET;
  hal_nrf_set_power_mode(HAL_NRF_PWR_UP);
    hal_nrf_write_tx_payload(payload,length);
  DEBUGPIN_CLEAR;
  radio_busy = true;
  // Toggle radio CE signal to start transmission 
  CE_PULSE();
  
  
}

void RADIO_setup(void)
{
  volatile uint8_t status;
  procPayload = false; 
  is_reuse = false;
  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PortOutSet(RFCS_PORT, RFCS); 
  GPIO_PortOutClear(RFCS_PORT, RFCS); 
  GPIO_PortOutSet(RFCS_PORT, RFCS); 
 //  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPullDrive, 0);   /* CS */
   
  /* Configure Nordic CE pin*/
  GPIO_PinModeSet(RFCE_PORT, RFCE_N, gpioModePushPull, 0);
  /* Configure Nordic IRQ pin*/
  GPIO_PinModeSet(RFIRQ_PORT, RFIRQ_N, gpioModeInput, 1);
   /* Enable GPIO_ODD interrupt vector in NVIC */
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  
  /* Configure RF IRQ interrupt on falling edge */
  GPIO_IntConfig(RFIRQ_PORT, RFIRQ_N, false, true, true);
  CMU_ClockEnable(cmuClock_USART0, true);
   SPI_setup(0, NRF_USART_LOCATION, true);
  // USART0->CMD   = USART_CMD_MASTEREN | USART_CMD_TXEN | USART_CMD_RXEN;
  CSN_HIGH();
  
  //  hal_nrf_set_power_mode(HAL_NRF_PWR_DOWN);
        // Power up radio, dont want to use spi during crystal startup mode (1.5ms)
  hal_nrf_set_power_mode(HAL_NRF_PWR_UP);
  
  hal_nrf_enable_ack_payload(1);
  
  hal_nrf_enable_dynamic_payload(1);
  hal_nrf_setup_dynamic_payload(1); // Set up PIPE 0 to handle dynamic lengths
  hal_nrf_set_rf_channel(125); // 2525 MHz
  hal_nrf_set_auto_retr(5, 250); // Retry 5x
  
  status = hal_nrf_read_reg(SETUP_RETR);
  status = hal_nrf_read_reg(RF_CH);
  hal_nrf_flush_tx();
  hal_nrf_flush_rx();
    // Configure radio as primary(PTX) 
  hal_nrf_set_operation_mode(HAL_NRF_PTX);
   status = hal_nrf_nop();
 hal_nrf_set_power_mode(HAL_NRF_PWR_DOWN);
  // Enable receiver
 // CE_HIGH();
  status = hal_nrf_nop();
    
//  RADIO_TX(payloadAddr,2);
  __NOP();
}



/**************************************************************************//**
 * @brief Setup a USART as SPI
 * @param spiNumber is the number of the USART to use (e.g. 1 USART1)
 * @param location is the GPIO location to use for the device
 * @param master set if the SPI is to be master
 *****************************************************************************/
void SPI_setup(uint8_t spiNumber, uint8_t location, bool master)
{
  USART_TypeDef *spi;
  uint32_t spi_perck_frequency = CMU_ClockFreqGet(cmuClock_HFPER);
  
/* Enable clock for USART module */
  
  
  /* Determining USART */
  switch (spiNumber)
  {
  case 0:
    spi = USART0;
    CMU_ClockEnable(cmuClock_USART0, true);
    break;
  case 1:
    spi = USART1;
    CMU_ClockEnable(cmuClock_USART1, true);
    break;
/*  case 2:
    spi = USART2;
    break;
    */
  default:
    return;
  }

  /* Setting baudrate */
  spi->CLKDIV = 128 * (spi_perck_frequency / SPI_BAUDRATE - 2);

  /* Configure SPI */
  /* Using synchronous (SPI) mode*/
  spi->CTRL = USART_CTRL_SYNC | USART_CTRL_MSBF;
  /* Clearing old transfers/receptions, and disabling interrupts */
  spi->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
  spi->IEN = 0;
  /* Enabling pins and setting location */
  // spi->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN | (location << 8);
  
  /* Set to master and to control the CS line */
  if (master)
  {
    /* Enabling Master, TX and RX */
    spi->CMD   = USART_CMD_MASTEREN | USART_CMD_TXEN | USART_CMD_RXEN;
   // spi->CTRL |= USART_CTRL_AUTOCS;
  }
  else
  {
    /* Enabling TX and RX */
    spi->CMD = USART_CMD_TXEN | USART_CMD_RXEN;
  }

  /* Clear previous interrupts */
  spi->IFC = _USART_IFC_MASK;

  /* IO configuration */
  switch(spiNumber)
  {
    case 0: switch(location)
            {
              case 0: /* IO configuration (USART 0, Location #0) */
                      GPIO_PinModeSet(gpioPortE, 10, gpioModePushPull, 0);  /* MOSI */
                      GPIO_PinModeSet(gpioPortE, 11, gpioModeInput, 0);     /* MISO */
                      GPIO_PinModeSet(gpioPortE, 13, gpioModePushPull, 0);  /* CS */
                      GPIO_PinModeSet(gpioPortE, 12, gpioModePushPull, 0);  /* Clock */
                      break;
              case 1: /* IO configuration (USART 0, Location #1) */
                      GPIO_PinModeSet(gpioPortE, 7, gpioModePushPull, 0);   /* MOSI */ 
                      GPIO_PinModeSet(gpioPortE, 6, gpioModeInput, 0);      /* MISO */
                      GPIO_PinModeSet(gpioPortE, 4, gpioModePushPull, 0);   /* CS */
                      GPIO_PinModeSet(gpioPortE, 5, gpioModePushPull, 0);   /* Clock */
                      break;
              case 2: /* IO configuration (USART 0, Location #2) */
                      GPIO_PinModeSet(gpioPortC, 11, gpioModePushPull, 0);  /* MOSI */
                      GPIO_PinModeSet(gpioPortC, 10, gpioModeInput, 0);     /* MISO */
                      GPIO_PinModeSet(gpioPortC,  8, gpioModePushPull, 0);  /* CS */
                      GPIO_PinModeSet(gpioPortC,  9, gpioModePushPull, 0);  /* Clock */
                      break;
            default: break;
            }
            break;
    case 1: switch(location)
            {
              case 0: /* IO configuration (USART 1, Location #0) */
                      GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 0);   /* MOSI */
                      GPIO_PinModeSet(gpioPortC, 1, gpioModeInput, 0);      /* MISO */
                      GPIO_PinModeSet(gpioPortB, 8, gpioModePushPull, 0);   /* CS */
                      GPIO_PinModeSet(gpioPortB, 7, gpioModePushPull, 0);   /* Clock */
                      break;
              case 1: /* IO configuration (USART 1, Location #1) */
                      GPIO_PinModeSet(gpioPortD, 0, gpioModePushPullDrive, 0);   /* MOSI */
                      GPIO_PinModeSet(gpioPortD, 1, gpioModeInput, 0);      /* MISO */
                      GPIO_PinModeSet(gpioPortD, 3, gpioModePushPullDrive, 0);   /* CS */
                      GPIO_PinModeSet(gpioPortD, 2, gpioModePushPullDrive, 0);   /* Clock */
                      break;              
            default: break;
            }
            break;
    case 2: switch(location)
            {
              case 0: /* IO configuration (USART 2, Location #0) */
                      GPIO_PinModeSet(gpioPortC, 2, gpioModePushPull, 0);   /* MOSI */
                      GPIO_PinModeSet(gpioPortC, 3, gpioModeInput, 0);      /* MISO */
                      GPIO_PinModeSet(gpioPortC, 5, gpioModePushPull, 0);   /* CS */
                      GPIO_PinModeSet(gpioPortC, 4, gpioModePushPull, 0);   /* Clock */
                      break;
              case 1: /* IO configuration (USART 2, Location #1) */
                      GPIO_PinModeSet(gpioPortB, 3, gpioModePushPull, 0);   /* MOSI */
                      GPIO_PinModeSet(gpioPortB, 4, gpioModeInput, 0);      /* MISO */
                      GPIO_PinModeSet(gpioPortB, 6, gpioModePushPull, 0);   /* CS */
                      GPIO_PinModeSet(gpioPortB, 5, gpioModePushPull, 0);   /* Clock */
                      break;              
            default: break;
            }
            break;
    default: break;  
  }
}

/**************************************************************************//**
 * @brief USART1 RX IRQ Handler Setup
 * @param receiveBuffer points to where to place recieved data
 * @param receiveBufferSize indicates the number of bytes to receive
 *****************************************************************************/
void SPI1_setupRXInt(char* receiveBuffer, int receiveBufferSize)
{
  USART_TypeDef *spi = USART1;

  /* Setting up pointer and indexes */
  slaveRxBuffer      = receiveBuffer;
  slaveRxBufferSize  = receiveBufferSize;
  slaveRxBufferIndex = 0;

  /* Clear RX */
  spi->CMD = USART_CMD_CLEARRX;

  /* Enable interrupts */
  NVIC_ClearPendingIRQ(USART1_RX_IRQn);
  NVIC_EnableIRQ(USART1_RX_IRQn);
  spi->IEN |= USART_IEN_RXDATAV;
}



/**************************************************************************//**
 * @brief USART1 TX IRQ Handler Setup
 * @param transmitBuffer points to the data to send
 * @param transmitBufferSize indicates the number of bytes to send
 *****************************************************************************/
void SPI1_setupTXInt(char* transmitBuffer, int transmitBufferSize)
{
  USART_TypeDef *spi = USART1;

  /* Setting up pointer and indexes */
  slaveTxBuffer      = transmitBuffer;
  slaveTxBufferSize  = transmitBufferSize;
  slaveTxBufferIndex = 0;

  /* Clear TX */
  spi->CMD = USART_CMD_CLEARTX;

  /* Enable interrupts */
  NVIC_ClearPendingIRQ(USART1_TX_IRQn);
  NVIC_EnableIRQ(USART1_TX_IRQn);
  spi->IEN |= USART_IEN_TXBL;
}

/**************************************************************************//**
 * @brief USART1 IRQ Handler Setup
 * @param receiveBuffer points to where received data is to be stored
 * @param receiveBufferSize indicates the number of bytes to receive
 * @param transmitBuffer points to the data to send
 * @param transmitBufferSize indicates the number of bytes to send
 *****************************************************************************/
void SPI1_setupMasterInt(char* receiveBuffer, int receiveBufferSize, char* transmitBuffer, int transmitBufferSize)
{
  SPI1_setupRXInt(receiveBuffer, receiveBufferSize);
  SPI1_setupTXInt(transmitBuffer, transmitBufferSize);
}

