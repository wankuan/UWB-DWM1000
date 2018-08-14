/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/

#include "USART.h"
#include "stm32f4xx_hal.h"
#include "sys.h"
#include "delay.h"
#include "KEY.h"
#include "LED.h"
#include "24C02.h"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "spi.h"
#include "i2c.h"
#include "dwOps.h"
#include "cfg.h"
#include "mac.h"

#include "eeprom.h"
#include "led.h"

/* USER CODE BEGIN Includes */

extern UART_HandleTypeDef UART_Handler; //UART句柄
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

static CfgMode mode = modeAnchor;

const uint8_t *uid = (uint8_t*)0x1FFFF7AC;
//const uint8_t *uid = (uint8_t*)MCU_ID_ADDRESS;

int initDwm1000(void);

#define N_NODES 9

uint8_t address[] = {0,0,0,0,0,0,0xcf,0xbc};
uint8_t base_address[] = {0,0,0,0,0,0,0xcf,0xbc};

// Static system configuration
#define MAX_ANCHORS 6
uint8_t anchors[MAX_ANCHORS];

// The four packets for ranging
#define POLL 0x01   // Poll is initiated by the tag
#define ANSWER 0x02
#define FINAL 0x03
#define REPORT 0x04 // Report contains all measurement from the anchor

//Justin add -- start
//#pragma anon_unions
//Tag计算距离之后，发送tag和测距基站的地址，还有距离信息，规定只是发送给地址1的基站
typedef struct {
	uint8_t tagAddress[8];
  uint8_t anchorAddress[8];
  uint32_t distance;
} __attribute__ ((packed)) reportDistance_t;
//Justin add -- end

typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];

  float pressure;
  float temperature;
  float asl;
  uint8_t pressure_ok;
} __attribute__ ((packed)) reportPayload_t;

//#pragma anon_unions
typedef union timestamp_u {
  uint8_t raw[5];
  uint64_t full;
  struct {
    uint32_t low32;
    uint8_t high8;
  } __attribute__ ((packed));
  struct {
    uint8_t low8;
    uint32_t high32;
  } __attribute__ ((packed));
}__attribute__ ((packed)) timestamp_t;

// Timestamps for ranging
dwTime_t poll_tx;
dwTime_t poll_rx;
dwTime_t answer_tx;
dwTime_t answer_rx;
dwTime_t final_tx;
dwTime_t final_rx;

uint32_t rangingTick;

float pressure, temperature, asl;
bool pressure_ok;

const double C = 299792458.0;       // Speed of light
const double tsfreq = 499.2e6 * 128;  // Timestamp counter frequency

#define ANTENNA_OFFSET 154.6   // In meter
#define ANTENNA_DELAY  (ANTENNA_OFFSET*499.2e6*128)/299792458.0 // In radio tick

packet_t rxPacket;
packet_t txPacket;
static volatile uint8_t curr_seq = 0;

// Sniffer queue
#define QUEUE_LEN 16
packet_t snifferPacketQueue[QUEUE_LEN];
int snifferPacketLength[QUEUE_LEN];
int queue_head = 0;
int queue_tail = 0;
volatile uint32_t dropped = 0;

static void restConfig(void);
static void changeAddress(uint8_t addr);
static void handleInput(char ch);
static void changeMode(CfgMode newMode);
static void printMode(void);
static void help(void);

// #define printf(...)
#define debug(...)

//Justin Add
uint32_t timeRangingComplete = 0;
uint8_t rangingComplete = 0;
volatile bool rangingLock = false;//当处于测距状态的时候rangingLock设置为true
uint32_t rangingLockTick = 0;//
uint8_t rangingTagAddr = 0xFF;//正在测距的tag地址
#define rangingOverTime 10 //over time 10ms,normally measure ranging time about 5ms

//#define OLEDDISPLAY
#define USE_FTDI_UART

//#define APP_NAME "DWM1000DISCOVERY"
//#define AUTHOR_NAME "    -- By Justin"
//#define DISTANCE_NAME "Distance:"

/* String used to display measured distance on LCD screen (16 characters maximum). */
char dist_str[16] = {0};

//uart packet
uint8_t uartPacket[32] = {0x55,0xAA,};

//Justin Add end


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void txcallback(dwDevice_t *dev)
{
  dwTime_t departure;
  dwGetTransmitTimestamp(dev, &departure);

  debug("TXCallback: ");

  switch (txPacket.payload[0]) {
    case POLL:
      rangingTick = HAL_GetTick();
      debug("POLL\r\n");
      poll_tx = departure;
      break;
    case ANSWER:
    debug("ANSWER to %02x at %04x\r\n", txPacket.destAddress[0], (unsigned int)departure.low32);
      answer_tx = departure;
      break;
    case FINAL:
      debug("FINAL\r\n");
      final_tx = departure;
      break;
    case REPORT:
      debug("REPORT\r\n");
      break;
  }
}

#define TYPE 0
#define SEQ 1
uint64_t distCount = 0;

void rxcallback(dwDevice_t *dev) {
  dwTime_t arival;
  int dataLength = dwGetDataLength(dev);

  if (dataLength == 0) return;

	//TODO
  //bzero(&rxPacket, MAC802154_HEADER_LENGTH);
	memset(&rxPacket,0,MAC802154_HEADER_LENGTH);

  debug("RXCallback(%d): ", dataLength);

  if (mode == modeSniffer) {
    if ((queue_head+1)%QUEUE_LEN == queue_tail) {
      dropped++;
      dwNewReceive(dev);
      dwSetDefaults(dev);
      dwStartReceive(dev);
      return;
    }

    // Queue the received packet, the main loop will print it on the console
    dwGetData(dev, (uint8_t*)&snifferPacketQueue[queue_head], dataLength);
    snifferPacketLength[queue_head] = dataLength;
    queue_head = (queue_head+1)%QUEUE_LEN;
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
    return;
  }

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);
	
	
  if (memcmp(rxPacket.destAddress, address, 8)) {
    debug("Not for me! for %02x with %02x\r\n", rxPacket.destAddress[0], rxPacket.payload[0]);
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
    return;
  }
	
	//如果处于测距状态，但是其他的tag同时也要测距，直接返回
	if(rangingLock == true && !(rangingTagAddr == rxPacket.sourceAddress[0]))
	{
		debug("ranging with tag %02x,but tag %02x want to rang\r\n", rxPacket.destAddress[0], rangingTagAddr);
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
		return;
	}

  //dwGetReceiveTimestamp(dev, &arival);
	debug("ReceiveTimestamp\r\n");
	
  memcpy(txPacket.destAddress, rxPacket.sourceAddress, 8);
  memcpy(txPacket.sourceAddress, rxPacket.destAddress, 8);

  switch(rxPacket.payload[TYPE]) {
    // Anchor received messages
    case POLL:
      debug("POLL from %02x at %04x\r\n", rxPacket.sourceAddress[0], (unsigned int)arival.low32);
		
			rangingTagAddr = rxPacket.sourceAddress[0];//记录Anchor正在跟哪个tag进行测距
			rangingLock = true;//Anchor处于测距状态，lock住
			rangingLockTick = HAL_GetTick();
		
      rangingTick = HAL_GetTick();

      //poll_rx = arival;

      txPacket.payload[TYPE] = ANSWER;
      txPacket.payload[SEQ] = rxPacket.payload[SEQ];

      dwNewTransmit(dev);
      dwSetDefaults(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);
		
			dwGetReceiveTimestamp(dev, &arival);
      poll_rx = arival;
			break;
    case FINAL:
    {
      reportPayload_t *report = (reportPayload_t *)(txPacket.payload+2);

      debug("FINAL\r\n");
			
			dwGetReceiveTimestamp(dev, &arival);
      final_rx = arival;

      txPacket.payload[TYPE] = REPORT;
      txPacket.payload[SEQ] = rxPacket.payload[SEQ];
      memcpy(&report->pollRx, &poll_rx, 5);
      memcpy(&report->answerTx, &answer_tx, 5);
      memcpy(&report->finalRx, &final_rx, 5);
      report->pressure = pressure;
      report->temperature = temperature;
      report->asl = asl;
      report->pressure_ok = pressure_ok;

      dwNewTransmit(dev);
      dwSetDefaults(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2+sizeof(reportPayload_t));

      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);
			
			//测距完成
			rangingTagAddr = 0xFF;//记录Anchor正在跟哪个tag进行测距
			rangingLock = false;//Anchor处于测距状态，lock住
			
      break;
    }
    // Tag received messages
    case ANSWER:
      debug("ANSWER\r\n");

      if (rxPacket.payload[SEQ] != curr_seq) {
        debug("Wrong sequence number!\r\n");
        return;
      }

      //answer_rx = arival;

      txPacket.payload[0] = FINAL;
      txPacket.payload[SEQ] = rxPacket.payload[SEQ];

      dwNewTransmit(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);
			
			dwGetReceiveTimestamp(dev, &arival);
			answer_rx = arival;
      
			break;
    case REPORT:
    {
			//Justin add -- start
			/*
			if (mode == modeAnchor) {
				reportDistance_t *collectDistance = (reportDistance_t *)(rxPacket.payload+2);
				uint8_t anchorAddress, tagAddress;
				uint32_t aDistance;
				
				anchorAddress = collectDistance->anchorAddress[0];
				tagAddress = collectDistance->tagAddress[0];
				aDistance = collectDistance->distance;
				
				printf("tagAddress: %d,anchorAddress: %d, distance : %5d mm\r\n", tagAddress, anchorAddress, aDistance);
				return;
			}*/
			//Justin add -- end
						
      reportPayload_t *report = (reportPayload_t *)(rxPacket.payload+2);
      double tround1, treply1, treply2, tround2, tprop_ctn, tprop, distance;
			
      debug("REPORT\r\n");

      if (rxPacket.payload[SEQ] != curr_seq) {
        debug("Wrong sequence number!\r\n");
        return;
      }

      memcpy(&poll_rx, &report->pollRx, 5);
      memcpy(&answer_tx, &report->answerTx, 5);
      memcpy(&final_rx, &report->finalRx, 5);

      debug("%02x%08x ", (unsigned int)poll_tx.high8, (unsigned int)poll_tx.low32);
      debug("%02x%08x\r\n", (unsigned int)poll_rx.high8, (unsigned int)poll_rx.low32);
      debug("%02x%08x ", (unsigned int)answer_tx.high8, (unsigned int)answer_tx.low32);
      debug("%02x%08x\r\n", (unsigned int)answer_rx.high8, (unsigned int)answer_rx.low32);
      debug("%02x%08x ", (unsigned int)final_tx.high8, (unsigned int)final_tx.low32);
      debug("%02x%08x\r\n", (unsigned int)final_rx.high8, (unsigned int)final_rx.low32);

      tround1 = answer_rx.low32 - poll_tx.low32;
      treply1 = answer_tx.low32 - poll_rx.low32;
      tround2 = final_rx.low32 - answer_tx.low32;
      treply2 = final_tx.low32 - answer_rx.low32;

      debug("%08x %08x\r\n", (unsigned int)tround1, (unsigned int)treply2);
      debug("\\    /   /     \\\r\n");
      debug("%08x %08x\r\n", (unsigned int)treply1, (unsigned int)tround2);

      tprop_ctn = ((tround1*tround2) - (treply1*treply2)) / (tround1 + tround2 + treply1 + treply2);

      debug("TProp (ctn): %d\r\n", (unsigned int)tprop_ctn);

      tprop = tprop_ctn/tsfreq;
			debug("tprop : %d\r\n", (unsigned int)tprop);
      distance = C * tprop;
			
			distCount++;

			printf("distance %d: %6dmm, ", rxPacket.sourceAddress[0], (unsigned int)(distance*1000));
			
			#ifdef OLEDDISPLAY
				//OLED_ShowString(0,6,"                 ");//clear the line
				sprintf(dist_str, "DIST: %6dmm", (unsigned int)(distance*1000));

				OLED_ShowString(0,6,(uint8_t*)dist_str);
			#endif
			//if(rxPacket.sourceAddress[0] == 3)
			{
				printf("\r\n");
			}

			dwGetReceiveTimestamp(dev, &arival);
      debug("Total in-air time (ctn): 0x%08x\r\n", (unsigned int)(arival.low32-poll_tx.low32));
		//	printf("Total in-air time (ctn): %.4f s\r\n", (float)((arival.low32-poll_tx.low32)/tsfreq));
			
			rangingComplete = 1;

      break;
    }
  }
}

bool contains(int* list, int length, int value)
{
  int i;

  for (i=0; i<length; i++) {
    if (list[i] == value) {
      return true;
    }
  }

  return false;
}
/* USER CODE END 0 */
dwDevice_t dwm_device;
dwDevice_t *dwm = &dwm_device;

int main(void)
{
	int result=0;
  int i=0;
  char ch=0;
  bool selftestPasses = true;
	
	uint8_t anchorListSize = 0;

  bool ledState = false;
  uint32_t ledTick = 0;
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  
   HAL_Init();                     //初始化HAL库
  /* Configure the system clock */
    Stm32_Clock_Init(360,25,2,8); 		//设置时钟,180Mhz
    delay_init(180);                //初始化延时函数
				LED_Init();
				MX_GPIO_Init();
				MX_SPI1_Init();
		  eepromInit();
				
				UART_Init();
			//printf("%x\r\n",AT24CXX_Check());


 
	//MX_USART2_UART_Init();

	
  // Light up all LEDs to test
	
		printf("CONFIG\t: EEPROM self-test\r\n");

  cfgInit();

  if (cfgReadU8(cfgAddress, &address[0])) {
    printf("CONFIG\t: Address is 0x%X\r\n", address[0]);
  } else {
    printf("CONFIG\t: Address not found!\r\n");
  }

  if (cfgReadU8(cfgMode, &mode)) {
    printf("CONFIG\t: Mode is ");
    switch (mode) {
      case modeAnchor: printf("Anchor\r\n"); break;
      case modeTag: printf("Tag\r\n"); break;
      case modeSniffer: printf("Sniffer\r\n"); break;
      default: printf("UNKNOWN\r\n"); break;
    }
  } else {
    printf("Device mode: Not found!\r\n");
  }


  if (cfgFieldSize(cfgAnchorlist, &anchorListSize)) {
    if (cfgReadU8list(cfgAnchorlist, (uint8_t*)&anchors, anchorListSize)) {
      printf("CONFIG\t: Tag mode anchor list (%i): ", anchorListSize);
      for (i = 0; i < anchorListSize; i++) {
        printf("0x%02X ", anchors[i]);
      }
      printf("\r\n");
    } else {
      printf("CONFIG\t: Tag mode anchor list: Not found!\r\n");
    }
  }
	
	printf("CONFIG\t: Initialize DWM1000 ... \r\n");
	dwInit(dwm, &dwOps);       // Init libdw
  dwOpsInit(dwm);
  result = dwConfigure(dwm); // Configure the dw1000 chip
  if (result == 0) {
    printf("System Test Ok\r\n");
    dwEnableAllLeds(dwm);
  } else {
    printf("System Test Error: %s\r\n", dwStrError(result));
    selftestPasses = false;
  }
	
	delay_ms(100);

	
		dwTime_t delay = {.full = ANTENNA_DELAY/2};//delay =16475.679
		
  dwSetAntenaDelay(dwm, delay);

  dwAttachSentHandler(dwm, txcallback);
		
  dwAttachReceivedHandler(dwm, rxcallback);

  dwNewConfiguration(dwm);
  dwSetDefaults(dwm);
	//dwEnableMode(dwm, MODE_LONGDATA_RANGE_LOWPOWER);	
	//dwEnableMode(dwm, MODE_SHORTDATA_FAST_LOWPOWER);	
	//dwEnableMode(dwm, MODE_LONGDATA_FAST_LOWPOWER);		
			dwEnableMode(dwm, MODE_SHORTDATA_FAST_ACCURACY);	
	//dwEnableMode(dwm, MODE_LONGDATA_FAST_ACCURACY);
	//dwEnableMode(dwm, MODE_LONGDATA_RANGE_ACCURACY);	
  dwSetChannel(dwm, CHANNEL_2);
  dwSetPreambleCode(dwm, PREAMBLE_CODE_64MHZ_9);

  dwCommitConfiguration(dwm);

  printf("SYSTEM\t: Node started ...\r\n");
  printf("SYSTEM\t: Press 'h' for help.\r\n");
	
	
  // Initialize the packet in the TX buffer
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  txPacket.pan = 0xbccf;

  if (mode == modeAnchor || mode == modeSniffer) {
    dwNewReceive(dwm);
    dwSetDefaults(dwm);
    dwStartReceive(dwm);
  }
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
					LED_R=~LED_R;
					LED_G=~LED_G;
				if (HAL_UART_Receive(&UART_Handler, (uint8_t*)&ch, 1, 0) == HAL_OK) 
					{
      handleInput(ch);
    }
    if (mode == modeSniffer) {
      static uint32_t prevDropped = 0;

      if (dropped != prevDropped) {
        printf("Dropped!\r\n");
        prevDropped = dropped;
      }

      if (queue_tail != queue_head) {
        printf("From %02x to %02x data ", snifferPacketQueue[queue_tail].sourceAddress[0],
                                          snifferPacketQueue[queue_tail].destAddress[0]);
        for (int i=0; i<(snifferPacketLength[queue_tail] - MAC802154_HEADER_LENGTH); i++) {
          printf("0x%02x ", snifferPacketQueue[queue_tail].payload[i]);
        }
        queue_tail = (queue_tail+1)%QUEUE_LEN;
        printf("\r\n");
      }
    }

    if (mode == modeTag) {
       for (i=0; i<anchorListSize; i++) {
         //printf ("Interrogating anchor %d\r\n", anchors[i]);
         base_address[0] = anchors[i];
         dwIdle(dwm);

         txPacket.payload[TYPE] = POLL;
         txPacket.payload[SEQ] = ++curr_seq;

         memcpy(txPacket.sourceAddress, address, 8);
         memcpy(txPacket.destAddress, base_address, 8);

         dwNewTransmit(dwm);
								
         dwSetDefaults(dwm);
								
         dwSetData(dwm, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

         dwWaitForResponse(dwm, true);
								
         dwStartTransmit(dwm);

         //Justin add
         //HAL_Delay(30);
				 //measure the ranging time,then printf the tag to every anchor ranging time information
				 timeRangingComplete = HAL_GetTick();
				 rangingComplete = 0;
				 while((rangingComplete ==0)&&(HAL_GetTick()<(timeRangingComplete+rangingOverTime)));
				 if(HAL_GetTick()<(timeRangingComplete+rangingOverTime))
				 {
					debug("tag and anchor[%d] ranging time: %04d ms\r\n", (i+1),(unsigned int)(HAL_GetTick() - timeRangingComplete));
				 }
				 else
				 {
					debug("anchor[%d] maybe no exist, ranging time over: %04d ms\r\n", (i+1),(unsigned int)(HAL_GetTick() - timeRangingComplete));
				 }
				 //Justin add end
       }
     }
		
		 if(HAL_GetTick() > rangingLockTick + 10)
		 {
				rangingLock = false;
				rangingTagAddr = 0xFF;
		 }



     //ledOff(ledSync);

     switch (mode)
						{
       case modeTag:
         break;
       case modeAnchor:
         break;
       case modeSniffer: 
								 break;
       default:
         break;
				  }

  }

}



void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOB_CLK_ENABLE();
  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}


#endif


static void handleInput(char ch) {
  bool configChanged = true;

  switch (ch) {
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
      changeAddress(ch - '0');
      break;
    case 'a': changeMode(modeAnchor); break;
    case 't': changeMode(modeTag); break;
    case 's': changeMode(modeSniffer); break;
    case 'd': restConfig(); break;
    case 'h':
      help();
      configChanged = false;
      break;
    case '#':
      //productionTestsRun();
      printf("System halted, reset to continue\r\n");
      while(true){}
      break;
    default:
      configChanged = false;
      break;
  }

  if (configChanged) {
    printf("EEPROM configuration changed, restart for it to take effect!\r\n");
  }
}

static void restConfig(void) {
  printf("Resetting EEPROM configuration...");
  if (cfgReset()) {
    printf("OK\r\n");
  } else {
    printf("ERROR\r\n");
  }
}

static void changeAddress(uint8_t addr) {
  printf("Updating address from 0x%02X to 0x%02X\r\n", address[0], addr);
  cfgWriteU8(cfgAddress, addr);
  if (cfgReadU8(cfgAddress, &address[0])) {
    printf("Device address: 0x%X\r\n", address[0]);
  } else {
    printf("Device address: Not found!\r\n");
  }
}

static void changeMode(CfgMode newMode) {
    printf("Previous device mode: ");
    printMode();

    cfgWriteU8(cfgMode, newMode);

    printf("New device mode: ");
    printMode();
}

static void printMode(void) {
  CfgMode mode;

  if (cfgReadU8(cfgMode, &mode)) {
    switch (mode) {
      case modeAnchor: printf("Anchor"); break;
      case modeTag: printf("Tag"); break;
      case modeSniffer: printf("Sniffer"); break;
      default: printf("UNKNOWN"); break;
    }
  } else {
    printf("Not found!");
  }

  printf("\r\n");
}

static void help(void) {
  printf("Help\r\n");
  printf("-------------------\r\n");
  printf("0-9 - set address\r\n");
  printf("a   - anchor mode\r\n");
  printf("t   - tag mode\r\n");
  printf("s   - sniffer mode\r\n");
  printf("d   - reset configuration\r\n");
  printf("h   - This help\r\n");
}


/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
