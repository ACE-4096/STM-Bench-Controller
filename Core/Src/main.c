/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RefCalOffset 0.0255

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Structure to hold the voltage and current

// external types
extern SPI_HandleTypeDef hspi2;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_DescriptorsTypeDef CDC_Desc;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
RTC_HandleTypeDef hrtc;
TIM_HandleTypeDef htim3;

// Function prototypes

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

static void MX_USB_OTG_FS_Init(void);
void SendStrToUSB(char *);
float Measure(int8_t, uint8_t);
ssize_t containsStr(const char *needle, const char *haystack);
ssize_t containsChar(char needle, const char *haystack);
void Output(uint8_t Channel, uint8_t State);
void HandleCmd();
void nullstr(char*, uint16_t);
void nullint8(uint8_t* pointer, uint16_t size);
float getVref(uint16_t);
volatile uint32_t * getVrefCalData(int);
uint8_t ReadInputs(void);
void PWM(uint16_t value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Var declaration

// uart comms buffers
char strBuf[1024];
char SerialStr[1024];
char strOutBuf[1024];
uint8_t hexBuf[255];
uint32_t pos32;

int encoderState = 0x00;
int EncoderCounterLimit = 4096;
int EncoderCount = 0;




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
  MX_RTC_Init();
  MX_USB_Device_Init();
  MX_USB_OTG_FS_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  while (1)
     {
	  //if (encoderPrevCount != (TIM2->CNT)){
		//  encoderPrevCount = (TIM2->CNT);
	  //}

 	  // Button inputs
 	  uint8_t values = ReadInputs();
 	  if ((values & 0b10000000) > 0){ // button 1

 	  }if ((values & 0b01000000) > 0){ // button 2

 	  }if ((values & 0b00100000) > 0){ // button 3

 	  }if ((values & 0b00010000) > 0){ // button 4

 	  }if ((values & 0b00001000) > 0){ // Encoder button

 	  }if ((values & 0b00000011) > 0){ // Encoder Up

 	  }

 	  // Serial COmms
 	  if (VCP_retrieveInputData(hexBuf,&pos32) != 0)
 	   	 {
 	   		 // you could do data processing here.
 	   		 //by demo, i just send it back to PC
 	   		 sprintf(strBuf,"%s", hexBuf);
 	   		 nullint8(hexBuf,255);
 	   		 // combine the new chars with the old
 	   		 sprintf(strOutBuf, "%s%s", SerialStr, strBuf);
 	   		 strcpy(SerialStr, strOutBuf);
 	   		 nullstr(strOutBuf,1024);
 	   		 nullstr(strBuf,1024);

 	   		 // loopback raw
 	   		 //sprintf(strOutBuf,"data ='%s' (%d)\n\r", strBuf, strlen(strBuf));
 	   		 //SendStrToUSB(strOutBuf);
 	   		 // loopback current cmd

 	   		 nullstr(strOutBuf, 1024);
 	   		 if (containsChar('\n', SerialStr) >= 0){
 	   			 HandleCmd();
 	   		 }
 	   	 }
 	   	 HAL_Delay(10);
 		  /* USER CODE END WHILE */
 	 }
    /* USER CODE END 3 */
  }


 uint8_t ReadInputs(){
 	uint8_t InputStatus = 0b00000000;
 	// Buttons
 	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)){
 		// delay
 		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)){
 			InputStatus |= 0b10000000;
 		}
 	}
 	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)){
 			// delay
 		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)){
 			InputStatus |= 0b01000000;
 		}
 	}
 	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)){
 			// delay
 		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)){
 			InputStatus |= 0b00100000;
 		}
 	}
 	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)){
 		// delay
 		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)){
 			InputStatus |= 0b00010000;
 		}
 	}
 	// Encoder Button
 	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){
 		// delay
 		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){
 			InputStatus |= 0b00001000;
 		}
 	}

 	// Encoder
 	int nextState = 0;
 	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15))
	{
		nextState = 0;
	}else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) && ! HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15))
	{
		nextState = 1;
	}else if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) && !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15))
	{
		nextState = 2;
	}else if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15))
	{
		nextState = 3;
	}

	int diff = (int)encoderState + (int)nextState;

	if (diff != 0){
/*
		 * sprintf(strOutBuf, "encoderState: %d\n\r", encoderState);
		SendStrToUSB(strOutBuf);
		nullstr(strOutBuf, 1024);

		sprintf(strOutBuf, "nextState: %d\n\r", nextState);
		SendStrToUSB(strOutBuf);
		nullstr(strOutBuf, 1024);
		sprintf(strOutBuf, "Diff: %d\n\r", diff);
		SendStrToUSB(strOutBuf);
		nullstr(strOutBuf, 1024);
*/

		encoderState = nextState;
/*
		sprintf(strOutBuf, "EncoderCount: %d\n\r", EncoderCount);
		SendStrToUSB(strOutBuf);
		nullstr(strOutBuf, 1024);
*/

		if (diff < 0) diff += 4;


		EncoderCount += diff;

		if (EncoderCount >= EncoderCounterLimit){
			EncoderCount = 0;
		}else if (EncoderCount < 0){
			EncoderCount = EncoderCounterLimit;
		}


		InputStatus &= 0b11111100;
		if (diff < 0){
			InputStatus |= 0b00000010;
		}else if (diff > 0){
			InputStatus |= 0b00000001;
		}
		nextState = 0;
 	 }
 	return InputStatus;
 }




  /* USER CODE BEGIN */
 /* COMMANDS
   * MEAS:VOLT:[X]
   * MEAS:CURR:[X]
   * CTRL:[X] <OFF|ON>
   * SET:VOLT [X]
   * OVP:[X]
   * */
  void HandleCmd(){
 	if (containsStr("MEAS:", SerialStr) >= 0){
 		 // Get Mode
 		 // 0 = Current
 		 // 1 = Voltage
 		 // 2 = Internal Ref
 		 int8_t Mode = -1;
 		 if (containsStr("VOLT:", SerialStr) == 5){
 			 Mode = 0;
 		 }else if (containsStr("CURR:", SerialStr) == 5){
 			 Mode = 1;
 		 }else if (containsStr("VREF", SerialStr) == 5){
 			 getVref(4000);
 			 nullstr(SerialStr, 1024);
 			 return;
 		 }else{
 			SendStrToUSB("INVALID COMMAND\r\nERROR:-3\r\n");
 			nullstr(SerialStr, 1024);
 			return;
 		 }
 		 sprintf(strOutBuf,"Mode: %d\n\r", Mode);
 		 SendStrToUSB(strOutBuf);

 		 // Output Line
 		 if (strlen(SerialStr) >= 11){
 			 uint8_t OutputChannel = SerialStr[10] - 48;
 			 if (OutputChannel >= 0 && OutputChannel <= 3){
 				 float value = Measure(Mode, OutputChannel);
 				 sprintf(strOutBuf, "%0.6f%s\r\n", value, Mode == 1 ? "A" : "V");
 				 SendStrToUSB(strOutBuf);
 			 }else{
 				SendStrToUSB("INVALID COMMAND\r\nERROR:-1\r\n");
 				nullstr(SerialStr, 1024);
 				return;
 			 }
 		 }else{
 			SendStrToUSB("INVALID COMMAND\r\nERROR:-2\r\n");
 			nullstr(SerialStr, 1024);
 			return;
 		 }

 	 }

 	if (containsStr("CTRL:", SerialStr) >= 0){
 		 if (strlen(SerialStr) >= 6){
 			 // Channel
 			uint8_t OutputChannel = SerialStr[5] - 48;
 			 if (!(OutputChannel >= 0 && OutputChannel <= 3)){
 				 SendStrToUSB("INVALID COMMAND\r\nERROR:-7\r\n");
 					nullstr(SerialStr, 1024);
 				 return;
 			 }
 			 // New State
 			 uint8_t State = 0;
 			 if (containsStr(" ON", SerialStr)){
 				 State = 1;
 			 }else if (containsStr(" OFF", SerialStr)){
 				 State = 0;
 			 }else{
 				 SendStrToUSB("INVALID COMMAND\r\nERROR:-9\r\n");
 					nullstr(SerialStr, 1024);
 				 return;
 			 }

 			 Output(OutputChannel, State);

 		 }else{
 			SendStrToUSB("INVALID COMMAND\r\nERROR:-8\r\n");
 			nullstr(SerialStr, 1024);
 			return;
 		 }

 	 }

 	if (containsStr("SET:VOLT", SerialStr) >= 0 && strlen(SerialStr) > 10){
 		nullstr(strOutBuf, 1024);
 		char *start =  &SerialStr[9];
 		char *end =  &SerialStr[14];
 		char *substr = (char *)calloc(1, end - start + 1);
 		memcpy(substr, start, end - start);
 		float TargetVoltage = atof(substr);
 		sprintf(strOutBuf, "Set to %s V\r\n", substr);
		SendStrToUSB(strOutBuf);
 		nullstr(strOutBuf, 1024);

 		float ActualVoltage = Measure(0, Vadj);

 		uint16_t CurrentPWM = 0;
 		int PWMIncrement = TargetVoltage > ActualVoltage ? 10 : -10;
 		while(ActualVoltage < TargetVoltage && CurrentPWM < 1000){
 			CurrentPWM += PWMIncrement;
 			PWM(CurrentPWM);
 			ActualVoltage = Measure(0, Vadj);
 			float VREF_plus = (float)1.212 * ((float)*getVrefCalData(0) / (float)*getVrefCalData(1));
 		 	float Vref = (VREF_plus / (float)getVref(4000)) * 4095;
 		 	float OutputVoltage = ((Vref / 4095) * CurrentPWM);
 	 		sprintf(strOutBuf, "DAC: %d / %0.3f V - Vadj: %0.3f V\r\n", CurrentPWM, OutputVoltage, ActualVoltage);
 			SendStrToUSB(strOutBuf);
 	 		nullstr(strOutBuf, 1024);
 		}




 	}
  }
void PWM(uint16_t value)
{
	TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void nullstr(char* str, uint16_t size){
for (int i = 0; i < size; i++){
	str[i] = '\0';
}
}
void nullint8(uint8_t* pointer, uint16_t size){
for (int i = 0; i < size; i++){
	pointer[i] = 0;
}
}

void Output(uint8_t Channel, uint8_t State){
GPIO_TypeDef* port;
uint8_t pin = 0;
switch(Channel){
	case Vadj:
		port = RelayPort0;
		pin = RelayPin0;
		break;
	case TwelveV:
		port = RelayPort1;
		pin = RelayPin1;
		break;
	case FiveV:
		port = RelayPort2;
		pin = RelayPin2;
		break;
	case ThreeV3:
		port = RelayPort3;
		pin = RelayPin3;
		break;
	default:
		SendStrToUSB("INVALID COMMAND\r\nERROR:-10\r\n");
		return;
}
HAL_GPIO_WritePin(port, pin, State);
char output[255];
sprintf(output, "Set Pin: %d to %d\r\n", pin, State);
SendStrToUSB(output);
}

float getVref(uint16_t Samples){


ADC_ChannelConfTypeDef sConfig = {0};

sConfig.Channel = ADC_CHANNEL_VREFINT;
sConfig.Rank = ADC_REGULAR_RANK_1;
sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
sConfig.SingleDiff = ADC_SINGLE_ENDED;
sConfig.OffsetNumber = ADC_OFFSET_NONE;
sConfig.Offset = 0;

float adcAvg = 0UL;
	for(uint16_t i = 0; i < Samples; i++){

		// Take reading
		// Start ADC Conversion
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		if (HAL_ADC_Start(&hadc1) != HAL_OK)
		{
			Error_Handler();
		}
		 // Wait for conversion to complete
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

		// Get ADC reading
		// Assuming Vref is 3.3V and 12-bit resolution

		adcAvg += (float)HAL_ADC_GetValue(&hadc1);

	}
	adcAvg /= Samples;


//sprintf(strOutBuf, "Reference Voltage: %0.6f \r\n", (float)adcAvg);
//SendStrToUSB(strOutBuf);
return adcAvg;
}
// gets the calibration data from the memory location selected by Index 0 to 1
volatile uint32_t * getVrefCalData(int Index){
volatile uint32_t * refAddress = ( volatile uint32_t *)(Index == 0 ? 0x1FFF75AA : 0x1FFF75AB);
//sprintf(strOutBuf,"VrefCal(%d): %ld\n\r", Index, *refAddress);
//SendStrToUSB(strOutBuf);
//nullstr(strOutBuf, 1014);
return refAddress;
}


float Measure(int8_t Mode, uint8_t Channel)
{

// Set ADC Channel
ADC_ChannelConfTypeDef sConfig = {0};

// Set Channel
// 0 = Voltage
// 1 = Current
uint32_t adcChannel;
ADC_HandleTypeDef* hadc;
//sprintf(strOutBuf,"Channel: %d\n\r", Channel);
//SendStrToUSB(strOutBuf);
switch(Mode){
	case 0:
		switch(Channel){
			case Vadj:
				// B12
				adcChannel = ADC_CHANNEL_11;
				hadc = &hadc1;
				break;
			case TwelveV:
				// B1
				adcChannel = ADC_CHANNEL_12;
				hadc = &hadc1;
				break;
			case FiveV:
				// B14
				adcChannel = ADC_CHANNEL_15;
				hadc = &hadc1;
				break;
			case ThreeV3:
				// B11
				adcChannel = ADC_CHANNEL_14;
				hadc = &hadc2;
				break;
			default:
				SendStrToUSB("INVALID COMMAND\r\nERROR:-4\r\n");
				return -4;
		}
		break;
	case 1:
		switch(Channel){
			case Vadj:
				// C4
				adcChannel = ADC_CHANNEL_5;
				hadc = &hadc2;
				break;
			case TwelveV:
				// B2
				adcChannel = ADC_CHANNEL_12;
				hadc = &hadc2;
				break;
			case FiveV:
				// B0
				adcChannel = ADC_CHANNEL_15;
				hadc = &hadc1;
				break;
			case ThreeV3:
				// A7
				adcChannel = ADC_CHANNEL_4;
				hadc = &hadc2;
				break;
			default:
				SendStrToUSB("INVALID COMMAND\r\nERROR:-5\r\n");
				return -5;
		}
		break;
	default:
		SendStrToUSB("INVALID COMMAND\r\nERROR:-6\r\n");
		return -6;
}

sConfig.Channel = adcChannel;
sConfig.Rank = ADC_REGULAR_RANK_1;
sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
sConfig.SingleDiff = ADC_SINGLE_ENDED;
sConfig.OffsetNumber = ADC_OFFSET_NONE;
sConfig.Offset = 0;

// Take reading
// Start ADC Conversion
// 4k Samples
float adcAvg = 0UL;
for(uint16_t i = 0; i < 4000; i++){

	// Take reading
	// Start ADC Conversion
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_ADC_Start(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	 // Wait for conversion to complete
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

	// Get ADC reading

	adcAvg += HAL_ADC_GetValue(&hadc1);

}

adcAvg /= 4000;

//sprintf(strOutBuf,"adcAvg: %0.6f \n\r", adcAvg);
//SendStrToUSB(strOutBuf);
//nullstr(strOutBuf, 1024);

// Convert ADC reading to voltage/current

float VREF_plus = (float)1.212 * ((float)*getVrefCalData(0) / (float)*getVrefCalData(1));
float Vref = (VREF_plus / (float)getVref(4000)) * 4095;
float ChannelVoltage = ((Vref / 4095) * adcAvg); // + (float)RefCalOffset; // Assuming Vref is 3.3V and 12-bit resolution
//float VoltageReference = ((float)1.212 * getVref(4000) / *getVrefCalData(0));

//sprintf(strOutBuf,"Voltage Reference: %0.6f \n\r", VREF_plus);
//SendStrToUSB(strOutBuf);
//nullstr(strOutBuf, 1024);

//Channel Voltage = Vref+_Charac * VREFINT_CAL * ADC_DATA / ( VREFINT_DATA * FULL_SCALE )

// Convert ADC reading to voltage/current
//sprintf(strOutBuf,"Calibrated Reference Voltage: %0.6f\n\r", Vref);
//SendStrToUSB(strOutBuf);
//nullstr(strOutBuf, 1024);

// Convert ADC reading to voltage/currentVREF_plus
//sprintf(strOutBuf,"Calibrated Channel Voltage(ref): %0.6f\n\r", ChannelVoltage);
//(strOutBuf);
//nullstr(strOutBuf, 1024);
// Convert to Current reading or attenuated voltage reading
float Reading = 0;
switch(Mode){
	case 0:
		switch(Channel){
			case Vadj:
			case TwelveV:
				// B12
				// B1
				Reading = ChannelVoltage * ( 33.6 + 9.72 ) / 9.72;
				break;
			case FiveV:
				// B14
				Reading = ChannelVoltage * ( 15 + 10 ) / 10;
				break;
			case ThreeV3:
				// B11
				Reading = ChannelVoltage * ( 5 + 10 ) / 10;
				break;
			default:
				SendStrToUSB("INVALID COMMAND\r\nERROR:-11\r\n");
				return -11;
		}
		break;
	case 1:
		// C4
		// B2
		// B0
		// A7
		Reading = ChannelVoltage / (2500 * 0.01);
		break;
	default:
		SendStrToUSB("INVALID COMMAND\r\nERROR:-12\r\n");
		return -12;
}

return Reading;
}
ssize_t containsChar(char needle, const char *haystack)
{
 if (!haystack)
	 return -1;

 const char *needle_in_haystack = strchr(haystack, needle);
 return needle_in_haystack ? needle_in_haystack - haystack : -1;
}
ssize_t containsStr(const char * needle, const char *haystack)
{
 char *needle_in_haystack;
 if(!needle || !haystack) return -1;
 needle_in_haystack = strstr(haystack, needle);
 return needle_in_haystack ? needle_in_haystack - haystack : -1;
}

void SendStrToUSB(char* buf){
char outbuffer[4096];
strcpy (outbuffer, buf);
CDC_Transmit_FS((uint8_t *)outbuffer, (uint16_t)strlen(outbuffer));

nullstr(outbuffer,4096);
}

static void MX_USB_OTG_FS_Init(void)
{
hUsbDeviceFS.pData = NULL;
hUsbDeviceFS.pClassData = NULL;
hUsbDeviceFS.pClass = NULL;
hUsbDeviceFS.dev_state = USBD_STATE_DEFAULT;
hUsbDeviceFS.id = 204;

/* Init Device Library, add supported class and start the library. */
if (USBD_Init(&hUsbDeviceFS, &CDC_Desc, DEVICE_FS) != USBD_OK)
{
 Error_Handler();
}
if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
{
 Error_Handler();
}
if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
{
 Error_Handler();
}
if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
{
 Error_Handler();
}
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 18;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_6|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_8
                          |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC14 PC6 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_6|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC15 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA3 PA5 PA8
                           PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_8
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
