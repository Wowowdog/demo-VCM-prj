/* USER CODE BEGIN Header */
/**
  ******************************************************************************
	this is 15501 ctrl script based on 15501fra02, which is based on 15501fra02, 195fra_01, 195fra_02 and 195fra_04 code that has done:
	(1)add new complex function based on Measuring Frequency Response by Tim Wescott
	(2)modify delay time in myiic.c/IIC_Send_Byte, to shorten time required
	(3)add close loop fra script
	(4)has right multple ADC settings
	
	this code is primar to 
	 - do multiple sample OLM fra simutenously (TESTMODE 3)
	 - do /tune multiple sample CLM fra simutenously (TESTMODE 2)
	
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "key.h"
#include "delay.h"
#include "myiic.h"
#include "usbd_cdc_if.h"
#include "current.h"
#include "ois195y.h"
#include "functions.h"
#include "string.h"
#include <complex.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define UARTENABLE     1
#define CYCLE          25             // FRA sine wave cycle per freq.
#define FREQPTS        80
                       // Frequency points of FRA test
uint8_t   k_stamp=0   ;              // number use to ID the signal is kth freq. signal
uint16_t  amplitu[20]= {100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000};
const float  timer_inSec_offset   =0;  // time before activating PWM
const float  timer_inSec_period   =70; // duration of PWM
const float  freq_start           =1;  // FRA start freq (Hz)
const float  freq_stop            =100;// FRA stop freq (Hz)
float        timer_inSec_previous =0;
float       *ptimer_inSec_previous = &timer_inSec_previous;
float    phase        =0;
float   *pphase       =&phase;
float    Curr           ;
float    Curr2          ;
float phaseAcc          ;
float setpoint          ;
float setpoint2         ;
float *psetpoint = &setpoint;
uint32_t timer_ini      ;
float  freqHz[FREQPTS]  ;      
uint32_t timer          ;
float    timer_inSec =0 ;
uint16_t OMRON_ADC      ;
uint16_t HALL_ADC       ;
float    OMRON_ADC_filter        ;
float    HALL_ADC_filter         ;
int    iteration                 ;
float delta_error             =0 ;
float *pdelta_error=&delta_error;
float yinPre                     ;
float youtPre                    ;
uint16_t  k_stamp2[50] ={0}      ; // number used to ID the signal is kth signal in complex response calculation
uint8_t initialized[50]={0}      ; // flag value used to control initial code in complexResponse2 function 

/*complex number for fra*/
double complex Xw[50];
double complex Yw[50];
float Xw_real[50];
float Xw_imag[50];
float Yw_real[50];
float Yw_imag[50];

/*UART Tx*/
char buf[800];
char buf_float_1[80];
char buf_float_2[80];
char buf_float_3[80];
char buf_float_4[80];
char buf_float_5[80];
char buf_float_6[80];

/*USB Tx*/
u8 sendData[100];
u8 sendData2[100];
u8 cData[2]={0}         ;       // buf for hex2bin convert
u8 cData2[2]={0}         ;       // buf for hex2bin convert
u8 currP[2]={0x00,0x80};
u8 currN[2]={0x03,0x80};
//u8 currP[2]={0x00,0x00};
//u8 currN[2]={0x03,0xff};
uint16_t ADC_value_1=0;
uint16_t ADC_value_2=0;
uint16_t ADC_value[2]   ={0};
uint16_t ADC_value2[2]  ={0};
float ADCvalue_filter =0; 
void motorrun(void);
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
  //KEY_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */
	KEY_Init();
	IIC_Init();
	CURR_Init();
	LED_Init();
//TIM3_Init(399,499);
	rtU.in=500;
  rtU.in_o=800;
  rtU.in_b=1000;
//	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
//	sinroute();
//	KEY_Scan();
//  my_RxBuf[3]=0x3f;
//	my_RxBuf[4]=0x01;
//	my_RxBuf[7]=0x32;
//	balltest();

//IIC_Write_Arr(0,0x18,0x03,2,curr00);
//IIC_Write_Arr(3,0x18,0x03,2,currff);
//IIC_Write_Arr(6,0x18,0x03,2,currff);
HAL_DAC_Start(&hdac1,DAC1_CHANNEL_1);
HAL_DAC_SetValue(&hdac1,DAC1_CHANNEL_1,DAC_ALIGN_12B_R,1000 );
//HAL_DAC_SetValue(&hdac1,DAC1_CHANNEL_1,DAC_ALIGN_12B_R,1875 );
	
	timer_ini = GetMicrosFromISR();
  FraFreq(freqHz,freq_start,freq_stop,FREQPTS,2);
	//recive_flag =1;
	//my_RxBuf[2]=0x04;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
			for (int i=0;i<(sizeof(freqHz)/sizeof(freqHz[0]));i++) 
			{
				/*reset initial timer*/
				timer_ini = GetMicrosFromISR();
				timer = GetMicrosFromISR_Reset(&timer_ini);
				timer_inSec = GetSecFromISR_Reset(&timer);	
				//*initialized=0;
				
					while(timer_inSec<1000)
					{
					/*timer*/
					timer = GetMicrosFromISR_Reset(&timer_ini);
					timer_inSec = GetSecFromISR_Reset(&timer);	
					/*output, ADC sample*/
					readADC_multiChannel(&hadc1, ADC_value,2); 
					ADC_value_1 = ADC_value[0];
					ADC_value_2 = ADC_value[1];	 
					//readADC_multiChannel(&hadc2, ADC_value2,1); 
						
					#if TESTMODE==1
					/*input*/
					//Curr = discreteFreqSignal_inTime(CHRIP_AMPLITUDE_OLM, freqHz[i],1,timer_inSec,&k_stamp)/2; // dividing 2 for 2ea DW9800
					Curr = 40*sin(2*M_PI*freqHz[i]*timer_inSec);
					Curr2CodeDw9800(Curr,cData);
					IIC_Write_Arr(0,0x18,0x03,2,cData); // (0,1),(3,4),(6,7) on the bridge, (8,9) on board
					IIC_Write_Arr(1,0x18,0x03,2,cData); 
						
					/*cal. FRA G(w)=Y(w)/X(w)*/
					//if(timer_inSec>2){complexResponse(&Xw[i],&Yw[i],freqHz[i], Curr, ADC_value, &yinPre, &youtPre, timer_inSec-1, 0.001, &k_stamp2, 1);}
					//complexResponse(&Xw[i],&Yw[i],freqHz[i], Curr, ADC_value, &yinPre, &youtPre, timer_inSec, 0.001, &k_stamp2, 1);
					complexResponse2(&Xw[i],&Yw[i],freqHz[i], Curr, ADC_value, &yinPre, &youtPre, timer_inSec, 0.001, &k_stamp2[i], 10, &initialized[i]);
					//if(timer_inSec>1){complexResponse(&Xw[i],&Yw[i],freqHz[i], Curr, ADC_value, &yinPre, &youtPre, timer_inSec, 0.001, &k_stamp2,2);}

				  /*send real time data*/
					float2str(buf_float_1,&Curr);

					sprintf (sendData, "%lu,%s,%d\r\n", timer, buf_float_1,ADC_value);

					//sprintf (sendData, "%lu,%s,%d\r\n", timer, buf_float_1,ADC_value);
					CDC_Transmit_HS(sendData,strlen(sendData));		
				  }
						
					#elif TESTMODE==2	
										/*input*/
					//Curr = discreteFreqSignal_inTime(CHRIP_AMPLITUDE_OLM, freqHz,1,timer_inSec,&k_stamp)/2; // dividing 2 for 2ea DW9800
					setpoint = ADCMIDDLE +2000*sin(2*M_PI*freqHz[i]*timer_inSec);
					setpoint2= ADCMIDDLE2+2000*sin(2*M_PI*freqHz[i]*timer_inSec);
  	  	  Curr     = PID_Discrete(setpoint,ADC_value[0],1.0f,timer_inSec);	
					Curr2    = PID_Discrete2(setpoint2,ADC_value[1],1.0f,timer_inSec);	
					Curr2CodeDw9800(Curr,cData);
					Curr2CodeDw9800(Curr2,cData2);
					IIC_Write_Arr(0,0x18,0x03,2,cData); // (0,1),(3,4),(6,7) on the bridge, (8,9) on board
					//IIC_Write_Arr(1,0x18,0x03,2,cData); 	
					IIC_Write_Arr(3,0x18,0x03,2,cData2); // (0,1),(3,4),(6,7) on the bridge, (8,9) on board
					//IIC_Write_Arr(4,0x18,0x03,2,cData); 
					
					/*cal. FRA G(w)=Y(w)/X(w)*/
					//if(timer_inSec>2){complexResponse(&Xw[i],&Yw[i],freqHz[i], Curr, ADC_value, &yinPre, &youtPre, timer_inSec-1, 0.001, &k_stamp2, 1);}
					//complexResponse(&Xw[i],&Yw[i],freqHz[i], setpoint, ADC_value, &yinPre, &youtPre, timer_inSec, 0.001, &k_stamp2, 2);
					//if(timer_inSec>1){complexResponse(&Xw[i],&Yw[i],freqHz[i], Curr, ADC_value, &yinPre, &youtPre, timer_inSec, 0.001, &k_stamp2,2);}
						
				  /*send real time data*/
					float2str(buf_float_1,&setpoint);
					//sprintf (sendData, "%lu,%s,%d\r\n", timer, buf_float_1,ADC_value);
					//sprintf (sendData, "%lu,%s,%d,%d\r\n", timer, buf_float_1,ADC_value,(int)ADCvalue_filter);
					//sprintf (sendData, "%lu,%s,%d,%d,\r\n", timer, buf_float_1,ADC_value[0],ADC_value[1]);
					sprintf (sendData, "%lu\r\n", timer);
					CDC_Transmit_HS(sendData,strlen(sendData));		
					
					RecieEven();
				  }						
					#elif TESTMODE==3

					/*input*/
					//Curr = discreteFreqSignal_inTime(CHRIP_AMPLITUDE_OLM, freqHz[i],1,timer_inSec,&k_stamp)/2; // dividing 2 for 2ea DW9800
					Curr = 80*sin(2*M_PI*freqHz[i]*timer_inSec);
					Curr2CodeDw9800(Curr,cData);

					//IIC_Write_Arr(0,0x18,0x03,2,cData); // (0,1),(3,4),(6,7) on the bridge, (8,9) on board
				  IIC_Write_Arr(1,0x18,0x03,2,cData); 
					IIC_Write_Arr(3,0x18,0x03,2,cData); 

					/*cal. FRA G(w)=Y(w)/X(w)*/
					//if(timer_inSec>2){complexResponse(&Xw[i],&Yw[i],freqHz[i], Curr, ADC_value, &yinPre, &youtPre, timer_inSec-1, 0.001, &k_stamp2, 1);}
					//complexResponse(&Xw[i],&Yw[i],freqHz[i], Curr, ADC_value, &yinPre, &youtPre, timer_inSec, 0.001, &k_stamp2, 1);
				  complexResponse3(&Xw[i],&Yw[i],freqHz[i], Curr, ADC_value[0], &yinPre, &youtPre, timer_inSec, 0.001, &k_stamp2[i], 1, &initialized[i]);
					//if(timer_inSec>1){complexResponse(&Xw[i],&Yw[i],freqHz[i], Curr, ADC_value, &yinPre, &youtPre, timer_inSec, 0.001, &k_stamp2,2);}

				  /*send real time data*/
					float2str(buf_float_1,&Curr);
					sprintf (sendData, "%lu,%s,%d,%d\r\n", timer, buf_float_1,ADC_value[0],ADC_value[1]);
					//sprintf (sendData, "%lu,%s,%d,%d,%d\r\n", timer, buf_float_1,ADC_value[0],ADC_value[1],ADC_value2[0]);
					//sprintf (sendData, "%lu,%s,%d,%d\r\n", timer, buf_float_1,ADC_value[0],ADC_value[1]);
					//sprintf (sendData, "%lu,%s,%d\r\n", timer, buf_float_1,ADC_value);
					CDC_Transmit_HS(sendData,strlen(sendData));	
					
					RecieEven();
				  }
										
					#elif TESTMODE==4
					/*input*/
					//Curr = discreteFreqSignal_inTime(CHRIP_AMPLITUDE_OLM, freqHz[i],1,timer_inSec,&k_stamp)/2; // dividing 2 for 2ea DW9800
					if (timer_inSec<1){Curr=50*sin(Curr=2*M_PI*0.5*timer_inSec);}else{Curr=0;}
					Curr2CodeDw9800(Curr,cData);
					IIC_Write_Arr(0,0x18,0x03,2,cData); // (0,1),(3,4),(6,7) on the bridge, (8,9) on board
					IIC_Write_Arr(1,0x18,0x03,2,cData); 
						
					/*cal. FRA G(w)=Y(w)/X(w)*/
					//if(timer_inSec>2){complexResponse(&Xw[i],&Yw[i],freqHz[i], Curr, ADC_value, &yinPre, &youtPre, timer_inSec-1, 0.001, &k_stamp2, 1);}
					complexResponse(&Xw[i],&Yw[i],freqHz[i], Curr, ADC_value, &yinPre, &youtPre, timer_inSec, 0.001, &k_stamp2, 2);
					//if(timer_inSec>1){complexResponse(&Xw[i],&Yw[i],freqHz[i], Curr, ADC_value, &yinPre, &youtPre, timer_inSec, 0.001, &k_stamp2,2);}

				  /*send real time data*/
					float2str(buf_float_1,&Curr);
					sprintf (sendData, "%lu,%s,%d\r\n", timer, buf_float_1,ADC_value);
					//sprintf (sendData, "%lu,%s,%d\r\n", timer, buf_float_1,ADC_value);
					CDC_Transmit_HS(sendData,strlen(sendData));		
				  }
					
					
					#endif	

				/*register complex result*/
				Xw_real[i] = crealf(Xw[i]);
				Xw_imag[i] = cimagf(Xw[i]);
				Yw_real[i] = crealf(Yw[i]);
				Yw_imag[i] = cimagf(Yw[i]);
				RecieEven();
			}
			
			/*send result data*/	
			for (int i=0;i<(sizeof(freqHz)/sizeof(freqHz[0]));i++) 
			{
				/*clean buffer value*/
				memset(buf_float_2, 0, sizeof(buf_float_2));memset(buf_float_3, 0, sizeof(buf_float_3));memset(buf_float_4, 0, sizeof(buf_float_4));memset(buf_float_5, 0, sizeof(buf_float_5));
				
				float2str(buf_float_2,&Xw_real[i]);
				float2str(buf_float_3,&Xw_imag[i]);
				float2str(buf_float_4,&Yw_real[i]);
				float2str(buf_float_5,&Yw_imag[i]);
				float2str(buf_float_6,&freqHz[i]);
				
				sprintf(sendData2, "%s,%s,%s,%s,%s\r\n", buf_float_2, buf_float_3,buf_float_4,buf_float_5,buf_float_6);
				//HAL_UART_Transmit(&huart2,(uint8_t*)&buf,strlen(buf),100);
				CDC_Transmit_HS(sendData2,strlen(sendData2));	
			}
			
			/*clean buffer value*/
			memset(Xw, 0, sizeof(Xw));memset(Yw, 0, sizeof(Yw));memset(Xw_real, 0, sizeof(Xw_real));memset(Yw_real, 0, sizeof(Yw_real));memset(Xw_imag, 0, sizeof(Xw_imag));memset(Yw_imag, 0, sizeof(Yw_imag));
				
			recive_flag=1;
			my_RxBuf[2]=0x04;
			RecieEven();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
  }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 16;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void convertUnsignedShort2u8(unsigned short nValue, u8* cData)
{
//	cData[0] = (nValue >> 8) & 0xff;
//	cData[1] = nValue & 0xff;
	*cData = (nValue >> 8) & 0xff;
	*(cData+1) = nValue    & 0xff;
}

void Curr2CodeDw9800(float inputCurr,u8 dataBuf[])
{
		if (inputCurr>0)
			{
		uint16_t curr_code = inputCurr*511/100; 
		curr_code = (curr_code>511)?511:curr_code;
		convertUnsignedShort2u8(curr_code, dataBuf);
		*dataBuf = *dataBuf | 0x02;               // for positive current, 10th bit is 1
			} 
		else
			{
		uint16_t curr_code = 512+inputCurr*511/100; 
		curr_code = (curr_code<0)?0:curr_code;
		convertUnsignedShort2u8(curr_code, dataBuf);
		*dataBuf = *dataBuf | 0x00;               // for negative current, 10th bit is 0
			}	
}

void RecieEven(void)
{
		if(recive_flag==1)
		{
		recive_flag=0;
		switch(my_RxBuf[2])
		{
		case 0x04: while(my_RxBuf[2]==0x04){HAL_Delay(2000);}; // wait here until new my_RxBuf[2] != 04
		case 0x05: timer_ini = GetMicrosFromISR();           
		}
		}
}	

void RecieEven2(void)
{
		if(recive_flag==1)
		{
			recive_flag=0;
			switch(my_RxBuf[2])
			{           
        case 0x04:IIC_Write_Arr(8,0X18,0X03,2,my_RxBuf+3);break;
				case 0x05:IIC_Write_Arr(9,0X18,0X03,2,my_RxBuf+3);break;
				case 0x06:ad_conv();break;
				case 0x07:ad_conv();break;
				case 0x11:IIC_Write();break;
				case 0x12:IIC_Read();break;
   			case 0x13:IICboardcast();break;
				case 0x14:CL_AF_CURR();break;
				case 0x15:CL_OIS_CURR();break;
				case 0x16:IIC_Read_GT24(0X1212,1);CDC_Transmit_HS(my_iic,1);break;
				case 0x17:balltest();break;
				case 0x19:sinroute();break;
				case 0x1b:hall_dect_x2();break;
				case 0x20:hall_dect_x1();break;
				case 0x30:hall_dect_y();break;
				case 0x40:hall_dect_x1();break;
				default:break;
			}
		}
		KEY_Scan();
}	
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

