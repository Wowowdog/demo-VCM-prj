#include "main.h"
#include "functions.h"

///*controller*/
float sensed_output  =0;
float control_signal =0;
float control_signal_discrete[2] ={0,0};

float last_sensed    =0;
float last_time_inSec=0;
float error;
float error_array[3]  ;
float error_array_filter[3];
float total_error;
float last_error;
const float max_control    = 2048;
const float min_control    =-2048;

int   k_pre           ;
float t_pre           ;

/*filter*/
#if FILPARAM ==1             // order 2, Fs = 100, dB = -80
	float    bk[3]        ={0.000130587224717315,	-0.000135656915371369,	0.00013058722717315};
	float    ak[2]        ={-1.98409406526296,	0.984219582797028};
	float    inputBuf[3];
	float    outputBuf[2];
	float    inputBuf_2[3];
	float    outputBuf_2[2];
	size_t   nb           =3;
	size_t   na           =2;



#elif FILPARAM ==2           // chebyshev 2, order 2,Fstop 100Hz, dB = -80, ts = 350us
	double    bk[3]        ={0.01987868763704398,0.03640216477384969,0.01987868763704398};
	double    ak[2]        ={-1.634879045899268,0.7203314553042037};
	double    inputBuf[3];
	double    outputBuf[2];
	float    inputBuf_2[3];
	float    outputBuf_2[2];
	size_t   nb           =3;
	size_t   na           =2;


#elif FILPARAM ==3           // order 10, Fpass = 20Hz, Fstop 25Hz, dB = -40
	float    bk[4]        ={0.0018187,-0.0017805,-0.0017805,0.0018187};
	float    ak[3]        ={-2.9151,2.8338,-0.91862};
	float    inputBuf[4];
	float    outputBuf[3];
	size_t   nb           =4;
	size_t   na           =3;



#elif FILPARAM ==4           // order 1, Fpass = 20Hz, Fstop 25Hz, dB = -40
	float    bk[3]        ={0.015503910683361,0.027570087324856,0.015503910683361};
	float    ak[2]        ={-1.686004274144930,1.686004274144930};
	float    inputBuf[3];
	float    outputBuf[2];
	size_t   nb           =3;
	size_t   na           =2;
#elif FILPARAM ==5           // order 10, Fpass = 20Hz, Fstop 25Hz, dB = -40
	float    bk[2]        ={0.366025403784439,0.366025403784439};
	float    ak[1]        ={0.267949192431123};
	float    inputBuf[2];
	float    outputBuf[1];
	size_t   nb           =2;
	size_t   na           =1;
#endif

#if PIDPARAM ==1
	float Kp         =1;     // kp = 0.56 is the parameter with 45d PM, kp = 10 is for test
#elif PIDPARAM ==2           // this setup has nice tracking at lower frequency, but has no extra gain and very unstable (PM 5deg)
	float Kp        = 0.3;   // this one is decent with no hunting phonamena, follows kpkd > ki rule
	float Ki        = 0.0299;
	float Kd        = 0.1;
#elif PIDPARAM ==3           // this one is based on above with Ki slightly greater, for more gain
	float Kp        = 0.3;
	float Ki        = 0.05;
	float Kd        = 0.1;
#elif PIDPARAM ==4           // this is got 20dB gain at 10Hz
	float Kp        =0.149;
	float Ki        = 4.4;
	float Kd        = 0.0749;
#elif PIDPARAM ==5           // this is for test,
	float Kp        = 0.231;
	float Ki        = 31.8 ;
	float Kd        = 0.00457;
#elif PIDPARAM ==11           // this is for test,
	float Kp        =0.0149;
	float Ki        = 0.44;
	float Kd        = 0.00749;
#endif
/******************************************************************************
* Function Name: PID_Discrete
* Description :
* the difference eq is: u(n) = u(n-1)+du, du = kp{e(n)-e(n-1)+ dt/Ti*e(n) + Td/dt*[e(n)-2*e(n-1)+e(n-2)]      }
* this eq is derived form u(t) = kp*[e(t) + 1/Ti int(e(t)) + Td d/dt(e(t))]
* if PID coef. is derived from Matlab, it is the form u(t) = kp*e(t) + ki int(e(t)) + kd d/dt(e(t))
* so Ti = kp/ki , Td = kd/kp
* Arguments :
* Return Value :
******************************************************************************/
float PID_Discrete(float setpoint,float y,float adjustGain, float timer_inSec)
{
  float control_output_increment;
  float current_time = timer_inSec; //returns the number of milliseconds passed since the Arduino started running the program
  float Ti           = Kp/Ki;
  float Td           = Kd/Kp;
  #if PIDMODE ==2 ||3
  float delta_time = current_time - last_time_inSec; //delta time interval
  #endif

	sensed_output = y;  //measured PV
	float target = setpoint;
	error_array[0] = target - sensed_output;
//    total_error += error; //accumulates the error - integral term
//    delta_error = error - last_error; //for derivative on error
//    *pdelta_error = -1*(sensed_output - last_sensed); //for derivative on PV

	 /*anti wind-up, the algorithm follows Bisoffi: Reset integral control for improved settling of PID-based motion systems with friction*/



  #if PIDMODE ==1
	control_signal = Kp*adjustGain*error;
  #elif PIDMODE ==2
	control_signal = Kp*adjustGain*error + Ki*adjustGain*delta_time*total_error;
  #elif PIDMODE ==3
	control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]+error_array[0]*delta_time/Ti+Td*(error_array[0]-2*error_array[1]+error_array[2])/delta_time);
  #elif PIDMODE ==4
	error_array_filter[0] = 0.9*error_array_filter[1]+0.1*error_array[1];
	control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]+error_array[0]*delta_time/Ti+Td*(error_array_filter[0]-2*error_array_filter[1]+error_array_filter[2])/delta_time);

		control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]+error_array[0]*delta_time/Ti+Td*(error_array_filter[0]-2*error_array_filter[1]+error_array_filter[2])/delta_time);
  #endif
	control_signal_discrete[0] = control_signal_discrete[1] +control_output_increment;

	if (control_signal_discrete[0]>= max_control) control_signal_discrete[0] = max_control;
	else if (control_signal_discrete[0] <= min_control) control_signal_discrete[0] = min_control;

	error_array[2] = error_array[1];
	error_array[1] = error_array[0];
	control_signal_discrete[1] = control_signal_discrete[0];
	last_time_inSec = current_time;
  #if PIDMODE ==4
	error_array_filter[2]=error_array_filter[1];
	error_array_filter[1]=error_array_filter[0];
  #endif
	last_sensed = sensed_output; // for derivative on PV
	int8_t sign_error    = error_array[0]>0?1:(error_array[0] < 0?-1:0);
  return control_signal_discrete[0]+300*sign_error;
}

/******************************************************************************
* Function Name: PID_Discrete with anti_windup option (unfinished)
* Description :
* the difference eq is: u(n) = u(n-1)+du, du = kp{e(n)-e(n-1)+ dt/Ti*e(n) + Td/dt*[e(n)-2*e(n-1)+e(n-2)]      }
* this eq is derived form u(t) = kp*[e(t) + 1/Ti int(e(t)) + Td d/dt(e(t))]
* if PID coef. is derived from Matlab, it is the form u(t) = kp*e(t) + ki int(e(t)) + kd d/dt(e(t))
* so Ti = kp/ki , Td = kd/kp
* Arguments :
* Return Value :
******************************************************************************/
float PID_Discrete_AntiWindup(float setpoint,float y,float adjustGain, float timer_inSec)
{
  float control_output_increment;
  float current_time = timer_inSec; //returns the number of milliseconds passed since the Arduino started running the program
  float Ti           = Kp/Ki;
  float Td           = Kd/Kp;
  #if PIDMODE ==2 ||3
  float delta_time = current_time - last_time_inSec; //delta time interval
  #endif

	sensed_output = y;  //measured PV
	float target = setpoint;
	error_array[0] = target - sensed_output;
//    total_error += error; //accumulates the error - integral term
//    delta_error = error - last_error; //for derivative on error
//    *pdelta_error = -1*(sensed_output - last_sensed); //for derivative on PV

	 /*anti wind-up, the algorithm follows Bisoffi: Reset integral control for improved settling of PID-based motion systems with friction*/



  #if PIDMODE ==1
	control_signal = Kp*adjustGain*error;
  #elif PIDMODE ==2
	control_signal = Kp*adjustGain*error + Ki*adjustGain*delta_time*total_error;
  #elif PIDMODE ==3
	control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]+error_array[0]*delta_time/Ti+Td*(error_array[0]-2*error_array[1]+error_array[2])/delta_time);
  #elif PIDMODE ==4
	error_array_filter[0] = 0.9*error_array_filter[1]+0.1*error_array[1];
		#if ANTIWINDUP ==1
		int8_t sign_error    = error_array[0]>0?1:(error_array[0] < 0?-1:0);
		int8_t sign_velicity = (sensed_output-last_sensed)>0?1:((sensed_output-last_sensed) < 0?-1:0);
		int8_t sign_CO       = control_signal_discrete[0]>0?1:(control_signal_discrete[0] < 0?-1:0);
		if (sign_error*sign_CO<0 && sign_velicity*sign_CO<0)
			{
			float alpha = 0.5;
			float COi_increment = sign_error*300;
			control_output_increment = COi_increment+Kp*adjustGain*(error_array[0]-error_array[1]+Td*(error_array_filter[0]-2*error_array_filter[1]+error_array_filter[2])/delta_time);
			}
		else{
			control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]+error_array[0]*delta_time/Ti+Td*(error_array_filter[0]-2*error_array_filter[1]+error_array_filter[2])/delta_time);
			}
       #else
		control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]+error_array[0]*delta_time/Ti+Td*(error_array_filter[0]-2*error_array_filter[1]+error_array_filter[2])/delta_time);
   	   #endif
  #endif
	control_signal_discrete[0] = control_signal_discrete[1] +control_output_increment;

	if (control_signal_discrete[0]>= max_control) control_signal_discrete[0] = max_control;
	else if (control_signal_discrete[0] <= min_control) control_signal_discrete[0] = min_control;

	error_array[2] = error_array[1];
	error_array[1] = error_array[0];
	control_signal_discrete[1] = control_signal_discrete[0];
	last_time_inSec = current_time;
  #if PIDMODE ==4
	error_array_filter[2]=error_array_filter[1];
	error_array_filter[1]=error_array_filter[0];
  #endif
    last_sensed = sensed_output; // for derivative on PV

  return control_signal_discrete[0];
}

/******************************************************************************
* Function Name: iirFilter
* Description : shift signal and implement iir filter with defined coef.
* Arguments : input signal, input and output buffer, coefficients and sizes
* Return Value : filtered signal
******************************************************************************/
//float iirFilter(float input, float *inputBuf, float *outputBuf, float *ak, float *bk, size_t na,size_t nb){
//float iirFilter(float input, float *ak, float *bk, size_t na,size_t nb){
float iirFilter(float input){
	float sum_yn=0,sum_xn=0, res=0;
	//delay x[n]
	for (int i=0; i<nb; i+=1){
//		i == nb ? inputBuf[nb-i]=input : inputBuf[nb-i]=inputBuf[nb-1-i];
		if (i == nb-1){
			inputBuf[0]=input;
		}
		else {
			inputBuf[nb-i-1]=inputBuf[nb-i-2];
		}
	}

	//y[n] summation
	for (int i=0; i<na; i+=1){
//		float y_ = -1*ak[i]*outputBuf[i];
		float y_ = -1*ak[i]*outputBuf[i];
		sum_yn=sum_yn+y_;
	}

	//x[n] summation
	for (int i=0; i<nb; i+=1){
//		float x_ = bk[i]*inputBuf[i];
		float x_ = bk[i]*inputBuf[i];
		sum_xn=sum_xn+x_;
	}

	//%delay yn
	for (int i=0; i<na; i+=1){
//		i == na ? outputBuf[na-i]=sum_yn+sum_xn  : outputBuf[na-i]=outputBuf[na-1-i];
		if(i == na-1) {
			res = sum_yn+sum_xn;
			outputBuf[0]=res;
		}
		else {
			outputBuf[na-i-1]=outputBuf[na-i-2];
	}
  }
	return res;
}
/******************************************************************************
* Function Name: iirFilter_2
* Description : use double (16 decimal precision compared to 7 of float type ), to avoid instability by truncation, since many filters have poles very close unit circle
* Arguments : input signal, input and output buffer, coefficients and sizes
* Return Value : filtered signal
******************************************************************************/
double iirFilter_2(double input, double *inputBuf, double *outputBuf, double *ak, double *bk, size_t na,size_t nb){
	double sum_yn=0,sum_xn=0, res=0;

	//delay x[n]
	for (int i=0; i<nb; i+=1){
//		i == nb ? inputBuf[nb-i]=input : inputBuf[nb-i]=inputBuf[nb-1-i];
		if (i == nb-1){
			inputBuf[0]=input;
		}
		else {
			inputBuf[nb-i-1]=inputBuf[nb-i-2];
		}
	}

	//y[n] summation
	for (int i=0; i<na; i+=1){
//		float y_ = -1*ak[i]*outputBuf[i];
		double y_ = -1*ak[i]*outputBuf[i];
		sum_yn=sum_yn+y_;
	}

	//x[n] summation
	for (int i=0; i<nb; i+=1){
//		float x_ = bk[i]*inputBuf[i];
		double x_ = bk[i]*inputBuf[i];
		sum_xn=sum_xn+x_;
	}

	//%delay yn
	for (int i=0; i<na; i+=1){
//		i == na ? outputBuf[na-i]=sum_yn+sum_xn  : outputBuf[na-i]=outputBuf[na-1-i];
		if(i == na-1) {
			res = sum_yn+sum_xn;
			outputBuf[0]=res;
		}
		else {
			outputBuf[na-i-1]=outputBuf[na-i-2];
	}
  }
	return res;
}
/******************************************************************************
* Function Name: GetMicrosFromISR
* Description : micro-second timer from ISR interrupt
* Arguments : none
* Return Value : micro-second time stamp
******************************************************************************/
uint32_t GetMicrosFromISR()
{
    uint32_t st = SysTick->VAL;
    uint32_t pending = SCB->ICSR & SCB_ICSR_PENDSTSET_Msk;
    uint32_t ms = HAL_GetTick();
    if (pending == 0)
        ms++;
    return ms * 1000 - st / ((SysTick->LOAD + 1) / 1000);
}
/******************************************************************************
* Function Name: GetMicrosFromISR_Reset
* Description : micro-second timer from ISR interrupt, start at zero initially
* Arguments : (p) timer variable, (p)initial time stamp
* Return Value : nona
******************************************************************************/
uint32_t GetMicrosFromISR_Reset(uint32_t *iniTime)
{
	return (GetMicrosFromISR()- *iniTime);
}

/******************************************************************************
* Function Name: GetSecFromISR_Reset
* Description : second timer from ISR interrupt, start at zero initially
* Arguments : (p) timer variable, (p)initial time stamp
* Return Value : nona
******************************************************************************/
float GetSecFromISR_Reset(uint32_t *timer)
{
	return (float)*timer/1e6;
}

/******************************************************************************
* Function Name: float2string
* Description : convert float point to char array buffer, for later UART
* Arguments : char array, floating value
* Return Value : none
******************************************************************************/
void float2str(char *str, float *value){
	*str = (char*)malloc(sizeof(char)*80);  //pre-allocate string size
	char *valSign = (*value < 0) ? "-" : "";
	float Val     = (*value < 0) ? -(*value) : *value;

	int   Int1 = Val;                  // Get the integer
	float Frac = Val - Int1;           // Get fraction
	int   Int2 = trunc(Frac * 10000);  // Turn into integer
	sprintf(str, "%s%d.%d", valSign,Int1, Int2);
}
/******************************************************************************
* Function Name: chirp_linear
* Description : get chirp signal where freq. changes linearly with time
* Arguments : amplitude, start freq., stop freq., current time, stop time
* Return Value : chirp signal
******************************************************************************/
float chirp_linear(float amp, float f1,float f2, float time, float timer_inSec_offset, float time_max)
{
	float res;
	res = amp*sin(f1*2*M_PI*(time-timer_inSec_offset)+(f2-f1)*2*M_PI*(time-timer_inSec_offset)*(time-timer_inSec_offset)/time_max/2);
	return res;
}
/******************************************************************************
* Function Name: chirp_log
* Description : get chirp signal where freq. changes logarithmically w/ time
* Arguments : amplitude, start freq., stop freq., current time, stop time, (p)previous time, (p)phase
* Return Value : chirp signal
******************************************************************************/
float chirp_log(float amp, float f1,float f2, float time, float timer_inSec_offset, float time_max, float *timer_previous, float *phase)
{
	float delta_time, a0, a1, phase_increment, res;
	delta_time   =  time-timer_inSec_offset-*timer_previous;
	a0 = log10(f1*2*M_PI);
	a1 = (log10(f2*2*M_PI)-log10(f1*2*M_PI))/time_max;
	phase_increment = pow(10,(a0+a1*(time-timer_inSec_offset)))*delta_time;
	phase_increment = pow(10,(a0+a1*(time-timer_inSec_offset)))*delta_time;
	*phase+=phase_increment;
	*timer_previous = time-timer_inSec_offset;
	res = amp*sin(*phase);
	return res;
}

/******************************************************************************
* Function Name: PID_Control
* Description :
* Arguments :
* Return Value : control output
******************************************************************************/
float PID_Control(float setpoint,float y,float adjustGain, float *timer_inSec,float *pdelta_error)
{
  float current_time = *timer_inSec; //returns the number of milliseconds passed since the Arduino started running the program
  #if PIDMODE ==2 ||3
  float delta_time = current_time - last_time_inSec; //delta time interval
  #endif

  if (1){
    sensed_output = y;  //measured PV

    error = setpoint - sensed_output;
    total_error += error; //accumulates the error - integral term
//    delta_error = error - last_error; //for derivative on error
    *pdelta_error = -1*(sensed_output - last_sensed); //for derivative on PV

  #if PIDMODE ==1
    control_signal = Kp*adjustGain*error;
  #elif PIDMODE ==2
    control_signal = Kp*adjustGain*error + Ki*adjustGain*delta_time*total_error;
  #elif PIDMODE ==3
    control_signal = Kp*adjustGain*error + Ki*adjustGain*delta_time*total_error + Kd*adjustGain/delta_time*(*pdelta_error);
  #endif

    if (control_signal >= max_control) control_signal = max_control;
    else if (control_signal <= min_control) control_signal = min_control;

    last_error = error;
    last_time_inSec = current_time;
    last_sensed = sensed_output; // for derivative on PV
    }
  return control_signal;
}


/******************************************************************************
* Function Name: FraFreq
* Description : get FRA test frequencies linearly (type==1) or log linearly (type==2)
* Arguments :
* Return Value : derive FRA frequency
******************************************************************************/
void FraFreq(float *freqHz,float freq_start, float freq_stop, float point, uint8_t type)
{
	if (type==1)
	{
	for (int i=0;i<=point;i++)
		{
		freqHz[i] = freq_start+(freq_stop-freq_start)/(point-1) *i;
		 }
	} else if (type==2)
	{
	for (int i=0;i<=point;i++)
		{
		freqHz[i] = pow(10,log10(freq_start)+1/(point-1)*(log10(freq_stop)-log10(freq_start))*i);
		 }
	}
}

/******************************************************************************
* Function Name: discreteFreqSignal_inTime
* Description : sweep freq. under a constant span
* Arguments :
* Return Value : (1)sine wave which each freq. has constant span (2)record the timing of chaning freq.
******************************************************************************/
float discreteFreqSignal_inTime(float amp, float *freqList, float time_each, float timer_inSec, uint8_t *k_stamp )
{
    /* initialize once*/
    static int initialized = 0;
    static float t_reset;
    if(!initialized)
    	{
    	t_reset = timer_inSec;
    	initialized = 1;
    	}

    *k_stamp = floor(timer_inSec/time_each);
    if  (*k_stamp!=k_pre)
    	{
    	t_reset = timer_inSec;
    	}
    float phase = 2*M_PI*freqList[*k_stamp]*(timer_inSec-t_reset);
    k_pre = *k_stamp;
    return amp*sin(phase);


}

/******************************************************************************
* Function Name: discreteFreqSignal_inTime
* Description : sweep freq. under a constant cycle
* Arguments :
* Return Value : (1)sine wave which each freq. has constant span (2)record the timing of chaning freq.
******************************************************************************/
float discreteFreqSignal_inCycle(float amp, float *freqList,float *phaseAcc, float cycle, float timer_inSec, uint8_t *k_stamp )
{
    /* initialize once*/
    static int initialized = 0;
    static float t_reset;
    if(!initialized)
    	{
    	t_reset = timer_inSec;
    	initialized = 1;
    	}

    * k_stamp = floor(*phaseAcc/2/M_PI/cycle);
    if  (*k_stamp!=k_pre)
    	{
    	t_reset = timer_inSec;
    	}
    float phase = 2*M_PI*freqList[*k_stamp]*(timer_inSec-t_reset);
    *phaseAcc+= 2*M_PI*freqList[*k_stamp]*(timer_inSec-t_pre);
    k_pre = *k_stamp;
    t_pre = timer_inSec;
    return amp*sin(phase);
}

/******************************************************************************
* Function Name: read ADC
* Description :
* Arguments :
* Return Value :
******************************************************************************/
void readADC(ADC_HandleTypeDef *ADC_Handle,uint16_t *ADC_variable)
{
  	  HAL_ADC_Start(ADC_Handle);
    	  if(HAL_ADC_PollForConversion(ADC_Handle, 10)==HAL_OK){
    		*ADC_variable = HAL_ADC_GetValue(ADC_Handle);
    	  }
      HAL_ADC_Stop(ADC_Handle);
}
