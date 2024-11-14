/** ===================================================================
**     Function Name : void Button_Task(void)
**     Description    : The state of seven buttons

Open-loop mode display:
Mode: Open
Freq: 100.00KHz
Du/DT: 48.0/2.0%
ADC: 0.000V

Closed-loop mode display:
Mode: Close
Freq: 100.00KHz
Du/DT: xx.x/x.x%
ADC: x.xxxV

;---
Open-loop functionality / Button functionality:
1. PA6 -> T1, TB1, TA2, TB2 simultaneously increase frequency
          -> Upper limit 125K

2. PA7 -> T1, TB1, TA2, TB2 simultaneously decrease frequency
          -> Lower limit 77K

3. PB4 -> TA1, TB1 increase dead time
           -> Upper limit 2%, 50-48 = 2%

4. PB5 -> TA1, TB1 decrease dead time
           -> Lower limit 10%, 50-40 = 10%

5. PB6 -> TA2, TB2 simultaneously increase duty cycle
            -> Upper limit 45%

6. PB7  -> TA2, TB2 simultaneously decrease duty cycle
            -> Minimum 5%

7. PB9  -> Switch operating mode
            -> Open loop / Close loop
;---
Closed-loop functionality
1. When switching to closed-loop mode, display closed-loop mode
2. In closed-loop mode, buttons 1 to 6 are disabled
3. ADC voltage 0-3.3V corresponds to cycle 0-50% range
4. When ADC changes, the periods of four PWM channels TA1, TB1, TA2, TB2 change simultaneously, with TA1 equal to TB1 and TA2 equal to TB2
5. The relationship between target voltage, ADC voltage, and PWM output? Needs further establishment
6. Whether in open-loop or closed-loop, should perform soft start upon activation
;---
Pending functionalities:
1. Soft start
;---
Time base adjustment
1. Interrupt every 100£gS
2. Button task every 20mS
3. OLED every 1mS
;---

PWM Functionality:

TA1 -> Positive duty cycle 48%
TB1 -> Positive duty cycle 48%
TA2 -> Positive duty cycle 48%
TB2 -> Positive duty cycle 48%

TA1 and TB1 waveforms are complementary, TA2 and TB2 waveforms are complementary
-> When TA1 duty cycle is 48%, TB1 duty cycle is 52%
-> When TA2 duty cycle is 48%, TB2 duty cycle is 52%

TA1 and TA2 waveforms are consistent, TB1 and TB2 waveforms are consistent
TA1 and TB1 waveforms are complementary, TA2 and TB2 waveforms are complementary

1. TA1 and TB1 are complementary
2. TA2 and TB2 are complementary 
3. Dead time between TA1 and TB1 is from 2% to 50%, cannot be less than 2%
4. Dead time between TA2 and TB2 is from 2% to 50%, cannot be less than 2%
5. Initial frequency is 100KHz
6. Duty is 50%


**     Parameters  :
**     Returns     :
** ===================================================================*/

#include "function.h"
#include "CtlLoop.h"
#include "stdio.h"
#include "string.h"

#include "stm32g4xx_hal_def.h"

// Soft-start state flag
SState_M STState = SSInit;

// OLED refresh counter, increments every 5ms in the 5ms interrupt
uint16_t OLEDShowCnt = 0;

// Define control modes
#define MODE_OPEN 0
#define MODE_CLOSE 1

#define MODE_OPEN_LOOP 0
#define MODE_CLOSE_LOOP 1

volatile uint8_t currentMode = MODE_OPEN_LOOP;

// Minimum and maximum frequencies (unit: Hz)
#define FreqMin 11200 // 70KHz
#define FreqMax 20800 //130KHz
#define FreqStepPercent 160 //1%
//#define FreqStepPercent 16 //0.1%

// Dead time parameters (unit: 0.1%, i.e., adjust by 0.1% each step)
#define DeadTimeStepPercent 160 //1%

// Duty cycle parameters (unit: 0.1%, i.e., adjust by 0.1% each step)
#define DutyStepPercent 160 //1%

// Global variables
extern HRTIM_HandleTypeDef hhrtim1;
extern HRTIM_TimeBaseCfgTypeDef pGlobalTimeBaseCfg;

volatile float currentPWMFreq = 100000.0f; // Initial frequency 100 kHz
volatile uint8_t gCurrentDeadTimePercent = 2; // Initial dead time 2%

// Initialize duty cycle
volatile uint8_t gCurrentDutyPercent_TA1_TB1 = 48; // TA1/TB1 initial duty cycle 48%
volatile uint8_t gCurrentDutyPercent_TA2_TB2 = 48; // TA2/TB2 initial duty cycle 48%

// Time base configuration structure
HRTIM_TimeBaseCfgTypeDef pGlobalTimeBaseCfg;

// Current PLL frequency
volatile uint32_t currentPLLFreq = 160000000; // 160 MHz, based on fHRCK settings

int gPerioid = 16000; //100KHz
int gHalf = 8000; //50%
int gDeadTime = 360; //2%
int gDuty = 7680; //48%


/** ===================================================================
**     Function Name : Button_Task
**     Description : 
**     Parameters  :
**     Returns     :
** ===================================================================*/
void Button_Task(void)
{
	// KEY1/PA6 : T1, TB1, TA2, TB2 simultaneously increase frequency
	if (Key_Scan(KEY1_INC_Freq_GPIO_Port, KEY1_INC_Freq_Pin) == KEY_ON)
	{
		if(gPerioid < FreqMax)
		{
			if(gPerioid > FreqMax)
				gPerioid = FreqMax;

			gPerioid += FreqStepPercent;
			UpdateHRTIM(gPerioid, gHalf, gDuty, gDeadTime);

			DisplayFrequency(gPerioid);
		}
	}
	
	// KEY2/PA7 : T1, TB1, TA2, TB2 simultaneously decrease frequency
	if (Key_Scan(KEY2_DEC_Freq_GPIO_Port, KEY2_DEC_Freq_Pin) == KEY_ON)
	{
		if(gPerioid > FreqMin)
		{
			if(gPerioid < FreqMin)
				gPerioid = FreqMin;

			gPerioid -= FreqStepPercent;
			UpdateHRTIM(gPerioid, gHalf, gDuty, gDeadTime);

			DisplayFrequency(gPerioid);
		}
	}

	// KEY3/PB4: Increase dead time of TA1/TB1
	if (Key_Scan(KEY3_INC_DT_GPIO_Port, KEY3_INC_DT_Pin) == KEY_ON)
	{
		gDeadTime += DeadTimeStepPercent;
		UpdateHRTIM(gPerioid, gHalf, gDuty, gDeadTime);

		float deadTimePercent = ((float)gDeadTime) / 180.0f; // 360 / 180.0f = 2.0%
		DisplayDeadTime(deadTimePercent);
	}

	// KEY4/PB5: Decrease dead time of TA1/TB1
	if (Key_Scan(KEY4_DEC_DT_GPIO_Port, KEY4_DEC_DT_Pin) == KEY_ON)
	{
		gDeadTime -= DeadTimeStepPercent;
		UpdateHRTIM(gPerioid, gHalf, gDuty, gDeadTime);
		float deadTimePercent = ((float)gDeadTime) / 180.0f; // 360 / 180.0f = 2.0%
		DisplayDeadTime(deadTimePercent);

	}

	// KEY5/PB6: Simultaneously increase duty cycle of TA2/TB2
	if (Key_Scan(KEY5_INC_DUTY_GPIO_Port, KEY5_INC_DUTY_Pin) == KEY_ON)
	{
		gDuty += DutyStepPercent;
		UpdateHRTIM(gPerioid, gHalf, gDuty, gDeadTime);
		float dutyPercent = ((float)gDuty / (float)gPerioid) * 100.0f; // 7680 /16000 *100 = 48.0%
		DisplayDutyCycle(dutyPercent);


	}
	// KEY6/PB7: Simultaneously decrease duty cycle of TA2/TB2
	if (Key_Scan(KEY6_DEC_DUTY_GPIO_Port, KEY6_DEC_DUTY_Pin) == KEY_ON)
	{
		gDuty -= DutyStepPercent;
		UpdateHRTIM(gPerioid, gHalf, gDuty, gDeadTime);
		float dutyPercent = ((float)gDuty / (float)gPerioid) * 100.0f; // 7680 /16000 *100 = 48.0%
		DisplayDutyCycle(dutyPercent);

	}

	if (Key_Scan(KEY7_SWITCH_MODE_GPIO_Port, KEY7_SWITCH_MODE_Pin) == KEY_ON)
	{
		Mode_Switch(); // Switch mode
	}
	// Update OLED display
		//UpdateDisplay();
		UpdateDutyDisplay();

}

/** ===================================================================
**     Funtion Name :void Key_Scan(void)
**     Description : The state of two buttons
**       Default state of KEYFlag is 0. When pressed, Flag becomes 1, and when pressed again, Flag becomes 0, cycling in this way.
**       When the machine is running normally or during startup, pressing the button will turn off the output and enter standby mode.
**     Parameters  :
**     Returns     :
** ===================================================================*/
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{			
	/* Check if the button is pressed */
	if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == KEY_OFF)  
	{	 
		/* Wait for the button to be released */
		while(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_OFF);   
		return 	KEY_ON;	 
	}
	else
		return KEY_OFF;

}

/*
** ===================================================================
**     Function Name :   void ADCSample(void)
**     Description :    Samples Vin, Iin, Vout, and Iout
**     Parameters  :
**     Returns     :
** ===================================================================
*/
struct _ADI SADC={2048,2048,0,0,2048,2048,0,0,0,0}; // Input and output parameter sampling values and average values
struct _Ctr_value CtrValue={0,0,0,MIN_BUKC_DUTY,0,0,0}; // Control parameters
struct _FLAG DF={0,0,0,0,0,0,0,0}; // Control flag bits
uint16_t ADC1_RESULT[4]={0,0,0,0}; // DMA data storage register for transferring ADC samples from peripheral to memory

CCMRAM void ADCSample(void)
{
	// Declare variables for averaging Vin, Iin, Vout, and Iout
	static uint32_t VinAvgSum=0, IinAvgSum=0, VoutAvgSum=0, IoutAvgSum=0;
	
	// Convert ADC readings using calibration factors (Q15 format), including offset compensation
	SADC.Vin  = ((uint32_t)ADC1_RESULT[0] * CAL_VIN_K >> 12) + CAL_VIN_B;
	SADC.Iin  = ((uint32_t)ADC1_RESULT[1] * CAL_IIN_K >> 12) + CAL_IIN_B;
	SADC.Vout = ((uint32_t)ADC1_RESULT[2] * CAL_VOUT_K >> 12) + CAL_VOUT_B;
	SADC.Iout = ((uint32_t)ADC1_RESULT[3] * CAL_IOUT_K >> 12) + CAL_IOUT_B;

	// Check for invalid readings; if Vin is below the threshold, set it to 0
	if(SADC.Vin < 100) 
		SADC.Vin = 0;
	
	// If Iin is less than 2048, set it to 2048 (0A)
	if(SADC.Iin < 2048) 
		SADC.Iin = 2048;
	
	if(SADC.Vout < 100)
		SADC.Vout = 0;
	
	if(SADC.Iout < 2048)
		SADC.Iout = 2048;

	// Calculate average values of Vin, Iin, Vout, and Iout using moving average
	VinAvgSum = VinAvgSum + SADC.Vin - (VinAvgSum >> 2); // Add current Vin value and subtract the oldest value
	SADC.VinAvg = VinAvgSum >> 2; // Update Vin average
	
	IinAvgSum = IinAvgSum + SADC.Iin - (IinAvgSum >> 2); // Add current Iin value and subtract the oldest value
	SADC.IinAvg = IinAvgSum >> 2; // Update Iin average
	
	VoutAvgSum = VoutAvgSum + SADC.Vout - (VoutAvgSum >> 2); // Add current Vout value and subtract the oldest value
	SADC.VoutAvg = VoutAvgSum >> 2; // Update Vout average
	
	IoutAvgSum = IoutAvgSum + SADC.Iout - (IoutAvgSum >> 2); // Add current Iout value and subtract the oldest value
	SADC.IoutAvg = IoutAvgSum >> 2; // Update Iout average
}


/**
  * @brief  Mode switch function
  * @retval None
  */
void Mode_Switch(void)
{
    if (currentMode == MODE_OPEN_LOOP)
    {
        currentMode = MODE_CLOSE_LOOP;

		gPerioid = 16000;	//100KHz
		gHalf = 8000;	//50%
		gDeadTime = 360; //2%
		gDuty = 7680; //48%
		UpdateHRTIM(gPerioid, gHalf, gDuty, gDeadTime);

        // Display mode change
        OLED_ShowStr(55, 0, "Close", 2);

		Close_Mode_Init();
    }
    else
    {
        currentMode = MODE_OPEN_LOOP;

		gPerioid = 16000;	//100KHz
		gHalf = 8000;	//50%
		gDeadTime = 360; //2%
		gDuty = 7680; //48%
		UpdateHRTIM(gPerioid, gHalf, gDuty, gDeadTime);

        OLED_ShowStr(55, 0, "      ", 2); // Clear original display
        OLED_ShowStr(55, 0, "Open", 2);   // Display mode change

		Open_Mode_Init();
    }

    HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); // Blink LED to indicate mode change
}


/**
  * @brief  Update OLED display function
  * @retval None
  */
void UpdateDutyDisplay(void)
{
    if (currentMode == MODE_CLOSE_LOOP)
    {
        // 1. Start ADC sampling and use DMA to transfer data
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_RESULT, 4);
        HAL_ADC_Start(&hadc1);

        // 2. Sample ADC data
        ADCSample();

        // 3. Convert ADC value to voltage (0-3.3V)
        float adc_voltage = (SADC.VinAvg / 4095.0f) * 3.3f;

        // 4. Map ADC voltage to 0-50% duty cycle range
        float duty_percent = (adc_voltage / 3.3f) * 50.0f;

        // 5. Limit duty cycle to 0-50%
        if (duty_percent < 0.0f)
            duty_percent = 0.0f;
        if (duty_percent > 50.0f)
            duty_percent = 50.0f;

        // 6. Update global duty cycle value gDuty
        // gDuty = (duty percentage / 100) * period value
        gDuty = (int)((duty_percent / 100.0f) * gPerioid);

		gDeadTime = gHalf - gDuty;

		// 7. Call UpdateHRTIM function to update PWM configuration
		UpdateHRTIM(gPerioid, gHalf, gDuty, gDeadTime);

        // 8. Display frequency
        // Calculate frequency based on gPerioid and display
        DisplayFrequency(gPerioid);

        // 9. Display ADC voltage
        // Display in mV and show
        float adc_voltage_display = (SADC.VinAvg / 4095.0f) * 3.3f;
        unsigned char adcStr[10];
        sprintf((char*)adcStr, "%.3f ", adc_voltage_display); // For example, "1.650V"
        OLED_ShowStr(50, 6, adcStr, 2); // Display at position (50,6)

        // 10. Display duty cycle
        // Use existing DisplayDutyCycle function to display duty cycle
        DisplayDutyCycle(duty_percent);

		DisplayDeadTime(50.0f - duty_percent);
    }
    else
    {
        // Open-loop mode: display preset frequency

        // Display frequency, reserve decimal places
        //unsigned char freqStr[10];
        //sprintf(freqStr, "%.2fKHz", currentPWMFreq / 1000.0f); // For example, "100.00KHz"
        //OLED_ShowStr(45, 2, freqStr, 2); // Display frequency at position (45,2)
    }

    // Optionally: display other data, such as dead time
    // DisplayDeadTime(...); // Call as needed
}


/** ===================================================================
**     Function Name : void MX_OLED_Init(void)
**     Description : OLED initialization routine
**     Initialize the OLED display interface
**     Parameters  :
**     Returns     :
** ===================================================================*/
void Close_Mode_Init(void)
{
	currentMode = MODE_CLOSE_LOOP;

	// Initialize the OLED
	OLED_Init();
	OLED_CLS(); // Clear the OLED display
	
	// Display default frequency 100KHz
	gPerioid = 16000;
	DisplayFrequency(gPerioid);

	// Display initial text labels on the OLED
	OLED_ShowStr(0, 0, "Mode:", 2);
	OLED_ShowStr(55, 0, "Close", 2);   // Display mode change

	OLED_ShowStr(0, 2, "Freq:", 2);
	OLED_ShowStr(68, 2, ".", 2);
	OLED_ShowStr(100, 2, "KHz", 2);

	OLED_ShowStr(0, 4, "Du/DT:", 2);
	OLED_ShowStr(85, 4, "/", 2);
	OLED_ShowStr(120, 4, "%", 2);

	// Display dead time
	gCurrentDeadTimePercent = 2;
	DisplayDeadTime((float)gCurrentDeadTimePercent);


	// Display duty cycle
	gCurrentDutyPercent_TA2_TB2 = 48;
	DisplayDutyCycle(gCurrentDutyPercent_TA2_TB2);


	OLED_ShowStr(0, 6, "ADC:", 2);
	OLED_ShowStr(60, 6, ".", 2);
	OLED_ShowStr(98, 6, "V", 2);

	// Display ADC voltage
	uint8_t Vtemp[4] = {0};
	OLEDShowData(50, 6, Vtemp[0]);
	OLEDShowData(65, 6, Vtemp[1]);
	OLEDShowData(75, 6, Vtemp[2]);
	OLEDShowData(85, 6, Vtemp[3]);


	OLED_ON(); // Turn on the OLED display
}

/** ===================================================================
**     Function Name : void MX_OLED_Init(void)
**     Description : OLED initialization routine
**     Initialize the OLED display interface
**     Parameters  :
**     Returns     :
** ===================================================================*/
void Open_Mode_Init(void)
{
	currentMode = MODE_OPEN_LOOP;

	// Initialize the OLED
	OLED_Init();
	OLED_CLS(); // Clear the OLED display
	
	// Display default frequency 100KHz
	gPerioid = 16000;
	DisplayFrequency(gPerioid);

	// Display initial text labels on the OLED
	OLED_ShowStr(0, 0, "Mode:", 2);
	OLED_ShowStr(55, 0, "Open", 2);   // Display mode change

	OLED_ShowStr(0, 2, "Freq:", 2);
	OLED_ShowStr(68, 2, ".", 2);
	OLED_ShowStr(100, 2, "KHz", 2);

	OLED_ShowStr(0, 4, "Du/DT:", 2);
	OLED_ShowStr(85, 4, "/", 2);
	OLED_ShowStr(120, 4, "%", 2);

	// Display dead time
	gCurrentDeadTimePercent = 2;
	DisplayDeadTime((float)gCurrentDeadTimePercent);


	// Display duty cycle
	gCurrentDutyPercent_TA2_TB2 = 48;
	DisplayDutyCycle(gCurrentDutyPercent_TA2_TB2);


	OLED_ShowStr(0, 6, "ADC:", 2);
	OLED_ShowStr(60, 6, ".", 2);
	OLED_ShowStr(98, 6, "V", 2);

	// Display ADC voltage
	uint8_t Vtemp[4] = {0};
	OLEDShowData(50, 6, Vtemp[0]);
	OLEDShowData(65, 6, Vtemp[1]);
	OLEDShowData(75, 6, Vtemp[2]);
	OLEDShowData(85, 6, Vtemp[3]);


	OLED_ON(); // Turn on the OLED display
}


/**
  * @brief  Display frequency on OLED
  * @param  period - Current period value (gPerioid)
  * @retval None
  */
void DisplayFrequency(int period)
{

    // Calculate frequency (Hz)
    float frequency = (float)currentPLLFreq / (float)period;

    // Convert to KHz
    float frequency_kHz = frequency / 1000.0f;

    // Format frequency string, keep two decimal places
    unsigned char freqStr[10];
    sprintf((char *)freqStr, "%.2f", frequency_kHz); // For example, "100.00"

    // Display frequency string at (45, 2) position on OLED
    OLED_ShowStr(45, 2, freqStr, 2);

    // Display "KHz" at (95, 2) position
    OLED_ShowStr(95, 2, "KHz", 2);
}




/**
  * @brief  Display duty cycle on OLED
  * @param  duty_percent - Current duty cycle percentage (e.g., 36.0 means 36.0%)
  * @retval None
  */
void DisplayDutyCycle(float duty_percent)
{
    // Limit duty cycle within allowed range
    //if (duty_percent < (float)DUTY_MIN_PX10 / 10.0f)
    //    duty_percent = (float)DUTY_MIN_PX10 / 10.0f;
    //if (duty_percent > (float)DUTY_MAX_PX10 / 10.0f)
    //    duty_percent = (float)DUTY_MAX_PX10 / 10.0f;

    // Format duty cycle string, keep one decimal place
    unsigned char dutyStr[10];
    sprintf((char *)dutyStr, "%.1f", duty_percent);

    // Display duty cycle, adjust x, y coordinates to fit your display layout
    // Assuming "Duty:" label is at (0,4), value displayed at (50,4)
    OLED_ShowStr(50, 4, dutyStr, 2); // Adjust x, y coordinates to fit your display layout
}

/**
  * @brief  Display dead time on OLED
  * @param  dead_time_percent - Current dead time percentage (e.g., 2.0 means 2.0%)
  * @retval None
  */
void DisplayDeadTime(float dead_time_percent)
{
    // Limit dead time within allowed range
    //if (dead_time_percent < (float)DEADTIME_MIN_PX1000 / 10.0f)
    //    dead_time_percent = (float)DEADTIME_MIN_PX1000 / 10.0f;
    //if (dead_time_percent > (float)DEADTIME_MAX_PX1000 / 10.0f)
    //    dead_time_percent = (float)DEADTIME_MAX_PX1000 / 10.0f;

    // Format dead time string, keep one decimal place
    unsigned char deadTimeStr[10];
    sprintf((char *)deadTimeStr, "%.1f", dead_time_percent);

    // Display dead time, adjust x, y coordinates to fit your display layout
    // Assuming "Du/DT:" label is at (0,4), value displayed at (50,4)
    OLED_ShowStr(95, 4, deadTimeStr, 2); // Adjust x, y coordinates to fit your display layout
}




/**
  * @brief  Update the PWM frequency of HRTIM
  * @param  period: Timer period
  * @param  half_period: Half period
  * @param  duty_cycle: Duty cycle
  * @param  dead_time: Dead time
  * @retval HAL_StatusTypeDef: Returns HAL_OK on success, HAL_ERROR on failure
  */
void UpdateHRTIM(int period, int half_period, int duty_cycle, int dead_time)
{
    /* USER CODE END HRTIM1_Init 0 */

    HRTIM_TimeBaseCfgTypeDef timeBaseConfig = {0};
    HRTIM_TimerCfgTypeDef timerConfig = {0};
    HRTIM_CompareCfgTypeDef compareConfig = {0};
    HRTIM_TimerCtlTypeDef timerControl = {0};
    HRTIM_OutputCfgTypeDef outputConfig = {0};

    /* USER CODE BEGIN HRTIM1_Init 1 */

    /* USER CODE END HRTIM1_Init 1 */
    hhrtim1.Instance = HRTIM1;
    hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
    hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
    if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_3) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
    {
        Error_Handler();
    }

    // Configure the time base of the master timer (MASTER)
    timeBaseConfig.Period = period;
    timeBaseConfig.RepetitionCounter = 0x00; // Set repetition counter to 0
    timeBaseConfig.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL16; // Set prescaler ratio to 16x
    timeBaseConfig.Mode = HRTIM_MODE_CONTINUOUS; // Set timer to continuous mode

    // Apply the time base configuration to the MASTER timer, call error handler if configuration fails
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &timeBaseConfig) != HAL_OK)
    {
        Error_Handler();
    }

    // Configure the control parameters of the MASTER timer
    timerConfig.InterruptRequests = HRTIM_MASTER_IT_NONE; // Disable MASTER timer interrupt requests
    timerConfig.DMARequests = HRTIM_MASTER_DMA_NONE;       // Disable MASTER timer DMA requests
    timerConfig.DMASrcAddress = 0x0000;                   // Set DMA source address to 0
    timerConfig.DMADstAddress = 0x0000;                   // Set DMA destination address to 0
    timerConfig.DMASize = 0x1;                            // Set DMA transfer size to 1
    timerConfig.HalfModeEnable = HRTIM_HALFMODE_DISABLED; // Disable half mode
    timerConfig.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED; // Disable interleaved mode
    timerConfig.StartOnSync = HRTIM_SYNCSTART_DISABLED;    // Disable synchronous start
    timerConfig.ResetOnSync = HRTIM_SYNCRESET_DISABLED;    // Disable synchronous reset
    timerConfig.DACSynchro = HRTIM_DACSYNC_NONE;           // Disable DAC synchronization
    timerConfig.PreloadEnable = HRTIM_PRELOAD_DISABLED;    // Disable preload
    timerConfig.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT; // Set update gating to independent
    timerConfig.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK; // Set burst mode to maintain clock
    timerConfig.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED; // Disable repetition update
    timerConfig.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL; // Set resync update to unconditional

    // Apply the control configuration to the MASTER timer, call error handler if configuration fails
    if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &timerConfig) != HAL_OK)
    {
        Error_Handler();
    }

    // Configure the compare unit 1 of the MASTER timer to set the reset point for TA1 and TB1
    compareConfig.CompareValue = half_period * period / 16000; // Set compare value, duty 50%
    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, &compareConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &timeBaseConfig) != HAL_OK)
    {
        Error_Handler();
    }

    // Configure Timer A's control parameters
    timerControl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UP; // Set count mode to up-counting
    timerControl.TrigHalf = HRTIM_TIMERTRIGHALF_DISABLED; // Disable half-cycle trigger
    timerControl.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL; // Set compare condition to equal
    timerControl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED; // Disable dual channel DAC

    // Apply Timer A's control configuration, call error handler if configuration fails
    if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &timerControl) != HAL_OK)
    {
        Error_Handler();
    }

    // Configure Timer A's compare unit 1 to reset TA1 (advance deadtime)
    timerConfig.InterruptRequests = HRTIM_TIM_IT_NONE; // Disable Timer A interrupt requests
    timerConfig.DMARequests = HRTIM_TIM_DMA_NONE;       // Disable Timer A DMA requests
    timerConfig.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED; // Disable push-pull mode
    timerConfig.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;    // Disable faults
    timerConfig.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;   // Set fault lock to read/write
    timerConfig.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED; // Disable dead time insertion
    timerConfig.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED; // Disable delayed protection mode
    timerConfig.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE; // Disable update trigger
    timerConfig.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER; // Set reset trigger source to MASTER timer period
    timerConfig.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED; // Disable update on reset

    // Apply Timer A's compare unit configuration, call error handler if configuration fails
    if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &timerConfig) != HAL_OK)
    {
      Error_Handler();
    }


    // Set Timer B's reset trigger source to the MASTER timer's compare unit 1
    timerConfig.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_CMP1;

    // Apply Timer B's compare unit configuration, call error handler if configuration fails
    if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &timerConfig) != HAL_OK)
    {
        Error_Handler();
    }

    // Set Timer A's compare unit 1, set new compare value to (half_period * period / 16000) - dead_time
    compareConfig.CompareValue = half_period * period / 16000 - dead_time; // 7680 / frequency minus deadtime 48%
    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &compareConfig) != HAL_OK)
    {
        Error_Handler();
    }
    // Set Timer A's compare unit 2, set duty cycle to duty_cycle * period / 16000
    compareConfig.CompareValue = duty_cycle * period / 16000; // 3600 / period 36%
    compareConfig.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    compareConfig.AutoDelayedTimeout = 0x0000;

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &compareConfig) != HAL_OK)
    {
        Error_Handler();
    }
    // Configure TA1 and TB1's output parameters
    outputConfig.Polarity = HRTIM_OUTPUTPOLARITY_HIGH; // Set output polarity to high
    outputConfig.SetSource = HRTIM_OUTPUTSET_TIMPER;    // Set output set source to MASTER timer period
    outputConfig.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1; // Set output reset source to compare unit 1
    outputConfig.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE; // Set idle mode to none
    outputConfig.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE; // Set idle level to inactive
    outputConfig.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE; // Set fault level to none
    outputConfig.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED; // Disable chopper mode
    outputConfig.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR; // Set burst mode entry to regular mode

    // Configure TA1 output, call error handler if configuration fails
    if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &outputConfig) != HAL_OK)
    {
        Error_Handler();
    }
    // Configure TB1 output, call error handler if configuration fails
    if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, &outputConfig) != HAL_OK)
    {
        Error_Handler();
    }

    // Change ResetSource to compare unit 2, used for resetting TA2 and TB2
    outputConfig.ResetSource = HRTIM_OUTPUTRESET_TIMCMP2;

    // Configure TA2 output, call error handler if configuration fails
    if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &outputConfig) != HAL_OK)
    {
        Error_Handler();
    }
    // Configure TB2 output, call error handler if configuration fails
    if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB2, &outputConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &timeBaseConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &timerControl) != HAL_OK)
    {
        Error_Handler();
    }
    // Set Timer B's compare unit 1, set compare value to (half_period * period / 16000) - dead_time
    compareConfig.CompareValue = half_period * period / 16000 - dead_time; // 7680 / frequency minus deadtime 48%
    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &compareConfig) != HAL_OK)
    {
        Error_Handler();
    }
    // Set Timer B's compare unit 2, set duty cycle to duty_cycle * period / 16000
    compareConfig.CompareValue = duty_cycle * period / 16000; // 3600 / period 36%

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &compareConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE BEGIN HRTIM1_Init 2 */
    // Store global time base configuration
    pGlobalTimeBaseCfg = timeBaseConfig;

    // Start four PWM outputs (TA1, TA2, TB1, TB2)
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2); // Enable all PWM outputs
    // Start timers A and B
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B); // Start both PWM timers

    /* USER CODE END HRTIM1_Init 2 */
    HAL_HRTIM_MspPostInit(&hhrtim1);
}





















