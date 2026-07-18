/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
*/

/*
* 	+-----------------------------------+
* 	|   AxxBeam Specifications          |
*	+-----------------------------------+
*
*	Weight: 210 g
*	Input voltage: 7-20 V
*	Electrical Power: 90 W (e.g 7,5 A at 12 V)
*	Light flux: 10000 lm
*	LEDs: 8 CREE XM-L2
*	Glass: 3 mm thick Polycarbonate
*	Cord: 500 mm long, 4.7 mm diameter with Andersson PowerPole 15 connector.
*	Mount: GoPro mount
*	Light intensities: 5%, 50%, 100% (Configurable)
*	Temperature regulation: At max temperature the lamp will dim down in order to not over heat.
*
*
* 	+-----------------------------------+
* 	|   AxxBeam Programming Interface   |
* 	+-----------------------------------+
*	Settings done with this programming interface will be written to non-volatile flash memory.
*
* 	Hold button during start-up (plugging in power) to enter programming mode.
* 	Blue indicator light will flash 10 times rapidly to confirm programming mode.
* 	Blinks for the top level menu will follow and be repeated: 1 BLINK ---(delay)--- 2 BLINK ---(delay)--- 3 BLINK.
* 	To enter a sub-menu, short press the button after the chosen blink sequence.
* 	The main LEDs will confirm the chosen sub-menu with the same number of blinks
* 	The sub-menu will be presented in a similar way as the main menu
* 	1 BLINK ---(delay)--- 2 BLINK ---(delay)--- 3 BLINK and so on....
* 	To select a setting, short press the button and the main LEDs will once again confirm the selection.
* 	The main menu will once again be presented.
* 	To exit the menu, long press the button.
* 	The blue indication light will flash 10 times rapidly and the lamp will exit programming mode and boot up as normal.
*
* 	Eg. To change the power source to 12V lead acid battery:
* 		- Hold button while plugging in the lamp to a power source
* 		- The blue indicator light flashes 10 times rapidly (button can be released now)
* 		- 1 BLINK --- 2 BLINK ---(short button press) --- (2 BLINK with the main LEDs as confirmation for entering sub-menu 2) ---
* 		- 1 BLINK --- 2 BLINK --- 3 BLINK ---(short button press)--- (3 BLINK with the main LEDs as confirmation for choosing setting 3) ---
* 		- 1 BLINK --- (long button press) --- The blue indicator light flashes 10 times rapidly and the programming interface is exited.
*
*	- Top level menu
*		- Sub-menu
*
*	1 BLINK (Power modes)
*		1 BLINK: Power modes -> [5%, 20%, 50%, 70%, 100%]
*		2 BLINK: Power modes -> [5%, 50%, 100%] (default)
*		3 BLINK: Power modes -> [5%, 100%]
*		4 BLINK: Power modes -> [5%, 70%]
*		5 BLINK: Power modes -> [100%]
*		6 BLINK: Power modes -> [50%]
*		7 BLINK: Power modes -> [5%]
*
*	2 BLINK (Power source, The LEDs will dim down at the end of the battery capacity.
*	         On an empty battery the lamp latches into a ~2 W reserve ("get home") mode
*	         after two quick dips of the main beam; replacing the battery releases it.)
*		1 BLINK: 3S Li-ion Pack (9 V -> 10% lED power, reserve at 8.7 V) (default)
*		2 BLINK: 4S Li-ion Pack (12 V -> 10% lED power, reserve at 11.6 V)
*		3 BLINK: 12V Lead Acid battery (11.5 V -> 10% lED power, reserve at 10.5 V)
*		4 BLINK: DC Source (7 V - 20 V, no reserve mode)
*
*	3 BLINK (Start-up sequence)
*		1 BLINK: Start LEDs with button (default)
*		2 BLINK: Start Automatically when power is available
*
*	4 BLINK (Remember last power setting)
*		1 BLINK: The lamp always start at the lowest power setting of the chosen power modes (default)
*		2 BLINK: The lamp remembers the current power setting when it shut down, once started again this is the power setting used.
*		3 BLINK: The lamp remembers the current power setting when it shut down, once started again this power setting PLUS ONE STEP is used.
*
*	5 BLINK (Idle indicator LED)
*		1 BLINK: Pulse mode (Default)
*		2 BLINK: Breath mode
*		3 BLINK: Indicator LED off
*
*	6 BLINK (Factory reset)
*		1 BLINK: Reset all parameters to Default?  YES
*		2 BLINK: Reset all parameters to Default?  NO
*
*	7 BLINK (Version info)
*/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "moving_average.h"
#include "TMP1075.h"
#include "flash.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Independent watchdog: LSI/64 with full reload gives a ~8.2 s timeout.
 * Kicked from every loop that can legitimately run long (blink sequences,
 * BLE setup, settings menu, main loop). Cannot be stopped once started. */
#define WATCHDOG_KICK() (IWDG->KR = 0x0000AAAAU)

/* Magic value in a backup register requesting a jump to the ROM bootloader
 * on the next boot, before the watchdog is started (the IWDG would reset
 * the bootloader in the middle of an OTA flash otherwise). */
#define BOOTLOADER_REQUEST_MAGIC 0xB007ACCEU

/* Thermal regulation: PI controller that computes a power ceiling holding
 * the PCB temperature at maxTemp. Below maxTemp the ceiling saturates at
 * 100 % and has no effect. */
#define THERMAL_KP 10.0f  // % power per degC of temperature error
#define THERMAL_KI 0.5f   // % power per degC per second

/* Temperature sensor fault handling: after this many consecutive failed
 * reads the sensor is considered dead. Over-temperature protection is
 * blind without it, so power is limited to a safe ceiling and the status
 * LED blinks rapidly. */
#define TEMP_SENSOR_FAULT_LIMIT 50
#define TEMP_FAULT_POWER_CEILING 20.0f  // % power allowed with a dead sensor

/* Battery reserve ("get home") mode: when the battery stays below its
 * chemistry-specific empty threshold for RESERVE_DEBOUNCE_MS under load,
 * latch to a small fixed power instead of running the pack down to the
 * 7 V electronics limit. Releases only when the voltage rises well above
 * the threshold, i.e. the battery was replaced. */
#define RESERVE_POWER 2.0f          // % power in reserve mode (~2 W)
#define RESERVE_DEBOUNCE_MS 5000    // must stay below threshold this long

/* Settings storage: 6 setting words followed by a CRC word */
#define SETTINGS_WORDS 6

/* Health log storage */
#define HEALTH_MAGIC 0x48501001U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac3;

I2C_HandleTypeDef hi2c2;

OPAMP_HandleTypeDef hopamp1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t major_version = 4;
uint8_t minor_version = 1;

/* Debug variables */
uint8_t time = 1;
uint8_t ON = 0;
uint8_t P = 5;
uint8_t P_red = 0;

/* Timing constants */

char buffer[40]; //Buffer for UART print
uint8_t UART1_rxBuffer[12] = {0};
uint8_t rx_flag = 0;
uint8_t byte, rx; //Was previously char
volatile int line_valid = 0;
#define LINEMAX 40 // Maximal allowed/expected line length
static char line_buffer[LINEMAX]; // Holding buffer with space for terminating NUL
static char rx_buffer[LINEMAX]; // Local holding buffer to build line

/* UART send buffers */
#define MAX_BUFFER_LEN 250
uint8_t UART_transmit_buffer[MAX_BUFFER_LEN];
uint8_t UART_packet_index = 0;
uint8_t UART_packet_length = 0;

float temperatureLightReduction = 0.0;
float voltageLightReduction = 0.0;

uint32_t status_previousMillis = 0;
uint32_t status_Interval = 500;           // interval at which to send status messages

uint32_t blink_previousMillis = 0;
uint32_t blink_Interval = 1000;

uint32_t previous_millis_debug = 0;
uint32_t interval_debug = 50;

uint32_t SOSmode_previousMillis = 0;
uint32_t SOSmode_delay = 3000;

static uint32_t next_blink_number = 1;
static uint32_t next_pulse_length = 50;

float lowVoltage = 7.2;
float minVoltage = 7.0;

/* Battery reserve mode state. Thresholds are set per power source at
 * start-up; reserveVoltage == 0 disables the feature (DC source). */
float reserveVoltage = 0.0f;
float reserveReleaseVoltage = 0.0f;
uint8_t ReserveMode = 0;
uint8_t reserve_below = 0;
uint32_t reserve_belowSinceMillis = 0;

float LedPower = 0.0;
const float *LedPowerArray;
uint8_t LedPowerArrayLenght;
uint8_t arrayIndex = 0;

uint32_t settings_data_default[] = {2, 1, 1, 1, 1, 0};

float maxTemp = 75.0;

uint8_t LowBusVoltage = 0;
float cutOffVoltage= 7.0;

//Constants for the Indicator led breathing
uint16_t led_pulse_index = 0;
uint32_t led_pulse_previousMillis = 0;
uint16_t led_pulse_Interval = 10;

//Constants for the SOS signal
uint16_t SOS_pulse_index = 0;
uint32_t SOS_pulse_previousMillis = 0;
uint16_t SOS_pulse_Interval = 200;

uint8_t SOSmode = 0;

//Constants for the Low Voltage blink
uint16_t LowVoltageBlink_pulse_index = 0;
uint32_t LowVoltageBlink_previousMillis = 0;
uint16_t LowVoltageBlink_pulse_Interval = 100;


#define BUS_VOLTAGE_FACTOR 0.00488281084
#define LED_CURRENT_FACTOR 29.7

float Current_SetCurrentPercent = 0;

uint8_t shortPress=0;
uint8_t longPress=0;
uint8_t longPressSOS=0;

uint8_t Pressed=0;
uint8_t Pressed_SOSmode=0;

uint16_t SHORT_PRESS_TIME = 800; // 800 milliseconds
uint16_t DEBOUNCE_PRESS_TIME = 60; // 60 milliseconds
uint16_t SOSMODE_PRESS_TIME = 4000;

uint32_t pressedTime  = 0;
uint32_t releasedTime = 0;
uint32_t pressDuration = 0;

uint8_t immediateStart = 0;
uint8_t remember_power = 0;
uint8_t remember_power_plus_one = 0;
uint8_t idle_blink_mode = 0;
uint32_t Flash_Rx_Data[6];
uint32_t Flash_Tx_Data[6];


uint16_t statusLedArr_pulse[230] = {0,26,52,78,105,131,157,184,210,236,263,289,315,342,368,394,421,447,473,
		500,500,473,447,421,394,368,342,315,289,263,236,210,184,157,131,105,78,52,26,
		0,0,42,84,126,168,210,252,294,336,378,421,463,505,547,589,631,673,715,757,
		800,800,786,772,759,745,732,719,706,693,680,668,655,643,630,618,606,594,582,571,
		559,548,536,525,514,503,492,481,471,460,450,439,429,419,409,399,390,380,371,361,
		352,343,334,325,316,308,299,291,283,275,267,259,251,243,236,228,221,214,207,200,
		193,186,179,173,167,160,154,148,142,136,131,125,120,115,109,104,99,95,90,85,
		81,76,72,68,64,60,56,53,49,46,43,39,36,33,31,28,25,23,20,18,
		16,14,12,11,9,7,6,5,4,3,2,1,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0};

	/* Created By Python3 script:
	import matplotlib.pyplot as plt
	import numpy as np

	ramplength = 20
	arr = np.array([])
	arr = np.append(arr, np.linspace(0, 500, num=ramplength))
	arr = np.append(arr, np.linspace(500, 0, num=ramplength))
	arr = np.append(arr, np.linspace(0, 800, num=ramplength))
	x = np.linspace(0, 120, num=120)
	xx = np.array([0,60,120])
	yy = np.array([800,190,0])
	plt.plot(xx+ramplength*3,yy,'r*-')
	z = np.polyfit(xx, yy, 2)
	y = np.polyval(z, x)
	arr = np.append(arr, y)
	arr = np.append(arr, np.linspace(0, 0, num=50))
	plt.plot(arr)
	plt.grid(True)
	plt.show()
	for i,j in enumerate(arr):
		if i > 0:
			print(",", end='')
		if((i+1)%20 == 0):
			print('')
		print("%d" % j, end='')

	print("\n\nArray length: ")
	print(len(arr))
	*/

uint16_t statusLedArr_breath[600] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,2,2,3,
		4,4,5,6,7,8,9,10,11,12,13,14,16,17,18,20,21,23,24,26,27,
		29,31,33,34,36,38,40,42,44,47,49,51,53,56,58,60,63,65,68,71,73,
		76,79,82,84,87,90,93,96,99,103,106,109,112,116,119,123,126,130,133,137,141,
		144,148,152,156,160,164,168,172,176,180,185,189,193,198,202,206,211,216,220,225,230,
		234,239,244,249,254,259,264,269,274,279,285,290,295,301,306,312,317,323,329,334,340,
		346,352,358,363,369,375,382,388,394,400,406,413,419,426,432,439,445,452,458,465,472,
		479,486,493,500,499,503,507,511,514,518,521,524,527,529,532,534,536,538,540,542,543,
		545,546,547,548,548,549,549,549,549,549,549,548,548,547,546,545,543,542,540,538,536,
		534,532,529,527,524,521,518,514,511,507,503,500,500,495,491,487,483,479,475,471,467,
		463,459,455,452,448,444,440,436,432,428,425,421,417,413,410,406,402,399,395,391,388,
		384,380,377,373,370,366,363,359,356,352,349,346,342,339,335,332,329,325,322,319,316,
		312,309,306,303,299,296,293,290,287,284,281,278,275,272,269,266,263,260,257,254,251,
		248,245,242,239,236,234,231,228,225,223,220,217,214,212,209,206,204,201,199,196,193,
		191,188,186,183,181,178,176,174,171,169,166,164,162,159,157,155,152,150,148,146,143,
		141,139,137,135,133,131,128,126,124,122,120,118,116,114,112,110,109,107,105,103,101,
		99,97,96,94,92,90,88,87,85,83,82,80,78,77,75,74,72,70,69,67,66,
		64,63,61,60,59,57,56,54,53,52,50,49,48,47,45,44,43,42,40,39,38,
		37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,19,18,
		17,16,16,15,14,13,13,12,11,11,10,10,9,8,8,7,7,6,6,5,5,
		5,4,4,3,3,3,2,2,2,1,1,1,1,1,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0};

	/* Created By Python3 script:
	import matplotlib.pyplot as plt
	import numpy as np

	ramplength = 100
	arr = np.array([])
	arr = np.append(arr, np.linspace(0, 300, num=ramplength))
	x = np.linspace(0, 150, num=150)
	xx = np.array([0,60,150])
	yy = np.array([300,150,0])
	plt.plot(xx+ramplength,yy,'r*--')
	z = np.polyfit(xx, yy, 2)
	y = np.polyval(z, x)

	arr = np.append(arr, y)
	arr = np.append(arr, np.linspace(0, 0, num=150))
	plt.plot(arr)
	plt.grid(True)
	plt.show()
	for i,j in enumerate(arr):
		if i > 0:
			print(",", end='')
		if((i+1)%20 == 0):
			print('')
		print("%d" % j, end='')

	print("\n\nArray length: ")
	print(len(arr))
	*/

uint8_t SOS_morse[40] = {0,1,0,1,0,1,0,0,0,1,1,1,0,0,0,1,1,1,0,0,0,1,1,1,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0};

typedef enum {
    ST_IDLE,
	ST_RUN,
} mainstates;
mainstates activeState = ST_IDLE;

/* Settings menu layout. Each top-level menu has a number of selectable
 * options and a location in the settings array (Flash_Rx_Data) where the
 * chosen option number is stored. See the menu description in the file
 * header for what the options mean. */
typedef struct {
	uint8_t option_count;   // number of options in the sub-menu (0 = no sub-menu)
	uint8_t settings_index; // index in the settings array where the choice is stored
} SettingsMenuDef;

#define SETTINGS_MENU_COUNT 7
#define SETTINGS_FACTORY_RESET_MENU 5   // 0-based index of the factory reset menu
#define SETTINGS_ITEM_INTERVAL 2000     // ms between presented menu items

static const SettingsMenuDef settings_menus[SETTINGS_MENU_COUNT] = {
	{7, 0}, // Menu 1: Power modes
	{4, 1}, // Menu 2: Power source
	{2, 2}, // Menu 3: Start-up sequence
	{3, 3}, // Menu 4: Remember last power setting
	{3, 4}, // Menu 5: Idle indicator LED
	{2, 5}, // Menu 6: Factory reset (YES/NO)
	{0, 0}, // Menu 7: Version info (no sub-menu)
};


#define ADC1_BUF_LEN 80
uint16_t adc1_buf[ADC1_BUF_LEN];
float pcbTemperature_temp;

struct sensor_values_struct {
	float pcbTemperature;
	float busVoltage;
	float esitmatedBusCurrent;
	float batteryVoltage;
	uint8_t buttonState;
	uint8_t last_buttonState;

};
struct sensor_values_struct sensor_values  = { .pcbTemperature = 20,
        									.busVoltage = 20,
											.esitmatedBusCurrent = 0,
											.batteryVoltage = 20,
											.buttonState = 0,
											.last_buttonState = 0};

FilterTypeDef temperature_filterStruct;
FilterTypeDef voltage_filterStruct;
FilterTypeDef button_filterStruct;

/* Thermal regulator state. The integrator is the steady-state power
 * ceiling in percent; it starts at 100 (no limit) and is pulled down by
 * the PI controller when the PCB runs above maxTemp. */
float thermal_integrator = 100.0f;
uint32_t thermal_previousMillis = 0;
uint16_t temp_read_failures = 0;  // consecutive failed TMP1075 reads

/* Health log, persisted in its own flash page. All fields are 32-bit so
 * the struct maps directly onto the word-based flash routines. */
typedef struct {
	uint32_t magic;
	uint32_t boot_count;           // number of power-ups
	uint32_t on_minutes;           // accumulated time with the main LEDs on
	int32_t  max_temperature_dC;   // highest PCB temperature seen, in 0.1 degC
	uint32_t min_voltage_mV;       // lowest bus voltage seen while running
	uint32_t thermal_dim_events;   // times the thermal limiter engaged
	uint32_t low_voltage_events;   // times low-voltage reduction engaged
	uint32_t crc;                  // over all preceding fields
} HealthLog;
#define HEALTH_WORDS (sizeof(HealthLog)/4)

HealthLog health;
uint8_t health_dirty = 0;              // unsaved record/event changes
uint32_t health_lastSaveMillis = 0;
uint32_t health_run_ms = 0;            // run time not yet counted into on_minutes
uint32_t health_saved_minutes = 0;     // on_minutes at the last flash save

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC3_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ------------------------- Watchdog ------------------------- */

/* Start the independent watchdog (~8.2 s timeout). It cannot be stopped
 * again; every loop that can run longer than the timeout must call
 * WATCHDOG_KICK(). Frozen while halted by a debugger. */
void watchdog_start(void){
	DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP;
	IWDG->KR = 0x0000CCCCU;  // start the watchdog
	IWDG->KR = 0x00005555U;  // enable register access
	IWDG->PR = 4;            // prescaler LSI/64
	IWDG->RLR = 0x0FFF;      // full reload: ~8.2 s at 32 kHz LSI
	while (IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)){}
	WATCHDOG_KICK();
}

/* -------------------- Bootloader entry ---------------------- */

/* Request the ROM bootloader via a backup register and reset. The jump
 * happens early on the next boot, before the watchdog is started, so an
 * OTA flash is not interrupted by a watchdog reset. */
void request_bootloader_reboot(void){
	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_RTCAPB_CLK_ENABLE();
	TAMP->BKP0R = BOOTLOADER_REQUEST_MAGIC;
	NVIC_SystemReset();
}

/* Check (and consume) a pending bootloader request */
uint8_t bootloader_reboot_requested(void){
	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_RTCAPB_CLK_ENABLE();
	if (TAMP->BKP0R == BOOTLOADER_REQUEST_MAGIC){
		TAMP->BKP0R = 0;
		return 1;
	}
	return 0;
}

/* --------------------- Settings storage --------------------- */

/* The 6 setting words are stored followed by a CRC word. A CRC mismatch
 * (erased, corrupt or pre-CRC flash content) falls back to defaults. */
uint32_t settings_crc(uint32_t *data){
	return HAL_CRC_Calculate(&hcrc, data, SETTINGS_WORDS*4); // length in bytes
}

void settings_save(uint32_t *data){
	uint32_t block[SETTINGS_WORDS + 1];
	memcpy(block, data, SETTINGS_WORDS*4);
	block[SETTINGS_WORDS] = settings_crc(block);
	Flash_Write_Data(FLASH_USER_SETTINGS_ADDR, block, SETTINGS_WORDS + 1);
}

/* Returns 1 and fills data when the stored settings are valid, else 0 */
uint8_t settings_load(uint32_t *data){
	uint32_t block[SETTINGS_WORDS + 1];
	Flash_Read_Data(FLASH_USER_SETTINGS_ADDR, block, SETTINGS_WORDS + 1);
	if (settings_crc(block) != block[SETTINGS_WORDS]){
		return 0;
	}
	memcpy(data, block, SETTINGS_WORDS*4);
	return 1;
}

/* ----------------------- Health log ------------------------- */

void health_save(void){
	health.magic = HEALTH_MAGIC;
	health.crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)&health, (HEALTH_WORDS - 1)*4);
	Flash_Write_Data(FLASH_USER_HEALTH_ADDR, (uint32_t *)&health, HEALTH_WORDS);
	health_dirty = 0;
	health_saved_minutes = health.on_minutes;
	health_lastSaveMillis = HAL_GetTick();
}

void health_load(void){
	Flash_Read_Data(FLASH_USER_HEALTH_ADDR, (uint32_t *)&health, HEALTH_WORDS);
	uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)&health, (HEALTH_WORDS - 1)*4);
	if (health.magic != HEALTH_MAGIC || health.crc != crc){
		// New device or corrupt log: start over
		memset(&health, 0, sizeof(health));
		health.max_temperature_dC = -400; // -40.0 degC
		health.min_voltage_mV = 99000;    // "no reading yet"
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    static uint16_t rx_index = 0;

    if (huart->Instance == USART1){
        char rx = byte;

        // Re-arm interrupt ASAP
        HAL_UART_Receive_IT(&huart1, &byte, 1);

        if (rx == '\n' || rx == '\r'){
            if (rx_index > 0){
                if (!line_valid){ // main loop is done with the previous line
                    memcpy(line_buffer, rx_buffer, rx_index);
                    line_buffer[rx_index] = '\0';

                    line_valid = 1;   // signal main loop, which also echoes the line on huart2
                }
                // else: previous line not consumed yet - drop this one
                // instead of tearing the buffer mid-parse

                rx_index = 0;     // reset buffer
            }
        }
        else{
            if (rx_index < LINEMAX - 1){
                rx_buffer[rx_index++] = rx;
            }
            else{
                // overflow → reset or ignore
                rx_index = 0;
            }
        }
    }
}

/* Any UART error (framing, noise, overrun - e.g. bytes arriving while a
 * flash erase stalls the CPU) makes HAL abort the interrupt reception,
 * and nothing re-arms it: all BLE commands including __OTA__ would be
 * dead until a power cycle. Re-arm reception here. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART1){
		HAL_UART_Receive_IT(&huart1, &byte, 1);
	}
}

/* Function to clamp d between the limits min and max */
float clamp(float d, float min, float max) {
    if (d < min) return min;
    if (d > max) return max;
    return d;
}

//This sets the DAC with a OPAMP follower to the desired CTRL voltage for the LT3741
void SetDAC(float setCurrentPercent){
	Current_SetCurrentPercent = clamp(setCurrentPercent, 0.0f, 100.0f);
	HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (Current_SetCurrentPercent*LED_CURRENT_FACTOR)); // the compensation value to get 90W at 100%
}

//Control the ENABLE signal to LT3741
void EnableLedArray(int i){
	if(i){
		HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);
	}
}

//Status led PWM (range: 0-1000)
void SetStatusLED(int i){
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, i);
}

//Low voltage warning: status LED alternates bright/dim every 100 ms
void lowVoltageWarningBlink(void){
	if(HAL_GetTick() - LowVoltageBlink_previousMillis >= LowVoltageBlink_pulse_Interval){
		if(LowVoltageBlink_pulse_index){
			SetStatusLED(1000);
			LowVoltageBlink_pulse_index = 0;
		}
		else{
			SetStatusLED(300);
			LowVoltageBlink_pulse_index = 1;
		}
		LowVoltageBlink_previousMillis = HAL_GetTick();
	}
}

float powerReduction(float powerRequest){
	//CHECK IF CURRENT SHOULD BE REDUCED DUE TO TEMPERATURE
	//PI controller: the integrator settles on the power ceiling that holds
	//the PCB at maxTemp. Below maxTemp the ceiling saturates at 100 % and
	//nothing is reduced.
	uint32_t now = HAL_GetTick();
	float error = maxTemp - sensor_values.pcbTemperature; // positive = below target
	float dt = (now - thermal_previousMillis)/1000.0f;
	if (dt >= 0.1f){
		// powerReduction is not called while idle, so dt can span a whole
		// idle period. Integrate at most 1 s of it: otherwise a stale dt
		// multiplied by the momentary error slams the integrator to a rail
		// and the learned steady-state ceiling is lost.
		if (dt > 1.0f){
			dt = 1.0f;
		}
		if (temp_read_failures < TEMP_SENSOR_FAULT_LIMIT){
			thermal_integrator = clamp(thermal_integrator + THERMAL_KI*error*dt, 0.0f, 100.0f);
		}
		thermal_previousMillis = now;
	}
	float thermalCeiling = clamp(thermal_integrator + THERMAL_KP*error, 0.0f, 100.0f);

	// With a dead temperature sensor the reading is frozen and the
	// protection is blind: allow only a safe power level.
	if (temp_read_failures >= TEMP_SENSOR_FAULT_LIMIT && thermalCeiling > TEMP_FAULT_POWER_CEILING){
		thermalCeiling = TEMP_FAULT_POWER_CEILING;
	}

	if (thermalCeiling < powerRequest){
		temperatureLightReduction = powerRequest - thermalCeiling;
	}
	else{
		temperatureLightReduction = 0.0;
	}

	// Count limiter engagements in the health log (edge triggered)
	static uint8_t thermal_was_limiting = 0;
	if (temperatureLightReduction > 0.0f && !thermal_was_limiting){
		health.thermal_dim_events++;
		health_dirty = 1;
	}
	thermal_was_limiting = (temperatureLightReduction > 0.0f);

	//CHECK IF CURRENT SHOULD BE REDUCED DUE TO LOW VOLTAGE
	if (sensor_values.batteryVoltage < lowVoltage){
		voltageLightReduction = (powerRequest-5)*((lowVoltage-sensor_values.batteryVoltage)/(lowVoltage-minVoltage));
		if ((powerRequest - voltageLightReduction) < 5){
			voltageLightReduction = powerRequest - 5;
		}
	}
	else{
		voltageLightReduction = 0.0;
	}

	static uint8_t lv_was_active = 0;
	uint8_t lv_active = (sensor_values.batteryVoltage < lowVoltage) || (sensor_values.busVoltage < cutOffVoltage);
	if (lv_active && !lv_was_active){
		health.low_voltage_events++;
		health_dirty = 1;
	}
	lv_was_active = lv_active;

	//HARD LOW BUS VOLTAGE CUTOFF. Evaluated unconditionally so that no other
	//branch can bypass it. Latched until the bus recovers 0.5 V above the
	//cutoff (an unloaded battery rebounds once the load is removed).
	if (sensor_values.busVoltage <= cutOffVoltage){
		LowBusVoltage = 1;
	}
	else if (sensor_values.busVoltage > cutOffVoltage + 0.5f){
		LowBusVoltage = 0;
	}
	if (LowBusVoltage){
		lowVoltageWarningBlink();
		LedPower = 0;
		return LedPower;
	}

	//BATTERY RESERVE ("get home") MODE. See the defines for the concept.
	//The entry step is made deliberately noticeable (two quick dips), in
	//contrast to the thermal limiter which is meant to be invisible.
	if (reserveVoltage > 0.0f){
		if (!ReserveMode){
			if (sensor_values.batteryVoltage < reserveVoltage){
				if (!reserve_below){
					reserve_below = 1;
					reserve_belowSinceMillis = now;
				}
				else if (now - reserve_belowSinceMillis >= RESERVE_DEBOUNCE_MS){
					ReserveMode = 1;
					for (int d = 0; d < 2; d++){
						WATCHDOG_KICK();
						SetDAC(0);
						HAL_Delay(120);
						SetDAC(RESERVE_POWER);
						HAL_Delay(120);
					}
				}
			}
			else{
				reserve_below = 0;
			}
		}
		else if (sensor_values.batteryVoltage > reserveReleaseVoltage){
			//Far above the empty threshold: the battery was replaced
			ReserveMode = 0;
			reserve_below = 0;
		}
	}
	if (ReserveMode){
		lowVoltageWarningBlink();
		LedPower = (powerRequest > RESERVE_POWER) ? RESERVE_POWER : powerRequest;
		return LedPower;
	}

	if((temperatureLightReduction > 0.0f) || (sensor_values.batteryVoltage < lowVoltage)){
		if(temperatureLightReduction > voltageLightReduction){
			if (temp_read_failures >= TEMP_SENSOR_FAULT_LIMIT){
				//Rapid blink: temperature sensor fault
				SetStatusLED(((HAL_GetTick()/150) & 1) ? 1000 : 0);
			}
			else{
				SetStatusLED(1000);
			}
			LedPower=powerRequest-temperatureLightReduction;
		}
		else{
			lowVoltageWarningBlink();
			LedPower=powerRequest-voltageLightReduction;
		}
	}
	else{
		SetStatusLED(0);
    	LedPower = powerRequest;
	}

	return LedPower;
}

/* Smoothly ramp the LED power from one level to another over duration_ms.
 * The ramp is linear in perceived lightness (CIE: perception ~ cube root
 * of output), so the transition looks even to the eye instead of
 * front-loaded. Every step passes through powerReduction, so the thermal,
 * low-voltage and sensor-fault limits stay in force during the ramp. */
void ramp_LedPower(float from, float to, uint32_t duration_ms){
	WATCHDOG_KICK();
	if (duration_ms == 0){
		SetDAC(powerReduction(to));
		return;
	}
	float lightness_from = cbrtf(from);
	float lightness_to = cbrtf(to);
	for (uint32_t t = 0; t <= duration_ms; t++){
		float lightness = lightness_from + (lightness_to - lightness_from)*((float)t/(float)duration_ms);
		SetDAC(powerReduction(lightness*lightness*lightness));
		HAL_Delay(1);
	}
}

//Returns the average of 40 n'th reading in the adc1_buf vector
float get_mean_ADC_reading(uint8_t index){
	float ADC_filter_Mean = 0;
	for(int n=index;n<80;n=n+2){
		ADC_filter_Mean += adc1_buf[n];
	}
	return ADC_filter_Mean/40.0;
}

//Updates the PCB temperature in deg C. A failed I2C read is discarded so the
//error value never enters the moving average; the last good value is kept.
//Consecutive failures are counted: a permanently dead sensor would freeze
//the reading and blind the over-temperature protection, so powerReduction
//limits the power once the failure count reaches TEMP_SENSOR_FAULT_LIMIT.
void get_PCBTemp(){
	float temperature = TMP1075_Read_Temperature(&hi2c2);
	if (temperature > TMP1075_READ_ERROR + 1.0f){
		sensor_values.pcbTemperature = Moving_Average_Compute(temperature, &temperature_filterStruct);
		temp_read_failures = 0;
	}
	else if (temp_read_failures < TEMP_SENSOR_FAULT_LIMIT){
		temp_read_failures++;
	}
}

void get_BusAndBatteryVoltage(){
	// Get the DC voltage, reduce by voltage drop (approx 0.4 V at full power)
	//Index 1 is bus Voltage
	sensor_values.busVoltage = Moving_Average_Compute(get_mean_ADC_reading(1), &voltage_filterStruct)*BUS_VOLTAGE_FACTOR;
	sensor_values.esitmatedBusCurrent = (Current_SetCurrentPercent*0.9)/sensor_values.busVoltage;
	sensor_values.batteryVoltage = sensor_values.busVoltage + sensor_values.esitmatedBusCurrent*0.06;
}

void get_HallButtonState(){
	//Index 0 is Hall Button state
	float button = Moving_Average_Compute(get_mean_ADC_reading(0), &button_filterStruct);
	if (button < 2000){
		sensor_values.buttonState = 0;
		HAL_GPIO_WritePin(GPIOA, RED_LED_Pin, RESET);
	}
	else{
		sensor_values.buttonState = 1;
		HAL_GPIO_WritePin(GPIOA, RED_LED_Pin, SET);
	}
}

void blink_mainLED(uint32_t blink_number, uint32_t pulse_on_length, uint32_t pulse_off_length, float power){
	SetDAC(powerReduction(power));
	for(int i=0;i<blink_number;i++){
		WATCHDOG_KICK();
		EnableLedArray(1);
		HAL_Delay(pulse_on_length);
		EnableLedArray(0);
		HAL_Delay(pulse_off_length);
	}
	SetDAC(0);
}

void blink_statusLED(uint32_t blink_number, uint32_t blinkspeed){
	for(int i=0;i<blink_number;i++){
		WATCHDOG_KICK();
		SetStatusLED(1000);
		HAL_Delay(20);
		SetStatusLED(0);
		HAL_Delay(blinkspeed);
	}
}

void get_ButtonStatus(){
	get_HallButtonState();
	// CHECK IF BUTTON IS PRESSED
	if(!sensor_values.last_buttonState  && sensor_values.buttonState) {       // button is pressed
    pressedTime = HAL_GetTick();
  	Pressed = 1;
  	Pressed_SOSmode = 1;
	}

	// CHECK IF BUTTON IS RELEASED
	else if(sensor_values.last_buttonState && !sensor_values.buttonState) { // button is released
		releasedTime = HAL_GetTick();
		pressDuration = releasedTime - pressedTime;
		Pressed = 0;
		Pressed_SOSmode = 0;
		if(pressDuration < SHORT_PRESS_TIME && pressDuration > DEBOUNCE_PRESS_TIME){
			shortPress = 1;
		}
	}

	// CHECK IF BUTTON IS longPressed
	if (Pressed && (HAL_GetTick() - pressedTime) >= SHORT_PRESS_TIME ){
		longPress = 1;
		Pressed = 0;
	}

	if(Pressed_SOSmode && (releasedTime<pressedTime ) && ((HAL_GetTick() - pressedTime) >= SOSMODE_PRESS_TIME) && activeState == ST_RUN){
		SOSmode=1;
		SOS_pulse_index = 0;
		Pressed_SOSmode = 0;
	}

  // save the the last state
  sensor_values.last_buttonState = sensor_values.buttonState;

}

/* Blink the number of the presented menu item on the status LED and restart
 * the item presentation timer. */
void settings_present_item(uint8_t item_number){
	blink_statusLED(item_number, 300);
	status_previousMillis = HAL_GetTick();
}

/* Programming mode. Entered by holding the button while plugging in power.
 *
 * Navigation (see the menu table in the file header):
 *  - The status LED presents the current item by blinking its number.
 *    Every 2 s the menu advances to the next item (wrapping around).
 *  - Short press selects the presented item:
 *      * Top menu: the main LEDs confirm with the menu number and the
 *        sub-menu is entered. Menu 7 blinks the firmware version instead.
 *      * Sub-menu: the choice is stored in the pending settings (RAM), the
 *        main LEDs confirm with the option number and the top menu restarts.
 *  - Long press writes the pending settings to flash and exits.
 *  - Factory reset (menu 6, option 1 = YES) writes the default settings to
 *    flash and exits immediately. Option 2 = NO stores 0 and returns to the
 *    top menu.
 */
void run_settings_menu(){
	uint8_t menu = 0;       // current top-menu item (0-based)
	uint8_t option = 0;     // current sub-menu option (0-based)
	uint8_t in_submenu = 0;
	uint8_t end_settings = 0;

	blink_statusLED(10,50); // 10 rapid flashes: programming mode entered
	HAL_Delay(1000);
	settings_present_item(1);

	// The button may still be held from power-on. Sync the edge detector so
	// the hold is not re-detected as a fresh press, which would select the
	// presented item or (after 800 ms) instantly save and exit.
	get_HallButtonState();
	sensor_values.last_buttonState = sensor_values.buttonState;
	Pressed = 0;
	Pressed_SOSmode = 0;
	shortPress = 0;
	longPress = 0;

	while (!end_settings){

		WATCHDOG_KICK();
		get_ButtonStatus();

		if (shortPress){
			shortPress = 0;

			if (!in_submenu){
				// Confirm the chosen menu with the main LEDs
				blink_mainLED(menu + 1, 20, 300, 5);

				if (settings_menus[menu].option_count == 0){
					// Menu 7: blink firmware version, then restart the top menu
					HAL_Delay(300);
					blink_statusLED(major_version,300);
					HAL_Delay(600);
					blink_statusLED(minor_version,300);
					HAL_Delay(300);
					menu = 0;
				}
				else{
					in_submenu = 1;
					option = 0;
				}
				settings_present_item(1);
			}
			else if (menu == SETTINGS_FACTORY_RESET_MENU && option == 0){
				// Factory reset YES: write defaults to flash and exit
				end_settings = 1;
				blink_statusLED(20,50); // 20 rapid flashes as confirmation
				settings_save(settings_data_default);
			}
			else{
				// Store the choice in the pending settings.
				// Factory reset NO stores 0, everything else its option number.
				uint8_t value = (menu == SETTINGS_FACTORY_RESET_MENU) ? 0 : option + 1;
				Flash_Rx_Data[settings_menus[menu].settings_index] = value;

				// Confirm the choice with the main LEDs, then restart the top menu
				blink_mainLED(value, 20, 300, 5);
				in_submenu = 0;
				menu = 0;
				settings_present_item(1);
			}
		}

		// Present the next item when the current one has been shown long enough
		if (HAL_GetTick() - status_previousMillis >= SETTINGS_ITEM_INTERVAL){
			if (!in_submenu){
				menu = (menu + 1) % SETTINGS_MENU_COUNT;
				settings_present_item(menu + 1);
			}
			else{
				option = (option + 1) % settings_menus[menu].option_count;
				settings_present_item(option + 1);
			}
		}

		// Long press: write the pending settings to flash and exit
		if (longPress){
			longPress = 0;
			settings_save(Flash_Rx_Data);
			end_settings = 1;
			blink_statusLED(10,50); // 10 rapid flashes: leaving programming mode
		}
	}
}

void AnimateIdleLED(){
	// Update Indicator led
	if(HAL_GetTick() - led_pulse_previousMillis >= led_pulse_Interval){

		if(idle_blink_mode == 1){ //Pulse mode
			if(led_pulse_index>=230){
				led_pulse_index=0;
			}
			SetStatusLED(statusLedArr_pulse[led_pulse_index]);
		}
		else if(idle_blink_mode == 2){ //Breath mode
			if(led_pulse_index>=600){
				led_pulse_index=0;
			}
			SetStatusLED(statusLedArr_breath[led_pulse_index]);
		}
		else{ //LED off
			SetStatusLED(0);
		}
		led_pulse_previousMillis = HAL_GetTick();
		led_pulse_index++;
	}
}

void get_lastPowerSetting(){
	Flash_Read_Data(FLASH_USER_LAST_POWER_ADDR, Flash_Tx_Data, 1);
	arrayIndex = Flash_Tx_Data[0];
	if(arrayIndex >= LedPowerArrayLenght){
		// Saved index is not valid for the currently configured power modes
		// (e.g. the power mode setting was changed to a shorter list)
		arrayIndex = 0;
	}
	if(remember_power_plus_one){
		arrayIndex = (arrayIndex + 1) % LedPowerArrayLenght;
		Flash_Tx_Data[0] = arrayIndex;
		Flash_Write_Data(FLASH_USER_LAST_POWER_ADDR, (uint32_t *)Flash_Tx_Data, 1);
		HAL_Delay(10);
	}
}

void GetSensorData(){
	get_PCBTemp();
	get_BusAndBatteryVoltage();
	get_ButtonStatus();
}

/* Accumulate run time and health records. Saves are rate limited to
 * protect flash endurance: record/event changes at most every 2 min,
 * run time every 30 minutes of accumulated on-time. */
void health_task(void){
	static uint32_t previousMillis = 0;
	uint32_t now = HAL_GetTick();
	if (previousMillis == 0){ // first call: just start the clock
		previousMillis = now;
		return;
	}
	uint32_t delta = now - previousMillis;
	previousMillis = now;

	if (activeState == ST_RUN){
		health_run_ms += delta;
		while (health_run_ms >= 60000){
			health_run_ms -= 60000;
			health.on_minutes++;
		}

		// Lowest bus voltage under load. Ignore implausible readings.
		uint32_t voltage_mV = (uint32_t)(sensor_values.busVoltage*1000.0f);
		if (voltage_mV > 1000 && voltage_mV + 100 < health.min_voltage_mV){
			health.min_voltage_mV = voltage_mV;
			health_dirty = 1;
		}
	}

	// Highest PCB temperature seen (at least +0.5 degC to limit writes)
	int32_t temperature_dC = (int32_t)(sensor_values.pcbTemperature*10.0f);
	if (temperature_dC > health.max_temperature_dC + 5){
		health.max_temperature_dC = temperature_dC;
		health_dirty = 1;
	}

	if (health_dirty && (now - health_lastSaveMillis) >= 120000){
		health_save();
	}
	else if (health.on_minutes >= health_saved_minutes + 30){
		health_save();
	}
}

/* Blocking UART prints. At 115200 baud a full line takes a few ms, which the
 * main loop tolerates. Interrupt-based transmit silently dropped messages
 * (HAL_BUSY) and reused its buffer while a previous transfer was in flight. */
#define UART_TX_TIMEOUT_MS 100

void debugPrint(UART_HandleTypeDef *huart, char _out[]){
	HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), UART_TX_TIMEOUT_MS);
}

void debugPrintln(UART_HandleTypeDef *huart, char _out[]) {
	debugPrint(huart, _out);
	debugPrint(huart, "\r\n");
}

void debugPrintDUO(UART_HandleTypeDef *huart, char _out1[], char _out2[]) {
	debugPrint(huart, _out1);
	debugPrint(huart, _out2);
}

void debugPrintTRIO(UART_HandleTypeDef *huart, char _out1[], char _out2[], char _out3[]) {
	debugPrint(huart, _out1);
	debugPrint(huart, _out2);
	debugPrint(huart, _out3);
}

//Commands for initializing RN4870/71 Bluetooth module
void setup_BLE_module(){
	WATCHDOG_KICK();
	HAL_Delay(500);
	blink_statusLED(1,300); // one blink
	debugPrintDUO(&huart2,"SEND: ", "$$$\n");
	debugPrint(&huart1,"$$$");// Enter Command Mode

	WATCHDOG_KICK();
	HAL_Delay(500);
	blink_statusLED(1,300); // one blink
	debugPrintDUO(&huart2,"SEND: ", "SF, 1\n");
	debugPrintln(&huart1,"SF,1");// Factory reset

	WATCHDOG_KICK();
	HAL_Delay(500);
	blink_statusLED(1,300); // one blink
	debugPrintDUO(&huart2,"SEND: ", "$$$\n");
	debugPrint(&huart1,"$$$");// Enter Command Mode

	WATCHDOG_KICK();
	HAL_Delay(500);
	blink_statusLED(1,300); // one blink
	debugPrintDUO(&huart2,"SEND: ", "V\n");
	debugPrintln(&huart1,"V");// Get version

	WATCHDOG_KICK();
	HAL_Delay(500);
	blink_statusLED(1,300); // one blink
	debugPrintDUO(&huart2,"SEND: ", "S:,0000,010077777777\n");// Set MAC Address to 777777770001
	debugPrintln(&huart1,"S:,0000,010077777777");// Get version

	WATCHDOG_KICK();
	HAL_Delay(500);
	blink_statusLED(1,300); // one blink
	debugPrintDUO(&huart2,"SEND: ", "S-,AxxNNN\n");
	debugPrintln(&huart1,"S-,AxxBeam");// Set Device Name

	//HAL_Delay(1000);
	//blink_statusLED(1,300); // one blink
	//debugPrintDUO(&huart2,"SEND: ", "SA,3\n");
	//debugPrintln(&huart1,"SA,1");// Set Authentication method
	//HAL_Delay(1000);
	//blink_statusLED(1,300); // one blink
	//debugPrintDUO(&huart2,"SEND: ", "SP,123456\n");
	//debugPrintln(&huart1,"SP,123456");// Set Security Pin

	WATCHDOG_KICK();
	HAL_Delay(500);
	blink_statusLED(1,300); // one blink
	debugPrintDUO(&huart2,"SEND: ", "SS,C0\n");
	debugPrintln(&huart1,"SS,C0");// Support Device Info and UART Transparent services


	WATCHDOG_KICK();
	HAL_Delay(500);
	blink_statusLED(1,300); // one blink
	debugPrintDUO(&huart2,"SEND: ", "SB,03,01,02,00\n");// 115200, even parity
	debugPrintln(&huart1,"SB,03,01,02,00");// 115200, even parity

	WATCHDOG_KICK();
	HAL_Delay(500);
	blink_statusLED(1,300); // one blink
	debugPrintDUO(&huart2,"SEND: ", "R,1\n");
	debugPrintln(&huart1,"R,1");// Reboot device for configuration to take effect

	blink_statusLED(5,100); // 5 blinks
}
void factory_reset_BLE_module(){
	WATCHDOG_KICK();
	HAL_Delay(2000);
	blink_statusLED(1,300); // one blink
	debugPrintDUO(&huart2,"SEND: ", "$$$\n");
	debugPrint(&huart1,"$$$");// Enter Command Mode
	WATCHDOG_KICK();
	HAL_Delay(2000);
	blink_statusLED(1,300); // one blink
	debugPrintDUO(&huart2,"SEND: ", "SF, 1\n");
	debugPrintln(&huart1,"SF,1");// Factory reset
	WATCHDOG_KICK();
	HAL_Delay(2000);
}
// Reset up Microchip RN4870/71 BLE
void BLE_reset(){
	HAL_GPIO_WritePin(GPIOA, BLE_MODE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BLE_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOA, BLE_RST_Pin, GPIO_PIN_SET);
}

void jump2STinternalBootloader(void){
	void (*SysMemBootJump)(void);
	volatile uint32_t addr = 0x1FFF0000;

	/**
	* Disable RCC, set it to default (after reset) settings
	* Internal clock, no PLL, etc.
	*/
	HAL_RCC_DeInit();

	/**
	* Disable systick timer and reset it to default values
	*/
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	//reset all periphals
	HAL_DeInit();

	__disable_irq();

	/**
	* Remap system memory to address 0x0000 0000 in address space
	*/
	__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

	/**
	* Set jump memory location for system memory
	* Use address with 4 bytes offset which specifies jump location where program starts
	*/
	SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4)));

	/**
	* Set main stack pointer.
	* This step must be done last otherwise local variables in this function
	* don't have proper value since stack pointer is located on different position
	*
	* Set direct address location which specifies stack pointer in SRAM location
	*/
	__set_MSP(*(uint32_t *)addr);

	/**
	* Re-enable interrupts so the system bootloader starts in the same state
	* as after a reset (PRIMASK cleared). Leaving them disabled can prevent
	* the ROM bootloader from communicating.
	*/
	__enable_irq();

	/**
	* Actually call our function to jump to set location
	* This will start system memory execution
	*/
	SysMemBootJump();
	while(1);
}

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC3_Init();
  MX_OPAMP1_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	/* Jump to the ROM bootloader if requested before the reset. Must happen
	   before the watchdog is started: the IWDG cannot be stopped and would
	   reset the bootloader in the middle of an OTA flash. */
	if (bootloader_reboot_requested()){
		jump2STinternalBootloader();
	}

	watchdog_start();

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buf, ADC1_BUF_LEN);//Start ADC DMA
	HAL_TIM_Base_Start(&htim3);//Start Indicator LED Timer
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2); //Start Indicator LED Pwm signal
	HAL_OPAMP_Start(&hopamp1);//Start OPAMP
	HAL_DAC_Start(&hdac3, DAC_CHANNEL_1);//Start DAC
	HAL_UART_Receive_IT(&huart1, &byte, 1);

	BLE_reset();

	//Init Moving Average function
	Moving_Average_Init(&temperature_filterStruct, 20);
	Moving_Average_Init(&voltage_filterStruct, 20);
	Moving_Average_Init(&button_filterStruct, 10);

	/* Init and fill filter structures with initial values */
	for (int i = 0; i<200;i++){
		WATCHDOG_KICK();
		get_PCBTemp();
		get_BusAndBatteryVoltage();
		get_HallButtonState();
	}

	// Load the health log and count this power-up. The increment is
	// persisted later by the rate-limited save in health_task: writing
	// flash on every plug-in would wear the page and open a torn-write
	// window at the exact moment the connector is most likely to bounce.
	health_load();
	health.boot_count++;
	health_dirty = 1;

  	// First boot after full chip erase?
  	Flash_Read_Data(FLASH_USER_FIRST_START_ADDR, Flash_Rx_Data, 1);

	// If first boot
  	if(Flash_Rx_Data[0] != 0xAAAAAAAA){

  		settings_save(settings_data_default);

  		uint32_t data_first_boot[] = {0xAAAAAAAA};
  		Flash_Write_Data(FLASH_USER_FIRST_START_ADDR, (uint32_t *)data_first_boot, 1);
  		uint32_t data_last_power[] = {0x00000000};
  		Flash_Write_Data(FLASH_USER_LAST_POWER_ADDR, (uint32_t *)data_last_power, 1);

  		// Set up Microchip RN4870/71 BLE
  		BLE_reset();
  		WATCHDOG_KICK();
  		HAL_Delay(4000);
  		setup_BLE_module();
  	}


  	// If the button is held at power-on: enter programming mode
  	if(sensor_values.buttonState){
  		if (!settings_load(Flash_Rx_Data)){
  			memcpy(Flash_Rx_Data, settings_data_default, sizeof(settings_data_default));
  		}
  		run_settings_menu();
  	}

  	// Read settings at start-up. Invalid CRC (erased, corrupt or pre-CRC
  	// flash content) restores the defaults.
  	if (!settings_load(Flash_Rx_Data)){
  		memcpy(Flash_Rx_Data, settings_data_default, sizeof(settings_data_default));
  		settings_save(Flash_Rx_Data);
  	}

  	/* Power mode tables. Must be static: LedPowerArray keeps pointing at the
  	   selected table for the lifetime of the program. */
  	static const float powerModeArray_1[5] = {5, 20, 50, 70, 100}; //100% is 90 W
  	static const float powerModeArray_2[3] = {5, 50, 100}; //Default
  	static const float powerModeArray_3[2] = {5, 100};
  	static const float powerModeArray_4[2] = {5, 70};
  	static const float powerModeArray_5[1] = {100};
  	static const float powerModeArray_6[1] = {50};
  	static const float powerModeArray_7[1] = {5};

  	if(Flash_Rx_Data[0]== 1){
  		LedPowerArray = powerModeArray_1;
  		LedPowerArrayLenght = 5;
  	}
  	else if(Flash_Rx_Data[0]== 2){
  		LedPowerArray = powerModeArray_2;
  		LedPowerArrayLenght = 3;
  	}
  	else if(Flash_Rx_Data[0]== 3){
  		LedPowerArray = powerModeArray_3;
  		LedPowerArrayLenght = 2;
  	}
  	else if(Flash_Rx_Data[0]== 4){
  		LedPowerArray = powerModeArray_4;
  		LedPowerArrayLenght = 2;
  	}
  	else if(Flash_Rx_Data[0]== 5){
  		LedPowerArray = powerModeArray_5;
  		LedPowerArrayLenght = 1;
  	}
  	else if(Flash_Rx_Data[0]== 6){
  		LedPowerArray = powerModeArray_6;
  		LedPowerArrayLenght = 1;
  	}
  	else if(Flash_Rx_Data[0]== 7){
  		LedPowerArray = powerModeArray_7;
  		LedPowerArrayLenght = 1;
  	}
  	else{
  		// Unknown or corrupt setting: fall back to the default power modes
  		LedPowerArray = powerModeArray_2;
  		LedPowerArrayLenght = 3;
  	}

  	// Power source: dimming thresholds plus the reserve mode thresholds.
  	// Reserve latches at ~2.9 V/cell under load and releases only at a
  	// voltage an empty pack cannot rebound to (i.e. battery replaced).
  	if(Flash_Rx_Data[1]== 1){ // 3S Li-ion
  		lowVoltage = 9;
  		minVoltage = 8.1;
  		reserveVoltage = 8.7;
  		reserveReleaseVoltage = 10.7;
  	  	}
  	  	else if(Flash_Rx_Data[1]== 2){ // 4S Li-ion
			lowVoltage = 12;
			minVoltage = 10;
			reserveVoltage = 11.6;
			reserveReleaseVoltage = 14.2;
  	  	}
  	  	else if(Flash_Rx_Data[1]== 3){ // 12V lead acid
			lowVoltage = 11.5;
			minVoltage = 10.5;
			reserveVoltage = 10.5;
			reserveReleaseVoltage = 12.4;
  	  	}
  	  	else if(Flash_Rx_Data[1]== 4){ // DC source: no battery to protect
			lowVoltage = 7.1;
			minVoltage = 7;
			reserveVoltage = 0;
			reserveReleaseVoltage = 0;
	}

	if(Flash_Rx_Data[2]== 1){
		immediateStart = 0;
	}
	else if(Flash_Rx_Data[2]== 2){
		immediateStart = 1;
	}


	if(Flash_Rx_Data[3]== 1){
		remember_power = 0;
		remember_power_plus_one = 0;
	}
	else if(Flash_Rx_Data[3]== 2){
		remember_power = 1;
		remember_power_plus_one = 0;
		}
	else if(Flash_Rx_Data[3]== 3){
		remember_power = 1;
		remember_power_plus_one = 1;
		}

	if(Flash_Rx_Data[4]== 1){
			idle_blink_mode = 1;
		}
	else if(Flash_Rx_Data[4]== 2){
			idle_blink_mode = 2;
		}
	else if(Flash_Rx_Data[4]== 3){
			idle_blink_mode = 3;
		}

  	if (immediateStart){
  		activeState = ST_RUN;
  	}
  	else{
  		activeState = ST_IDLE;
  	}

  	if(remember_power){
  		get_lastPowerSetting();
	}


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  WATCHDOG_KICK();
	  GetSensorData();
	  health_task();
	  //Main Switch
	  switch (activeState) {
            case ST_IDLE: {
                // Turn off LED
    			SetDAC(0);
    			EnableLedArray(0);

    			shortPress =  0; // ignore shortpresses

    			//longPress brings Switch to next Case
    			if (longPress){
    				longPress=0;
	                activeState = ST_RUN;
	                EnableLedArray(1);

	                if(remember_power){
	                	get_lastPowerSetting();
	                }
	                else{
	                	arrayIndex = 0;
	                }

					ramp_LedPower(0, LedPowerArray[arrayIndex], 400);
					LedPower = LedPowerArray[arrayIndex];
    			}

    			AnimateIdleLED();

    			break;
            }

            case ST_RUN: {
        		//Random Blink mode
				if(LedPowerArray[arrayIndex] == 101){
					EnableLedArray(0);
					if(HAL_GetTick() - blink_previousMillis >= blink_Interval){
						blink_mainLED(next_blink_number, next_pulse_length, 10, 90);
						blink_Interval = (rand() % 5900)*10 + 1000;
						blink_previousMillis = HAL_GetTick();

						if(blink_Interval < 32000 && blink_Interval > 30000)
						{
							next_blink_number = 1;
							next_pulse_length = 2000;
						}
						else if(blink_Interval < 40000 && blink_Interval > 35000)
						{
							next_blink_number = 10;
							next_pulse_length = 50;
						}

						else if(blink_Interval < 2000)
						{
							next_blink_number = 5;
							next_pulse_length = 50;
						}
					}
				}
				// SOS morse code
				else if(SOSmode){
					if(HAL_GetTick() - SOS_pulse_previousMillis >= SOS_pulse_Interval){
						EnableLedArray(SOS_morse[SOS_pulse_index]);
						SetDAC(powerReduction(90));
						SOS_pulse_index++;
						if(SOS_pulse_index>39){
							SOS_pulse_index=0;
						}
						SOS_pulse_previousMillis = HAL_GetTick();
					}

				}
        		else{
					//Set LED Power
					EnableLedArray(1);
					LedPower = powerReduction(LedPowerArray[arrayIndex]);
					SetDAC(LedPower);
				}

    			// Cycle between output power levels in "LedPowerArray"
    			if (shortPress){
    				shortPress=0;
    				ramp_LedPower(LedPower, LedPowerArray[(arrayIndex+1) % LedPowerArrayLenght], 400);
	                arrayIndex = (arrayIndex+1) % LedPowerArrayLenght;


	                if(remember_power){
	                	Flash_Tx_Data[0] = arrayIndex;
	                	Flash_Write_Data(FLASH_USER_LAST_POWER_ADDR, (uint32_t *)Flash_Tx_Data, 1);
	                	HAL_Delay(10);
	                }
    			}

    			//Turn off LED if longPress
    			else if (longPress){
    				SOSmode = 0;
    				longPress=0;
    				//Dim down to 0
    				ramp_LedPower(LedPower, 0, 400);
					LedPower = 0;
    				activeState = ST_IDLE;
    				arrayIndex = 0;
    			}
                break;
            }
        }

		if (line_valid==1){ // A new line has arrived
			// line_valid stays set while parsing: the RX interrupt will not
			// touch line_buffer until it is cleared at the end.
			debugPrintTRIO(&huart2,"RESPONCE: ", line_buffer, "\n"); // echo received line

			if (strstr(line_buffer, "__OTA__") != NULL){
				debugPrintln(&huart1,"Received OTA Request, going into Bootloader");
				health_save(); // persist before the reset
				request_bootloader_reboot(); // resets; the jump happens on the next boot before the watchdog starts
			}

			else if (strstr(line_buffer, "__LONGPRESS__") != NULL){
				debugPrintln(&huart1,"Received LONG Press Request");
				longPress = 1;
			}

			else if (strstr(line_buffer, "__SHORTPRESS__") != NULL){
				debugPrintln(&huart1,"Received SHORT Press Request");
				shortPress = 1;
			}

			else if (strstr(line_buffer, "__VERSION__") != NULL){
		  		snprintf(buffer, sizeof(buffer), "AxxBeam Version: %u.%u", major_version, minor_version);
		  		debugPrintln(&huart1, buffer);
			}

			else if (strstr(line_buffer, "__HEALTH__") != NULL){
				char health_buffer[96];
				snprintf(health_buffer, sizeof(health_buffer),
					"Boots:%lu On:%luh%02lum MaxT:%ld.%ldC MinV:%lu.%02luV ThermDim:%lu LowVolt:%lu",
					health.boot_count,
					health.on_minutes/60, health.on_minutes%60,
					health.max_temperature_dC/10, labs(health.max_temperature_dC%10),
					health.min_voltage_mV/1000, (health.min_voltage_mV%1000)/10,
					health.thermal_dim_events, health.low_voltage_events);
				debugPrintln(&huart1, health_buffer);
			}

			line_valid = 0; // done with line_buffer, release it to the RX interrupt
		}
		if(HAL_GetTick() - status_previousMillis >= status_Interval){
			memset(&buffer, '\0', sizeof(buffer));
			snprintf(buffer, sizeof(buffer), "T:%3.1f V:%3.1f P:%3.1f C:%3.1f", sensor_values.pcbTemperature, sensor_values.batteryVoltage, LedPower, sensor_values.esitmatedBusCurrent);
			debugPrintln(&huart1,buffer);
			status_previousMillis = HAL_GetTick();
		}

		/* Send debug information */
		if(1==1){//flash_values.serial_debug_print == 1){
			if(HAL_GetTick() - previous_millis_debug >= interval_debug){
				/*UART_packet_length = 9*sizeof(float);
				pack_frame_start(UART_transmit_buffer, &UART_packet_index, UART_packet_length);
				pack_float(UART_transmit_buffer, &UART_packet_index, (float)3.33);
				pack_float(UART_transmit_buffer, &UART_packet_index, (float)3.33);
				pack_float(UART_transmit_buffer, &UART_packet_index, (float)3.33);
				pack_float(UART_transmit_buffer, &UART_packet_index, (float)3.33);
				pack_float(UART_transmit_buffer, &UART_packet_index, (float)3.33);
				pack_float(UART_transmit_buffer, &UART_packet_index, (float)3.33);
				pack_float(UART_transmit_buffer, &UART_packet_index, (float)3.33);
				pack_float(UART_transmit_buffer, &UART_packet_index, (float)3.33);
				pack_float(UART_transmit_buffer, &UART_packet_index, (float)3.33);

				HAL_UART_Transmit_DMA(&huart2,(uint8_t*)UART_transmit_buffer, UART_packet_length+2); // Add two for starting bit and packet length
				*/


				float values[3];

				values[0] = sensor_values.pcbTemperature;
				values[1] = sensor_values.busVoltage;
				values[2] = LedPower;


				//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)values, sizeof(values));

				previous_millis_debug = HAL_GetTick();
			}
		}

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DAC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC3_Init(void)
{

  /* USER CODE BEGIN DAC3_Init 0 */

  /* USER CODE END DAC3_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC3_Init 1 */

  /* USER CODE END DAC3_Init 1 */

  /** DAC Initialization
  */
  hdac3.Instance = DAC3;
  if (HAL_DAC_Init(&hdac3) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC3_Init 2 */

  /* USER CODE END DAC3_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00503D58;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp1.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_DAC;
  hopamp1.Init.InternalOutput = DISABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 80-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 256000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ENABLE_Pin|RED_LED_Pin|BLE_RST_Pin|BLE_MODE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENABLE_Pin RED_LED_Pin BLE_RST_Pin BLE_MODE_Pin */
  GPIO_InitStruct.Pin = ENABLE_Pin|RED_LED_Pin|BLE_RST_Pin|BLE_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
