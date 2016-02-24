
#include <asf.h>
#include "FT800.h"
#include "FT_GPU.h"

#include "Shiftreg.h"
#include "check.h"
#include "CANdefinitions.h"
#include "Interrupts_and_can.h"
#include "flashc_array.h"
//#include "revolve_logo.h"

void doneInitialize(void);
void getOldAdjustments(void);

//EXTERN VARIABLES

//Arrays 
extern ft_uint8_t number_of_adjustments[6];
extern ft_uint8_t type_of_adjustment[6];
extern ft_char8_t *adjustment_devices[6];
extern ft_char8_t *interval_onoff[6];
extern ft_char8_t *interval_5_postoneg[6];
extern ft_char8_t *interval_5_pos[6];
extern bool isAlive[DASH_ALIVE_SIZE]; 
extern bool isAlive_check[DASH_ALIVE_SIZE];
extern ft_uint8_t adjustment_value[6];
extern ft_uint8_t new_adjustment_value[6];
extern ft_uint16_t adjustment_value_ECU[3];

//Variables received on CAN
extern uint16_t rpm;
extern int16_t torque;
extern uint16_t motor_temp;
extern uint8_t battery_temp;
extern uint8_t min_battery_temp;
extern uint8_t max_battery_temp;
extern uint16_t IGBT_temp;
extern uint16_t launch_traction;
extern uint16_t old_launchActivated;
extern uint16_t DC_bus_voltage;
extern uint16_t BSPD_status;
extern uint8_t leftOnBattery;
extern uint8_t min_cell_voltage;
extern uint8_t max_cell_voltage;
extern uint8_t min_cell_id;
extern uint8_t max_cell_id;

extern uint16_t pack_voltage;
extern uint16_t brk_pres_rear;
extern uint16_t brk_pres_front;
extern uint16_t steer_ang;
extern uint16_t error_ecu;
extern uint16_t gearbox_temp;
extern uint16_t glv_voltage;
extern int16_t torque_encoder_ch0;
extern int16_t torque_encoder_ch1;
extern volatile uint8_t gotFeedback_ch0;
extern volatile uint8_t gotFeedback_ch1;
extern uint8_t pedal_position;
extern uint8_t tractive_status;

extern bool check_alive;
extern bool check_err_ecu;
extern bool check_drive;
extern bool play_RTDS;
extern bool driveEnabled;
extern bool monitor_status;
extern bool chechLaunch;
extern uint8_t launchActivated;
extern bool check_temp;
extern bool tryEnableDrive;
extern bool tryDisableDrive;
extern bool sendDriveEnable;
extern bool sendDriveDisable;
extern bool check_voltage;
extern uint8_t bms_fault_code;
extern uint8_t bms_warning;
extern bool valid_torque_ch1;
extern bool valid_torque_ch0;

//Timer variables
extern volatile bool timer_interrupted;
extern volatile uint8_t torqenc_timeout;
extern bool launchDoneCount;

//Interrupt variables
extern uint8_t launchAck;
extern uint8_t acknowledgeAck;
extern int8_t horizontal_position;
extern uint8_t vertical_level_adjustment_menu;
extern uint8_t vertical_level_adjustment;
extern uint8_t vertical_level_adjustment_spes;
extern uint8_t move_up;
extern uint8_t move_down;
extern uint8_t move_left;
extern uint8_t move_right;
extern uint8_t update;

can_msg_t msg_tx_ch0;
can_mob_t appli_tx_ch0;

wdt_opt_t opt = {
	.dar   = false,     // After a watchdog reset, the WDT will still be enabled.
	.mode  = WDT_BASIC_MODE,    // The WDT is in basic mode, only PSEL time is used.
	.sfv   = false,     // WDT Control Register is not locked.
	.fcd   = false,     // The flash calibration will be redone after a watchdog reset.
	.cssel = WDT_CLOCK_SOURCE_SELECT_RCSYS,       // Select the system RC oscillator (RCSYS) as clock source.
	.us_timeout_period = 1500000  // Timeout Value micro seconds
};

//deklarer power_on_reset globalt i main.c
//static volatile uint8_t power_on_reset = 0;

//funksjonsprototype. Legg funksjonen i main
void wdt_scheduler(void);
void wdt_scheduler(void) {
	// Watchdog reset
	if(AVR32_PM.RCAUSE.wdt) {
		/* Havner her hvis watchdogen har restartet kortet. 
		 * Reinitialiser data eller hent det frem hvis du har lagret det et sted */
		power_on_reset = 0;
		wdt_reenable();
	} else if (AVR32_PM.RCAUSE.por) {
		/* Havner her når kortet starter opp (power on reset) */
		power_on_reset = 1;
		wdt_enable(&opt);
	} else {
		power_on_reset = 0;
		wdt_enable(&opt);
	}
}

int main (void)
{	
	board_init();
	//Received from shift registers
	uint8_t receivedShift_SR2;
	uint8_t receivedShift_SR2_test;
	uint8_t receivedShift_SR1;
	uint8_t receivedShift_SR1_test;
	//Adjustment parameters
	uint16_t kersValue;
	uint16_t sliprefValue;
	uint8_t start_value;
	//Compare registers to find out if switch values have changed
	uint8_t shiftReg1_value = 0;
	uint8_t shiftReg2_value = 0;
	//Arrays to send correct values to ECU
	uint16_t slipVec[5] = {1, 2, 3, 4, 5};
	uint16_t kersVec[7] = {0, 50, 60, 80, 70, 90, 100};
	//Compare arrays to find right adjusmtent	
	uint8_t valueVector_kers[7] = {0b01111110, 0b01111101, 0b01111011, 0b01110111, 0b01101111, 0b01011111, 0b00111111};
	uint8_t valueVector_slip[5] = {0b01111110, 0b01111101, 0b01111011, 0b01110111, 0b00111111};
	
	//Initialization
	
	ShiftRegister_Init();
	sysclk_enable_peripheral_clock(EXAMPLE_TC);
	FT800_Init();
	//Turn off LEDS - Initialized high
	gpio_set_pin_low(ECU_ERR_LED);
	gpio_set_pin_low(TMP_LED);
	gpio_set_pin_low(WARNING_LC);
	gpio_set_pin_low(ALIVE_LED);
	gpio_set_pin_low(VOLTAGE_LED);
	gpio_set_pin_low(TRACTIVEDRIVE_LED);
	#ifdef USE_WDT
		wdt_scheduler();
	#else
	//Her kan du initialisere data (hvis du trenger) når du ikke bruker watchdog
	#endif
	startScreen();
	Interrupt_CAN_Init();
	//Get main screen
	horizontal_position=1;
	getBackDisplay();
	
	//Blink LEDs
	doneInitialize();
	//Load adjustments from EEPROM
	//getOldAdjustments();
	
	while(1){
		
		receivedShift_SR2=0x7F;
		receivedShift_SR2_test=0x01;
		receivedShift_SR1=0x7F;
		receivedShift_SR1_test=0x01;
		
		//Read shift registers for new values
		while((receivedShift_SR2_test!=receivedShift_SR2)||(receivedShift_SR1_test!=receivedShift_SR1)||(receivedShift_SR1==0x7F) ||(receivedShift_SR1==0x5F) || (receivedShift_SR2==0x7F)){
			receivedShift_SR1 = readShiftRegister(CS_3);
			receivedShift_SR2 = readShiftRegister(CS_2);
			myDelay(100);
			receivedShift_SR1_test = readShiftRegister(CS_3);
			receivedShift_SR2_test = readShiftRegister(CS_2);
		}				//update shiftregisters if value have changed
		if((shiftReg1_value!=receivedShift_SR1)||(shiftReg2_value!=receivedShift_SR2)||(sendDriveEnable)||(sendDriveDisable)){
			//Add CAN ID
			appli_tx_ch0.can_msg->id =DASH_PRI_ID1;
			
			//Update compare value
			shiftReg1_value = receivedShift_SR1;
			shiftReg2_value = receivedShift_SR2;
			
			start_value = receivedShift_SR1<<2;
			start_value = start_value>>7;
			
			//Check if it is allowed to try to enable drive
			
			//KODE UNDER MÅ UTKOMMENTERES FOR Å FÅ RETT OPPFØRSEL, MEN USIKKER PÅ BRAKE PRESSURE!
		
			if((start_value==1)&&(tractive_status==0)){
				writeString("Turn on tractive before enabling drive!", 29, 240, 136);
				myDelay(2000);
				start_value = 0;
			}
			
			if((start_value==1)&&(brk_pres_front<4400)){
				writeString("Push brake before enabling drive", 29, 240, 136);
				myDelay(2000);	
				start_value = 0;
			}
			
			if((start_value==1)&&(!driveEnabled)&&(!play_RTDS)){
				tryEnableDrive=true;
				sendDriveEnable = false;
			}else if((start_value==0)&&(driveEnabled)){
				tryDisableDrive = true;
				sendDriveDisable = false;
			}
			
			//Add CAN msg values
			appli_tx_ch0.can_msg->data.u8[0]= start_value;
			appli_tx_ch0.can_msg->data.u8[1]= tractive_status;
			
			//Compare shifted values to get switch position
			/* OLAV MOD */
			kersValue = receivedShift_SR2;
			if (kersValue==valueVector_kers[1])
			{
				launchAck=1;
			}
			/*for(int i=0; i<7; i++){
				if(kersValue==valueVector_kers[i]){
					kersValue = kersVec[i];
					break;
				}
			}*/
			
			sliprefValue = receivedShift_SR1|0b00110000;
			for(int i=0; i<5; i++){
				if(sliprefValue==valueVector_slip[i]){
					sliprefValue = slipVec[i];
					break;
				}
			}
			
			//Add CAN msg values
			appli_tx_ch0.can_msg->data.u16[1] = kersValue;
			appli_tx_ch0.can_msg->data.u16[2] = sliprefValue;
			
			//can msg = [start tractive kers kers slipref slipref]
			
			can_tx(0,
			appli_tx_ch0.handle,
			6,
			appli_tx_ch0.req_type,
			appli_tx_ch0.can_msg);
			if((!tryDisableDrive)&&(!tryEnableDrive)){
				writeShiftregChange(start_value, kersValue, sliprefValue);
				myDelay(2000);
			}

			getBackDisplay();
			
		}
	//----------Update if change has occured in data from CAN or interrupts----------//
		if(update){
			checkStatus();
			update=0;
		}
		
	}
	
	
}


void doneInitialize(void){
	gpio_set_pin_high(ECU_ERR_LED);
	myDelay(200);
	gpio_set_pin_low(ECU_ERR_LED);
	gpio_set_pin_high(TMP_LED);
	myDelay(200);
	gpio_set_pin_low(TMP_LED);
	gpio_set_pin_high(WARNING_LC);
	myDelay(200);
	gpio_set_pin_low(WARNING_LC);
	gpio_set_pin_high(VOLTAGE_LED);
	myDelay(200);
	gpio_set_pin_low(VOLTAGE_LED);
	gpio_set_pin_high(ALIVE_LED);
	myDelay(200);
	gpio_set_pin_low(ALIVE_LED);
	gpio_set_pin_high(TRACTIVEDRIVE_LED);
	myDelay(200);
	gpio_set_pin_low(TRACTIVEDRIVE_LED);
}


void getOldAdjustments(void){
	adjustment_value[DRIVER] = saved_adjustments[DRIVER];
	adjustment_value[DAVE] = saved_adjustments[DAVE];
	adjustment_value_ECU[0] = saved_adjustments[ECU];
	adjustment_value_ECU[1] = saved_adjustments[ECU+1];
	adjustment_value_ECU[2] = saved_adjustments[ECU+2];
}
