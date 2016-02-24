/*
 * Interrupt.c
 *
 * Created: 20.03.2014 15:21:43
 *  Author: Øystein
 */ 

#include <asf.h>
#include "CANdefinitions.h"
#include "Interrupts_and_can.h"
#include "FT800.h"
//#include "Interrupts_and_can.h"

uint8_t launchAck = 0;
uint8_t acknowledgeAck = 0;
uint8_t move_up=0;
uint8_t move_down=0;
uint8_t move_left=0;
uint8_t move_right=0;
uint8_t update = 0;
bool monitor_status = 0;
const uint8_t mycanbus_0 = 0;
const uint8_t mycanbus_1=1;

//Data received on CAN
uint16_t rpm = 0x0000;
int16_t torque = 0x0000;
uint16_t launch_traction = 0x0000;
uint16_t old_launchActivated = 0x0000;
uint16_t old_rpm = 0x0000;
int16_t old_power = 0x0000;
uint16_t motor_temp = 0x00;
uint16_t old_motor_temp = 0x00;
uint8_t battery_temp = 0x00;
uint8_t old_battery_temp = 0x00;
uint16_t IGBT_temp = 0x00;
uint16_t old_IGBT_temp = 0x00;
uint16_t DC_bus_voltage;
uint16_t old_DC_bus_voltage = 0x00;
int16_t torque_encoder_ch0 = 0x0000;
int16_t torque_encoder_ch1 = 0x0000;
int16_t old_torque_encoder_ch0 = 0x0000;
int16_t old_torque_encoder_ch1 = 0x0000;
uint8_t leftOnBattery = 0x00;
uint8_t old_leftOnBattery = 0x00;
uint16_t error_ecu = 0;
uint16_t old_error_ecu = 0;
uint16_t brk_pres_rear = 2000;
uint16_t brk_pres_front = 2000;
uint16_t old_brk_pres_rear = 2000;
uint16_t old_brk_pres_front = 2000;
uint16_t old_steer_ang = 0;
uint16_t steer_ang = 1800;
uint8_t min_battery_temp = 0;
uint8_t max_battery_temp = 0;
uint8_t min_cell_voltage;
uint8_t max_cell_voltage;
uint8_t old_min_cell_voltage = 0;
uint8_t old_max_cell_voltage = 0;
uint16_t pack_voltage;
uint16_t old_pack_voltage = 0;
uint8_t start_value;
uint8_t launchActivated;
volatile uint8_t gotFeedback_ch0;
volatile uint8_t gotFeedback_ch1;
uint8_t tractive_status = 0x00;
uint8_t old_tractive_status = 0x00;
uint8_t BSPD_status=0;
uint16_t gearbox_temp = 8000;
uint16_t old_gearbox_temp = 0;
uint16_t glv_voltage;
uint16_t old_glv_voltage =0;
uint8_t bms_fault_code = 0;
uint8_t bms_warning = 0;
uint8_t old_bms_fault_code = 0;
uint8_t old_bms_warning = 0;
uint8_t min_cell_id = 0;
uint8_t max_cell_id = 0;


//Timer variables
volatile bool timer_interrupted = false;
volatile uint8_t torqenc_timeout = 0x0A;
volatile uint16_t delay_counter = 0;
volatile static uint32_t tc_tick = 0;
int8_t countdown = -1;
static uint16_t watchdog_reset = 0;

//Check variables
uint8_t pedal_position;
bool play_RTDS = 0;
bool driveEnabled = 0;
bool check_drive = 0;
bool chechLaunch = 0;
bool isAlive[DASH_ALIVE_SIZE] = {0}; 
bool check_alive = false;
bool delay_active = false;
bool launchDoneCount = false;
bool old_launchDoneCount = false;
bool check_err_ecu = false;
bool check_temp = false;
bool tryEnableDrive = false;
bool tryDisableDrive = false;
bool sendDriveEnable = false;
bool sendDriveDisable = false;
bool check_voltage = false;
 bool valid_torque_ch1 = false;
 bool valid_torque_ch0 = false;
 bool old_valid_torque_ch0 = false;
 bool old_valid_torque_ch1 = false;

static uint8_t por_timer = 0;

uint16_t compare;
uint32_t testVar;

volatile avr32_tc_t *tc = EXAMPLE_TC;

//! Local allocation for MOB buffer in HSB_RAM
volatile can_msg_t mob_ram_ch0[NB_MOB_CHANNEL] __attribute__ ((section (".hsb_ram_loc")));
//! Local allocation for MOB buffer in HSB_RAM
volatile can_msg_t mob_ram_ch1[NB_MOB_CHANNEL] __attribute__ ((section (".hsb_ram_loc")));


//-----------------CAN Message resceive definitions-------------------//

// CAN Message Definition: ECU data input
can_msg_t msg_rx_ecu_data =
{
	{
		{
			.id = ECU_DATA_CH0,           // Identifier
			.id_mask  = 0xFF8,                // Mask
		},
	},
	.data.u64 = 0x0LL,                 // Data
};
// MOB Message Definition: ECU data input
can_mob_t appli_rx_ecu_data = {
	CAN_MOB_NOT_ALLOCATED, 			// Handle: by default CAN_MOB_NOT_ALLOCATED
	&msg_rx_ecu_data,	   				// Pointer on CAN Message
	8,		                		// Data length DLC
	CAN_DATA_FRAME,        	        // Request type : CAN_DATA_FRAME or CAN_REMOTE_FRAME
	CAN_STATUS_NOT_COMPLETED	    // Status: by default CAN_STATUS_NOT_COMPLETED
};


// CAN Message Definition: ECU data input
can_msg_t msg_rx_ecu_data_ch1 =
{
	{
		{
			.id = ECU_DATA_CH0,           // Identifier
			.id_mask  = 0xFF8,                // Mask
		},
	},
	.data.u64 = 0x0LL,                 // Data
};
// MOB Message Definition: ECU data input
can_mob_t appli_rx_ecu_data_ch1 = {
	CAN_MOB_NOT_ALLOCATED, 			// Handle: by default CAN_MOB_NOT_ALLOCATED
	&msg_rx_ecu_data_ch1,	   				// Pointer on CAN Message
	8,		                		// Data length DLC
	CAN_DATA_FRAME,        	        // Request type : CAN_DATA_FRAME or CAN_REMOTE_FRAME
	CAN_STATUS_NOT_COMPLETED	    // Status: by default CAN_STATUS_NOT_COMPLETED
};


// CAN Message Definition: ECU cmd input
can_msg_t msg_rx_ecu_pri =
{
	{
		{
			.id = ECU_PRI_CH0,           // Identifier
			.id_mask  = 0xFF8,                // Mask
		},
	},
	.data.u64 = 0x0LL,                 // Data
};
// MOB Message Definition: ECU cmd input
can_mob_t appli_rx_ecu_pri = {
	CAN_MOB_NOT_ALLOCATED, 			// Handle: by default CAN_MOB_NOT_ALLOCATED
	&msg_rx_ecu_pri,	   				// Pointer on CAN Message
	2,		                		// Data length DLC
	CAN_DATA_FRAME,        	        // Request type : CAN_DATA_FRAME or CAN_REMOTE_FRAME
	CAN_STATUS_NOT_COMPLETED	    // Status: by default CAN_STATUS_NOT_COMPLETED
};

// CAN Message Definition: BSPD input
can_msg_t msg_rx_bspd_data =
{
	{
		{
			.id = BSPD_DATA_CH0,           // Identifier
			.id_mask  = 0xFFF,                // Mask
		},
	},
	.data.u64 = 0x0LL,                 // Data
};
// MOB Message Definition: BSPD input
can_mob_t appli_rx_bspd_data = {
	CAN_MOB_NOT_ALLOCATED, 			// Handle: by default CAN_MOB_NOT_ALLOCATED
	&msg_rx_bspd_data,	   				// Pointer on CAN Message
	1,		                		// Data length DLC
	CAN_DATA_FRAME,        	        // Request type : CAN_DATA_FRAME or CAN_REMOTE_FRAME
	CAN_STATUS_NOT_COMPLETED	    // Status: by default CAN_STATUS_NOT_COMPLETED
};


can_msg_t msg_rx_torqueenc_data_ch1 =
{
	{
		{
			.id = TRQENC_PRI,           // Identifier
			.id_mask  = 0xFF8,                // Mask
		},
	},
	.data.u64 = 0x0LL,                 // Data
};
// MOB Message Definition: torqu encoders input
can_mob_t appli_rx_torqueenc_data_ch1 = {
	CAN_MOB_NOT_ALLOCATED, 			// Handle: by default CAN_MOB_NOT_ALLOCATED
	&msg_rx_torqueenc_data_ch1,	   				// Pointer on CAN Message
	2,		                		// Data length DLC
	CAN_DATA_FRAME,        	        // Request type : CAN_DATA_FRAME or CAN_REMOTE_FRAME
	CAN_STATUS_NOT_COMPLETED	    // Status: by default CAN_STATUS_NOT_COMPLETED
};


can_msg_t msg_rx_torqueenc_data_ch0 =
{
	{
		{
			.id = TRQENC_PRI,           // Identifier
			.id_mask  = 0xFF8,                // Mask
		},
	},
	.data.u64 = 0x0LL,                 // Data
};
// MOB Message Definition: torque encoder input
can_mob_t appli_rx_torqueenc_data_ch0 = {
	CAN_MOB_NOT_ALLOCATED, 			// Handle: by default CAN_MOB_NOT_ALLOCATED
	&msg_rx_torqueenc_data_ch0,	   				// Pointer on CAN Message
	2,		                		// Data length DLC
	CAN_DATA_FRAME,        	        // Request type : CAN_DATA_FRAME or CAN_REMOTE_FRAME
	CAN_STATUS_NOT_COMPLETED	    // Status: by default CAN_STATUS_NOT_COMPLETED
};

can_msg_t msg_rx_bms_data_ch0 =
{
	{
		{
			.id = BMS_CMD_CH0,           // Identifier
			.id_mask  = 0xFF0,                // Mask
		},
	},
	.data.u64 = 0x0LL,                 // Data
};
// MOB Message Definition: BMS input
can_mob_t appli_rx_bms_data_ch0 = {
	CAN_MOB_NOT_ALLOCATED, 			// Handle: by default CAN_MOB_NOT_ALLOCATED
	&msg_rx_bms_data_ch0,	   				// Pointer on CAN Message
	7,		                		// Data length DLC
	CAN_DATA_FRAME,        	        // Request type : CAN_DATA_FRAME or CAN_REMOTE_FRAME
	CAN_STATUS_NOT_COMPLETED	    // Status: by default CAN_STATUS_NOT_COMPLETED
};


can_msg_t msg_rx_alive_ch0 =
{
	{
		{
			.id = ISALIVE,           // Identifier
			.id_mask  = 0xFF8,                // Mask
		},
	},
	.data.u64 = 0x0LL,                 // Data
};
// MOB Message Definition: ISALIVE input
can_mob_t appli_rx_alive_ch0 = {
	CAN_MOB_NOT_ALLOCATED, 			// Handle: by default CAN_MOB_NOT_ALLOCATED
	&msg_rx_alive_ch0,	   				// Pointer on CAN Message
	3,		                		// Data length DLC
	CAN_DATA_FRAME,        	        // Request type : CAN_DATA_FRAME or CAN_REMOTE_FRAME
	CAN_STATUS_NOT_COMPLETED	    // Status: by default CAN_STATUS_NOT_COMPLETED
};

can_msg_t msg_rx_alive_ch1 =
{
	{
		{
			.id = ISALIVE,           // Identifier
			.id_mask  = 0xFF8,                // Mask
		},
	},
	.data.u64 = 0x0LL,                 // Data
};
// MOB Message Definition: ISALIVE input
can_mob_t appli_rx_alive_ch1 = {
	CAN_MOB_NOT_ALLOCATED, 			// Handle: by default CAN_MOB_NOT_ALLOCATED
	&msg_rx_alive_ch1,	   				// Pointer on CAN Message
	3,		                		// Data length DLC
	CAN_DATA_FRAME,        	        // Request type : CAN_DATA_FRAME or CAN_REMOTE_FRAME
	CAN_STATUS_NOT_COMPLETED	    // Status: by default CAN_STATUS_NOT_COMPLETED
};

can_msg_t msg_rx_adc_ch1 =
{
	{
		{
			.id = CANR_FCN_DATA_ID|CANR_GRP_SENS_BRK_ID,           // Identifier
			.id_mask  = 0xFF8,                // Mask
		},
	},
	.data.u64 = 0x0LL,                 // Data
};
// MOB Message Definition: adc input
can_mob_t appli_rx_adc_ch1 = {
	CAN_MOB_NOT_ALLOCATED, 			// Handle: by default CAN_MOB_NOT_ALLOCATED
	&msg_rx_adc_ch1,	   				// Pointer on CAN Message
	2,		                		// Data length DLC
	CAN_DATA_FRAME,        	        // Request type : CAN_DATA_FRAME or CAN_REMOTE_FRAME
	CAN_STATUS_NOT_COMPLETED	    // Status: by default CAN_STATUS_NOT_COMPLETED
};

can_msg_t msg_rx_strang_ch1 =
{
	{
		{
			.id = STRANG_DATA,           // Identifier
			.id_mask  = 0xFF8,                // Mask
		},
	},
	.data.u64 = 0x0LL,                 // Data
};
// MOB Message Definition: steering angle input
can_mob_t appli_rx_strang_ch1 = {
	CAN_MOB_NOT_ALLOCATED, 			// Handle: by default CAN_MOB_NOT_ALLOCATED
	&msg_rx_strang_ch1,	   				// Pointer on CAN Message
	3,		                		// Data length DLC
	CAN_DATA_FRAME,        	        // Request type : CAN_DATA_FRAME or CAN_REMOTE_FRAME
	CAN_STATUS_NOT_COMPLETED	    // Status: by default CAN_STATUS_NOT_COMPLETED
};

can_msg_t msg_rx_geartmp_ch1 =
{
	{
		{
			.id = GEARTMP_DATA,           // Identifier
			.id_mask  = 0xFFF,                // Mask
		},
	},
	.data.u64 = 0x0LL,                 // Data
};
// MOB Message Definition: steering angle input
can_mob_t appli_rx_geartmp_ch1 = {
	CAN_MOB_NOT_ALLOCATED, 			// Handle: by default CAN_MOB_NOT_ALLOCATED
	&msg_rx_geartmp_ch1,	   				// Pointer on CAN Message
	2,		                		// Data length DLC
	CAN_DATA_FRAME,        	        // Request type : CAN_DATA_FRAME or CAN_REMOTE_FRAME
	CAN_STATUS_NOT_COMPLETED	    // Status: by default CAN_STATUS_NOT_COMPLETED
};

can_msg_t msg_rx_glv_ch1 =
{
	{
		{
			.id = GLV_DATA,           // Identifier
			.id_mask  = 0xFFF,                // Mask
		},
	},
	.data.u64 = 0x0LL,                 // Data
};
// MOB Message Definition: steering angle input
can_mob_t appli_rx_glv_ch1 = {
	CAN_MOB_NOT_ALLOCATED, 			// Handle: by default CAN_MOB_NOT_ALLOCATED
	&msg_rx_glv_ch1,	   				// Pointer on CAN Message
	2,		                		// Data length DLC
	CAN_DATA_FRAME,        	        // Request type : CAN_DATA_FRAME or CAN_REMOTE_FRAME
	CAN_STATUS_NOT_COMPLETED	    // Status: by default CAN_STATUS_NOT_COMPLETED
};


can_msg_t msg_rx_telemetry_ch0 =
{
	{
		{
			.id = TELEMETRY_DATA,           // Identifier
			.id_mask  = 0xFF8,                // Mask
		},
	},
	.data.u64 = 0x0LL,                 // Data
};
// MOB Message Definition: ISALIVE input
can_mob_t appli_rx_telemetry_ch0 = {
	CAN_MOB_NOT_ALLOCATED, 			// Handle: by default CAN_MOB_NOT_ALLOCATED
	&msg_rx_telemetry_ch0,	   				// Pointer on CAN Message
	8,		                		// Data length DLC
	CAN_DATA_FRAME,        	        // Request type : CAN_DATA_FRAME or CAN_REMOTE_FRAME
	CAN_STATUS_NOT_COMPLETED	    // Status: by default CAN_STATUS_NOT_COMPLETED
};



//-----------------CAN Message transceive definitions-------------------//

can_msg_t msg_tx_ch0 = {
	{
		{
			.id = 0x000,                      // Identifier
			.id_mask  = 0,                // Mask
		},
	},
	.data.u64 = 0xFFFFAAAALL,                 // Data
};

can_mob_t appli_tx_ch0 = {
	CAN_MOB_NOT_ALLOCATED, //Handle
	&msg_tx_ch0, // CAN Message
	8, // DLC
	CAN_DATA_FRAME, // Data Frame Type
	CAN_STATUS_NOT_COMPLETED // Status
};

can_msg_t msg_tx_ch1 = {
	{
		{
			.id = 0x000,                      // Identifier
			.id_mask  = 0,                // Mask
		},
	},
	.data.u64 = 0xFFFFAAAALL,                 // Data
};

can_mob_t appli_tx_ch1 = {
	CAN_MOB_NOT_ALLOCATED, //Handle
	&msg_tx_ch1, // CAN Message
	8, // DLC
	CAN_DATA_FRAME, // Data Frame Type
	CAN_STATUS_NOT_COMPLETED // Status
};

//------------------------------------------------------------------//
//-------------------Interrupt handles-----------------------------//
//----------------------------------------------------------------//

//port number = floor(gpio_number/32) = floor(107/32)=3
//line number: 96/32 = 3 --> line0= gpio96->103, line1=gpio104->111
//Acknowledge = gpio107, launchcontrol = gpio109
__attribute__((__interrupt__)) static void gpio_interrupt_handler_port3_line1(void)  //__attribute__((__interrupt__)) må være foran for at det skal forstå at man skal fortsette etter interrupten
{
	//----------Joystick right interrupt----------//
	if (gpio_get_pin_interrupt_flag(JOYSTICK_RIGHT)){
		gpio_clear_pin_interrupt_flag(JOYSTICK_RIGHT);
		move_right=1;
		update = 1;
	}
	//--------Joystick left interrupt------------//
	if (gpio_get_pin_interrupt_flag(JOYSTICK_LEFT)){
		gpio_clear_pin_interrupt_flag(JOYSTICK_LEFT);
		move_left=1;
		update = 1;
	}
	//---------Joystick up interrupt-------------//
	if (gpio_get_pin_interrupt_flag(JOYSTICK_UP)){
		gpio_clear_pin_interrupt_flag(JOYSTICK_UP);
		move_up=1;
		update = 1;	
	}
	//--------Joystick down interrupt------------//
	if (gpio_get_pin_interrupt_flag(JOYSTICK_DOWN)){
		gpio_clear_pin_interrupt_flag(JOYSTICK_DOWN);	
		move_down=1;
		update = 1;	
	}
	
}

__attribute__((__interrupt__)) static void gpio_interrupt_handler_port3_line0(void){
	
	
	//-------Acknoeldge interrupt---------------//
	if (gpio_get_pin_interrupt_flag(ACKNOWLEDGE)){
		// Clear the pin change interrupt flag
		gpio_clear_pin_interrupt_flag(ACKNOWLEDGE);
		acknowledgeAck = 1;
		update = 1;
	}
	
}

__attribute__((__interrupt__)) static void gpio_interrupt_handler_port2_line0(void){
	//------Launch control interrupt--------------//
	if (gpio_get_pin_interrupt_flag(LAUNCHCONTROL)){
		gpio_clear_pin_interrupt_flag(LAUNCHCONTROL);
		launchAck = 1;
		update = 1;
		//SET CAN BIT OR SOMETHING TO SEND CAN MSG
	}
}

// TIMER interrupt
__attribute__((__interrupt__)) static void tc_irq(void)
{
	// Increment the 10ms seconds counter
	tc_tick++;
	delay_counter++;
	
	// Clear the interrupt flag. This is a side effect of reading the TC SR.
	tc_read_sr(EXAMPLE_TC, EXAMPLE_TC_CHANNEL);
	if(watchdog_reset==80){
		if (power_on_reset == 1) {
			por_timer++;
			} else {
			por_timer = 0;
		}
		
		if (por_timer <2) {
			/* Clear watchdog timer */
			wdt_clear();
		} else {
			asm("nop");
		}
		watchdog_reset = 0;
	}
	watchdog_reset++;
	//Virker som det er noe rart med klokka til timeren, derfor 120 og ikke 100
	if(tc_tick>=120){
		
		check_alive = true;
		if(tractive_status==1 && driveEnabled==0){
			gpio_toggle_pin(TRACTIVEDRIVE_LED);
			
		}
		if(launchActivated==4){	//If we are counting down
			countdown++;
			switch(countdown){	
				case 0:
					writeString("3", 31, 240, 136);
					break;
				case 1:
					writeString("2", 31, 240, 136);
					break;
				case 2: 
					writeString("1", 31, 240, 136);
					break;
				case 3:
				//Mulig det skjer for mye her i interruptet......
					writeString("Launch!", 31, 240, 136);
					myDelay(2000);
					launchActivated=5;
					countdown=-1;
					update=1;
					chechLaunch = true;
					break;
					
			}
			
		}
		if(torqenc_timeout<5){
			torqenc_timeout++;
		}
		if(tryEnableDrive){
			sendDriveEnable = true;
		}
		if(tryDisableDrive){
			sendDriveDisable = true;
		}
		tc_tick=0;
		update = 1;
		
	}
	
}


//--------------------------------------------------------------------//
//------------------CAN callback functions---------------------------//
//-------------------------------------------------------------------//

 void can_out_callback_channel0(U8 handle, U8 event){
	 //Disable_global_interrupt();
	
	//ECU data msg on channel 0
 	if (handle == appli_rx_ecu_data.handle){
		appli_rx_ecu_data.can_msg->data.u64 = can_get_mob_data(mycanbus_0, handle).u64;
		appli_rx_ecu_data.can_msg->id = can_get_mob_id(mycanbus_0,handle);
		appli_rx_ecu_data.dlc = can_get_mob_dlc(mycanbus_0,handle);
		appli_rx_ecu_data.status = event;
		
		testVar = appli_rx_ecu_data.can_msg->id;
		
		switch(testVar){
			case ECU_DATA_CH0_ID0:
				DC_bus_voltage = appli_rx_ecu_data.can_msg->data.u16[0];// >> 48;
				error_ecu = appli_rx_ecu_data.can_msg->data.u16[1];
				rpm = appli_rx_ecu_data.can_msg->data.u16[2]; //>> 16;
				torque = appli_rx_ecu_data.can_msg->data.u16[3];
				break;
			
			/*case ECU_DATA_CH0_ID1:
				motor_temp = appli_rx_ecu_data.can_msg->data.u64 >> 48;
				IGBT_temp = appli_rx_ecu_data.can_msg->data.u64 >> 32;
				break;*/
		}
		//Setup for a new receive
		can_rx(0,
		appli_rx_ecu_data.handle,
		appli_rx_ecu_data.req_type,
		appli_rx_ecu_data.can_msg);
	
	//ECU PRI msg on channel 0	
	}else if(handle == appli_rx_ecu_pri.handle){
		appli_rx_ecu_pri.can_msg->data.u64 = can_get_mob_data(mycanbus_0, handle).u64;
		appli_rx_ecu_pri.can_msg->id = can_get_mob_id(mycanbus_0,handle);
		appli_rx_ecu_pri.dlc = can_get_mob_dlc(mycanbus_0,handle);
		appli_rx_ecu_pri.status = event;
		
		testVar = appli_rx_ecu_pri.can_msg->id;
		
		switch(testVar){
			//Drive enable/disable msg
			case ECU_PRI_CH0|CANR_MODULE_ID0_ID:
				if((appli_rx_ecu_pri.can_msg->data.u16[0])==0x0002){
					play_RTDS= true;
					tryEnableDrive = false;
		
				}else if((appli_rx_ecu_pri.can_msg->data.u16[0])==0x0003){
					check_drive = true;
					driveEnabled = true;
				
				}else if((appli_rx_ecu_pri.can_msg->data.u16[0])==0x0004){
					check_drive = true;
					driveEnabled = false;
					tryDisableDrive = false;
				}
				break;
			
			//Launch control msg
			case ECU_PRI_CH0|CANR_MODULE_ID1_ID:
				if((appli_rx_ecu_pri.can_msg->data.u8[0])==0x01){
					//launch_traction = 1;
					launchActivated =2;
				
				}else if((appli_rx_ecu_pri.can_msg->data.u8[0])==0x02){
					//launch_traction = 2;
					launchActivated = 3;
				}else if((appli_rx_ecu_pri.can_msg->data.u8[0])==255){
					//launch_traction =4;
					launchActivated = 6;
				}
				break;
		}
		//Setup for a new receive
		can_rx(0,
		appli_rx_ecu_pri.handle,
		appli_rx_ecu_pri.req_type,
		appli_rx_ecu_pri.can_msg);
	
	//BSPD data msg on channel 0	
	}else if(handle == appli_rx_bspd_data.handle){
		appli_rx_bspd_data.can_msg->data.u64 = can_get_mob_data(mycanbus_0, handle).u64;
		appli_rx_bspd_data.can_msg->id = can_get_mob_id(mycanbus_0,handle);
		appli_rx_bspd_data.dlc = can_get_mob_dlc(mycanbus_0,handle);
		appli_rx_bspd_data.status = event;
		
		switch(appli_rx_bspd_data.can_msg->id){
			case BSPD_DATA_CH0_ID0:
				BSPD_status = appli_rx_bspd_data.can_msg->data.u8[0];
				break;
		}

		//Setup for a new receive	
		can_rx(0,
		appli_rx_bspd_data.handle,
		appli_rx_bspd_data.req_type,
		appli_rx_bspd_data.can_msg);
		
	//Torque encoder priority msg on channel 0	
	}else if(handle == appli_rx_torqueenc_data_ch0.handle){
		appli_rx_torqueenc_data_ch0.can_msg->data.u64 = can_get_mob_data(mycanbus_0, handle).u64;
		appli_rx_torqueenc_data_ch0.can_msg->id = can_get_mob_id(mycanbus_0,handle);
		appli_rx_torqueenc_data_ch0.dlc = can_get_mob_dlc(mycanbus_0,handle);
		appli_rx_torqueenc_data_ch0.status = event;
		
		testVar = appli_rx_torqueenc_data_ch0.can_msg->id;
		
		switch(testVar){
			case (TRQENC_PRI|CANR_MODULE_ID0_ID):
				if(appli_rx_torqueenc_data_ch0.can_msg->data.u8[2] != 0xFF){
					torque_encoder_ch0 = appli_rx_torqueenc_data_ch0.can_msg->data.s16[0];
					valid_torque_ch0 = true;
				}else{
					torque_encoder_ch0 = appli_rx_torqueenc_data_ch0.can_msg->data.s16[0];
					valid_torque_ch0 = false;
				}
				break;
				
			case (TRQENC_PRI|CANR_MODULE_ID2_ID):
				if(((appli_rx_torqueenc_data_ch0.can_msg->data.u8[0]==0xF0)&&(pedal_position==1)) || ((appli_rx_torqueenc_data_ch0.can_msg->data.u8[0]==0x0F)&&(pedal_position==0))){
					gotFeedback_ch0=1;
				
				}else{
					gotFeedback_ch0=2;  //=2 if something is wrong
				}
				break;
		}
		//Setup for a new receive
		can_rx(0,
		appli_rx_torqueenc_data_ch0.handle,
		appli_rx_torqueenc_data_ch0.req_type,
		appli_rx_torqueenc_data_ch0.can_msg);
	
	//BMS data msg on channel 0
	
	//Alive msg from (almost) all units
	}else if(handle == appli_rx_alive_ch0.handle){
		appli_rx_alive_ch0.can_msg->data.u64 = can_get_mob_data(mycanbus_0, handle).u64;
		appli_rx_alive_ch0.can_msg->id = can_get_mob_id(mycanbus_0,handle);
		appli_rx_alive_ch0.dlc = can_get_mob_dlc(mycanbus_0,handle);
		appli_rx_alive_ch0.status = event;
		
		switch(appli_rx_alive_ch0.can_msg->id){
			case ISALIVE|CANR_MODULE_ID7_ID:
				if(appli_rx_alive_ch0.can_msg->data.u8[0] == CANR_CMD_ALIVE){
					if(appli_rx_alive_ch0.can_msg->data.u8[2] == CANR_ALIVE_STATE_OPERATIVE){
						isAlive[appli_rx_alive_ch0.can_msg->data.u8[1]] = 1;	//Byte 1 says which unit, and equals a point in the array.
					}
				}
				break;
		}
		//Setup for a new receive
		can_rx(0,
		appli_rx_alive_ch0.handle,
		appli_rx_alive_ch0.req_type,
		appli_rx_alive_ch0.can_msg);
	
	//Telemetry data. Used to control screen
	}else if(handle == appli_rx_telemetry_ch0.handle){
		appli_rx_telemetry_ch0.can_msg->data.u64 = can_get_mob_data(mycanbus_0, handle).u64;
		appli_rx_telemetry_ch0.can_msg->id = can_get_mob_id(mycanbus_0,handle);
		appli_rx_telemetry_ch0.dlc = can_get_mob_dlc(mycanbus_0,handle);
		appli_rx_telemetry_ch0.status = event;
		
		switch(appli_rx_telemetry_ch0.can_msg->id){
			case TELEMETRY_DATA|CANR_MODULE_ID0_ID:
				if(appli_rx_telemetry_ch0.can_msg->data.u8[0] == 0x00){
					move_left = 1;
				
				}else if(appli_rx_telemetry_ch0.can_msg->data.u8[0] == 0x01){
					move_up = 1;
				
				}else if(appli_rx_telemetry_ch0.can_msg->data.u8[0] == 0x02){
					move_right = 1;
				
				}else if(appli_rx_telemetry_ch0.can_msg->data.u8[0] == 0x03){
					move_down = 1;
				}
				update = 1;
				break;
		}
		
		can_rx(0,
		appli_rx_telemetry_ch0.handle,
		appli_rx_telemetry_ch0.req_type,
		appli_rx_telemetry_ch0.can_msg);
	}
	
	//Check if variables have changed. 
	//ER DET BEDRE Å BARE OPPDATERE SKJERM HELE TIDEN? ELLER UNØDVENDIG??
	if((old_rpm!=rpm)||(old_power!=torque)||(BSPD_status==0x42)||(old_DC_bus_voltage!=DC_bus_voltage)||(old_IGBT_temp!=IGBT_temp)||(old_motor_temp!=motor_temp)||(old_torque_encoder_ch0!=torque_encoder_ch0)||(play_RTDS==1)||(old_launchActivated!=launchActivated)||(error_ecu!=old_error_ecu)||(old_valid_torque_ch0!=valid_torque_ch0)){
		update=1;
		monitor_status=1;
		//for some reason the rpm is a bit unstable from the ECU when the car is standing still. SO have to do a check
		if((old_rpm==0)&&(rpm>2000)){
			rpm = 0;
		}
		old_rpm=rpm;
		old_power=torque;
		old_torque_encoder_ch0=torque_encoder_ch0;
		old_valid_torque_ch0 = valid_torque_ch0;
		
		if((old_launchActivated!=launchActivated)){
			chechLaunch = 1;
			old_launchActivated = launchActivated;
		}
		if(error_ecu!=old_error_ecu){
			check_err_ecu = true;
			old_error_ecu = error_ecu;
		}
		check_voltage = true;
		old_DC_bus_voltage=DC_bus_voltage;
		//Bytte fra != til battery_temp > grense. Så skal vi sjekke i check.c for å toggle led osv..
		//Sjekker her for spenninger osv også f.eks
	
		
		
	}
	//Enable_global_interrupt();
	
}

void can_out_callback_channel1(U8 handle, U8 event){
	//Disable_global_interrupt();
	
	//Torque encoder priority msg on channel 0
	if(handle == appli_rx_torqueenc_data_ch1.handle){
		appli_rx_torqueenc_data_ch1.can_msg->data.u64 = can_get_mob_data(1, handle).u64;
		appli_rx_torqueenc_data_ch1.can_msg->id = can_get_mob_id(1,handle);
		appli_rx_torqueenc_data_ch1.dlc = can_get_mob_dlc(1,handle);
		appli_rx_torqueenc_data_ch1.status = event;
		
		testVar = appli_rx_torqueenc_data_ch1.can_msg->id;
		
		switch(testVar){
			case TRQENC_PRI|CANR_MODULE_ID1_ID:
				if(appli_rx_torqueenc_data_ch1.can_msg->data.u8[2] != 0xFF){
					torque_encoder_ch1 = appli_rx_torqueenc_data_ch1.can_msg->data.s16[0];
					valid_torque_ch1 = true;
				}else{
					torque_encoder_ch1 = appli_rx_torqueenc_data_ch1.can_msg->data.s16[0];
					valid_torque_ch1 = false;
				}
				break;
			
			case TRQENC_PRI|CANR_MODULE_ID2_ID:
				if(((appli_rx_torqueenc_data_ch1.can_msg->data.u8[0]==0xF0)&&(pedal_position==1)) || ((appli_rx_torqueenc_data_ch1.can_msg->data.u8[0]==0x0F)&&(pedal_position==0))){
					gotFeedback_ch1=1;
				
				}else{
					gotFeedback_ch0=2;	//=2 if something is wrong
				}
				break;
		}
		
		//Setup for a new resceive
		can_rx(1,
		appli_rx_torqueenc_data_ch1.handle,
		appli_rx_torqueenc_data_ch1.req_type,
		appli_rx_torqueenc_data_ch1.can_msg);
		
	//Alive msg on channel 1
	}else if(handle == appli_rx_alive_ch1.handle){
		appli_rx_alive_ch1.can_msg->data.u64 = can_get_mob_data(1, handle).u64;
		appli_rx_alive_ch1.can_msg->id = can_get_mob_id(1,handle);
		appli_rx_alive_ch1.dlc = can_get_mob_dlc(1,handle);
		appli_rx_alive_ch1.status = event;
	
		switch(appli_rx_alive_ch1.can_msg->id){
			case ISALIVE|CANR_MODULE_ID7_ID:
			if(appli_rx_alive_ch1.can_msg->data.u8[0] == CANR_CMD_ALIVE){
				if(appli_rx_alive_ch1.can_msg->data.u8[2] == CANR_ALIVE_STATE_OPERATIVE){
					isAlive[appli_rx_alive_ch1.can_msg->data.u8[1]] = 1;
				}
			}
			break;
		}
	
		can_rx(1,
		appli_rx_alive_ch1.handle,
		appli_rx_alive_ch1.req_type,
		appli_rx_alive_ch1.can_msg);
	
	//ADC data msg on channel 1
	}else if(handle == appli_rx_adc_ch1.handle){
		appli_rx_adc_ch1.can_msg->data.u64 = can_get_mob_data(1, handle).u64;
		appli_rx_adc_ch1.can_msg->id = can_get_mob_id(1,handle);
		appli_rx_adc_ch1.dlc = can_get_mob_dlc(1,handle);
		appli_rx_adc_ch1.status = event;
	
		switch(appli_rx_adc_ch1.can_msg->id){
			case CANR_FCN_DATA_ID|CANR_GRP_SENS_BRK_ID|CANR_MODULE_ID0_ID:
				brk_pres_front = appli_rx_adc_ch1.can_msg->data.u16[0];
				break;
			
			case CANR_FCN_DATA_ID|CANR_GRP_SENS_BRK_ID|CANR_MODULE_ID1_ID:
				brk_pres_rear = appli_rx_adc_ch1.can_msg->data.u16[0];
				break;
		}
		
		can_rx(1,
		appli_rx_adc_ch1.handle,
		appli_rx_adc_ch1.req_type,
		appli_rx_adc_ch1.can_msg);
		
	//Steering angle msg on channel 1
	}else if(handle == appli_rx_strang_ch1.handle){
		appli_rx_strang_ch1.can_msg->data.u64 = can_get_mob_data(1, handle).u64;
		appli_rx_strang_ch1.can_msg->id = can_get_mob_id(1,handle);
		appli_rx_strang_ch1.dlc = can_get_mob_dlc(1,handle);
		appli_rx_strang_ch1.status = event;
	
		switch(appli_rx_strang_ch1.can_msg->id){
			case STRANG_DATA|CANR_MODULE_ID0_ID:
				if(appli_rx_strang_ch1.can_msg->data.u8[0] == 0){
					steer_ang = ((appli_rx_strang_ch1.can_msg->data.u8[1])<<8)|(appli_rx_strang_ch1.can_msg->data.u8[2]);
				}else{
					steer_ang = 0xFFFF;
				}
				break;
		}
		
		can_rx(1,
		appli_rx_strang_ch1.handle,
		appli_rx_strang_ch1.req_type,
		appli_rx_strang_ch1.can_msg);
		
	}else if(handle == appli_rx_geartmp_ch1.handle){
	appli_rx_geartmp_ch1.can_msg->data.u64 = can_get_mob_data(1, handle).u64;
	appli_rx_geartmp_ch1.can_msg->id = can_get_mob_id(1,handle);
	appli_rx_geartmp_ch1.dlc = can_get_mob_dlc(1,handle);
	appli_rx_geartmp_ch1.status = event;
	
	switch(appli_rx_geartmp_ch1.can_msg->id){
		case GEARTMP_DATA:
			gearbox_temp = appli_rx_geartmp_ch1.can_msg->data.u16[0];
		break;
	}
	
	can_rx(1,
	appli_rx_geartmp_ch1.handle,
	appli_rx_geartmp_ch1.req_type,
	appli_rx_geartmp_ch1.can_msg);
	
	}else if(handle == appli_rx_glv_ch1.handle){
	appli_rx_glv_ch1.can_msg->data.u64 = can_get_mob_data(1, handle).u64;
	appli_rx_glv_ch1.can_msg->id = can_get_mob_id(1,handle);
	appli_rx_glv_ch1.dlc = can_get_mob_dlc(1,handle);
	appli_rx_glv_ch1.status = event;
	
	switch(appli_rx_glv_ch1.can_msg->id){
		case GLV_DATA:
			glv_voltage = appli_rx_glv_ch1.can_msg->data.u16[0];
		break;
	}
	
	can_rx(1,
	appli_rx_glv_ch1.handle,
	appli_rx_glv_ch1.req_type,
	appli_rx_glv_ch1.can_msg);
	
	}else if(handle == appli_rx_bms_data_ch0.handle){
		appli_rx_bms_data_ch0.can_msg->data.u64 = can_get_mob_data(1, handle).u64;
		appli_rx_bms_data_ch0.can_msg->id = can_get_mob_id(1,handle);
		appli_rx_bms_data_ch0.dlc = can_get_mob_dlc(1,handle);
		appli_rx_bms_data_ch0.status = event;
		
		isAlive[DASH_ALIVE_BMS] = 1;
	
		switch(appli_rx_bms_data_ch0.can_msg->id){
			case (BMS_CMD_CH0 + CANR_MODULE_ID2_ID) :
				if(((appli_rx_bms_data_ch0.can_msg->data.u8[3])&8) == 8){
					tractive_status=0x01;
			
				}else{
					tractive_status=0x00;
					gpio_set_pin_low(TRACTIVEDRIVE_LED);
				}
				bms_warning = appli_rx_bms_data_ch0.can_msg->data.u8[6];
				bms_fault_code = appli_rx_bms_data_ch0.can_msg->data.u8[4];
				break;
		
			case (BMS_CMD_CH0 + CANR_MODULE_ID3_ID):
				pack_voltage = appli_rx_bms_data_ch0.can_msg->data.u16[0];
				min_cell_voltage = appli_rx_bms_data_ch0.can_msg->data.u8[2];
				min_cell_id = appli_rx_bms_data_ch0.can_msg->data.u8[3];
				max_cell_voltage = appli_rx_bms_data_ch0.can_msg->data.u8[4];
				max_cell_id = appli_rx_bms_data_ch0.can_msg->data.u8[5];
				break;
		
			/*case (BMS_CMD_CH0 + CANR_MODULE_ID6_ID) : //MODULE_ID_6
				leftOnBattery = appli_rx_bms_data_ch0.can_msg->data.u8[0];
				break;*/
		
			case (BMS_CMD_CH0 + CANR_MODULE_ID7_ID):  //module_ID_7
				battery_temp = appli_rx_bms_data_ch0.can_msg->data.u8[0];
				min_battery_temp = appli_rx_bms_data_ch0.can_msg->data.u8[2];
				max_battery_temp = appli_rx_bms_data_ch0.can_msg->data.u8[4];
				break;
		
		}
	//Setup for a new receive
		can_rx(1,
		appli_rx_bms_data_ch0.handle,
		appli_rx_bms_data_ch0.req_type,
		appli_rx_bms_data_ch0.can_msg);
	
	}else if(handle == appli_rx_ecu_data_ch1.handle){
		appli_rx_ecu_data_ch1.can_msg->data.u64 = can_get_mob_data(1, handle).u64;
		appli_rx_ecu_data_ch1.can_msg->id = can_get_mob_id(1,handle);
		appli_rx_ecu_data_ch1.dlc = can_get_mob_dlc(1,handle);
		appli_rx_ecu_data_ch1.status = event;
		
		testVar = appli_rx_ecu_data_ch1.can_msg->id;
		
		switch(testVar){
			case ECU_DATA_CH0_ID1:
				motor_temp = appli_rx_ecu_data_ch1.can_msg->data.u64 >> 48;
				IGBT_temp = appli_rx_ecu_data_ch1.can_msg->data.u64 >> 32;
				break;
		}
		//Setup for a new receive
		can_rx(1,
		appli_rx_ecu_data_ch1.handle,
		appli_rx_ecu_data_ch1.req_type,
		appli_rx_ecu_data_ch1.can_msg);
	}
	
	
	//Check for changed data
	if((old_tractive_status!=tractive_status)||(old_torque_encoder_ch1!=torque_encoder_ch1)||(old_brk_pres_front!=brk_pres_front)||(old_brk_pres_rear!=brk_pres_rear)||(steer_ang!=old_steer_ang)||(old_leftOnBattery!=leftOnBattery)||(old_battery_temp!=battery_temp)||(old_pack_voltage!=pack_voltage)||(old_min_cell_voltage!=min_cell_voltage)||(old_max_cell_voltage!=max_cell_voltage)||(old_motor_temp!=motor_temp)||(old_IGBT_temp!=IGBT_temp)||(old_gearbox_temp!=gearbox_temp)||(old_glv_voltage!=glv_voltage)||(old_bms_warning!=bms_warning)||(old_bms_fault_code!=bms_fault_code)||(old_valid_torque_ch1!=valid_torque_ch1)){
		update=1;
		monitor_status=1;
		old_torque_encoder_ch1=torque_encoder_ch1;	
		old_valid_torque_ch1 = valid_torque_ch1;
		old_brk_pres_front = brk_pres_front;
		old_brk_pres_rear = brk_pres_rear;
		old_leftOnBattery = leftOnBattery;
		old_tractive_status = tractive_status;
		old_battery_temp = battery_temp;
		old_IGBT_temp = IGBT_temp;
		old_motor_temp = motor_temp;
		old_max_cell_voltage = max_cell_voltage;
		old_min_cell_voltage = min_cell_voltage;
		old_pack_voltage = pack_voltage;
		old_gearbox_temp = gearbox_temp;
		old_steer_ang = steer_ang;
		old_glv_voltage = glv_voltage;
		old_bms_fault_code = bms_fault_code;
		old_bms_warning = bms_warning;
		check_voltage = true;
		check_temp = true;
		
	}
	//Enable_global_interrupt();
}


void Interrupt_CAN_Init(void){
	//This function initialize CAN, timer interrupts and I/O interrupts
	
	Disable_global_interrupt();//cpu_irq_disable(); //disable global interrupts

	/* Setup the generic clock for CAN output */
	scif_gc_setup(
	AVR32_SCIF_GCLK_CANIF,
	SCIF_GCCTRL_OSC0,
	AVR32_SCIF_GC_NO_DIV_CLOCK,
	0
	);
	
	/* Now enable the generic clock input for the CAN module */
	scif_gc_enable(AVR32_SCIF_GCLK_CANIF);
	
	const gpio_map_t CAN_GPIO_MAP =
	{
		{AVR32_CANIF_RXLINE_0_3_PIN, AVR32_CANIF_RXLINE_0_3_FUNCTION},
		{AVR32_CANIF_TXLINE_0_3_PIN, AVR32_CANIF_TXLINE_0_3_FUNCTION},
		{AVR32_CANIF_RXLINE_1_1_PIN, AVR32_CANIF_RXLINE_1_1_FUNCTION},
		{AVR32_CANIF_TXLINE_1_1_PIN, AVR32_CANIF_TXLINE_1_1_FUNCTION}
	};
		
	// Assign GPIO to CAN.
	gpio_enable_module(CAN_GPIO_MAP, sizeof(CAN_GPIO_MAP) / sizeof(CAN_GPIO_MAP[0]));
		
	/* Initialize interrupt vectors. */
	INTC_init_interrupts();
	
	appli_rx_ecu_data.handle = 0;
	appli_rx_bspd_data.handle = 1;
	appli_rx_torqueenc_data_ch1.handle = 2;
	appli_rx_torqueenc_data_ch0.handle = 3;
	appli_tx_ch0.handle = 4;
	appli_rx_ecu_pri.handle = 5;
	appli_tx_ch1.handle = 6;
	appli_rx_bms_data_ch0.handle = 7;
	appli_rx_alive_ch0.handle = 8;
	appli_rx_alive_ch1.handle = 9;
	appli_rx_telemetry_ch0.handle = 10;
	appli_rx_adc_ch1.handle = 11;
	appli_rx_ecu_data_ch1.handle = 12;
	appli_rx_strang_ch1.handle = 13;
	appli_rx_geartmp_ch1.handle = 14;
	appli_rx_glv_ch1.handle = 15;
	   		
	// Initialize CAN channel
	gpio_set_pin_low(LED1);
	delay_ms(500);
	gpio_set_pin_high(LED1);
	can_init(mycanbus_0, ((U32)&mob_ram_ch0[0]), CANIF_CHANNEL_MODE_NORMAL, can_out_callback_channel0);
	can_init(mycanbus_1, ((U32)&mob_ram_ch1[0]), CANIF_CHANNEL_MODE_NORMAL, can_out_callback_channel1);
		
	can_rx(0,
	appli_rx_ecu_data.handle,
	appli_rx_ecu_data.req_type,
	appli_rx_ecu_data.can_msg);
	
	can_rx(1,
	appli_rx_ecu_data_ch1.handle,
	appli_rx_ecu_data_ch1.req_type,
	appli_rx_ecu_data_ch1.can_msg);
	
	can_rx(0,
	appli_rx_ecu_pri.handle,
	appli_rx_ecu_pri.req_type,
	appli_rx_ecu_pri.can_msg);
	
	can_rx(0,
	appli_rx_bspd_data.handle,
	appli_rx_bspd_data.req_type,
	appli_rx_bspd_data.can_msg);
	
	can_rx(0,
	appli_rx_torqueenc_data_ch0.handle,
	appli_rx_torqueenc_data_ch0.req_type,
	appli_rx_torqueenc_data_ch0.can_msg);
	
	can_rx(1,
	appli_rx_torqueenc_data_ch1.handle,
	appli_rx_torqueenc_data_ch1.req_type,
	appli_rx_torqueenc_data_ch1.can_msg);
	
	can_rx(1,
	appli_rx_bms_data_ch0.handle,
	appli_rx_bms_data_ch0.req_type,
	appli_rx_bms_data_ch0.can_msg);
	
	can_rx(0,
	appli_rx_alive_ch0.handle,
	appli_rx_alive_ch0.req_type,
	appli_rx_alive_ch0.can_msg);
	
	can_rx(1,
	appli_rx_alive_ch1.handle,
	appli_rx_alive_ch1.req_type,
	appli_rx_alive_ch1.can_msg);
	
	can_rx(1,
	appli_rx_adc_ch1.handle,
	appli_rx_adc_ch1.req_type,
	appli_rx_adc_ch1.can_msg);
	
	can_rx(1,
	appli_rx_geartmp_ch1.handle,
	appli_rx_geartmp_ch1.req_type,
	appli_rx_geartmp_ch1.can_msg);
	
	can_rx(1,
	appli_rx_glv_ch1.handle,
	appli_rx_glv_ch1.req_type,
	appli_rx_glv_ch1.can_msg);
	
	can_rx(1,
	appli_rx_strang_ch1.handle,
	appli_rx_strang_ch1.req_type,
	appli_rx_strang_ch1.can_msg);
	
	can_rx(0,
	appli_rx_telemetry_ch0.handle,
	appli_rx_telemetry_ch0.req_type,
	appli_rx_telemetry_ch0.can_msg);
	
	//Init I/O interrupts 
	gpio_enable_pin_interrupt(ACKNOWLEDGE, GPIO_FALLING_EDGE);
	gpio_enable_pin_interrupt(LAUNCHCONTROL, GPIO_FALLING_EDGE);
	gpio_enable_pin_interrupt(JOYSTICK_LEFT, GPIO_FALLING_EDGE);
	gpio_enable_pin_interrupt(JOYSTICK_UP, GPIO_FALLING_EDGE);
	gpio_enable_pin_interrupt(JOYSTICK_RIGHT, GPIO_FALLING_EDGE);
	gpio_enable_pin_interrupt(JOYSTICK_DOWN, GPIO_FALLING_EDGE);
	INTC_register_interrupt(&gpio_interrupt_handler_port3_line0, AVR32_GPIO_IRQ_0 + ACKNOWLEDGE/8, AVR32_INTC_INT0); //siste element sier noe om viktighet, altså hvilke man tar først hvis to interrupter skjer samtidig. Bør gjøres noe med
	INTC_register_interrupt(&gpio_interrupt_handler_port2_line0, AVR32_GPIO_IRQ_0 + LAUNCHCONTROL/8, AVR32_INTC_INT0);
	INTC_register_interrupt(&gpio_interrupt_handler_port3_line1, AVR32_GPIO_IRQ_0 + JOYSTICK_LEFT/8, AVR32_INTC_INT0);
	INTC_register_interrupt(&gpio_interrupt_handler_port3_line1, AVR32_GPIO_IRQ_0 + JOYSTICK_UP/8, AVR32_INTC_INT0);
	INTC_register_interrupt(&gpio_interrupt_handler_port3_line1, AVR32_GPIO_IRQ_0 + JOYSTICK_RIGHT/8, AVR32_INTC_INT0);
	INTC_register_interrupt(&gpio_interrupt_handler_port3_line1, AVR32_GPIO_IRQ_0 + JOYSTICK_DOWN/8, AVR32_INTC_INT0);
	
	//Init timer interrupt
	INTC_register_interrupt(&tc_irq, EXAMPLE_TC_IRQ, EXAMPLE_TC_IRQ_PRIORITY);
	
	Enable_global_interrupt(); //cpu_irq_enable(); //enable global interrupt	
	
	//Timer counter init
	// Options for waveform genration.
	static const tc_waveform_opt_t waveform_opt = {
		// Channel selection.
		.channel  = EXAMPLE_TC_CHANNEL,
		// Software trigger effect on TIOB.
		.bswtrg   = TC_EVT_EFFECT_NOOP,
		// External event effect on TIOB.
		.beevt    = TC_EVT_EFFECT_NOOP,
		// RC compare effect on TIOB.
		.bcpc     = TC_EVT_EFFECT_NOOP,
		// RB compare effect on TIOB.
		.bcpb     = TC_EVT_EFFECT_NOOP,
		// Software trigger effect on TIOA.
		.aswtrg   = TC_EVT_EFFECT_NOOP,
		// External event effect on TIOA.
		.aeevt    = TC_EVT_EFFECT_NOOP,
		// RC compare effect on TIOA.
		.acpc     = TC_EVT_EFFECT_NOOP,
		/* RA compare effect on TIOA.
		 * (other possibilities are none, set and clear).
		 */
		.acpa     = TC_EVT_EFFECT_NOOP,
		/* Waveform selection: Up mode with automatic trigger(reset)
		 * on RC compare.
		 */
		.wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,
		// External event trigger enable.
		.enetrg   = false,
		// External event selection.
		.eevt     = 0,
		// External event edge selection.
		.eevtedg  = TC_SEL_NO_EDGE,
		// Counter disable when RC compare.
		.cpcdis   = false,
		// Counter clock stopped with RC compare.
		.cpcstop  = false,
		// Burst signal selection.
		.burst    = false,
		// Clock inversion.
		.clki     = false,
		// Internal source clock 3, connected to fPBA / 8.
		.tcclks   = TC_CLOCK_SOURCE_TC4
	};
		static const tc_interrupt_t tc_interrupt = {
			.etrgs = 0,
			.ldrbs = 0,
			.ldras = 0,
			.cpcs  = 1, // Enable interrupt on RC compare alone
			.cpbs  = 0,
			.cpas  = 0,
			.lovrs = 0,
			.covfs = 0
		};
	
		// Initialize the timer/counter.
		tc_init_waveform(tc, &waveform_opt);
		/*
		* Set the compare triggers.
		* We configure it to count every second
		* We want: (1 / (fPBA / 8)) * RC = 1s, hence RC = (fPBA / 8)
		* to get an interrupt every 1 secnd
		 */
		
		tc_write_rc(tc, EXAMPLE_TC_CHANNEL, ((uint32_t)(sysclk_get_pba_hz() / 800.0)));
		// configure the timer interrupt
		tc_configure_interrupts(tc, EXAMPLE_TC_CHANNEL, &tc_interrupt);
		// Start the timer/counter.
		tc_start(tc, EXAMPLE_TC_CHANNEL);
	
}

uint8_t myDelay(uint16_t timeinms){
	//Delay function
	
	timeinms = timeinms/10;
	delay_counter=0;
	//Waiting for timer 
	while(delay_counter<timeinms){
		
	}
	return 0;
}
