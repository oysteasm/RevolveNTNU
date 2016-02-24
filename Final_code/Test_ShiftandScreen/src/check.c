/*
 * check.c
 *
 * Created: 27.03.2014 21:38:02
 *  Author: Øystein
 */ 
#include <asf.h>
#include "FT800.h"
#include "CANdefinitions.h"
#include "Interrupts_and_can.h"
#include "flashc_array.h"
//#include "revolve_logo.h"

uint8_t launchAck;
uint8_t acknowledgeAck;
uint8_t move_up;
uint8_t move_down;
bool monitor_status;
bool adjustment_done;
uint8_t move_left;
uint8_t move_right;
int8_t horizontal_position=1;
uint8_t vertical_level_adjustment_menu = 0;	//Store the vertical position you are in in the adjustment menu
uint8_t vertical_level_adjustment = 0;	//Store the vetical position you are in in a adjusmtent for a device
uint8_t vertical_level_adjustment_spes = 0;
uint8_t pedal_position = 0;

uint8_t tractive_status;
uint8_t BSPD_status;
bool play_RTDS;
bool driveEnabled;
bool check_drive;
uint16_t launch_traction;
bool chechLaunch;
uint8_t launchActivated = 0;
volatile uint8_t gotFeedback_ch0 = 0;
volatile uint8_t gotFeedback_ch1 = 0;
volatile bool timer_interrupted;
volatile uint8_t torqenc_timeout;
uint16_t error_ecu;

ft_uint8_t number_of_adjustments[6] = {6, 0, 12, 5, 4, 2};  //Number of adjustments on each device.
ft_uint8_t adjustment_value[6] = {0, 0, 1, 0, 0, 0};		//The current value on device
ft_uint16_t adjustment_value_ECU[5] = {0, 0, 0, 5, 3};
ft_uint8_t adjustment_value_CCU[3] = {0, 0, 0};
ft_uint8_t number_of_adjustments_special[5] = {5, 5, 5, 20, 19};
ft_uint8_t lc_init_trq_val[20] = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
ft_uint8_t lc_rise_time_val[19] = {25, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200};

ft_uint8_t new_adjustment_value[6] = {0, 0, 1, 0, 0, 0};	//A vector to store possible new values
ft_uint8_t new_adjustment_value_ECU[5] = {0, 0, 0, 5, 3};
ft_uint8_t new_adjustment_value_CCU[3] = {0, 0, 0};
ft_uint8_t type_of_adjustment[6] = {DRIVER_INTERVaL, NONE, DAVE_INTERVAL, EXTRA_MENU, MAX_TRQ, CALIB}; //Type of adjustments on each device
ft_uint8_t adjusments_extra_menu[6] = {3, 3};
uint16_t ccuecu_values[5] = {0, 500, 1000, 1500, 2000};
uint16_t max_trq_values[4] = {100, 75, 50, 5};
bool check_alive;
bool check_err_ecu;
bool isAlive[DASH_ALIVE_SIZE];
bool isAlive_check[DASH_ALIVE_SIZE] = {0};
bool all_alive = false;
bool check_temp;
bool check_voltage;
uint8_t dave_msg[10] = {0xF9, 0xFB, 0xFC, 0xFD, 0xFE, 0x01, 0x02, 0x03, 0xDE, 0x32};
	
uint8_t battery_temp;
uint16_t motor_temp;
uint16_t IGBT_temp;
uint8_t min_cell_voltage = 39;
uint8_t max_cell_voltage = 39;
uint16_t pack_voltage = 550;
uint16_t DC_bus_voltage = 550;
uint16_t gearbox_temp;
uint16_t glv_voltage = 8500;
	
can_msg_t msg_tx_ch0;
can_mob_t appli_tx_ch0;
can_msg_t msg_tx_ch1;
can_mob_t appli_tx_ch1;




void sendChangeCAN(ft_uint8_t menu, ft_uint8_t device){
	//This function send CAN msg if adjustments have been done 
	
		switch (device){
			case  DRIVER:
			appli_tx_ch0.can_msg->id =DASH_DATA|CANR_MODULE_ID3_ID;
			appli_tx_ch0.can_msg->data.u8[0] = adjustment_value[DRIVER];
			appli_tx_ch0.can_msg->data.u8[1]= 0x00; //ingen Ack
			
			
			can_tx(0,
			appli_tx_ch0.handle,
			2,
			appli_tx_ch0.req_type,
			appli_tx_ch0.can_msg);
			
			/*saved_adjustments[DRIVER] = adjustment_value[DRIVER];*/
			break;
			
			case DEVICESTATUS:
			
			
			break;
			
			case DAVE:
			appli_tx_ch0.can_msg->id =DASH_DATA|CANR_MODULE_ID2_ID;
			if(adjustment_value[DAVE]==0){
				appli_tx_ch0.can_msg->data.u8[0]= 0x01; //DAVE ON
			}else if(adjustment_value[DAVE]==1){
				appli_tx_ch0.can_msg->data.u8[0]= 0x02; //DAVE OFF
			}else{
				appli_tx_ch0.can_msg->data.u8[0] = 0x03; //DAVE test
				appli_tx_ch0.can_msg->data.u8[1] = dave_msg[adjustment_value[DAVE]-2]; //DAVE test nummer
					//Hvis byte 1 er 0x03 så må jeg legge til et nummer i byte 2 som sier hvilken testlyd jeg vil kjøre!!!!
			}
			
			can_tx(0,
			appli_tx_ch0.handle,
			2,
			appli_tx_ch0.req_type,
			appli_tx_ch0.can_msg);
			break;
			
			case ECU:
			appli_tx_ch0.can_msg->id =DASH_PRI_ID2;
			appli_tx_ch0.can_msg->data.u16[0] = ccuecu_values[adjustment_value_ECU[0]];
			appli_tx_ch0.can_msg->data.u16[1] = ccuecu_values[adjustment_value_ECU[1]];
			appli_tx_ch0.can_msg->data.u16[2] = ccuecu_values[adjustment_value_ECU[2]];
			
			/*saved_adjustments[ECU] = adjustment_value_ECU[0];
			saved_adjustments[ECU+1] = adjustment_value_ECU[1];
			saved_adjustments[ECU+2] = adjustment_value_ECU[2];*/
			
			can_tx(0,
			appli_tx_ch0.handle,
			8,
			appli_tx_ch0.req_type,
			appli_tx_ch0.can_msg);
			
			//send launch control init torque on launch control rise time
			appli_tx_ch0.can_msg->id = DASH_PRI_ID4;
			appli_tx_ch0.can_msg->data.u8[0] = lc_init_trq_val[adjustment_value_ECU[3]];
			appli_tx_ch0.can_msg->data.u8[1] = lc_rise_time_val[adjustment_value_ECU[4]];
			
			can_tx(0,
			appli_tx_ch0.handle,
			2,
			appli_tx_ch0.req_type,
			appli_tx_ch0.can_msg);
			
			break;
			
			case MAXTRQ:
				appli_tx_ch0.can_msg->id =DASH_PRI_ID3;
				appli_tx_ch0.can_msg->data.u16[0] = max_trq_values[adjustment_value[MAXTRQ]];
				
				can_tx(0,
				appli_tx_ch0.handle,
				2,
				appli_tx_ch0.req_type,
				appli_tx_ch0.can_msg);
				
				cmd(CMD_DLSTART);
				cmd(CLEAR(1, 1, 1)); // clear screen
				cmd(COLOR_RGB(255,255,255));
				cmd_number(290, 136, 29, OPT_CENTER, max_trq_values[adjustment_value[MAXTRQ]]);
				cmd_text(200, 136, 29, OPT_CENTER, "Max torque:");
				cmd(DISPLAY()); // display the image
				cmd(CMD_SWAP);
				cmd_exec();
	
				myDelay(1500);
			break;
			
			case TRQENC_CALIB:
			appli_tx_ch0.can_msg->id =DASH_CMD|CANR_MODULE_ID4_ID;
			appli_tx_ch1.can_msg->id =DASH_CMD|CANR_MODULE_ID4_ID;
			if(pedal_position==1){
				appli_tx_ch0.can_msg->data.u8[0] = 0xF0;
				appli_tx_ch1.can_msg->data.u8[0] = 0xF0;
			}else if(pedal_position==0){
				appli_tx_ch0.can_msg->data.u8[0] = 0x0F;
				appli_tx_ch1.can_msg->data.u8[0] = 0x0F;
			}
			
			can_tx(0,
			appli_tx_ch0.handle,
			1,
			appli_tx_ch0.req_type,
			appli_tx_ch0.can_msg);
		
			can_tx(1,
			appli_tx_ch1.handle,
			1,
			appli_tx_ch1.req_type,
			appli_tx_ch1.can_msg);
			break;
		}//END switch
		
}//END function


void checkStatus(void){
	//This function update data on display and do things if buttons are pushed
	
	//----Launchcontrol-------------------------//
	if(launchActivated>0){
		if(launchAck==1){
			launchActivated=4;
			launchAck=0;
		}
	
		if(launchActivated==2){
			gpio_set_pin_high(WARNING_LC);
			writeString("Activating launch control. Pedal to max", 27, 235,135);
			//myDelay(3000);
		}else if(launchActivated==3){
			writeString("Launch control ready! Push button to confirm", 27, 235, 135);
		}else if(launchActivated==5){
			appli_tx_ch0.can_msg->id =DASH_PRI_ID5;
			appli_tx_ch0.can_msg->data.u8[0] = 2;
			
			can_tx(0,
			appli_tx_ch0.handle,
			1,
			appli_tx_ch0.req_type,
			appli_tx_ch0.can_msg);
			
		}else if(launchActivated==6){
			gpio_set_pin_low(WARNING_LC);
			//launchActivated = 0;
			writeString("Launch disabled", 28, 235, 135);
			myDelay(1500);
			launchActivated=0;
		}
		
	}else{
	
	if(launchAck==1){
		if(launchActivated==0){
			//Send CAN msg to ECU
			appli_tx_ch0.can_msg->id =DASH_PRI_ID5;
			appli_tx_ch0.can_msg->data.u8[0] = 0x01; //LC requested
			
			can_tx(0,
			appli_tx_ch0.handle,
			1,
			appli_tx_ch0.req_type,
			appli_tx_ch0.can_msg);
			
			writeString("Launch request sent", 28, 235, 135);
			myDelay(2000);
			getBackDisplay();
			launchActivated=1;
		}
		launchAck =0;
	}
	
	//----------Acknowledge--------------------//
	
	if(acknowledgeAck==1){
		
		//Send CAN msg to datalogger
		appli_tx_ch0.can_msg->id =DASH_DATA|CANR_MODULE_ID3_ID;
		appli_tx_ch0.can_msg->data.u8[0] = adjustment_value[DRIVER];
		appli_tx_ch0.can_msg->data.u8[1]= 0x01; //0x01 er ack. 0x00 er ikke ack
		
		can_tx(0,
		appli_tx_ch0.handle,
		2,
		appli_tx_ch0.req_type,
		appli_tx_ch0.can_msg);	
			
		writeString("Acknowledge sent", 28, 235, 135);
		myDelay(2000);
		getBackDisplay();
		acknowledgeAck = 0; 
	}
	
	//----------Joystick up-----------------//
	if(move_up==1){
		if(horizontal_position==ADJUSTMENT_MENU){	//If we are in the adjustment menu
			if(vertical_level_adjustment_menu>0){	//If we can move further up
				vertical_level_adjustment_menu-=1;
				
				//Update screen
				adjustmentMenu(vertical_level_adjustment_menu);
			}
			
		}else if (horizontal_position==ADJUSTMENT){	//If we are in an adjustment display
			if(vertical_level_adjustment>0){	//If we are able to move further up
				vertical_level_adjustment-=1;
				new_adjustment_value[vertical_level_adjustment_menu]=vertical_level_adjustment;
				
				//Update screen
				adjustment(vertical_level_adjustment_menu, vertical_level_adjustment, type_of_adjustment[vertical_level_adjustment_menu]);
			}
			
		}else if(horizontal_position==EXTRA_MENU){		//If we are in an adjusmtent screen where it is possible to move one more to the right then usual
			if(vertical_level_adjustment_spes>0){	//If we are able to move further up
				vertical_level_adjustment_spes-=1;
				if(vertical_level_adjustment_menu==ECU){
					new_adjustment_value_ECU[vertical_level_adjustment] = vertical_level_adjustment_spes;
					
					//Update screen
					adjustmentSpecial(vertical_level_adjustment, new_adjustment_value_ECU[vertical_level_adjustment]);
				//DONT think this one is in use!!!!!!!
				}else if(vertical_level_adjustment_menu==MAXTRQ){
					new_adjustment_value_CCU[vertical_level_adjustment] = vertical_level_adjustment_spes;
					adjustmentSpecial(vertical_level_adjustment, new_adjustment_value_CCU[vertical_level_adjustment]);
				}
			}
		}
		move_up=0; 
	}
	
	//-------------Joystick down-------------//
	
	if(move_down==1){
		if(horizontal_position==ADJUSTMENT_MENU){	//If we are in the adjustment menu
			if(vertical_level_adjustment_menu<NUMBER_OF_UNITS-1){	//If we can move further down
				vertical_level_adjustment_menu+=1;
				
				//Update screen
				adjustmentMenu(vertical_level_adjustment_menu);	
			}
			
		}else if(horizontal_position==ADJUSTMENT){	//If we are in an adjustment for a device
			if(vertical_level_adjustment<number_of_adjustments[vertical_level_adjustment_menu]-1){	//If we can move further down
				vertical_level_adjustment+=1;
				new_adjustment_value[vertical_level_adjustment_menu]=vertical_level_adjustment;	//Store the new adjustment value
				
				//Update Screen
				adjustment(vertical_level_adjustment_menu, vertical_level_adjustment, type_of_adjustment[vertical_level_adjustment_menu]);
			}
			
		}else if(horizontal_position==EXTRA_MENU){
			if(vertical_level_adjustment_spes<number_of_adjustments_special[vertical_level_adjustment]-1)
				vertical_level_adjustment_spes+=1;
				if(vertical_level_adjustment_menu==ECU){
					new_adjustment_value_ECU[vertical_level_adjustment] = vertical_level_adjustment_spes;
					
					//Update screen
					adjustmentSpecial(vertical_level_adjustment, new_adjustment_value_ECU[vertical_level_adjustment]);
				//DONT think this one is in use anymore!!!
				}else if(vertical_level_adjustment_menu==MAXTRQ){
					new_adjustment_value_CCU[vertical_level_adjustment] = vertical_level_adjustment_spes;
					adjustmentSpecial(vertical_level_adjustment, new_adjustment_value_CCU[vertical_level_adjustment]);
				}
		}
		move_down=0;
	}
	
	//-----------Joystick right------------//
	
	if(move_right==1){
		if((horizontal_position<NUMBER_OF_MENUES-1)){	//If we can move further right
			if (horizontal_position==ADJUSTMENT_MENU){	//If we are in the adjustment menu
				vertical_level_adjustment = adjustment_value[vertical_level_adjustment_menu]; //Get the current adjustment of the device
				//Update to new screen
				adjustment(vertical_level_adjustment_menu, adjustment_value[vertical_level_adjustment_menu], type_of_adjustment[vertical_level_adjustment_menu]);
			
			}else if(horizontal_position==MAINMENU){	//If we are in the main menu
				adjustmentMenu(vertical_level_adjustment_menu);
			
			}else if(horizontal_position==MAINMENU_2){ //If we are in main menu 2
				mainMenu();
			
			}else if(horizontal_position==MAINMENU_3){
				mainMenu2();
			
			}
			horizontal_position+=1;
		
		}else if((horizontal_position==ADJUSTMENT)&&(type_of_adjustment[vertical_level_adjustment_menu]==EXTRA_MENU)){ //If you are going into an extra menu
			if(vertical_level_adjustment_menu==ECU){
				vertical_level_adjustment_spes = adjustment_value_ECU[vertical_level_adjustment];
				//Update to new screen
				adjustmentSpecial(vertical_level_adjustment, new_adjustment_value_ECU[vertical_level_adjustment]);
			//not in use anymore!!
			}else if(vertical_level_adjustment_menu==MAXTRQ){
				vertical_level_adjustment_spes = adjustment_value_ECU[vertical_level_adjustment];
				//Update to new Screen
				adjustmentSpecial(vertical_level_adjustment, new_adjustment_value_CCU[vertical_level_adjustment] );
			}
			horizontal_position+=1;
		}
			
		move_right=0;		
	}
	
	
	
	//-----------Joystick left-----------//
	
	if(move_left==1){
		if (horizontal_position>-1){	//If we can move further left
			if(horizontal_position==EXTRA_MENU){
				//Update to new screen
				adjustment(vertical_level_adjustment_menu, vertical_level_adjustment, type_of_adjustment[vertical_level_adjustment_menu]);
				if(vertical_level_adjustment_menu==ECU){
					if(vertical_level_adjustment_spes!=adjustment_value_ECU[vertical_level_adjustment]){
						adjustment_value_ECU[vertical_level_adjustment] = new_adjustment_value_ECU[vertical_level_adjustment];
						//Send new value on CAN
						sendChangeCAN(horizontal_position, vertical_level_adjustment_menu);
					}
				}
			
			}else if(horizontal_position==ADJUSTMENT){ //If we are in the adjustment menu
				if(vertical_level_adjustment_menu==TRQENC_CALIB){
					if(vertical_level_adjustment==MAX_CALIB){	//Calibrate max throttle
						pedal_position = 1;
						writeString("Pedal to the metal", 29, 240, 136);
						myDelay(3000);
						
						//Send CAN msg to torque encoders that you would like to calibrate pedal
						sendChangeCAN(horizontal_position, vertical_level_adjustment_menu);
						torqenc_timeout=0;
						
						//Wait for reciept from torque encoders that calibration is OK. 
						//Brake if it takes more than 5 seconds.
						while((gotFeedback_ch0==0)&&(gotFeedback_ch1==0)&&(torqenc_timeout<5)){ //|| (gotFeedback_ch1==0)){
							
						}
						if((gotFeedback_ch0==1)&&(torqenc_timeout<5)){
							writeString("Calibration done!", 29, 240, 136);
						}else{
							writeString("Calibration failed!", 29, 240, 136);
						}
						torqenc_timeout = 0x0A;
						gotFeedback_ch0=0;
						gotFeedback_ch1=0;
						myDelay(2000);
					
					}else if(vertical_level_adjustment==ZERO_CALIB){	//Calibrate zero throttle
						pedal_position = 0;
						writeString("Remove foot from pedal", 29, 240, 136);
						myDelay(3000);
						
						//Send CAN msg to torque encoders that you would like to calibrate pedal
						sendChangeCAN(horizontal_position, vertical_level_adjustment_menu);
						torqenc_timeout = 0;
						
						//Wait for reciept from torque encoders that calibration is OK.
						//Brake if it takes more than 5 seconds.
						while((gotFeedback_ch0 == 0)&&(gotFeedback_ch1 == 0)&&(torqenc_timeout<5)){// || (gotFeedback_ch1 == 0)){
							
						}
						if((gotFeedback_ch0==1)&&(torqenc_timeout<5)){
							writeString("Calibration done!", 29, 240, 136);
						}else{
							writeString("Calibration failed!", 29, 240, 136);
						}
						torqenc_timeout = 0x0A;
						gotFeedback_ch0=0;
						gotFeedback_ch1=0;
						//delay_ms(2000);
						myDelay(2000);
					}
					adjustment_value[vertical_level_adjustment_menu] = new_adjustment_value[vertical_level_adjustment_menu];
				}else if((vertical_level_adjustment!=adjustment_value[vertical_level_adjustment_menu])&&(vertical_level_adjustment_menu!=ECU)){	//If the new value for a device is different from the old value 
					/*if((vertical_level_adjustment==0)||(vertical_level_adjustment==1)){
						saved_adjustments[DAVE] = adjustment_value[DAVE];
					}*/
					adjustment_value[vertical_level_adjustment_menu] = new_adjustment_value[vertical_level_adjustment_menu];	//Update the current value vector
					//Send CAN msg
					sendChangeCAN(horizontal_position, vertical_level_adjustment_menu);
				}
				adjustmentMenu(vertical_level_adjustment_menu);
			
			}else if(horizontal_position==ADJUSTMENT_MENU){
				mainMenu();
			
			}else if(horizontal_position==MAINMENU){
				mainMenu2();
			
			}else if(horizontal_position==MAINMENU_2){
				mainMenu3();
			}
			horizontal_position-=1;
		}
			
		move_left=0;		
	}
	
	//If changes has occured on data displayed
	if(monitor_status){
		if (horizontal_position==MAINMENU){
			mainMenu();
		}else if(horizontal_position==MAINMENU_2){
			mainMenu2();
		}else if (horizontal_position==MAINMENU_3){
			mainMenu3();
		}
		monitor_status=0;
	}
	
	//If BSPD have lost signal
	if(BSPD_status==0x42){
		writeString("Signal loss BSPD", 28, 235, 135);
		myDelay(2000);
		getBackDisplay();
		BSPD_status=0x00;
	}
	
	//If ECU say play RTDS
	if(play_RTDS){
		gpio_set_pin_high(BUZZER);
		delay_ms(1800);
		gpio_set_pin_low(BUZZER);
		
		appli_tx_ch0.can_msg->id = DASH_PRI_ID0;
		appli_tx_ch0.can_msg->data.u8[0] = 0x01;
		
		can_tx(0,
		appli_tx_ch0.handle,
		1,
		appli_tx_ch0.req_type,
		appli_tx_ch0.can_msg);
		
		play_RTDS=false;
	}
	
	//If msg received from ECU that drive is enabled
	if(check_drive){
		if(driveEnabled){
			gpio_set_pin_high(TRACTIVEDRIVE_LED);
			
			writeWarning("Ready to drive!");
			
			//writeString("Ready to drive!", 28, 235, 135);
			//delay_ms(500);
			myDelay(1500);
			getBackDisplay();
		}else{
			gpio_set_pin_low(TRACTIVEDRIVE_LED);
			writeString("Drive disabled!", 28, 235, 135);
			//delay_ms(500);
			myDelay(1500);
			getBackDisplay();
		}
		check_drive = false;
	}
	
	//If dashboard is done counting and ready to launch or launch turned off
	if(chechLaunch==1){
		if(launch_traction==0x0001){
			gpio_set_pin_high(WARNING_LC);
			writeString("Activating launch control. Pedal to max", 27, 235,135);
			//myDelay(3000);
			launchActivated = 1;
		}else if(launch_traction==0x0002){
			launchActivated = 2;
			writeString("Launch control ready! Push button to confirm", 27, 235, 135);
		}else if(launch_traction==3){
			appli_tx_ch0.can_msg->id =DASH_PRI_ID5;
			appli_tx_ch0.can_msg->data.u8[0] = 0x0002;
			
			can_tx(0,
			appli_tx_ch0.handle,
			1,
			appli_tx_ch0.req_type,
			appli_tx_ch0.can_msg);
			
		}else if(launch_traction==4){
			gpio_set_pin_low(WARNING_LC);
			//launchActivated = 0;
			writeString("Launch disabled", 28, 235, 135);
			myDelay(1500);
		}
		chechLaunch = false;
	}
	
	//If it is time to check alive. Timer counter set this variable
	if(check_alive){
		check_alive= false;
		all_alive = true;
		for (int i=0; i<DASH_ALIVE_SIZE; i++){
			isAlive_check[i] = isAlive[i];
			if(isAlive[i] == 0){ 
				all_alive = false;
			}else{
				isAlive[i]=0;
			}
		}
		if((gpio_get_pin_value(ALIVE_LED)==1)&&(all_alive==true)){
			gpio_set_pin_low(ALIVE_LED);
		}else if((gpio_get_pin_value(ALIVE_LED)==0)&&(all_alive==false)){
			gpio_set_pin_high(ALIVE_LED);
		}
		
		getBackDisplay();
	}
	
	//If msg received from ECU with error
	if(check_err_ecu){
		if(error_ecu==0){
			gpio_set_pin_low(ECU_ERR_LED);
		}else{
			gpio_set_pin_high(ECU_ERR_LED);
			//Legge inn kode for å tolke hvilke(n) feil som har oppstått.
		}
		check_err_ecu=false;
	}
	
	//If temp of some reason have been bigger than some value;
	//KANKSJE MULIG Å SJEKKE FOR DETTE DIREKTE DER VI SKRIVER TIL SKJERM
	//DA KAN MAN SKRIvE RØD SKRIFT OGSÅ HVIS FOR HØY OG TOGGLE LED SAMTIDIG
	if(check_temp){
		if((((motor_temp-9871.7)/54.2)>110)||(battery_temp>55)||(IGBT_temp>24000)||(-gearbox_temp*((float)50/2739)+189.28)>70){
			gpio_set_pin_high(TMP_LED);
		}else{
			gpio_set_pin_low(TMP_LED);
		}
		check_temp = false;
	}
	if(check_voltage){
		if((min_cell_voltage<32)||(max_cell_voltage>42)){
			gpio_set_pin_high(VOLTAGE_LED);
		}
		check_voltage = false;
	}
	}
}

