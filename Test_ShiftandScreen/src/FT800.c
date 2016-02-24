/*
 * FT600.c
 *
 * Created: 20.03.2014 15:17:52
 *  Author: Øystein
 */ 


#include "FT800.h"
#include <asf.h>
#include "spi_master.h"
#include "string.h"
#include "CANdefinitions.h"
#include "Interrupts_and_can.h"
#include "screen_pic.h"
#include "revolve_logo.h"



struct spi_device SPI_DEVICE_0 = {
	.id = 0
};

uint16_t rpm;
int16_t torque;
uint16_t compareVector[9] = {0b1, 0b10, 0b100, 0b1000, 0b10000, 0b100000, 0b1000000, 0b10000000, 0b100000000};
uint8_t compareVector1[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
/*void DelayMs(unsigned t);

void DelayMs(unsigned t) {
    unsigned int tWait;

    tWait = (FPB / 1000) * t;
    WriteCoreTimer(0);
    while (ReadCoreTimer() < tWait); // wait for the time to pass
}
*/

unsigned short dli = 0;
unsigned short cmd_offset = 0;
unsigned short screen_offset = 100;
ft_char8_t *adjustment_devices[6] = {"Driver", "Device status", "DAVE", "ECU", "Max torque", "Torque encoder calib"};
ft_char8_t * adjustment_devices_special[5] = {"Proportional gain", "Integral gain", "Derivat gain", "LC initial trq", "LC rise time"};
ft_char8_t *interval_onoff[2] = {"OFF", "ON"};
ft_char8_t *interval_calib[2] = {"Max position", "Zero position"};
ft_char8_t *interval_5_postoneg[5] = {"-2", "-1", "0", "1", "2"};
ft_char8_t *interval_5_pos[5] = {"1", "2", "3", "4", "5"};
ft_char8_t *interval_3_pos[5] = {"OFF", "ON", "Test" };
ft_char8_t *ecu_pos[5] = {"0", "500", "1000", "1500", "2000"};
ft_char8_t *lc_init_trq[20] = {"5", "10", "15", "20", "25", "30", "35", "40", "45", "50", "55", "60", "65", "70", "75", "80", "85", "90", "95", "100"};
ft_char8_t *lc_rise_time[19] = {"25", "30", "40", "50", "60", "70", "80", "90", "100", "110", "120", "130", "140", "150", "160", "170", "180", "190", "200"};
ft_char8_t * dave_interval[12] = {"ON", "OFF", "Test 1", "test 2", "Test 3", "Test 4", "test 5", "Test 6", "Test 7", "test 8", "Test 9", "Test 10"};
ft_char8_t *driver_interval[6] = {"Driver 1", "Driver 2", "Driver 3", "Driver 4", "Driver 5", "The Stig"};
ft_char8_t *devicestatus_interval[12] = {"ECU", "TRQ 0", "TRQ 1", "BSPD", "TEL", "DAT", "ADC 1", "ADC 2", "CCU", "FAN", "BMS", "Steer enc"};
ft_char8_t *maxtrq_interval[4] = {"100 %", "75 %", "50 %", "Shell ECO"};
ft_char8_t *ecu_error_names[9] = {"Lost BMS communication", "Inverter voltage < 0.9*batt pck voltage", "Lost inverter communication", "Internal error in inverter", "Lost torque sensor communication", "Lost speed sensor communication", "FRG_RUN is low", "AIR_PLUS is low", "BSPD loss signal"};
ft_char8_t *bms_fault_names[19] = {"Driving off while plugged in", "Interlock is tripped", "Comm fault with bank or cell", "Charge overcurrent", "Discharge overcurrent", "Over-temperature", "Under voltage", "Over voltage", "No battery voltage", "High voltage B- leak to chassis", "High voltage B+ leak to chassis", "Relay K1 i shorted", "Contactor K2 is shorted", "Contactor K3 is shorted", "Open K1 or K3, or shorted K2", "Open K2", "Excessive precharge time", "EEPROM stack overflow", "Loss of CAN from HVFE"};	
ft_char8_t *bms_warning_names[8] = {"Isolation fault", "Low SOH", "High temperature", "Low temperature", "Discharge overcurrent", "Charge overcurrent", "High voltge", "Low voltage"};
//ft_char8_t *interval_20_pos[20] = {"0", "100", "200", "300", "400", "500", "600", "700", "800",};
uint8_t vertical_level_adjustment_menu;
uint8_t vertical_level_adjustment;
int8_t horizontal_position;
uint8_t vertical_level_adjustment_spes;
uint8_t new_adjustment_value_ECU[3];
uint8_t new_adjustment_value_CCU[3];
ft_uint8_t adjustment_value[6];
ft_uint8_t type_of_adjustment[6];
ft_uint8_t new_adjustment_value[6];

ft_uint8_t number_of_adjustments[6];

bool isAlive[DASH_ALIVE_SIZE];
bool isAlive_check[DASH_ALIVE_SIZE];

uint16_t motor_temp;
uint16_t IGBT_temp;
uint16_t DC_bus_voltage;
int16_t torque_encoder_ch0;
int16_t torque_encoder_ch1;
uint8_t leftOnBattery;
uint8_t tractive_status;
uint8_t battery_temp;
bool driveEnabled;
uint16_t brk_pres_rear;
uint16_t brk_pres_front;
uint16_t steer_ang;
uint8_t min_battery_temp;
uint8_t max_battery_temp;
uint8_t min_cell_voltage;
uint8_t max_cell_voltage;
uint8_t min_cell_id;
uint8_t max_cell_id;
uint16_t pack_voltage;
uint16_t gearbox_temp;
uint16_t gearbox_temp_cal = 0;
bool check_temp;
bool check_voltage;
uint16_t glv_voltage;
float glv_voltage_calib = 0.0;
uint16_t error_ecu;
uint8_t bms_fault_code;
uint8_t bms_warning;
 bool valid_torque_ch1;
 bool valid_torque_ch0;

uint8_t launchActivated;
uint16_t launch_traction;

void FT800_StartTransmission(void){
	gpio_set_pin_low(CS_0);
	spi_select_device(&AVR32_SPI0, &SPI_DEVICE_0);
	

}

void FT800_StopTransmission(void){
	spi_deselect_device(&AVR32_SPI0, &SPI_DEVICE_0);
	gpio_set_pin_high(CS_0);
}


void tr8(int value){
	
	while(!spi_is_tx_ready(&AVR32_SPI0));
	spi_write_single((&AVR32_SPI0), value);
	
}
void tr16(int value){
	tr8((value) & 0xFF);
	tr8(((value) >> 8) & 0xFF);
}

void tr32(int value){
	tr16((value) & 0xFFFF);
	tr16(((value) >> 16) & 0xFFFF);
}


void host_command(ft_uint8_t command){
    FT800_StartTransmission();
    tr8(command);
    tr8(0);
    tr8(0);
    FT800_StopTransmission();
}

void wr8(unsigned long addr, ft_uint8_t value){
    FT800_StartTransmission();
    tr8(0x80 | (addr >> 16));
    tr8(addr >> 8);
    tr8(addr);
    tr8(value);
	
    FT800_StopTransmission();
}

void wr16(unsigned long addr, ft_uint16_t value){
    FT800_StartTransmission();
    tr8(0x80 | (addr >> 16));
    tr8(addr >> 8);
    tr8(addr);
    tr16(value);
    FT800_StopTransmission();
}

void wr32(unsigned long addr, ft_uint32_t value){
    FT800_StartTransmission();
    tr8(0x80 | (addr >> 16));
    tr8(addr >> 8);
    tr8(addr);
    tr32(value);
    FT800_StopTransmission();
}

ft_uint8_t wr8s(unsigned long addr, const ft_char8_t *s)
{
	FT800_StartTransmission();
	tr8(0x80 | (addr >> 16));
	tr8(addr >> 8);
	tr8(addr);
	ft_uint8_t l = strlen(s);
	ft_uint8_t i;
	for(i=0;i<=l;i++){
		tr8(s[i]);
	}
	for(;i%4>0;i++){
		tr8(0);
	}
	FT800_StopTransmission();
	return i;
}
/*
ft_uint8_t wr8s(unsigned long addr, const ft_char8_t *s)  //Her er det noe feil.......
{
    int SPI1BUF;
	FT800_StartTransmission();
    tr8(0x80 | (addr >> 16));
    tr8(addr >> 8);
    tr8(addr);
    ft_uint8_t l = strlen(s);
    ft_uint8_t i;
	wr32(RAM_DL+dli, CLEAR(1,1,1)); dli+=4;
	wr32(RAM_DL+dli, BEGIN(BITMAPS)); dli+=4;
    for(i=0;i<=l;i++){
        SPI1BUF = s[i];
		tr8(SPI1BUF);
        //while(!spi_is_tx_ready(&AVR32_SPI0));//SSPSTATbits.BF
			
    }
	wr32(RAM_DL+dli, END());
	wr32(RAM_DL+dli, DISPLAY());
	wr8(REG_DLSWAP, DLSWAP_FRAME);
	dli=0; screen_offset=100;
	delay_ms(2000);
	startScreen();
    for(;i%4>0;i++){
        SPI1BUF = 0;
        //while(!spi_is_tx_ready(&AVR32_SPI0));
		tr8(SPI1BUF);
    }
    FT800_StopTransmission();
    return i;
}*/

ft_uint8_t rd8(unsigned long addr){
    FT800_StartTransmission();
    tr8(addr >> 16);
    tr8(addr >> 8);
    tr8(addr);
    tr8(0); // dummy byte
    tr8(0);
	uint8_t rxdata = (uint8_t)spi_get(&AVR32_SPI0);
    FT800_StopTransmission();
	return rxdata;
}

ft_uint16_t rd16(unsigned long addr){
    FT800_StartTransmission();
    tr8(addr >> 16);
    tr8(addr >> 8);
    tr8(addr);
    tr8(0); // dummy byte
    tr8(0);
    ft_uint8_t temp =  (uint8_t)spi_get(&AVR32_SPI0);
    tr8(0);
	ft_uint8_t spibuf = (uint8_t)spi_get(&AVR32_SPI0);
    FT800_StopTransmission();
    return ((ft_uint16_t)spibuf<< 8) | temp;
}/*

ft_uint32_t rd32(unsigned long addr){
    FT800_StartTransmission();
    tr8(addr >> 16);
    tr8(addr >> 8);
    tr8(addr);
    tr8(0); // dummy byte
    tr8(0);
    ft_uint32_t temp = SPI1BUF;
    tr8(0);
    temp |= (ft_uint32_t)SPI1BUF << 8;
    tr8(0);
    temp |= (ft_uint32_t)SPI1BUF << 16;
    tr8(0);
    FT800_StopTransmission();
    return ((ft_uint32_t)SPI1BUF << 24) | temp;
}

/*
ft_char8_t * rd8s(unsigned long addr, ft_uint8_t len)
{
    FT800_StartTransmission();
    tr8(addr >> 16);
    tr8(addr >> 8);
    tr8(addr);
    tr8(0); // dummy byte
    ft_char8_t *s = (ft_char8_t*) malloc (len+1);
    ft_uint8_t i;
    for(i=0;i<=len;i++){
        tr8(0);
        s[i] = SPI1BUF;
    }
    s[len]=0;
    FT800_StopTransmission();
    return s;
}
*/

void cmd_incrementn(unsigned char n){
    cmd_offset=(cmd_offset+n)%4096;
}

void cmd(ft_uint32_t command){
    wr32(RAM_CMD + cmd_offset, command);cmd_increment4();
}

#define ACTIVE  0x00
#define STANDBY 0x41
#define SLEEP   0x42
#define PWRDOWN 0x50
#define CLKEXT  0x44
#define CLK48M  0x62
#define CLK36M  0x61
#define CORERST 0x68

void FT800_Init(void){
	//Initialize the screen controller
	
	spi_master_setup_device((&AVR32_SPI0), &SPI_DEVICE_0, SPI_MODE_0, SPI_BAUDRATE, 1);
	spi_enable((&AVR32_SPI0));
	delay_ms(20);
	gpio_set_pin_low(POWER_DOWN);
	delay_ms(200);
	gpio_set_pin_high(POWER_DOWN);
	delay_ms(20);
	
    host_command(ACTIVE); //send host command "ACTIVE" to FT800

    host_command(CLKEXT); //send command to "CLKEXT" to FT800

    host_command(CLK48M); //send command to "CLKEXT" to FT800
	
    wr16(REG_HSIZE, 480); // width resolution
    wr16(REG_VSIZE, 272); // height resolution
    wr16(REG_HCYCLE, 525); // number if horizontal cycles for display
    wr16(REG_HSYNC0, 0); // hsync falls
    wr16(REG_HSYNC1, 41); // hsync rise
    wr16(REG_HOFFSET, 43); // horizontal offset from starting signal
    wr16(REG_VCYCLE, 286); // number of vertical cycles for display
    wr16(REG_VSYNC0, 0); // vsync falls
    wr16(REG_VSYNC1, 10); // vsync rise
    wr16(REG_VOFFSET, 12); // vertical offset from start signal
    wr8(REG_CSPREAD, 0); // output clock spread enable
    //wr8(REG_DITHER, 0); // output number of bits
    //wr16(REG_OUTBITS, 0x01B6); // output bits resolution
    wr8(REG_SWIZZLE, 0); // output swizzle

    wr8(REG_PCLK_POL, 1); // clock polarity: 0 - rising edge, 1 - falling edge

    // write first display list
    wr32(RAM_DL+0,CLEAR_COLOR_RGB(0,0,0));
    wr32(RAM_DL+4,CLEAR(1,1,1));
    wr32(RAM_DL+8,DISPLAY());

    wr8(REG_DLSWAP,DLSWAP_FRAME);//display list swap
	
    wr8(REG_GPIO_DIR, 0x80);
    wr8(REG_GPIO, 0xff);

    wr8(REG_PCLK,5); // clock prescaler (0: disable, >0: 48MHz/pclock)

}

void writeString(char *s, ft_uint8_t fontsize, uint8_t x, uint8_t y ){
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	cmd(COLOR_RGB(255,255,255));
	cmd_text(x, y, fontsize, OPT_CENTER, s);
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
    
}

void writeShiftregChange(ft_uint8_t start1, ft_uint16_t kers, ft_uint16_t slipref){ 
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	
	if(start1 == 0x01){
		cmd_text(235, 100, 28, OPT_CENTER, "Drive:  Enabled");
	}else{
		cmd_text(235, 100, 28, OPT_CENTER, "Drive:  OFF");
	}
	cmd_text(220, 150, 28, OPT_CENTER, "KERS: ");
	cmd_number(285, 150, 28, OPT_CENTER, kers);
	cmd_text(225, 200, 28, OPT_CENTER, "Slip ref: ");
	cmd_number(290, 200, 28, OPT_CENTER, slipref);
	
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
	
	
}

//Draw the main menu
void mainMenu(void){
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	//cmd_text(236, 20, 29, OPT_CENTER, "Main");
	drawRevolve(240, 70);
	drawRPM();
	drawBattery();
	drawPower();
	drawTemperature(motor_temp, MAINMENU);
	drawLaunchTractiveStatus();
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();	
	//delay_ms(20);
	myDelay(10);
}

//Draw the system monitor
void mainMenu2(void){
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	cmd_text(236, 20, 29, OPT_CENTER, "System monitor");
	//drawDCbusVoltage();
	drawTorqueEncoder();
	drawBreakPres();
	drawSteerAng();
	drawECUerr();
	drawBMSerr();
	//drawVoltages();
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
	//delay_ms(20);
	myDelay(10);
	
	
}

//Draw temperature monitor
void mainMenu3(void){
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	cmd_text(236, 20, 29, OPT_CENTER, "Temperatures and voltages");
	drawTemperature(IGBT_temp, MAINMENU_3);
	drawVoltages();
	drawDCbusVoltage();
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
	//delay_ms(20);
	myDelay(10);
}

/*
void mainMenu4(void){
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	cmd_text(236, 20, 29, OPT_CENTER, "Voltages");
	drawVoltages();
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
	//delay_ms(20);
	myDelay(20);
}*/

//Draw the start up screen. Revolve LOGO
void startScreen(void){
	unsigned long teller = 0;
	//start rendring: 130560 lengde
	//start revolve: 79200
	
	//Write picture to FT800
	/*for(int i=0; i<64380; i++){
		wr16(teller, start_picture[i]);
		teller+=2;
	}
	cmd(CMD_DLSTART);
	cmd(CLEAR(1,1,1));
	//cmd(COLOR_RGB(30,30,0));
	cmd(BITMAP_SOURCE(0));
	cmd(BITMAP_LAYOUT(RGB565, 870, 148));  //rendirng: 960, 272,    logo: 960, 165
	cmd(BITMAP_SIZE(NEAREST, BORDER, BORDER, 435, 148)); //rendirng: 480, 272,  logo: 480, 165
	cmd(BEGIN(BITMAPS));
	cmd(VERTEX2II(22,52,0,0)); //52 er ca midt på
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
	delay_ms(3500);*/
	
	cmd(CMD_DLSTART);
	cmd(CLEAR(1,1,1));
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
	delay_ms(10);
	
	//write other pictures to FT800
	teller=0;
	for(int i=0; i<1024; i++){
		wr16(teller, revolvelogo[i]);
		teller+=2;
	}
	for(int i=0; i<5600; i++){
		wr16(teller, exclamation[i]);
		teller+=2;
	}
	
}

//Draw the adjustment menu
void adjustmentMenu(ft_uint8_t position){
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	cmd_text(236,20 , 29, OPT_CENTER, "Adjustment menu");
	if(position==0){
		cmd_fgcolor(0xb9b900);
		cmd(COLOR_RGB(0,0,0));
		cmd_button(170, 60, 140, 25, 26, 0, "Driver");
		cmd(COLOR_RGB(255,255,255));
		cmd_coldstart();
	}else{
		cmd_button(170, 60, 140, 25, 26, 0, "Driver");
	}
	if(position==1){
		cmd_fgcolor(0xb9b900);
		cmd(COLOR_RGB(0,0,0));
		cmd_button(170, 90, 140, 25,26, 0, "Device status");
		cmd(COLOR_RGB(255,255,255));
		cmd_coldstart();
	}else{
		cmd_button(170, 90, 140, 25,26, 0, "Device status");
	}
	if (position==2){
		cmd_fgcolor(0xb9b900);
		cmd(COLOR_RGB(0,0,0));
		cmd_button(170, 120, 140, 25,26, 0, "DAVE");
		cmd(COLOR_RGB(255,255,255));
		cmd_coldstart();
	}else{
		cmd_button(170, 120, 140, 25,26, 0, "DAVE");
	}
	if(position==3){
		cmd_fgcolor(0xb9b900);
		cmd(COLOR_RGB(0,0,0));
		cmd_button(170, 150, 140, 25,26, 0, "ECU");
		cmd(COLOR_RGB(255,255,255));
		cmd_coldstart();
		}else{
		cmd_button(170, 150, 140, 25,26, 0, "ECU");
	}
	if (position==4){
		cmd_fgcolor(0xb9b900);
		cmd(COLOR_RGB(0,0,0));
		cmd_button(170, 180, 140, 25,26, 0, "Max torque");
		cmd(COLOR_RGB(255,255,255));
		cmd_coldstart();
		}else{
		cmd_button(170, 180, 140, 25,26, 0, "Max torque");
	}
	if (position==5){
		cmd_fgcolor(0xb9b900);
		cmd(COLOR_RGB(0,0,0));
		cmd_button(170, 210, 140, 25,26, 0, "Torque encoder calib");
		cmd(COLOR_RGB(255,255,255));
		cmd_coldstart();
	}else{
		cmd_button(170, 210, 140, 25,26, 0, "Torque encoder calib");
	}
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
	//delay_ms(20);
	myDelay(10);
}

void adjustmentSpecial(ft_uint8_t device, ft_uint8_t val){
	ft_uint8_t y_coordinate = 70;
	ft_uint8_t x_coordinate = 95;
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	cmd_text(236, 20, 29, OPT_CENTER, adjustment_devices_special[device]);
	//ECU P, I and D
	if(device<3){
		for(int i=0; i<4; i++){
			if(i==val){
				cmd_fgcolor(0xb9b900);
				cmd_button(210, y_coordinate, 50, 25, 26, OPT_CENTER, ecu_pos[i]);
				cmd_coldstart();
			}else{
				cmd_button(210, y_coordinate, 50, 25, 26, OPT_CENTER, ecu_pos[i]);
			}
			y_coordinate+=30;
		}
	//LC initial torque
	}else if(device==3){
		for (int i=0; i<20; i++){
			if(i==val){
				cmd_fgcolor(0xb9b900);
				cmd_button(x_coordinate, y_coordinate, 50, 25, 26, OPT_CENTER, lc_init_trq[i]);
				cmd_coldstart();
				}else{
				cmd_button(x_coordinate, y_coordinate, 50, 25, 26, OPT_CENTER, lc_init_trq[i]);
			}
			if((i==4)||(i==9)||(i==14)){
				x_coordinate+=80;
				y_coordinate = 70;
			}else{
				y_coordinate+=30;
			}
		}
		
	//LC rise time
	}else{
		for (int i=0; i<19; i++){
			if(i==val){
				cmd_fgcolor(0xb9b900);
				cmd_button(x_coordinate, y_coordinate, 50, 25, 26, OPT_CENTER, lc_rise_time[i]);
				cmd_coldstart();
			}else{
				cmd_button(x_coordinate, y_coordinate, 50, 25, 26, OPT_CENTER, lc_rise_time[i]);
			}
			if((i==4)||(i==9)||(i==14)){
				x_coordinate+=80;
				y_coordinate= 70;	
			}else{
				y_coordinate+=30;
			}
		}
	}

	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
	myDelay(10);
}

void adjustment(ft_uint8_t device_adj, ft_uint8_t val, ft_uint8_t typeofdevice){
	ft_uint8_t y_coordinate = 100;
	ft_uint16_t x_coordinate = 0;
	ft_uint8_t bar_len = 0;
	
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	cmd_text(236, 20, 29, OPT_CENTER, adjustment_devices[device_adj]);
	
	if(typeofdevice==ONOFF){
		for(int i=0; i<2; i++){
			if(i==val){
				cmd_fgcolor(0xb9b900);
				cmd_button(210, y_coordinate, 50, 50, 26, OPT_CENTER, interval_onoff[i]);
				cmd_coldstart();
			}else{
				cmd_button(210, y_coordinate, 50, 50, 26, OPT_CENTER, interval_onoff[i]);
			}
			y_coordinate+=55;
		}
	
	}else if(typeofdevice==POSITIVE3){
		y_coordinate = 80;
		for(int i=0; i<3; i++){
			if(i==val){
				cmd_fgcolor(0xb9b900);
				cmd_button(210, y_coordinate, 50, 25, 26, OPT_CENTER, interval_3_pos[i]);
				cmd_coldstart();
				}else{
				cmd_button(210, y_coordinate, 50, 25, 26, OPT_CENTER, interval_3_pos[i]);
			}
			y_coordinate+=30;
		}
	
	}else if(typeofdevice==POSITIVE5){
		y_coordinate = 50;
		for(int i=0; i<5; i++){
			if(i==val){
				cmd_fgcolor(0xb9b900);
				cmd_button(210, y_coordinate, 50, 25, 26, OPT_CENTER, interval_5_pos[i]);
				cmd_coldstart();
			}else{
				cmd_button(210, y_coordinate, 50, 25, 26, OPT_CENTER, interval_5_pos[i]);
			}
			y_coordinate+=30;
		}
	
	}else if(typeofdevice==EXTRA_MENU){
		y_coordinate = 80;
		for(int i=0; i<5; i++){
			if(i==val){
				cmd(COLOR_RGB(0,0,0));
				cmd_fgcolor(0xb9b900);
				cmd_button(180, y_coordinate, 120, 30, 26, OPT_CENTER, adjustment_devices_special[i]);
				cmd(COLOR_RGB(255,255,255));
				cmd_coldstart();
			}else{
				cmd_button(180, y_coordinate, 120, 30, 26, OPT_CENTER, adjustment_devices_special[i]);
			}
			y_coordinate+=35;
		}
	
	}else if(typeofdevice==CALIB){
		y_coordinate=100;
		for(int i=0; i<2; i++){
			if(i==val){
				cmd(COLOR_RGB(0,0,0));
				cmd_fgcolor(0xb9b900);
				cmd_button(190, y_coordinate, 100, 50, 26, OPT_CENTER, interval_calib[i]);
				cmd(COLOR_RGB(255,255,255));
				cmd_coldstart();
			}else{
				cmd_button(190, y_coordinate, 100, 50, 26, OPT_CENTER, interval_calib[i]);
			}
			y_coordinate+=55;
		}
	
	}else if(typeofdevice == DAVE_INTERVAL){
		y_coordinate = 60;
		for(int i=0; i<6; i++){
			if(i==val){
				cmd(COLOR_RGB(0,0,0));
				cmd_fgcolor(0xb9b900);
				cmd_button(120, y_coordinate, 100, 25, 26, OPT_CENTER, dave_interval[i]);
				cmd(COLOR_RGB(255,255,255));
				cmd_coldstart();
			}else{
				cmd_button(120, y_coordinate, 100, 25, 26, OPT_CENTER, dave_interval[i]);
			}
			y_coordinate+=30;
		}
		y_coordinate = 60;
		for(int i=6; i<12; i++){
			if(i==val){
				cmd(COLOR_RGB(0,0,0));
				cmd_fgcolor(0xb9b900);
				cmd_button(260, y_coordinate, 100, 25, 26, OPT_CENTER, dave_interval[i]);
				cmd(COLOR_RGB(255,255,255));
				cmd_coldstart();
			}else{
				cmd_button(260, y_coordinate, 100, 25, 26, OPT_CENTER, dave_interval[i]);
			}
			y_coordinate+=30;
		}
	
	}else if(typeofdevice == DRIVER_INTERVaL){
		y_coordinate = 70;
		for(int i=0; i<6; i++){
			if(i==val){
				cmd(COLOR_RGB(0,0,0));
				cmd_fgcolor(0xb9b900);
				cmd_button(190, y_coordinate, 100, 25, 26, OPT_CENTER, driver_interval[i]);
				cmd(COLOR_RGB(255,255,255));
				cmd_coldstart();
			}else{
				cmd_button(190, y_coordinate, 100, 25, 26, OPT_CENTER, driver_interval[i]);
			}
			y_coordinate+=30;
		}
	}else if(typeofdevice == NONE){
		y_coordinate = 100;
		x_coordinate = 70;
		bar_len = 100;
		for(int i=0; i<DASH_ALIVE_SIZE; i++){
			if(isAlive_check[i]==0){		//Draw unit as red
				cmd(COLOR_RGB(255,255,255));
				cmd_fgcolor(0xFF0000);
				cmd_button(x_coordinate, y_coordinate, bar_len, 25, 26, OPT_CENTER, devicestatus_interval[i]);
			}else{							//Draw unit as green
				cmd(COLOR_RGB(0,0,0));
				cmd_fgcolor(0x00FF00);
				cmd_button(x_coordinate, y_coordinate, bar_len, 25, 26, OPT_CENTER, devicestatus_interval[i]);
			}
			if((i==3)||(i==7)){
				x_coordinate+=120;
				y_coordinate = 100;
			}else{
				y_coordinate+=30;
			}
		}
		cmd(COLOR_RGB(255,255,255));
		cmd_coldstart();
		
	}else if(typeofdevice==MAX_TRQ){
		y_coordinate = 80;
		for(int i=0; i<4; i++){
			if(i==val){
				cmd(COLOR_RGB(0,0,0));
				cmd_fgcolor(0xb9b900);
				cmd_button(180, y_coordinate, 120, 40, 26, OPT_CENTER, maxtrq_interval[i]);
				cmd(COLOR_RGB(255,255,255));
				cmd_coldstart();
			}else{
				cmd_button(180, y_coordinate, 120, 40, 26, OPT_CENTER, maxtrq_interval[i]);
			}
			y_coordinate+=45;
		}
	}
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
	//delay_ms(20);
	myDelay(10);
}

//Function to get back to the right display after some 
//message have been displayed
void getBackDisplay(void){
	if(horizontal_position==MAINMENU){
		mainMenu();	
	}else if(horizontal_position==ADJUSTMENT_MENU){
		adjustmentMenu(vertical_level_adjustment_menu);
	}else if(horizontal_position==ADJUSTMENT){
		adjustment(vertical_level_adjustment_menu, new_adjustment_value[vertical_level_adjustment_menu], type_of_adjustment[vertical_level_adjustment_menu]);
	}else if(horizontal_position==EXTRA_MENU){
		if(vertical_level_adjustment_menu==MAXTRQ){
			adjustmentSpecial(vertical_level_adjustment, new_adjustment_value_CCU[vertical_level_adjustment]);
		}else if(vertical_level_adjustment_menu==ECU){
			adjustmentSpecial(vertical_level_adjustment, new_adjustment_value_ECU[vertical_level_adjustment]);
		}
	}else if(horizontal_position==MAINMENU_2){
		mainMenu2();
	}else{
		mainMenu3();
	}
}

void drawRPM(void){
	
	cmd_text(100,230, 28, OPT_CENTER, "RPM" );
	cmd_text(100,250, 26, OPT_CENTER, "x1000" );
	cmd_gauge(100, 160, 110, OPT_NOPOINTER|OPT_NOBACK, 10,4,rpm, 5000);
	cmd(COLOR_RGB(255,0,0));
	cmd_gauge(100, 160, 110, OPT_NOBACK|OPT_NOTICKS, 10,4, rpm, 5000);
	cmd_number(40, 150, 29, OPT_CENTER, 1);
	cmd_number(75, 110, 29, OPT_CENTER, 2);
	cmd_number(130, 110, 29, OPT_CENTER, 3);
	cmd_number(160, 150, 29, OPT_CENTER, 4);
	cmd_number(140, 200, 29, OPT_CENTER, 5);
	cmd(COLOR_RGB(255,255,255));
}

void drawTemperature(ft_uint8_t temperature, ft_uint8_t menu ){
	if(menu==MAINMENU){
		cmd_text(240, 180, 26, OPT_CENTER, "Motor temp" );
		if(((motor_temp-9871.7)/54.2)>110){
			//check_temp=true;
			cmd(COLOR_RGB(255,0,0));
			cmd_number(230, 210, 31, OPT_CENTER, (motor_temp-9871.7)/54.2);
			cmd_text(275, 215, 29, OPT_CENTER, "C");
			cmd(COLOR_RGB(255,255,255));
			
		}else{
			cmd_number(230, 210, 31, OPT_CENTER, (motor_temp-9871.7)/54.2);
			cmd_text(275, 215, 29, OPT_CENTER, "C");
		}
	}else{
		//battery temps
		cmd_text(60, 200, 26, OPT_CENTER, "Battery min");
		cmd_number(50, 230, 31, OPT_CENTER, min_battery_temp);
		cmd_text(95, 235, 22, OPT_CENTER, "C");
		
		cmd_text(60, 130, 26, OPT_CENTER, "Battery avg");
		if(battery_temp>55){
			//check_temp=true;
			cmd(COLOR_RGB(255,0,0));
			cmd_number(50, 160, 31, OPT_CENTER, battery_temp);
			cmd_text(95, 165, 22, OPT_CENTER, "C");
			cmd(COLOR_RGB(255,255,255));
		}else{
			cmd_number(50, 160, 31, OPT_CENTER, battery_temp);
			cmd_text(95, 165, 22, OPT_CENTER, "C");
		}
		
		cmd_text(60, 60, 26, OPT_CENTER, "Battery max");
		if(max_battery_temp>55){
			//check_temp=true;
			cmd(COLOR_RGB(255,0,0));
			cmd_number(50, 90, 31, OPT_CENTER, max_battery_temp);
			cmd_text(95, 95, 22, OPT_CENTER, "C");
			cmd(COLOR_RGB(255,255,255));
		}else{
			cmd_number(50, 90, 31, OPT_CENTER, max_battery_temp);
			cmd_text(95, 95, 22, OPT_CENTER, "C");
		}
		
		//Motor temps
		cmd_text(180, 60, 26, OPT_CENTER, "Motor" );
		if(((motor_temp-9871.7)/54.2)>110){
			cmd(COLOR_RGB(255,0,0));
			cmd_number(170, 90, 31, OPT_CENTER, (motor_temp-9871.7)/54.2);
			cmd_text(215, 95, 29, OPT_CENTER, "C");
			cmd(COLOR_RGB(255,255,255));
		}else{
			cmd_number(170, 90, 31, OPT_CENTER, (motor_temp-9871.7)/54.2);
			cmd_text(215, 95, 29, OPT_CENTER, "C");
		}
		
		cmd_text(180, 130, 26, OPT_CENTER, "IGBT" );
		
		if(IGBT_temp>21125){
			//check_temp = true;
			cmd(COLOR_RGB(255,0,0));
		}
		if(IGBT_temp<19062){
			cmd_number(170, 160, 31, OPT_CENTER, (IGBT_temp-16000)/75.0);
		}else if(IGBT_temp<21125){
			cmd_number(170, 160, 31, OPT_CENTER, (IGBT_temp-16000)*(20.0/2063.0)+10);
		}else{
			cmd_number(170, 160, 31, OPT_CENTER, (IGBT_temp-16000)*(4.0/575.0)+25);
			
		}
		cmd_text(215, 165, 29, OPT_CENTER, "C");
		if(IGBT_temp>24000){
			cmd(COLOR_RGB(255,255,255));
		}
		//feil max gearbox temp enn så lenge
		cmd_text(180, 200, 26, OPT_CENTER, "Gearbox" );
		gearbox_temp_cal = -gearbox_temp*((float)50/2739)+189.28;  //Tidligere:((2.633-(gearbox_temp*0.000296))*73)+30;
		if(gearbox_temp_cal>70){
			check_temp = true;
			cmd(COLOR_RGB(255,0,0));
			cmd_number(170, 230, 31, OPT_CENTER, gearbox_temp_cal); 
			cmd_text(215, 235, 29, OPT_CENTER, "C");   
			cmd(COLOR_RGB(255,255,255));      
			
		}else{
			cmd_number(170, 230, 31, OPT_CENTER, gearbox_temp_cal);
			cmd_text(215, 235, 29, OPT_CENTER, "C");
		}
	}
	//check_temp = true;
}

void drawBattery(void){
	cmd_text(240, 110, 26, OPT_CENTER, "Battery level" );
	leftOnBattery = (min_cell_voltage-32)/0.09;
	if(leftOnBattery<12){
		cmd(COLOR_RGB(255,0,0));
		cmd_number(230, 140, 31, OPT_CENTER, leftOnBattery);
		cmd_text(275, 145, 29, OPT_CENTER, "%");
		cmd(COLOR_RGB(255,255,255));
	}else{
		cmd_number(230, 140, 31, OPT_CENTER, leftOnBattery);
		cmd_text(275, 145, 29, OPT_CENTER, "%");
	}
	
	
	
}

void drawPower(void){
	cmd_text(380,230, 28, OPT_CENTER, "Torque" );
	cmd_text(380,250, 26, OPT_CENTER, "x10%" );
	cmd_gauge(380, 160, 110, OPT_NOPOINTER|OPT_NOBACK, 10,4, max(torque,0), 32767);//torque*100/(2^15-1), 200);
	cmd(COLOR_RGB(255,0,0));
	cmd_gauge(380, 160, 110, OPT_NOBACK|OPT_NOTICKS, 10,4, max(torque,0), 32767);//torque*100/(2^15-1), 200);
	cmd_number(320, 150, 29, OPT_CENTER, 2);
	cmd_number(355, 110, 29, OPT_CENTER, 4);
	cmd_number(410, 110, 29, OPT_CENTER, 6);
	cmd_number(440, 150, 29, OPT_CENTER, 8);
	cmd_number(415, 200, 29, OPT_CENTER, 10);
	cmd(COLOR_RGB(255,255,255));
}

void drawDCbusVoltage(void){
	cmd_text(420, 130, 26, OPT_CENTER, "DC bus" );
	if(DC_bus_voltage>600){
		//check_voltage=true;
		cmd(COLOR_RGB(255,0,0));
		cmd_number(410, 160, 31, OPT_CENTER, DC_bus_voltage);
		cmd_text(455, 165, 22, OPT_CENTER, "V");
		cmd(COLOR_RGB(255,255,255));
	}else{
		cmd_number(410, 160, 31, OPT_CENTER, DC_bus_voltage);
		cmd_text(455, 165, 22, OPT_CENTER, "V");
	}
}

void drawTorqueEncoder(void){
	if(!valid_torque_ch0){
		cmd_text(60, 200, 26, OPT_CENTER, "Trq enc 0");
		cmd_number(50, 230, 31, OPT_CENTER, torque_encoder_ch0);
		cmd_text(95, 235, 22, OPT_CENTER, "E");
	}else{
		cmd_text(60, 200, 26, OPT_CENTER, "Trq enc 0");
		//cmd_number(50, 230, 31, OPT_CENTER, (uint16_t) torque_encoder_ch0);
		cmd_number(50, 230, 31, OPT_CENTER, max(torque_encoder_ch0/10 ,0));
		cmd_text(95, 235, 22, OPT_CENTER, "%");
	}
	if(!valid_torque_ch1){
		cmd_text(60, 130, 26, OPT_CENTER, "Trq enc 1");
		cmd_number(50, 230, 31, OPT_CENTER, torque_encoder_ch1);
		cmd_text(95, 235, 22, OPT_CENTER, "E");
	}else{
		cmd_text(60, 130, 26, OPT_CENTER, "Trq enc 1");
		//cmd_number(50, 160, 31, OPT_CENTER, (uint16_t) torque_encoder_ch1);
		cmd_number(50, 160, 31, OPT_CENTER, max(torque_encoder_ch1/10 ,0));
		cmd_text(95, 165, 22, OPT_CENTER, "%");
	}
}

void drawLaunchTractiveStatus(void){
	cmd_text(400, 20, 26, OPT_CENTER, "Launch control");
	cmd_text(80, 20, 26, OPT_CENTER, "Tractive system");
	cmd_text(240, 20, 26, OPT_CENTER, "Ready to drive");
	if((launch_traction==1)||(launch_traction==2)){
		cmd(COLOR_RGB(0,255,0));
		cmd_text(400, 40, 26, OPT_CENTER, "Activated" );
		
	}else{
		cmd(COLOR_RGB(255,0,0));
		cmd_text(400, 40, 26, OPT_CENTER, "OFF" );
	}
	if(tractive_status==1){
		cmd(COLOR_RGB(0,255,0));
		cmd_text(80, 40, 26, OPT_CENTER, "Activated" );
	}else{
		cmd(COLOR_RGB(255,0,0));
		cmd_text(80, 40, 26, OPT_CENTER, "OFF" );
	}
	if(driveEnabled){
		cmd(COLOR_RGB(0,255,0));
		cmd_text(240, 40, 26, OPT_CENTER, "Activated" );
	}else{
		cmd(COLOR_RGB(255,0,0));
		cmd_text(240, 40, 26, OPT_CENTER, "OFF" );
	}
	cmd(COLOR_RGB(255,255,255));
}

void drawRevolve(ft_uint8_t x, ft_uint8_t y){
	cmd(BITMAP_SOURCE(0));//2*131131));
	cmd(BITMAP_LAYOUT(RGB565, 64, 32));
	cmd(BITMAP_SIZE(NEAREST, BORDER, BORDER, 32, 32));
	cmd(BEGIN(BITMAPS));
	cmd(VERTEX2II(380-16,160-16,0,0));
	cmd(VERTEX2II(100-16, 160-16, 0, 0));
}

void writeWarning(char *war){
	cmd(CMD_DLSTART);
	cmd(CLEAR(1,1,1));
	//cmd(COLOR_RGB(30,30,0));
	cmd(BITMAP_SOURCE(2*1024));
	cmd(BITMAP_LAYOUT(RGB565, 160, 70));  //rendirng: 960, 272,    logo: 960, 165
	cmd(BITMAP_SIZE(NEAREST, BORDER, BORDER, 80, 70)); //rendirng: 480, 272,  logo: 480, 165
	cmd(BEGIN(BITMAPS));
	cmd(VERTEX2II(240-40,50,0,0)); //52 er ca midt på
	cmd_text(240, 150, 28, OPT_CENTER, war);
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
	
	myDelay(1000);
}

void drawBreakPres(void){
	cmd_text(180, 200, 26, OPT_CENTER, "Brk prs R");
	cmd_number(165, 230, 31, OPT_CENTER, (brk_pres_rear-3960)/120);	//tidligere: x/32.766
	cmd_text(225, 225, 22, OPT_CENTER, "%");
	
	cmd_text(180, 130, 26, OPT_CENTER, "Brk prs F");
	cmd_number(165, 160, 31, OPT_CENTER, (brk_pres_front-3960)/120);	//tidligere: x/32.766
	cmd_text(225, 165, 22, OPT_CENTER, "%");
	
}

void drawSteerAng(void){
	cmd_text(300, 200, 26, OPT_CENTER, "Steer ang");
	cmd_number(290, 230, 31, OPT_CENTER, (steer_ang-1800)*0.1);
	cmd_text(335, 235, 22, OPT_CENTER, "*");
}

void drawVoltages(void){
	cmd_text(300, 130, 26, OPT_CENTER, "Btry pack" );
	if((pack_voltage>600)||(pack_voltage<475)){
		//check_voltage=true;
		cmd(COLOR_RGB(255,0,0));
		cmd_number(290, 160, 31, OPT_CENTER, pack_voltage);
		cmd_text(335, 165, 22, OPT_CENTER, "V");
		cmd(COLOR_RGB(255,255,255));
	}else{
		cmd_number(290, 160, 31, OPT_CENTER, pack_voltage);
		cmd_text(335, 165, 22, OPT_CENTER, "V");
	}
	
	cmd_text(290, 200, 26, OPT_CENTER, "Min cell" );
	cmd_number(330, 200, 26, OPT_CENTER, min_cell_id);
	if(min_cell_voltage<32){
		//check_voltage=true;
		cmd(COLOR_RGB(255,0,0));
		cmd_number(280, 230, 31, OPT_CENTER, min_cell_voltage/10);
		cmd_text(295, 230, 31, OPT_CENTER, "." );
		cmd_number(310, 230, 31, OPT_CENTER, min_cell_voltage -(min_cell_voltage/10)*10);
		cmd_text(330, 235, 22, OPT_CENTER, "V");
		cmd(COLOR_RGB(255,255,255));
	}else{
		cmd_number(280, 230, 31, OPT_CENTER, min_cell_voltage/10);
		cmd_text(295, 230, 31, OPT_CENTER, "." );
		cmd_number(310, 230, 31, OPT_CENTER, min_cell_voltage -(min_cell_voltage/10)*10);
		cmd_text(330, 235, 22, OPT_CENTER, "V");
		//cmd_text(435, 215, 22, OPT_CENTER, "V");
	}
	
	cmd_text(290, 60, 26, OPT_CENTER, "Max cell" );
	cmd_number(335, 60, 26, OPT_CENTER, max_cell_id);
	if(max_cell_voltage>42){
		//check_voltage = true;
		cmd(COLOR_RGB(255,0,0));
		cmd_number(280, 95, 31, OPT_CENTER, max_cell_voltage/10);
		cmd_text(295, 95, 31, OPT_CENTER, "." );
		cmd_number(310, 95, 31, OPT_CENTER, max_cell_voltage -(max_cell_voltage/10)*10);
		cmd_text(330, 100, 22, OPT_CENTER, "V");
		cmd(COLOR_RGB(255,255,255));
	}else{
		cmd_number(280, 95, 31, OPT_CENTER, max_cell_voltage/10);
		cmd_text(295, 95, 31, OPT_CENTER, "." );
		cmd_number(310, 95, 31, OPT_CENTER, max_cell_voltage -(max_cell_voltage/10)*10);
		cmd_text(330, 100, 22, OPT_CENTER, "V");
	}
	
	cmd_text(420, 200, 26, OPT_CENTER, "GLV" );
	glv_voltage_calib = (float)(glv_voltage/800.0)+0.98;
	if(glv_voltage_calib<11.5){
		//check_voltage = true;
		cmd(COLOR_RGB(255,0,0));
		cmd_number(390, 230, 31, OPT_CENTER, glv_voltage_calib);
		cmd_text(420, 230, 31, OPT_CENTER, "." );
		cmd_number(435, 230, 31, OPT_CENTER, (((glv_voltage*10)/375)%10));
		cmd_text(455, 235, 22, OPT_CENTER, "V");
		cmd(COLOR_RGB(255,255,255));
	}else{
		cmd_number(390, 230, 31, OPT_CENTER, glv_voltage_calib);
		cmd_text(420, 230, 31, OPT_CENTER, "." );
		cmd_number(435, 230, 31, OPT_CENTER, (((glv_voltage*10)/375)%10));
		cmd_text(455, 235, 22, OPT_CENTER, "V");
	}
	//check_voltage = true;
}

void drawECUerr(void){
	cmd_text(120, 60, 28, OPT_CENTER, "ECU error");
	if(error_ecu==0){
		cmd_text(125, 90, 26, OPT_CENTER, "No errors");
	}else{
		cmd(COLOR_RGB(255,0,0));
		for(int i=0; i<9; i++){
			if(error_ecu == compareVector[i]){
				cmd_text(125, 90, 26, OPT_CENTER, ecu_error_names[i]);
				break;
			}
		}
	}
	cmd(COLOR_RGB(255,255,255));
}

void drawBMSerr(void){
	cmd_text(360, 60, 28, OPT_CENTER, "BMS fault code");
	cmd_text(360, 120, 28, OPT_CENTER, "BMS warning");
	if(bms_fault_code>0){
		cmd(COLOR_RGB(255,0,0));
		cmd_text(365, 90, 26, OPT_CENTER, bms_fault_names[bms_fault_code-1]);
		cmd(COLOR_RGB(255,255,255));
	}else{
		cmd_text(365, 90, 26, OPT_CENTER, "No faults");
	}
	if(bms_warning>0){
		cmd(COLOR_RGB(255,0,0));
		for(int i=0; i<8; i++){
			if(compareVector1[i]==bms_warning){
				cmd_text(365, 150, 26, OPT_CENTER, bms_warning_names[i]);
				break;
			}
		}
		cmd(COLOR_RGB(255,255,255));
	}else{
		cmd_text(365, 150, 26, OPT_CENTER, "No warnings");
	}
}