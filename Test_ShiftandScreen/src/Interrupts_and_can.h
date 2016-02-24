/*
 * Interrupt.h
 *
 * Created: 20.03.2014 15:21:55
 *  Author: Øystein
 */ 


#ifndef INTERRUPT_H_
#define INTERRUPT_H_

void Interrupt_CAN_Init(void);
__attribute__((__interrupt__)) static void gpio_interrupt_handler_port3_line1(void);
__attribute__((__interrupt__)) static void gpio_interrupt_handler_port3_line0(void);
__attribute__((__interrupt__)) static void gpio_interrupt_handler_port2_line0(void);
__attribute__((__interrupt__)) static void tc_irq(void);

void can_out_callback_channel0(U8 handle, U8 event);
void can_out_callback_channel1(U8 handle, U8 event);
uint8_t myDelay(uint16_t timeinms);

static volatile uint8_t power_on_reset = 0;

can_msg_t msg_tx_ch0;
can_mob_t appli_tx_ch0;
can_msg_t msg_tx_ch1;
can_mob_t appli_tx_ch1;
can_msg_t msg_rx_bspd_data;
can_mob_t appli_rx_bspd_data;
can_msg_t msg_rx_ecu_data;
can_mob_t appli_rx_ecu_data;
can_msg_t msg_rx_ecu_data_ch1;
can_mob_t appli_rx_ecu_data_ch1;
can_msg_t msg_rx_ecu_pri;
can_mob_t appli_rx_ecu_pri;
can_msg_t msg_rx_torqueenc_data_ch0;
can_mob_t appli_rx_torqueenc_data_ch0;
can_msg_t msg_rx_torqueenc_data_ch1;
can_mob_t appli_rx_torqueenc_data_ch1;
can_msg_t msg_rx_bms_data_ch0;
can_mob_t appli_rx_bms_data_ch0;
can_msg_t msg_rx_alive_ch0;
can_mob_t appli_rx_alive_ch0;
can_msg_t msg_rx_alive_ch1;
can_mob_t appli_rx_alive_ch1;
can_msg_t msg_rx_adc_ch1;
can_mob_t appli_rx_adc_ch1;
can_msg_t msg_rx_strang_ch1;
can_mob_t appli_rx_strang_ch1;
can_msg_t msg_rx_geartmp_ch1;
can_mob_t appli_rx_geartmp_ch1;
can_msg_t msg_rx_glv_ch1;
can_mob_t appli_rx_glv_ch1;
can_msg_t msg_rx_telemetry_ch0;
can_mob_t appli_rx_telemetry_ch0;

#endif /* INTERRUPT_H_ */