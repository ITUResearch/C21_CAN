/*
 * main.c
 *
 * Created: 12/20/2016 1:38:29 PM
 *  Author: xu
 */ 

/**
 * \file
 *
 * \brief SAM CAN basic Quick Start
 *
 * Copyright (C) 2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel micro-controller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
//#include "battery.h"
#include <asf.h>
#include <string.h>
#include <conf_can.h>
#include "sys_event_interrupt_hook.h"


//! [module_inst]
static struct usart_module cdc_instance;
static struct can_module can_instance;



//! [module_inst]


//! function prototypes
void configure_rtc_calendar(void);
void rtc_match_callback(void);
void configure_rtc_callbacks(void);
void configure_tsens(void);


static void can_send_standard_message(uint32_t id_value, uint8_t *data, uint32_t data_length);
static void can_fd_send_extended_message(uint32_t id_value, uint8_t *data);

//! [module_var]
//! [can_filter_setting]
#define CAN_RX_STANDARD_FILTER_INDEX_0    0
#define CAN_RX_STANDARD_FILTER_INDEX_1    1
#define CAN_RX_STANDARD_FILTER_ID_0     0x45A
#define CAN_RX_STANDARD_FILTER_ID_0_BUFFER_INDEX     2
#define CAN_RX_STANDARD_FILTER_ID_1     0x468
#define CAN_RX_EXTENDED_FILTER_INDEX_0    0
#define CAN_RX_EXTENDED_FILTER_INDEX_1    1
#define CAN_RX_EXTENDED_FILTER_ID_0     0x100000A5
#define CAN_RX_EXTENDED_FILTER_ID_0_BUFFER_INDEX     1
#define CAN_RX_EXTENDED_FILTER_ID_1     0x10000096
//! [can_filter_setting]

//! [can_transfer_message_setting]
#define CAN_TX_BUFFER_INDEX    0
static uint8_t tx_message_0[CONF_CAN_ELEMENT_DATA_SIZE];
static uint8_t tx_message_1[CONF_CAN_ELEMENT_DATA_SIZE];
//! [can_transfer_message_setting]

//! [can_receive_message_setting]
static volatile uint32_t standard_receive_index = 0;
static volatile uint32_t extended_receive_index = 0;
static struct can_rx_element_fifo_0 rx_element_fifo_0;
static struct can_rx_element_fifo_1 rx_element_fifo_1;
static struct can_rx_element_buffer rx_element_buffer;
//! [can_receive_message_setting]

//![temperature result]
//int32_t temp_result;

static int32_t delta_charge; //total electrical charge that been charged into battery plus discharged from battery in unit of mAsec.



enum commandType {
	commandType_null,
	commandType_set,
	commandType_request,
	commandType_response,
	commandType_report
};

enum commandAction{
	commandAction_null,
	commandAction_time,
	commandAction_data
};




#define CMD_TYPE_REQUEST    0x01
#define CMD_TYPE_SET        0x02

//******************************************** ADC & Battery *******************************************
#define  ADC_REFERENCE_INT1V_VALUE    0X01
#define  ADC_8BIT_FULL_SCALE_VALUE    0XFF

//TX_DATA_TYPE
#define TX_TYPE_BATTERY_DATA		  0x10
#define TX_TYPE_TIME_DATA		      0x11

#define OPAMP_GAIN					  13




/*
|START_FLAG|TX_DATA_TYPE|DATA_LENGTH|CHARGE_CURRENT|DISCHARGE_CURRENT|TEMPERATURE_DATA|soc percentage|BATTERY_STATUS|CHARGER_STATUS|YEAR   |MONTH  |DAY    |HOUR   |MIN   |SEC   |END_FLAG|
|1 BYTE    |1 BYTE      |1 BYTE     |2 BYTES       |2 BYTES          |2 BYTES         |1 BYTE        |1 BYTE        |1 BYTE        |2 BYTES|1 BYTE |1 BYTE |1 BYTE |1 BYTE|1 BYTE|1 BYTE  |
*/
#define BATTERY_DATA_LENGTH   16
#define TIME_DATA_LENGTH       7

uint8_t commandMsg[20] = {0};

uint8_t battery_data[BATTERY_DATA_LENGTH] = {0};
uint8_t time_data[TIME_DATA_LENGTH]	= {0};

uint16_t temprerature_value;

//for check if battery charging status changes
uint8_t battery_status_new = 0;
uint8_t battery_status_old = 0;


//**************SYSTEM COMMAND RELATED VARIABLE****************
/*set this variable to 1 when executing command,
set this variable back to 0 when command is done executing.*/
uint8_t canMsgInFlag = 0;
uint8_t system_busy_flag = 0;
uint8_t forceDataReportFlag = 0;
uint8_t dateReportFlag = 0;

uint8_t commandReady = 0;

uint32_t time_index = 0;

static volatile bool main_b_cdc_enable = false;

volatile bool adc_read_done = false;

//charge current samples on the ain8>PB00
uint16_t charge_current_adc_result;
//discharge current samples on the ain9>PB01
uint16_t charge_voltage_adc_result;

uint16_t discharge_current_adc_result;

uint16_t adc_voltage, adc_current;

//temperature result
uint16_t temprerature_adc_result;

uint16_t avg_charge_current_reading = 0;
uint16_t avg_discharge_current_reading = 0;

float adc_charge_signal_voltage;
float adc_discharge_signal_voltage;

/* To store raw_result of ADC output */
uint16_t raw_result;

//1 for changer presents,0 for charger not presents
uint8_t charger_status = 0;

//1 for charging, 2 for discharging, 0 for no current in/out.
uint8_t battery_status = 0;

uint8_t commandType = 0; //default = 0
uint8_t commandIndex = 0;//default = 0
uint8_t commandDataLength = 0;
uint8_t commandData[20] = {0};

uint64_t total_charge_current = 0;
uint64_t total_discharge_current = 0;

uint32_t charge_sample_num = 0;
uint32_t discharge_sample_num = 0;

uint32_t battery_capcity = 12240000; //listed total battery capacity in unit of mAsec
uint32_t calibrated_battery_capacity = 12240000;

int8_t charge_remain_percentage = 0;    // ?% of total capacity x 100;

uint8_t charge_from_empty_flag = 0;
uint8_t	discharge_from_full_flag = 0;

//static volatile uint32_t event_count = 0;

//! [adc_module_inst]
struct adc_module adc_instance;

//! [dac_module_inst]
struct dac_module dac_instance;

//! [rtc_module_instance]
struct rtc_module rtc_instance;

//! [alarm_struct]
//struct rtc_calendar_alarm_time alarm;

void ADC_event(struct events_resource *resource);

//! [GPIO_struct]
struct port_config config_port_pin;

void configure_port_pins(void);

void configure_adc(void);
void configure_adc_callbacks(void);
void adc_complete_callback(struct adc_module *const module);
void adc_sampling(void);

//digital read from GPIO PB06 to see if charger is connected
void charger_detection(void);

////! [functions for setup rtc]
//void rtc_match_callback(void);
//void configure_rtc_callbacks(void);
//void configure_rtc_calendar(void);
//void rtc_event_init(void);


void processCommandMsg(void);
void execute_system_command(void);

void adc_get_temperature(void);
uint16_t adc_start_read_temp(void);

uint8_t battery_status_update(void);

//smart battery issues commands
void send_battery_data(void);
void send_board_time_data(void);


//local host issues commands
void setParameter(void);
void setTime(void);

void battery_charge_calculation(uint8_t time);

#define ADC_TEMP_SAMPLE_LENGTH 4

//GPIO setup
void configure_port_pins(void)
{
	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_INPUT;
	config_port_pin.input_pull = PORT_PIN_PULL_NONE;
	
	
	//set EXT1_PIN_5(PIN_PA20) as input for charger detect, 1 for charger present/0 for charge not present
	
	
	//set EXT1_PIN_6(PIN_PA21) as LED output, on for charger present/off for charger not present
	//set EXT1_PIN_13(PIN_PA13) as LED output, on for charger present/off for charger not present
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(EXT1_PIN_5, &config_port_pin);
	port_pin_set_config(EXT1_PIN_6, &config_port_pin);
	//port_pin_set_config(EXT1_PIN_13, &config_port_pin);
}

//! [setup ADC]
void configure_adc(void)
{
	//! [setup_config]
	struct adc_config config_adc;
	
	//! [setup_config_defaults]
	adc_get_config_defaults(&config_adc);

	config_adc.clock_source = GCLK_GENERATOR_0;
	config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV16;
	config_adc.reference =  ADC_REFERENCE_AREFA;
	config_adc.positive_input = ADC_POSITIVE_INPUT_PIN8;
	config_adc.negative_input = ADC_NEGATIVE_INPUT_GND;
	config_adc.sample_length = ADC_TEMP_SAMPLE_LENGTH;

	//! [setup_set_config]
	adc_init(&adc_instance, ADC1, &config_adc);

	//ADC0->AVGCTRL.reg = ADC_AVGCTRL_ADJRES(2) | ADC_AVGCTRL_SAMPLENUM_4;
	//! [setup_enable]
	adc_enable(&adc_instance);
}

void configure_dac(void)
{
	struct dac_config config_dac;
	dac_get_config_defaults(&config_dac);
	config_dac.reference = DAC_REFERENCE_AVCC;
	dac_init(&dac_instance, DAC, &config_dac);
}
void configure_dac_channel(void)
{
	struct dac_chan_config config_dac_chan;
	dac_chan_get_config_defaults(&config_dac_chan);
	dac_chan_set_config(&dac_instance, DAC_CHANNEL_0, &config_dac_chan);
	dac_chan_enable(&dac_instance, DAC_CHANNEL_0);
	dac_enable(&dac_instance);
}



uint8_t adc_detect(uint16_t value) {
	uint16_t vol = 0;
	adc_set_positive_input(&adc_instance,ADC_POSITIVE_INPUT_PIN11);
	adc_start_conversion(&adc_instance);
	do {
		// Wait for conversion to be done and read out result
	} while (adc_read(&adc_instance, &vol) == STATUS_BUSY);
	vol = vol * 5000 / 4096;
	//printf("adc_detect %d\n", vol);
	return (vol > value)? 1 : 0;
};




void adc_sampling(void)
{
	//if (charger_status == 1) {
		//PB08-ADC_POSITIVE_INPUT_PIN4
		adc_set_positive_input(&adc_instance,ADC_POSITIVE_INPUT_PIN4);
		adc_start_conversion(&adc_instance);
		do {
		// Wait for conversion to be done and read out result
		} while (adc_read(&adc_instance, &charge_voltage_adc_result) == STATUS_BUSY);	
		
		
		
		//PB09-ADC_POSITIVE_INPUT_PIN5
		adc_set_positive_input(&adc_instance,ADC_POSITIVE_INPUT_PIN5);
		adc_start_conversion(&adc_instance);
		do {
			// Wait for conversion to be done and read out result 
		} while (adc_read(&adc_instance, &charge_current_adc_result) == STATUS_BUSY);
		//printf("%d current %d \r\n", time_index, adc_current * 5000 / 4096);
		
		charge_sample_num++;
		total_charge_current += (charge_current_adc_result * 5000) / 4096 / 2;
	//} else {
		//A2
		adc_set_positive_input(&adc_instance,ADC_POSITIVE_INPUT_PIN10);
		adc_start_conversion(&adc_instance);
		do {
		// Wait for conversion to be done and read out result
		} while (adc_read(&adc_instance, &discharge_current_adc_result) == STATUS_BUSY);
		discharge_sample_num ++;	
		total_discharge_current += (discharge_current_adc_result * 5000) / 4096 / 2;
	//}
}

uint8_t battery_status_update(void)
{
	if (charger_status == 1)
	{
		//printf("\n\rcharge_current_adc_result %d \r\n",  charge_current_adc_result);
		if (charge_current_adc_result > 0x00ff)
		{
			battery_status = 1; //charging, battery is not full
			//printf("\n\r here*********** \n\r");
			//the followed discharge is not from 100% remain.
			discharge_from_full_flag = 0;
		}
		else if (charge_current_adc_result < 0x20)
		{
			battery_status = 0;//no current in/out for battery, battery is full
			charge_current_adc_result = 0;
			//the follow discharge will be from 100% remain, set the discharge_from_full_flag = 1
			discharge_from_full_flag = 1;
		}
	}
	//need to add in self-discharge control logic and self-discharge to empty function
	else if (charge_voltage_adc_result > 0xff)
	{
		battery_status = 2;    //discharging
		charge_from_empty_flag = 0;
	}
	else //battery is empty
	{
		battery_status = 0;
		charge_voltage_adc_result = 0;
		charge_from_empty_flag = 1;	
	}
	return battery_status;
}

//SYSTEM EVENT for ADC sampling and state detection
void ADC_event(struct events_resource *resource)
{

	if(events_is_interrupt_set(resource, EVENTS_INTERRUPT_DETECT)) {
		//port_pin_toggle_output_level(LED_0_PIN);
		charger_detection();
		adc_sampling();
		battery_status_new = battery_status_update();
		if (battery_status_new != battery_status_old) //if battery charging status changes, send data to pc to update
		{	
			dateReportFlag = 1;
			battery_status_old = battery_status_new;
		}
		events_ack_interrupt(resource, EVENTS_INTERRUPT_DETECT);
	}
}



//charger detect function
void charger_detection(void)
{	
	
	charger_status = adc_detect(4200);
	adc_sampling();
	//printf("charger_status %d %d\r\n", charger_status, !charger_status);
	
	if (charger_status == 1) {
		charge_remain_percentage = delta_charge * 100 / calibrated_battery_capacity;
		//printf("\n\r Delta_charge > 0. It is: %d | per : %d \r\n", delta_charge, charge_remain_percentage);
		uint32_t voltage = (charge_voltage_adc_result <= charge_current_adc_result) ? 0 : (charge_voltage_adc_result - charge_current_adc_result) * 5000 / 4096;
		if (charge_remain_percentage > 95 || voltage >= 4200)
		{
			printf("\n\r Cutting off the charging circuit \r\n");
			port_pin_set_output_level(EXT1_PIN_6, 0);
		} else {
			port_pin_set_output_level(EXT1_PIN_6, 1);
		}
	} else {
		port_pin_set_output_level(EXT1_PIN_6, 0);
	}
	port_pin_set_output_level(EXT1_PIN_5,!charger_status);
	
	
}

void send_battery_data(void)
{	
	struct rtc_calendar_time current_time;
	rtc_calendar_get_time(&rtc_instance, &current_time);
	battery_data[0] = (uint8_t)(0x01);
	battery_data[1] = (uint8_t)((avg_charge_current_reading >> 4) & 0xff );
 	battery_data[2] = (uint8_t)((avg_discharge_current_reading >> 4) & 0xff);
	battery_data[3] = (uint8_t)((temprerature_value >> 8) & 0xff);
	battery_data[4] = (uint8_t)(charge_remain_percentage);
	battery_data[5] = (uint8_t)((battery_status << 1) | (charger_status & 1));
	//printf("###########battery_status : %d\r\n", battery_status);
	//printf("charge_remain_percentage : %d\r\n", charge_remain_percentage);
	//printf("avg_charge_current_reading : %d\r\n", (avg_charge_current_reading >> 4) & 0xff);
	//printf("avg_discharge_current_reading : %d\r\n", avg_discharge_current_reading);
	//battery_data[6] = charge_remain_percentage;
	//battery_data[7] = battery_status;
	//battery_data[8] = charger_status;
	//battery_data[9] = (uint8_t)((current_time.year >> 8) & 0xff);
	//battery_data[10] = (uint8_t)(current_time.year & 0xff);
	//battery_data[11] = current_time.month;
	//battery_data[12] = current_time.day;
	//battery_data[13] = current_time.hour;
	//battery_data[14] = current_time.minute;
	//battery_data[15] = current_time.second;
	
	avg_charge_current_reading = 0;
	avg_discharge_current_reading = 0;
	
	uint8_t tx_data[20];
	tx_data[0] = commandType_report;
	tx_data[1] = commandAction_data;
	for (uint8_t i = 0; i<6; i++)
	{
		tx_data[i+2] = battery_data[i];
	}
	can_send_standard_message(CAN_RX_STANDARD_FILTER_ID_1,tx_data,CONF_CAN_ELEMENT_DATA_SIZE);
	//can_send_extended_message(CAN_RX_EXTENDED_FILTER_ID_0, tx_data, CONF_CAN_ELEMENT_DATA_SIZE);	
	//uint8_t tx_data[20];
	//tx_data[0] = START_FLAG;
	//tx_data[1] = TX_TYPE_BATTERY_DATA;
	//tx_data[2] = 16;
	//for (uint8_t i = 0; i<sizeof(battery_data); i++)
	//{
		//tx_data[i+3] = battery_data[i];
	//}
	//tx_data[19] = END_FLAG;
	//udi_cdc_write_buf(tx_data,20);
	
}

void send_board_time_data(void)
{
	struct rtc_calendar_time current_time;
	rtc_calendar_get_time(&rtc_instance, &current_time);
	time_data[0] = (uint8_t)((current_time.year >> 8) & 0xff);
	time_data[1] = (uint8_t)(current_time.year & 0xff);
	time_data[2] = current_time.month;
	time_data[3] = current_time.day;
	time_data[4] = current_time.hour;
	time_data[5] = current_time.minute;
	time_data[6] = current_time.second;
	
	
	uint8_t tx_data[20];
	tx_data[0] = TX_TYPE_TIME_DATA;
	tx_data[1] = TIME_DATA_LENGTH;
	for (uint8_t i = 0; i<sizeof(battery_data); i++)
	{
		tx_data[i+2] = time_data[i];
	}
	//can_fd_send_extended_message(CAN_RX_EXTENDED_FILTER_ID_0,tx_data);
	//can_send_extended_message(CAN_RX_EXTENDED_FILTER_ID_0, tx_data, sizeof(tx_data));
	can_send_standard_message(CAN_RX_STANDARD_FILTER_ID_1,tx_data,CONF_CAN_ELEMENT_DATA_SIZE);
	//tx_data[0] = START_FLAG;
	//tx_data[1] = TX_TYPE_TIME_DATA;
	//tx_data[2] = 7;
	//for (uint8_t i=0;i<sizeof(time_data);i++)
	//{
		//tx_data[i+3] = time_data[i];
	//}
	//tx_data[10] = END_FLAG;
	//udi_cdc_write_buf(tx_data,11);
}

void setParameter(void)
{
	//to be done..
}

void setTime(void)
{
	struct rtc_calendar_time time;
	rtc_calendar_get_time_defaults(&time);
	time.year   = (uint16_t)((commandData[0] << 8) + commandData[1]);
	time.month  = commandData[2];
	time.day    = commandData[3];
	time.hour   = commandData[4];
	time.minute = commandData[5];
	time.second = commandData[6];
	//printf( "year %d month %d day %d \r\n", time.year, time.month, time.day);
	rtc_calendar_set_time(&rtc_instance, &time);
}

typedef void (*funcPtQuery)(void);
funcPtQuery requestAction[] = {&send_battery_data , &send_board_time_data};
const int requestActionNum = sizeof(requestAction)/sizeof(requestAction[0]);

typedef void (*funcPtUpdate)(void);
funcPtUpdate setAction[] = {&setParameter , &setTime};
const int setActionNum = sizeof(setAction)/sizeof(setAction[0]);


/*************************************************************
*	 commandMsg format:
*	|commandType|commandIndex  |commandDataLength|commandData
*	|1st Byte   |2nd Byte      |3rd Byte         | ..........
*   queryType command: commandDataLength and commandData are 0.
***************************************************************/
void processCommandMsg(void)
{	
	commandType = commandMsg[0]; //get commandType
	commandIndex = commandMsg[1];//get commandIndex
	commandDataLength = commandMsg[2];//get commandDataLength
		
	if (commandDataLength!= 0 )//if commandDataLength is not 0, get commandData.
	{
		for (int i=0;i<commandDataLength;i++)
		{
			commandData[i] = commandMsg[i+3];
		}
	}
	//reset commandMsg[] back to 0;
	for (uint8_t i = 0; i<sizeof(commandMsg); i++)
	{
		commandMsg[i] = 0;
	}
	canMsgInFlag = 0; //reset MsgIn flag
	//usbMsgInFlag = 0;//reset MsgIn flag
	commandReady = 1;//set commandReady to be executed
}




void execute_system_command()
{	
	system_busy_flag = 1;
	commandReady = 0;
	//printf("commandType == %d", commandType);
	if(commandType == CMD_TYPE_REQUEST)
	{	
		requestAction[commandIndex]();		
	}else if (commandType == CMD_TYPE_SET)
	{
		setAction[commandIndex]();
	}
	
	commandType = 0; //reset commandType to 0
	commandIndex = 0;//reset commandIndex to 0
	commandDataLength = 0;//reset commandDataLength to 0
	system_busy_flag = 0;
}

void battery_charge_calculation(uint8_t time)
{
	//printf("battery_charge_calculation\r\n");
	static uint16_t avg_charge_current_reading_old;
	static uint16_t avg_discharge_current_reading_old;
	
	int16_t diff = 0;
	
	if (charger_status == 1 && charge_sample_num >0)
	{
		//printf("total_charge_current: %d\r\n", total_charge_current);
		avg_charge_current_reading = (uint16_t)(total_charge_current / charge_sample_num);
		
		//printf("charge_sample_num: %d\r\n", charge_sample_num);
		//printf("avg_charge_current_reading: %d\r\n", avg_charge_current_reading);
		total_charge_current = 0;
		charge_sample_num = 0;
		diff = (int16_t)(avg_charge_current_reading - avg_charge_current_reading_old);
		if (diff > 20 || diff < -20)//20 ADC reading unit = 0.005v
		{
			dateReportFlag = 1;
			avg_charge_current_reading_old = avg_charge_current_reading;
		}
		
		//Calibration hand by hand to calculate the linear relation ship between adc_value and current
		delta_charge += avg_charge_current_reading * time;
		//printf("delta_charge: %d\r\n", delta_charge);
	} else if (discharge_sample_num > 0)
	{	
		avg_discharge_current_reading = (uint16_t)(total_discharge_current / discharge_sample_num);
		total_discharge_current = 0;
		discharge_sample_num = 0;
		diff = (int16_t)(avg_discharge_current_reading - avg_discharge_current_reading_old);
		if (diff > 20 || diff < -20)//20 ADC reading unit = 0.005v
		{
			dateReportFlag = 1;
			avg_discharge_current_reading_old = avg_discharge_current_reading;
		}
		delta_charge -= avg_discharge_current_reading * time;
		
	}
	
	delta_charge = max(0, delta_charge);
	//if ((((uint32_t)(delta_charge) > calibrated_battery_capacity) && (charge_from_empty_flag == 1) ) ||
		//(((uint32_t)(0 - delta_charge) > calibrated_battery_capacity) && (discharge_from_full_flag == 1)))
	//{
		//calibrated_battery_capacity = (uint32_t)(abs(delta_charge));
	//}
	
	if (delta_charge >= 0)
	{
		charge_remain_percentage = delta_charge * 100 / calibrated_battery_capacity;
		printf("#Delta_charge >= 0. It is: %d | per : %d\n", delta_charge, charge_remain_percentage);
		uint32_t voltage = (charge_voltage_adc_result <= charge_current_adc_result) ? 0 : (charge_voltage_adc_result - charge_current_adc_result) * 5000 / 4096;
		if (charge_remain_percentage > 95 || voltage >= 4200)
		{
			printf("#Cutting off the charging circuit\n");
			port_pin_set_output_level(EXT1_PIN_6, 0);
		} else {
			port_pin_set_output_level(EXT1_PIN_6, 1);
		}
	}
	else
	{
		
		charge_remain_percentage = (calibrated_battery_capacity + delta_charge) * 100 / calibrated_battery_capacity;
		printf("#Delta_charge < 0. It is: %d | per : %d\n", delta_charge, charge_remain_percentage);
		if (charge_remain_percentage < 1)
		{
			charge_remain_percentage = 1;
		}
	}
}

//******************************************** ADC & Battery *******************************************



//! [module_var]

//! [setup]
//******************************************** UART CONFIG *******************************************
//! [cdc_setup]
static void configure_usart_cdc(void)
{

	struct usart_config config_cdc;
	usart_get_config_defaults(&config_cdc);
	config_cdc.baudrate	 = 9600;
	config_cdc.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	config_cdc.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	config_cdc.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	config_cdc.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	config_cdc.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	stdio_serial_init(&cdc_instance, EDBG_CDC_MODULE, &config_cdc);
	usart_enable(&cdc_instance);
}
//! [cdc_setup]
//******************************************* UART CONFIG END *******************************************


// ******************************************* CAN CONFIG AND API*******************************************
static void can_set_standard_filter_0(void)
{
	struct can_standard_message_filter_element sd_filter;

	can_get_standard_message_filter_element_default(&sd_filter);
	sd_filter.S0.bit.SFID2 = CAN_RX_STANDARD_FILTER_ID_0_BUFFER_INDEX;
	sd_filter.S0.bit.SFID1 = CAN_RX_STANDARD_FILTER_ID_0;
	sd_filter.S0.bit.SFEC =
	CAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_STRXBUF_Val;

	can_set_rx_standard_filter(&can_instance, &sd_filter,
	CAN_RX_STANDARD_FILTER_INDEX_0);
	can_enable_interrupt(&can_instance, CAN_RX_BUFFER_NEW_MESSAGE);
}


static void can_send_standard_message(uint32_t id_value, uint8_t *data,
uint32_t data_length)
{
	uint32_t i;
	struct can_tx_element tx_element;

	can_get_tx_buffer_element_defaults(&tx_element);
	tx_element.T0.reg |= CAN_TX_ELEMENT_T0_STANDARD_ID(id_value);
	tx_element.T1.bit.DLC = data_length;
	for (i = 0; i < data_length; i++) {
		tx_element.data[i] = *data;
		data++;
	}

	can_set_tx_buffer_element(&can_instance, &tx_element,
	CAN_TX_BUFFER_INDEX);
	can_tx_transfer_request(&can_instance, 1 << CAN_TX_BUFFER_INDEX);
}


static void can_set_extended_filter_1(void)
{
	struct can_extended_message_filter_element et_filter;

	can_get_extended_message_filter_element_default(&et_filter);
	et_filter.F0.bit.EFID1 = CAN_RX_EXTENDED_FILTER_ID_1;

	can_set_rx_extended_filter(&can_instance, &et_filter,
	CAN_RX_EXTENDED_FILTER_INDEX_1);
	can_enable_interrupt(&can_instance, CAN_RX_FIFO_1_NEW_MESSAGE);
}


static void can_fd_send_extended_message(uint32_t id_value, uint8_t *data)
{
	uint32_t i;
	struct can_tx_element tx_element;

	can_get_tx_buffer_element_defaults(&tx_element);
	tx_element.T0.reg |= CAN_TX_ELEMENT_T0_EXTENDED_ID(id_value) |
	CAN_TX_ELEMENT_T0_XTD;
	tx_element.T1.reg = CAN_TX_ELEMENT_T1_EFC | CAN_TX_ELEMENT_T1_FDF |
	CAN_TX_ELEMENT_T1_BRS |
	CAN_TX_ELEMENT_T1_DLC(CAN_TX_ELEMENT_T1_DLC_DATA64_Val);
	for (i = 0; i < CONF_CAN_ELEMENT_DATA_SIZE; i++) {
		tx_element.data[i] = *data;
		data++;
	}

	can_set_tx_buffer_element(&can_instance, &tx_element,
	CAN_TX_BUFFER_INDEX);
	can_tx_transfer_request(&can_instance, 1 << CAN_TX_BUFFER_INDEX);
}


//! [can_init_setup]
static void configure_can(void)
{
	uint32_t i;
	/* Initialize the memory. */
	for (i = 0; i < CONF_CAN_ELEMENT_DATA_SIZE; i++) {
		tx_message_0[i] = i;
		tx_message_1[i] = i + 0x80;
	}

	/* Set up the CAN TX/RX pins */
	struct system_pinmux_config pin_config;
	system_pinmux_get_config_defaults(&pin_config);
	pin_config.mux_position = CAN_TX_MUX_SETTING;
	system_pinmux_pin_set_config(CAN_TX_PIN, &pin_config);
	pin_config.mux_position = CAN_RX_MUX_SETTING;
	system_pinmux_pin_set_config(CAN_RX_PIN, &pin_config);

	/* Initialize the module. */
	struct can_config config_can;
	can_get_config_defaults(&config_can);
	can_init(&can_instance, CAN_MODULE, &config_can);

	//can_enable_fd_mode(&can_instance);
	can_start(&can_instance);

	/* Enable interrupts for this CAN module */
	system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_CAN0);
	can_enable_interrupt(&can_instance, CAN_PROTOCOL_ERROR_ARBITRATION
	| CAN_PROTOCOL_ERROR_DATA);
}
//! [can_init_setup]


//! [can_interrupt_handler]
void CAN0_Handler(void)
{
	volatile uint32_t status, i, rx_buffer_index;
	status = can_read_interrupt_status(&can_instance);

	if (status & CAN_RX_BUFFER_NEW_MESSAGE) {
		can_clear_interrupt_status(&can_instance, CAN_RX_BUFFER_NEW_MESSAGE);
		for (i = 0; i < CONF_CAN0_RX_BUFFER_NUM; i++) {
			if (can_rx_get_buffer_status(&can_instance, i)) {
				rx_buffer_index = i;
				can_rx_clear_buffer_status(&can_instance, i);
				can_get_rx_buffer_element(&can_instance, &rx_element_buffer,
				rx_buffer_index);
				if (rx_element_buffer.R0.bit.XTD) {
					//printf("\n\r Extended FD message received in Rx buffer. The received data is: \r\n");
					} else {
					//printf("\n\r Standard FD message received in Rx buffer. The received data is: \r\n");
				}
				for (i = 0; i < CONF_CAN_ELEMENT_DATA_SIZE; i++) {
					//printf("  %d",rx_element_buffer.data[i]);
				}
				//printf("\r\n\r\n");
			}
		}
	}

	if (status & CAN_RX_FIFO_0_NEW_MESSAGE) {
		can_clear_interrupt_status(&can_instance, CAN_RX_FIFO_0_NEW_MESSAGE);
		can_get_rx_fifo_0_element(&can_instance, &rx_element_fifo_0,
		standard_receive_index);
		can_rx_fifo_acknowledge(&can_instance, 0,
		standard_receive_index);
		standard_receive_index++;
		if (standard_receive_index == CONF_CAN0_RX_FIFO_0_NUM) {
			standard_receive_index = 0;
		}
		if (rx_element_fifo_0.R1.bit.FDF) {
			//printf("\n\r Standard FD message received in FIFO 0. The received data is: \r\n");
			for (i = 0; i < CONF_CAN_ELEMENT_DATA_SIZE; i++) {
				//printf("  %d",rx_element_fifo_0.data[i]);
			}
			} else {
			//printf("\n\r Standard normal message received in FIFO 0. The received data is: \r\n");
			for (i = 0; i < rx_element_fifo_0.R1.bit.DLC; i++) {
				//printf("  %d",rx_element_fifo_0.data[i]);
			}
		}
		//printf("\r\n\r\n");
	}

	if (status & CAN_RX_FIFO_1_NEW_MESSAGE) {
		can_clear_interrupt_status(&can_instance, CAN_RX_FIFO_1_NEW_MESSAGE);
		can_get_rx_fifo_1_element(&can_instance, &rx_element_fifo_1,
		extended_receive_index);
		can_rx_fifo_acknowledge(&can_instance, 0,
		extended_receive_index);
		extended_receive_index++;
		if (extended_receive_index == CONF_CAN0_RX_FIFO_1_NUM) {
			extended_receive_index = 0;
		}

		//printf("\n\r Extended FD message received in FIFO 1. The received data is: \r\n");
		for (i = 0; i < CONF_CAN_ELEMENT_DATA_SIZE; i++) {
			commandMsg[i] = rx_element_fifo_1.data[i];
			//printf("  %d",rx_element_fifo_1.data[i]);
		}
		canMsgInFlag = 1;
		//printf("\r\n\r\n");
	}

	if ((status & CAN_PROTOCOL_ERROR_ARBITRATION)
	|| (status & CAN_PROTOCOL_ERROR_DATA)) {
		can_clear_interrupt_status(&can_instance, CAN_PROTOCOL_ERROR_ARBITRATION
		| CAN_PROTOCOL_ERROR_DATA);
		//printf("Protocol error, please double check the clock in two boards. \r\n\r\n");
	}
}
//! [can_interrupt_handler]

//! [user_menu]
static void display_menu(void)
{
	printf("Menu :\r\n"
	"  -- Select the action:\r\n"
	"  0: Set standard filter ID 0: 0x45A, store into Rx buffer. \r\n"
	"  1: Set standard filter ID 1: 0x468, store into Rx FIFO 0. \r\n"
	"  2: Send FD standard message with ID: 0x45A and 64 byte data 0 to 63. \r\n"
	"  3: Send FD standard message with ID: 0x468 and 64 byte data 128 to 191. \r\n"
	"  4: Set extended filter ID 0: 0x100000A5, store into Rx buffer. \r\n"
	"  5: Set extended filter ID 1: 0x10000096, store into Rx FIFO 1. \r\n"
	"  6: Send FD extended message with ID: 0x100000A5 and 64 byte data 0 to 63. \r\n"
	"  7: Send FD extended message with ID: 0x10000096 and 64 byte data 128 to 191. \r\n"
	"  a: Send normal standard message with ID: 0x468 and 8 byte data 0 to 7. \r\n"
	"  h: Display menu \r\n\r\n");
}
//! [user_menu]


//! ******************************************* [SYSTEM EVENT] *******************************************
static volatile uint32_t event_count = 0;
void event_counter(struct events_resource *resource);
static void configure_event_channel(struct events_resource *resource)
{
	struct events_config config;
	events_get_config_defaults(&config);
	config.generator = CONF_EVENT_GENERATOR;
	config.edge_detect = EVENTS_EDGE_DETECT_RISING;
	config.path = EVENTS_PATH_SYNCHRONOUS;
	config.clock_source = GCLK_GENERATOR_0;
	events_allocate(resource, &config);
}

static void configure_event_user(struct events_resource *resource)
{
	events_attach_user(resource, CONF_EVENT_USER);
}

static void configure_event_interrupt(struct events_resource *resource,
struct events_hook *hook)
{
	events_create_hook(hook, ADC_event);
	events_add_hook(resource, hook);
	events_enable_interrupt_source(resource, EVENTS_INTERRUPT_DETECT);
}

void event_counter(struct events_resource *resource)
{
	if(events_is_interrupt_set(resource, EVENTS_INTERRUPT_DETECT)) {
		port_pin_toggle_output_level(LED_0_PIN);
		////printf("@@@@@@@@@@@@@@@@@@@@@@@@@ event success @@@@@@@@@@@@@@@@@@@@@@@@ \r\n");
		event_count++;
		
		events_ack_interrupt(resource, EVENTS_INTERRUPT_DETECT);
	}
}
//! ******************************************* [SYSTEM EVENT END] *******************************************

//! ******************************************* [RTC CONFIG ] *******************************************

void configure_rtc_calendar(void)
{
	struct rtc_calendar_alarm_time alarm;

	/* Initialize RTC in calendar mode. */
	struct rtc_calendar_config config_rtc_calendar;
	rtc_calendar_get_config_defaults(&config_rtc_calendar);
	alarm.time.year = 2016;
	alarm.time.month = 1;
	alarm.time.day = 1;
	alarm.time.hour = 0;
	alarm.time.minute = 0;
	alarm.time.second = 0;
	config_rtc_calendar.clock_24h = true;
	config_rtc_calendar.alarm[0].time = alarm.time;
	config_rtc_calendar.alarm[0].mask = RTC_CALENDAR_ALARM_MASK_SEC;
	
	//! [init_rtc]
	rtc_calendar_init(&rtc_instance, RTC, &config_rtc_calendar);
	
	
	//[setup and initial RTC AND enable system event generate]
	struct rtc_calendar_events calendar_event;
	calendar_event.generate_event_on_periodic[7] = true;
	rtc_calendar_enable_events(&rtc_instance, &calendar_event);
	
	//! [enable]
	rtc_calendar_enable(&rtc_instance);
}
struct rtc_calendar_alarm_time alarm;

void rtc_match_callback(void)
{
	/* Do something on RTC alarm match here */
	////printf("###### alarm ######\r\n");
	
	static uint8_t second_count = 1;
	int32_t temp_result = 0;
	tsens_start_conversion();
	do {
		/* Wait for conversion to be done and read out temperature result */
	} while (tsens_read(&temp_result) != STATUS_OK);
	//printf("temperature :" );
	//printf("%d \r\n", temp_result);
	
	
	if (dateReportFlag == 1)
	{	
		second_count = 1;
		send_battery_data();
		battery_charge_calculation(5);
		if (charger_status == 1) {
			printf("%d voltage %d \r\n", time_index, charge_voltage_adc_result * 5000 / 4096);
			printf("%d c_current %d \r\n", time_index, charge_current_adc_result * 5000 / 4096);
		} else {
			printf("%d voltage %d \r\n", time_index, charge_voltage_adc_result * 5000 / 4096);
			printf("%d d_current %d \r\n\r\n", time_index++, discharge_current_adc_result * 5000 / 4096);
		}
		//reset flags
		dateReportFlag = 0;
	}
	
	
	/* Set new alarm in 5 seconds */
	alarm.mask = RTC_CALENDAR_ALARM_MASK_SEC;
	alarm.time.second += 1;
	alarm.time.second = alarm.time.second % 60;
	
	//forced data reporting every 10 seconds
	if ((alarm.time.second % 5) == 0)
	{
		dateReportFlag = 1;
	}
	
	second_count++;
	
	rtc_calendar_set_alarm(&rtc_instance, &alarm, RTC_CALENDAR_ALARM_0);
	
}

void configure_rtc_callbacks(void)
{
	rtc_calendar_register_callback(&rtc_instance, rtc_match_callback, RTC_CALENDAR_CALLBACK_ALARM_0);
	rtc_calendar_enable_callback(&rtc_instance, RTC_CALENDAR_CALLBACK_ALARM_0);
}
//! ******************************************* [RTC CONFIG END] *******************************************



/************************************************************************/
/* config temperature sensor                                            */
/************************************************************************/

void configure_tsens(void)
{
	struct tsens_config config_tsens;
	tsens_get_config_defaults(&config_tsens);
	tsens_init(&config_tsens);
	tsens_enable();
}

void battery_init(){
	//PB08
	adc_set_positive_input(&adc_instance,ADC_POSITIVE_INPUT_PIN4);
	adc_start_conversion(&adc_instance);
	do {
		// Wait for conversion to be done and read out result
	} while (adc_read(&adc_instance, &charge_voltage_adc_result) == STATUS_BUSY);
	
	//PB09
	adc_set_positive_input(&adc_instance,ADC_POSITIVE_INPUT_PIN5);
	adc_start_conversion(&adc_instance);
	do {
		// Wait for conversion to be done and read out result
	} while (adc_read(&adc_instance, &charge_current_adc_result) == STATUS_BUSY);
	//printf("%d current %d \r\n", time_index, adc_current * 5000 / 4096);
	delta_charge = 0;
	uint32_t voltage = (charge_voltage_adc_result <= charge_current_adc_result) ? 0 : (charge_voltage_adc_result - charge_current_adc_result) * 5000 / 4096;
	printf("Voltage %d \r\n", voltage);
	//delta_charge  = (voltage > 3000) ?  (voltage - 3000)  * calibrated_battery_capacity / 1200  : 0;
	printf("Initial delta_charge %d \r\n", delta_charge);
}


//! [setup]

int main(void)
{
	
	uint8_t key;

//! [setup_init]
	system_init();
	system_interrupt_enable_global();
	
//! setup USART
	configure_usart_cdc();
//! [setup_init]

	struct events_resource example_event;
	struct events_hook hook;

//!	setup GPIO
	configure_port_pins();
	
//! setup system event and call back function
	configure_event_channel(&example_event);
	configure_event_user(&example_event);
	configure_event_interrupt(&example_event, &hook);
	
	//setup RTC
	configure_rtc_calendar();
	configure_rtc_callbacks();
	rtc_calendar_enable(&rtc_instance);

//!config temperature sensor
	configure_tsens();
	
//! [configure_adc]
	configure_adc();
//! [configure_adc]

//! [configure_can]
	configure_can();
//! [configure_can]

//! [display_user_menu]
	//display_menu();
//! [display_user_menu]


	configure_dac();
	configure_dac_channel();
	
	

while (events_is_busy(&example_event)) {
	/* Wait for channel */
};

	can_set_standard_filter_0();
	
	battery_init();
	
	// open the Mosfet switch
	port_pin_set_output_level(EXT1_PIN_6, 1);
	uint16_t i = 0x3ff;
	
//! [main_loop]
	while(1) {
		dac_chan_write(&dac_instance, DAC_CHANNEL_0, i/2);
		//if (++i == 0x3ff) {
			//i = 0;
		//}
		
		if (1==canMsgInFlag){
			//printf("Command Received!\r\n");
			processCommandMsg();
		}
		if ((0 == system_busy_flag) && (1 == commandReady))
		{
			execute_system_command();
		}
		
		//send_board_time_data();
		//switch (key) {
		//case 'h':
			//display_menu();
			//break;
//
		//case '0':
			////printf("  0: Set standard filter ID 0: 0x45A, store into Rx buffer. \r\n");
			//can_set_standard_filter_0();
			//break;
//
		//case '1':
			////printf("  1: Set standard filter ID 1: 0x468, store into Rx FIFO 0. \r\n");
			//can_set_standard_filter_1();
			//break;
//
		//case '2':
			////printf("  2: Send standard message with ID: 0x45A and 4 byte data 0 to 3. \r\n");
			//can_send_standard_message(CAN_RX_STANDARD_FILTER_ID_0, tx_message_0,
					//CONF_CAN_ELEMENT_DATA_SIZE / 2);
			//break;
//
		//case '3':
			////printf("  3: Send standard message with ID: 0x468 and 4 byte data 128 to 131. \r\n");
			//can_send_standard_message(CAN_RX_STANDARD_FILTER_ID_1, tx_message_1,
					//CONF_CAN_ELEMENT_DATA_SIZE / 2);
			//break;
//
		//case '4':
			////printf("  4: Set extended filter ID 0: 0x100000A5, store into Rx buffer. \r\n");
			//can_set_extended_filter_0();
			//break;
//
		//case '5':
			////printf("  5: Set extended filter ID 1: 0x10000096, store into Rx FIFO 1. \r\n");
			//can_set_extended_filter_1();
			//break;
//
		//case '6':
			////printf("  6: Send extended message with ID: 0x100000A5 and 8 byte data 0 to 7. \r\n");
			//can_send_extended_message(CAN_RX_EXTENDED_FILTER_ID_0, tx_message_0,
					//CONF_CAN_ELEMENT_DATA_SIZE);
			//break;
//
		//case '7':
			////printf("  7: Send extended message with ID: 0x10000096 and 8 byte data 128 to 135. \r\n");
			//can_send_extended_message(CAN_RX_EXTENDED_FILTER_ID_1, tx_message_1,
					//CONF_CAN_ELEMENT_DATA_SIZE);
			//break;
		//case '8':
			////printf("  8: Test adc_sampling() function \r\n");
			//adc_sampling();
		//break;
//
		//default:
			//break;
		//}
		
		//scanf("%c", (char *)&key);
		//CONF_CLOCK_OSC48M_FREQ_DIV
	}
//! [main_loop]

//! [main_setup]
}

