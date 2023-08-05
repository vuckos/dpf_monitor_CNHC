#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <u8g2/csrc/u8x8.h>
#include <u8g2/csrc/u8g2.h>
#include "can.h"
#include "main.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/iwdg.h>


#define GET_SOOT_MAS_CALC     (uint8_t[8]){0x03, 0x22, 0x11, 0x4f, 0x00, 0x00, 0x00, 0x00}
#define GET_DISTANCE          (uint8_t[8]){0x03, 0x22, 0x11, 0x56, 0x00, 0x00, 0x00, 0x00}
#define GET_TIME_SINCE_REGEN  (uint8_t[8]){0x03, 0x22, 0x11, 0x5e, 0x00, 0x00, 0x00, 0x00}
#define GET_TEMP_DPF_IN       (uint8_t[8]){0x03, 0x22, 0x11, 0xb2, 0x00, 0x00, 0x00, 0x00}
#define GET_TEMP_DPF_OUT      (uint8_t[8]){0x03, 0x22, 0x10, 0xf9, 0x00, 0x00, 0x00, 0x00}
#define GET_SOOT_MAS_MEAS     (uint8_t[8]){0x03, 0x22, 0x11, 0x4e, 0x00, 0x00, 0x00, 0x00}
#define GET_DIFF_PRESSURE     (uint8_t[8]){0x03, 0x22, 0x14, 0xf5, 0x00, 0x00, 0x00, 0x00}
#define GET_POST_INJ_2        (uint8_t[8]){0x03, 0x22, 0x16, 0x3b, 0x00, 0x00, 0x00, 0x00}
#define GET_POST_INJ_3        (uint8_t[8]){0x03, 0x22, 0x16, 0x7d, 0x00, 0x00, 0x00, 0x00}


#define PIDS                                          \
(uint8_t [9][8])                                      \
{                                                     \
    { 0x03, 0x22, 0x11, 0x4f, 0x00, 0x00, 0x00, 0x00 }, /* GET_SOOT_MAS_CALC */     \
    { 0x03, 0x22, 0x11, 0x56, 0x00, 0x00, 0x00, 0x00 }, /* GET_DISTANCE */          \
    { 0x03, 0x22, 0x11, 0x5e, 0x00, 0x00, 0x00, 0x00 }, /* GET_TIME_SINCE_REGEN */  \
    { 0x03, 0x22, 0x11, 0xb2, 0x00, 0x00, 0x00, 0x00 }, /* GET_TEMP_DPF_IN */       \
    { 0x03, 0x22, 0x10, 0xf9, 0x00, 0x00, 0x00, 0x00 }, /* GET_TEMP_DPF_OUT */      \
    { 0x03, 0x22, 0x11, 0x4e, 0x00, 0x00, 0x00, 0x00 }, /* GET_SOOT_MAS_MEAS */     \
    { 0x03, 0x22, 0x14, 0xf5, 0x00, 0x00, 0x00, 0x00 }, /* GET_DIFF_PRESSURE */     \
    { 0x03, 0x22, 0x16, 0x3b, 0x00, 0x00, 0x00, 0x00 }, /* GET_POST_INJ_2 */        \
    { 0x03, 0x22, 0x16, 0x7d, 0x00, 0x00, 0x00, 0x00 }  /* GET_POST_INJ_3 */        \
}

#define PIDS_MSG (const char*[9]){"GET_SOOT_MAS_CALC\r\n", "GET_DISTANCE\r\n", "GET_TIME_SINCE_REGEN\r\n", "GET_TEMP_DPF_IN\r\n", "GET_TEMP_DPF_OUT\r\n", "GET_SOOT_MAS_MEAS\r\n", "GET_DIFF_PRESSURE\r\n", "GET_POST_INJ_2\r\n", "GET_POST_INJ_3\r\n"}


float calculate_soot_mass(void);
float calculate_distance_since(void);
float calculate_temp(void);
float calculate_time_since(void);
float calculate_diff_pressure(void);
float calculate_inj_quantity(void);

float (*func_ptr[9])(void) = {calculate_soot_mass, calculate_distance_since, calculate_time_since, calculate_temp, calculate_temp, calculate_soot_mass, calculate_diff_pressure, calculate_inj_quantity, calculate_inj_quantity };
float calc_val[9]={0,0,0,0,0,0,0,0,0};

#define NUMBER_OF_PIDS ((sizeof(calc_val) / sizeof(calc_val[0]))-1)
//Time in ms
#define MSG_TIMEOUT 300
#define MSG_SEND_DELAY 100
#define FULL_UPDATE_DELAY 4000
#define BTN_SWITCH_OFF_TIME 1200 //2000
#define DEBUG false
#define TIMEOUT_COUNT_POWEROFF 2
#define CAN_SEND_MAX_RETRY 3
#define MAX_ON_TIME (1000*60*60)*1.5
#define BTN_SHORT_PRESS_TIME 350
#define BLINK_DELAY 500

u8g2_t u8g2;
//extern struct ring output_ring;
//extern struct ring input_ring;
volatile unsigned int counter;
//volatile uint8_t status;
//volatile uint8_t commands_pending;
//uint8_t d_data[8];
uint8_t can_retry_conunter=0;

uint32_t expected_response=0;
volatile uint32_t current_time=0;
uint32_t can_response_timeout=0;
uint32_t can_next_msg_timer=0;
uint32_t btn_down_time=0;
bool btn_press_flag=false;
bool btn_short_press=false;
bool regeneration_started=false;

bool blink=false;

//float soot_mas=0.0;
//float soot_mas_meas=0.0;
//float distance_since_last_regen=0;
//float dpf_temp_in=0;
//float dpf_temp_out=0;
//float time_since_regen=0;


bool msg_RX=false;
bool print_once=true;
//uint8_t err_count=0;
uint16_t time_out_count=0, time_out_count_sum=0, poweroff_count=0;

uint32_t id;
uint8_t can_rx_data[8];
/*
typedef enum { SOOT_MAS_CALC,
               SOOT_MAS_MEAS,
               DISTANCE_SINCE,
               TEMP_DPF_IN,
               TEMP_DPF_OUT,
               TIME_SINCE_REGEN,              
               DELAY } obd_pid_states;
obd_pid_states obd_state = DELAY;
*/

static unsigned char dpf24x12_bits[] = {
   0xc0, 0xff, 0x03, 0x20, 0x00, 0x04, 0x10, 0x25, 0x08, 0x48, 0x00, 0x12,
   0x07, 0x44, 0xe0, 0x40, 0x00, 0x04, 0x00, 0x48, 0x00, 0x47, 0x02, 0xe2,
   0x08, 0x80, 0x10, 0x90, 0x10, 0x08, 0x20, 0x00, 0x04, 0xc0, 0xff, 0x03 };

static unsigned char black_24x12_bits[] = {
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

void update_display(void);
void power_off_display(void);
static void i2c_setup(void);
bool obd_send_command(uint16_t std_id, uint8_t *data);
void power_off(void);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static uint8_t u8x8_gpio_and_delay_cm3(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
	switch(msg) {
	case U8X8_MSG_GPIO_AND_DELAY_INIT:
		i2c_setup();  /* Init I2C communication */
		break;

	default:
		u8x8_SetGPIOResult(u8x8, 1);
		break;
	}

	return 1;
}

/* I2C hardware transfer based on u8x8_byte.c implementation */
static uint8_t u8x8_byte_hw_i2c_cm3(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
	static uint8_t buffer[32];   /* u8g2/u8x8 will never send more than 32 bytes */
	static uint8_t buf_idx;
	uint8_t *data;

	switch(msg) {
	case U8X8_MSG_BYTE_SEND:
		data = (uint8_t *)arg_ptr;
		while(arg_int > 0) {
			buffer[buf_idx++] = *data;
			data++;
			arg_int--;
		}
		break;
	case U8X8_MSG_BYTE_INIT:
		break;
	case U8X8_MSG_BYTE_SET_DC:
		break;
	case U8X8_MSG_BYTE_START_TRANSFER:
		buf_idx = 0;
		break;
	case U8X8_MSG_BYTE_END_TRANSFER:
		//i2c_transfer7(I2C1, 0x3D, buffer, buf_idx, NULL, 0);//display address
        i2c_transfer7(I2C1, 0x3C, buffer, buf_idx, NULL, 0);//display address
		break;
	default:
		return 0;
	}
	return 1;
}
#pragma GCC diagnostic pop


/* Wait a bit - the lazy version */
static void delay(uint32_t n) {
	for(uint32_t i = 0; i < n; i++)
		__asm__("nop");
}

static void gpio_setup(void)
{
    /* Enable GPIOA & GPIOB & GPIOC clock */
    /* A2 & A3 USART */
    rcc_periph_clock_enable(RCC_GPIOA);
    /* B8 & B9 CAN */
    rcc_periph_clock_enable(RCC_GPIOB);
    /* Preconfigure LED */
    gpio_set(GPIOC, GPIO13); /* LED green off */

    /* Configure LED&Osci GPIO */
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    /* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_USART3);   
    rcc_periph_clock_enable(RCC_DMA1);

    //power latch pin
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);
    gpio_clear(GPIOB, GPIO15); /* latch power off, need delay first */

    /* Power button sense. */
	/* Set GPIO14 (in GPIO port B) to 'input with pull-up resistor'. */
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO14);
	gpio_set(GPIOB, GPIO14);

}

static void systick_setup(void)
{
    /* 72MHz / 8 => 9000000 counts per second */
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
    /* 9000000/9000 = 1000 overflows per second - every 1ms one interrupt */
    /* SysTick interrupt every N clock pulses: set reload to N-1 */
    systick_set_reload(8999);
    systick_interrupt_enable();
    /* Start counting */
    systick_counter_enable();
}

void sys_tick_handler(void)
{
    /* We call this handler every 1ms so every 1ms = 0.001s*/

    current_time++;
    counter++;

    if(current_time==MAX_ON_TIME){
        gpio_clear(GPIOB, GPIO15); /* latch power off */
        for(;;){
            gpio_clear(GPIOB, GPIO15);
            power_off_display();
        }
    }

    static uint16_t btndbc = 0;
    btndbc=(btndbc<<1) | (gpio_get(GPIOB, GPIO14)>>13) | 0xe000;
    static bool pr1=false;

    if(DEBUG){
        if(btn_down_time!=0x0 && btn_press_flag==false && pr1==false){
            //size_t nbytes = snprintf(NULL, 0, "btn_down_time=%#08x\r\n", btn_down_time) + 1;
            size_t nbytes = snprintf(NULL, 0, "btn_down_time=%lu\r\n", btn_down_time) + 1;
            char *str = (char*)malloc(nbytes);
            snprintf(str, nbytes,"btn_down_time=%lu\r\n", btn_down_time);
            print(str);free(str);
            pr1=true;
        }
    }

    if (btndbc==0xfffe) { //btn up
        btn_press_flag=false;
    }

    if (btndbc==0xf000) { // High for 1 bits low for 12 (each bit is 1 loop thru this code).
        btn_down_time=0;
        btn_press_flag=true;
        if(DEBUG){ print("btndbc==0xf000\r\n"); pr1=false; } //print once
    }

    if(btndbc==0xE000){ //button down
        btn_down_time++;
    }

    if(btn_down_time>=BTN_SWITCH_OFF_TIME){
        btn_down_time=0;
        gpio_clear(GPIOB, GPIO15); /* latch power off */
        for(;;){gpio_clear(GPIOB, GPIO15); power_off_display();}
    }


    if(btn_down_time>30 && btn_down_time<=BTN_SHORT_PRESS_TIME && btn_press_flag==false){
        if(DEBUG){ print("short press\r\n"); pr1=false; }
        btn_down_time=0;
        btn_short_press=!btn_short_press;
    }


    if (counter == 2000) {
        counter = 0;
        gpio_toggle(GPIOC, GPIO13); /* toggle green LED */
    }
}


/* Initialize I2C1 interface */
static void i2c_setup(void) {
	/* Enable GPIOB and I2C1 clocks */
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_I2C1);

	/* Set alternate functions for SCL and SDA pins of I2C1 */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
				  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
				  GPIO_I2C1_SCL | GPIO_I2C1_SDA);

	/* Disable the I2C peripheral before configuration */
	i2c_peripheral_disable(I2C1);

	/* APB1 running at 36MHz */
    i2c_set_clock_frequency(I2C1, 36);

	/* 400kHz - I2C fast mode */
    i2c_set_fast_mode(I2C1);
	i2c_set_ccr(I2C1, 0x1e);	
    i2c_set_trise(I2C1, 0x0b);
	/* And go */
	i2c_peripheral_enable(I2C1);
}

void power_off(){
    power_off_display();
    //delay(800);
    gpio_clear(GPIOB, GPIO15); /* latch power off */
    for(;;){gpio_clear(GPIOB, GPIO15);}
}

float calculate_soot_mass(void){
    //return (((can_rx_data[4]*256)+can_rx_data[5])/100.0);
    return ((uint16_t)(((uint8_t)can_rx_data[4]<<8) | ((uint8_t)can_rx_data[5]<<0)))/100.0;
}

float calculate_distance_since(void){
    float distance_since_last_regen=(((can_rx_data[4]<<24) | (can_rx_data[5]<<16) | (can_rx_data[6]<<8) | can_rx_data[7]  )/1000.0);
    return (float)distance_since_last_regen;
}

float calculate_temp(void){
    //return ((((can_rx_data[4]*256)+can_rx_data[5])-2731)/10.0);
    return ((((can_rx_data[4]<<8) | can_rx_data[5])-2731)/10.0);
}

float calculate_time_since(void){
    uint16_t temp=(((uint8_t)can_rx_data[6]<<8) | ((uint8_t)can_rx_data[7]<<0));
    return (float)temp/60.0;
}

float calculate_diff_pressure(void){
    int16_t diff_press=(((uint8_t)can_rx_data[4]<<8) | ((uint8_t)can_rx_data[5]<<0));
    return (float)diff_press;
}

float calculate_inj_quantity(void){
    uint16_t post_inj=( ((uint8_t)can_rx_data[4]<<8) | (uint8_t)can_rx_data[5] );
    return (float)post_inj/100.0;
}

void usb_lp_can_rx0_isr(void)
{
    if(DEBUG){print("can isr" "\r\n");}
	
    if(!msg_RX){
    
        bool ext, rtr;
        uint8_t fmi, length;

        can_receive(CAN1, 0, false, &id, &ext, &rtr, &fmi, &length, can_rx_data, NULL);

        msg_RX=true;

        if(DEBUG){
            print("RX" "\r\n");
            gpio_toggle(GPIOC, GPIO13); /* toggle green LED */
            size_t nbytes = snprintf(NULL, 0, "id=%#08lx, ext=%d, rtr=%d, fmi=%#02x, length=%#02x, %#02x, %#02x, %#02x, %#02x, %#02x, %#02x, %#02x, %#02x \r\n", id, ext, rtr, fmi, length, can_rx_data[0], can_rx_data[1], can_rx_data[2], can_rx_data[3], can_rx_data[4], can_rx_data[5], can_rx_data[6], can_rx_data[7] ) + 1;
            char *str = (char*)malloc(nbytes);
            snprintf(str, nbytes,"id=%#08lx, ext=%d, rtr=%d, fmi=%#02x, length=%#02x, %#02x, %#02x, %#02x, %#02x, %#02x, %#02x, %#02x, %#02x \r\n", id, ext, rtr, fmi, length, can_rx_data[0], can_rx_data[1], can_rx_data[2], can_rx_data[3], can_rx_data[4], can_rx_data[5], can_rx_data[6], can_rx_data[7] );
            print(str);
            free(str);
        }

        can_fifo_release(CAN1, 0);
        if(DEBUG){print("can isr end" "\r\n");}
    }
}

bool obd_send_command(uint16_t std_id, uint8_t *data)
{
    if (can_transmit(CAN1,
        std_id,     /* (EX/ST)ID: CAN ID */
        false, /* IDE: CAN ID extended? */
        false, /* RTR: Request transmit? */
        8,     /* DLC: Data length */
        data) == -1)
    {
        expected_response = 0;
        msg_RX=false;
        return false;
    }else{ 
        can_next_msg_timer=current_time+MSG_SEND_DELAY;       
        msg_RX=false;
        expected_response = ( ( (uint8_t)data[1] | 1<<6 ) << 16 | ((uint8_t)data[2]) << 8 | ((uint8_t)data[3]) << 0 );
        
        const uint16_t id1 = (( (std_id|1<<3) << 5 ));// | // STDID
        can_filter_id_list_16bit_init(0, id1, id1, id1, id1, 0, true);

        can_response_timeout=current_time+MSG_TIMEOUT;
        return true;
    }

}

bool validate_response(void){
    if(DEBUG){print("validate_response" "\r\n");}
    uint32_t can_rx = ( ( (uint8_t)can_rx_data[1] | 1<<6 ) << 16 | ((uint8_t)can_rx_data[2]) << 8 | ((uint8_t)can_rx_data[3]) << 0 );
    
    if(expected_response==can_rx){
        can_rx_data[0]=0;
        msg_RX=false;
        expected_response=0;
        if(DEBUG){print("res OK!" "\r\n");}
        return true;
    }else{
        expected_response=0;
        msg_RX=false;
        can_rx_data[0]=0;
        if(DEBUG){print("res NOK!" "\r\n");}
        return false;
    }
}


void init_display(void){
    i2c_setup();
    //u8g2_Setup_ssd1306_i2c_128x64_noname_1(&u8g2, U8G2_R0, u8x8_byte_hw_i2c_cm3, u8x8_gpio_and_delay_cm3);
    u8g2_Setup_ssd1306_i2c_128x64_noname_2(&u8g2, U8G2_R0, u8x8_byte_hw_i2c_cm3, u8x8_gpio_and_delay_cm3);    
    delay(800000);
    u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
    u8g2_SetPowerSave(&u8g2, 0); // wake up display

    u8g2_FirstPage(&u8g2);
    do {
        u8g2_SetDrawColor(&u8g2,1);
        u8g2_DrawFrame(&u8g2,0, 0, 128, 64);
        u8g2_SetFont(&u8g2, u8g2_font_ncenB12_tf);
        u8g2_DrawStr(&u8g2, 10,37,"Initializing...");  
    } while ( u8g2_NextPage(&u8g2) );

}

void obd_msg_timeout(const char* msg){
    if(DEBUG){
        print("timeout ");
        print(msg);
        
    }
    delay(2);
    time_out_count+=1;
    msg_RX=false;
    can_rx_data[0]=0;
    expected_response=0;
    print_once=true;
}

static void iwdg_setup(uint32_t ms)
{
  iwdg_reset();
  iwdg_set_period_ms(ms);
  iwdg_start();
}


int main(void) {

	rcc_periph_clock_enable(RCC_GPIOC);
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    gpio_setup();
    gpio_clear(GPIOB, GPIO15); /* latch power off */
    can_setup();
    usart3_setup();
    systick_setup();
    //gpio_clear(GPIOB, GPIO15); /* latch power off */
    iwdg_setup(3000);
    init_display();

    uint32_t power_on_delay=current_time+1000;   
    while(power_on_delay>current_time){
        gpio_clear(GPIOB, GPIO15); /* latch power off */
    }

    gpio_set(GPIOB, GPIO15); /* latch power on */

    //iwdg_setup(100);

	while(true) {
		 
        static uint8_t i=0;
        static bool timer_set=false;
        static bool update_done=false;
        static uint32_t full_update_time=0, blink_time=0;

        iwdg_reset();

        if(i<=NUMBER_OF_PIDS && full_update_time<=current_time){
            timer_set=false;
            update_done=false;

            if(DEBUG){
                if(print_once){
                    print(PIDS_MSG[i]);
                    print_once=false;
                }
            }

            if(expected_response==0 && !msg_RX && (current_time>=can_next_msg_timer) ){
                if(obd_send_command(0x7e0, &PIDS[i][0])){
                    msg_RX=false;
                }else{
                    can_retry_conunter++;
                    if(DEBUG){ print("can error ");  print(PIDS_MSG[i]); }
                    msg_RX=false;
                    if(can_retry_conunter>=CAN_SEND_MAX_RETRY){
                        can_retry_conunter=0;
                        obd_msg_timeout(PIDS_MSG[i]);
                        i++;
                    }
                }
            }else if(can_rx_data[0] != 0 && msg_RX){
                if(validate_response()){
                    calc_val[i]=(*func_ptr[i])();
                }
                i++;
                print_once=true;
            }else if(current_time>=can_response_timeout){
                obd_msg_timeout(PIDS_MSG[i]);
                i++;
            }

        }else{
            i=0;
            update_done=true;
            if( time_out_count == (NUMBER_OF_PIDS+1) ){
                poweroff_count+=1;
            }

        }
        
        if(btn_press_flag || time_out_count_sum>0){
            update_display();
        }

        if((calc_val[7]>0 || calc_val[8]>0) && blink_time<=current_time){
            blink_time=current_time+BLINK_DELAY;
            update_display();
        }

        if( update_done && !timer_set ){
            if(DEBUG){ print("Full update done" "\r\n"); }
            full_update_time=current_time+FULL_UPDATE_DELAY;
            timer_set=true;
            
            
            if(calc_val[7]==0 || calc_val[8]==0){
                update_display();
            }

            if(time_out_count==0){
                time_out_count_sum=0;
                poweroff_count=0;
            }
            
            time_out_count=0;
            
            if(poweroff_count>=TIMEOUT_COUNT_POWEROFF){
                power_off();                    
            }   
            
            if(DEBUG){ print("============" "\r\n"); }
        }

	} //end while;

	return 0;
}


void update_display(){

  if(DEBUG){ print("display start" "\r\n"); }

  uint8_t btn_time_bar=0;
  if (btn_press_flag==true){
    float btn_press_percent=(btn_down_time*100)/BTN_SWITCH_OFF_TIME;
    btn_time_bar=(btn_press_percent*128)/100;
  }

  float percent=(calc_val[0]*100)/24;    
  
  //size_t nbytes = snprintf(NULL, 0, "%2.2fg,%3.0fkm", calc_val[0], distance_since_last_regen) + 1;
  size_t nbytes = snprintf(NULL, 0, "%2.2fg, %3.0fkm", calc_val[0], calc_val[1]) + 1;
  char *str = (char*)malloc(nbytes);
  snprintf(str, nbytes, "%2.2fg, %3.0fkm", calc_val[0], calc_val[1]);

  nbytes = snprintf(NULL, 0, "%2.0f%c", percent, (char)37) + 1;
  char *str1 = (char*)malloc(nbytes);
  snprintf(str1, nbytes, "%2.0f%c", percent, (char)37);
  uint8_t prog_bar=(percent*128)/100;

  nbytes = snprintf(NULL, 0, "%3.0f\xb0\x43", calc_val[3]) + 1;
  char *str_tmp_in = (char*)malloc(nbytes);
  snprintf(str_tmp_in, nbytes, "%3.0f\xb0\x43", calc_val[3]);

  nbytes = snprintf(NULL, 0, "%3.0f\xb0\x43", calc_val[4]) + 1;
  char *str_tmp_out = (char*)malloc(nbytes);
  snprintf(str_tmp_out, nbytes, "%3.0f\xb0\x43", calc_val[4]);


  //time since regen
  //nbytes = snprintf(NULL, 0, "%3.0f", calc_val[2]) + 1;
  //char *str_time_since = (char*)malloc(nbytes);
  //snprintf(str_time_since, nbytes, "%3.0f", calc_val[2]);
  
  //soot_mas_meas
  //nbytes = snprintf(NULL, 0, "%2.2f", calc_val[5]) + 1;
  //char *str_soot_mass_meas = (char*)malloc(nbytes);
  //snprintf(str_soot_mass_meas, nbytes, "%2.2f", calc_val[5]);

  //diff pressure
  //nbytes = snprintf(NULL, 0, "%3.0f", calc_val[6]) + 1;
  //char *diff_press = (char*)malloc(nbytes);
  //snprintf(diff_press, nbytes, "%3.0f", calc_val[6]);

  //time since regen + soot_mas_meas + diff pressure
  nbytes = snprintf(NULL, 0, "%3.0fm%2.2fg%3.0fhPa", calc_val[2], calc_val[5], calc_val[6]) + 1;
  char *time_soot_press = (char*)malloc(nbytes);
  snprintf(time_soot_press, nbytes, "%3.0fm%2.2fg%3.0fhPa", calc_val[2], calc_val[5], calc_val[6]);


  nbytes = snprintf(NULL, 0, "%1.2fg/str%1.2fg/str", calc_val[7], calc_val[8]) + 1;
  char *str_post_inj = (char*)malloc(nbytes);
  snprintf(str_post_inj, nbytes, "%1.2fg/str%1.2fg/str", calc_val[7], calc_val[8]);
  if(calc_val[7]>0 || calc_val[8]>0){
    blink=!blink;
  }else{blink=false;}



  char *str_time_out_count = (char*)malloc(1);
  if(time_out_count>0){
    free(str_time_out_count);
    nbytes = snprintf(NULL, 0, "ERR=%d/9, PD=%d/2", time_out_count, poweroff_count) + 1;
    //char *str_time_out_count = (char*)malloc(nbytes);
    str_time_out_count = (char*)malloc(nbytes);
    snprintf(str_time_out_count, nbytes, "ERR=%d/9, PD=%d/2", time_out_count, poweroff_count);
  }
 
 if(DEBUG){ print("d1" "\r\n"); }

    u8g2_FirstPage(&u8g2);
    do {
        u8g2_SetFont(&u8g2, u8g2_font_ncenB12_tf);
        u8g2_DrawStr(&u8g2, 0,12,str);
        u8g2_SetFontMode(&u8g2,1);
        u8g2_SetDrawColor(&u8g2,1);
        u8g2_DrawFrame(&u8g2,0, 16, 128, 14);
        u8g2_DrawBox(&u8g2,0, 17, prog_bar, 12);
        u8g2_SetFont(&u8g2,u8g2_font_pxplusibmvga9_tf);//u8g2_font_courB10_tf u8g2_font_pxplusibmvga9_tf
        u8g2_SetDrawColor(&u8g2,2);
        u8g2_DrawStr(&u8g2,49,28,str1);
        u8g2_SetFont(&u8g2, u8g2_font_ncenB12_tf);
        
        //u8g2_DrawStr(&u8g2, 0,43,str_tmp_in);
        //u8g2_DrawXBM(&u8g2, 52, 31, 24, 12, dpf24x12_bits);
        //u8g2_DrawStr(&u8g2,78,43,str_tmp_out);

        u8g2_DrawStr(&u8g2, 0,45,str_tmp_in);
        
        if (blink){
            u8g2_DrawXBM(&u8g2, 52, 33, 24, 12, black_24x12_bits);
        }else{
            u8g2_DrawXBM(&u8g2, 52, 33, 24, 12, dpf24x12_bits);
        }
        u8g2_DrawStr(&u8g2,78,45,str_tmp_out);

        if(time_out_count==0){
            u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tf);
            //u8g2_SetFont(&u8g2,u8g2_font_fur11_tf);
            //u8g2_SetFont(&u8g2,u8g2_font_t0_14b_tf);
            if(btn_short_press){
                u8g2_DrawStr(&u8g2, 0,62,str_post_inj);
            }else{
                u8g2_DrawStr(&u8g2, 0,61,time_soot_press);
            }
            //u8g2_DrawStr(&u8g2, 0,62,str_time_since);
            //u8g2_DrawStr(&u8g2,38,62,str_soot_mass_meas);
            //u8g2_DrawStr(&u8g2,80,62,diff_press); 
        }else{
            u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tf);
            u8g2_DrawStr(&u8g2, 0,62,str_time_out_count);
        }


        //u8g2_DrawFrame(&u8g2,0, 57, 128, 7);
        
        u8g2_DrawHLine(&u8g2, 0, 63, btn_time_bar);

    } while ( u8g2_NextPage(&u8g2) );

  if(DEBUG){ print("d1 end" "\r\n"); }
  free(str);
  free(str1);
  free(str_tmp_in);
  free(str_tmp_out);
  //free(str_time_since);
  //free(str_soot_mass_meas);
  //free(diff_press);
  free(time_soot_press);
  free(str_time_out_count);
  free(str_post_inj);

  if(DEBUG){ print("display end" "\r\n"); }
}

void power_off_display(){
    u8g2_FirstPage(&u8g2);
    do {
        u8g2_SetDrawColor(&u8g2,1);
        u8g2_DrawFrame(&u8g2,0, 0, 128, 64);
        u8g2_SetFont(&u8g2, u8g2_font_ncenB12_tf);
        u8g2_DrawStr(&u8g2, 10,37,"Power off...");     
    } while ( u8g2_NextPage(&u8g2) );
}