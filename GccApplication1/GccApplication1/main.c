/*
 * GccApplication1.c
 *
 * Created: 10.07.2025 21:15:47
 * Author : Илья
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>



#define size_buf 32
#define command_start 'q'
#define V_ref 1.1



uint8_t USART_data;
struct RingedBuffer input_buf;
uint8_t com_started = 0;
uint8_t len = 0;
uint8_t data_buffer[size_buf];
uint8_t sum_buffer[2];
uint8_t received = 0;
uint16_t U;
uint16_t goal = 0;
uint16_t up_U = 0;
uint16_t low_U = 0;
uint8_t five_Hz = 1;
uint8_t one_Hz = 0;
uint16_t adc_data = 0;

		/*USART*/

struct RingedBuffer{
	uint8_t begin;
	uint8_t end;
	uint8_t buf[size_buf];
	}; 
	
void init_buf(struct RingedBuffer* rb){
	rb->begin = 0;
	rb->end = 0;
}

void write_to_end(struct RingedBuffer* rb ,uint8_t data){
	rb->buf[rb->end++] = data;
	if(rb->end == size_buf){
		rb->end = 0;
	}
}

uint8_t read_from_begin(struct RingedBuffer* rb){
	uint8_t data;
	data = rb->buf[rb->begin++];
	if(rb->begin == size_buf){
		rb->begin = 0;
	}
	return data;
}

void init_USART0(){
	UCSR0B |= (1<<7) | (1<<4) | (1<<3); //RXCIE RXE TXE
	UCSR0C |= (1<<2) | (1<<1); //8-bit
	UBRR0H = 0;
	UBRR0L = 102; // 9600 BAUD
	sei();
}

uint8_t USART_recieve_data(){
	return UDR0;
}

void USART_send_data(uint8_t data){
	if(UCSR0A & (1 << 5)){
		UDR0 = data;
	}
}

void analyze_start(struct RingedBuffer* rb){
	uint8_t last_pos = rb->begin;
	if((read_from_begin(rb) == command_start) && (com_started == 0)){
		com_started = 1;
	}
	else{
		rb->begin = last_pos;
	}
}

void set_u(uint8_t* data, uint8_t l){
	for (uint8_t i = 0; i < l; i++)
	{
		U += data[i]*pow(10, l-1-i);
	}
}

uint8_t check_control_sum(uint8_t* input_data, uint8_t l1, uint8_t* input_sum, uint8_t l2){
	uint8_t sum1 = 0;
	uint8_t sum2 = 0;
	for (uint8_t i = 0; i < l1; i++)
	{
		sum1 += input_data[i];
	}
	for (uint8_t i = 0; i < l2; i++)
	{
		sum2 += input_sum[i]*pow(10, l2-1-i);
	}
	if(sum1 == sum2){
		return 1;
	}
	return 0;
}

ISR(USART0_RX_vect){
	USART_data = USART_recieve_data();
	write_to_end(&input_buf, USART_data);
	if(com_started){
		received++;
	}
}

          /*PWM*/
		  
void init_PWM(){
	DDRB |= 1<<2;
	PORTB &= ~(1<<2);
	TCCR1A |= 1<<5; // OC1B non-inverting
	TCCR1A |= (1<<0) | (1<<1);
	TCCR1B |= 1<<3; //fast PWM, 10 bit
	TCCR1B |= (1<<1); //psc = 8
	OCR1B = 100;
}

void set_compare(uint16_t comp){
	OCR1B = comp;
}

        /*ADC*/
		
void init_ADC(){
	ADCSRA |= (1<<7); //enable
	ADCSRA |= (1<<5); 
	ADCSRA |= (1<<3); //interrupt en
	ADCSRA |= (1<<2) | (1<<1); //divide by 64 (250KHz)
	ADMUX |= (1<<7) | (1<<6); //internal source
	ADMUX |= 1<<1; //using ADC2
	ADCSRA |= 1<<6; //start 
}

ISR(ADC_vect){
	adc_data = ((ADC*V_ref)/1024)*1000;
}

		/*LEDs*/
		
void init_LED_rx(){
	DDRB |= 1<<4;
	PORTB &= ~(1<<4);
}

void init_TIM3(){
	DDRB |= (1<<5);
	PORTB &= ~(1<<5);
	TCCR3B |= (1<<1) | (1<<0);
	TIMSK3 |= 1<<0;
}

ISR(TIMER3_OVF_vect){
	if(five_Hz == 1){
		PORTB ^= (1<<5);
	}
	else{
		one_Hz++;
		if(one_Hz == 5){
			PORTB ^= (1<<5);
			one_Hz = 0;
		}
	}
}

void delay_500us(){
	for (int i = 0; i < 8000000; i++)
	{
	}
}


int main(void)
{
	init_USART0();
	init_PWM();
	init_ADC();
	init_LED_rx();
	init_TIM3();
	init_buf(&input_buf);
	uint8_t i = 0;
	uint8_t j = 0;
    while (1) 
    {
		analyze_start(&input_buf);
		if(com_started == 1){
			if((received > 0) && (len == 0)){
				len = read_from_begin(&input_buf) - '0';
			}
			if((received > len) && (i < len)){
				data_buffer[i] = read_from_begin(&input_buf) - '0';
				i++;
			}
			if((received > len+2) && (j < 2) && (i == len)){
				sum_buffer[j] = read_from_begin(&input_buf) - '0';
				j++;
			}
			if((i == len) && (j == 2)){
				if(check_control_sum(data_buffer, len, sum_buffer, 2)){
					set_u(data_buffer, len);
					goal = (U/3300)*1024;
					up_U = U + ((U/100)*5);
					low_U = U - ((U/100)*5);
					set_compare(goal);
				}
				com_started = 0;
				received = 0;
				len = 0;
				i = 0;
				j = 0;
				U = 0;
				PORTB ^= (1<<4);
				delay_500us();
				PORTB ^= (1<<4);
			}
		}
		if((adc_data != 0) && (adc_data > up_U)){
			five_Hz = 1;
			one_Hz = 0;
			goal -= 10;
			set_compare(goal);
		}
		if((adc_data != 0) && (adc_data < low_U)){
			five_Hz = 1;
			one_Hz = 0;
			goal += 10;
			set_compare(goal);
		}
		if((adc_data != 0) && (adc_data > low_U) && (adc_data < up_U)){
			five_Hz = 0;
		}
    }
}

