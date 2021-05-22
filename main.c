/*
 * IR_Remote_Controller.c
 *
 * Created: 2020-07-08 오후 9:17:19
 * Author : 민규
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>

unsigned char Gflag=0,Bflag=0,Hflag=0,Rflag=0,Lflag=0;
// 스위치를 눌렀을 때 인터럽트문에서 세워줄 플래그들
volatile int G = 71,B=66,H=72,R=82,L=76; // 적외선에 실어서 보낼 데이터들
volatile unsigned char flag,cnt;
void UART0_TX(unsigned char udata);
unsigned char UART0_RX();

ISR(TIMER0_OVF_vect){ // Channel 0 TimerCounter Interrupt every 10us 
	TCNT0=48;
	++cnt;
	if(cnt==35){ // 35*13us= 455us -> 500us
		flag=1;
	}
}

void _one(){
	flag=0;
	cnt=0;
	while(!flag)
	{ // OOK 방식
		if(cnt%2)
		{
			PORTB = 0x00; // PB5에 IR이 물려있고 low일때는 Ired에 전류가 흐르지 않음
			//UART0_TX(48); //확인용 uart '0'
		}
		else
		{
			PORTB = 0xff; // PB5에 IR이 물려있고 high가 되어야 Ired에 전류가 흐름
			//UART0_TX(49); //확인용 uart '1'
		}
	}
	// 500us동안 1과0을 반복해서 보내어 수신부에서 1로 인식하게 함
	PORTB = 0x00;
	//UART0_TX(48); //확인용 uart '0'
	flag = 0;
	cnt = 0;
	while(!flag);
} // OOK방식

void _zero(){
	flag=0;
	cnt=0;
	while(!flag)
	{ // OOK 방식
		if(cnt%2)
		{
			PORTB = 0x00;
			//UART0_TX(48); //확인용 uart '0'
		}
		else
		{
			PORTB = 0xff;
			//UART0_TX(49); //확인용 uart '1'
		}
	}
	// 500us동안 1과0을 반복해서 보내어 수신부에서 1로 인식하게 함
	PORTB = 0x00;
	//UART0_TX(48); //확인용 uart '0'
	flag = 0;
	cnt = 0;
	while(!flag);
	// 500us동안 0을 보내어 수신부에서 0으로 인식하게 함
	PORTB = 0x00;
	//UART0_TX(48); //확인용 uart '0'
	flag = 0;
	cnt = 0;
	while(!flag);
	// 500us동안 0을 보내어 수신부에서 0으로 인식하게 함
}
	
void send_frame(unsigned char* data) {
	int i=0,j=0,size;
	size = strlen(data);
	_delay_ms(5); // 새로운 data frame을 판별하기 위해 기다림
	_one(); // start bit = 1
	
	while(i<8)
	{ // data의 길이, Atmega128이 8bit연산을 하기 때문에 8번 반복
		if((size<<i)&0x80) // size = 0b10000000이 되면 실행
			_one();
		else
			_zero();
		++i;
	}
	
	while(j<size)
	{ // data, Atmega128이 8bit연산을 하기 때문에 8번 반복
		i=0;
		while(i<8){
			if((*(data+j)<<i)&0x80) // data가 포인터 타입이므로 인덱싱을 통해 보냄
				_one();
			else
				_zero();
			++i;
		}
		++j;
	}
	_one(); // end bit = 1
	//UART0_TX(13); //확인용 uart
}

void UART0_TX(unsigned char udata){
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = udata;
}

unsigned char UART0_RX(){
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
}

ISR(INT0_vect){ // 0번 interrupt에서 수행할 명령
	Gflag = 1; // 전진하기 위한 flag
}

ISR(INT1_vect){ // 1번 interrupt에서 수행할 명령
	Bflag = 1; // 후진하기 위한 flag
}

ISR(INT2_vect){ // 2번 interrupt에서 수행할 명령
	Lflag = 1; // 좌회전하기 위한 flag
}

ISR(INT3_vect){ // 3번 interrupt에서 수행할 명령
	Rflag = 1; // 우회전하기 위한 flag
}

ISR(INT4_vect){ // 4번 interrupt에서 수행할 명령
	Hflag = 1; // 정지하기 위한 flag
}

void flagSearch(){
	if (Gflag)
	{	
		EIFR = 0xFE;
		Bflag=0;
		Lflag=0;
		Rflag=0;
		Hflag=0;
		for (int j=0; j<4; j++)
		{
			_delay_ms(500);
			send_frame(&G); // 'G'
			_delay_ms(100);
		}
		//for (int j=0; j<3; j++)
		//{
			//_delay_ms(200);
			//send_frame("Go");
		//}
		Gflag=0;
		cnt=0;
	}
	else if (Bflag)
	{	
		EIFR = 0xFD;
		Gflag=0;
		Lflag=0;
		Rflag=0;
		Hflag=0;
		for (int j=0; j<4; j++){
			_delay_ms(500);		
			send_frame(&B); // 'B
			_delay_ms(100);
		}
		//for (int j=0; j<3; j++)
		//{
			//_delay_ms(300);
			//send_frame("Back");
		//}
		Bflag=0;
		cnt=0;
	}
	else if (Hflag)
	{
		EIFR = 0xFB;
		Gflag=0;
		Bflag=0;
		Lflag=0;
		Rflag=0;
		for (int j=0; j<4; j++){
			_delay_ms(500);
			send_frame(&H); // 'H'
			_delay_ms(100);
		}
		//for (int j=0; j<3; j++)
		//{
			//_delay_ms(300);
			//send_frame("Hello");
		//}
		Hflag=0;
		cnt=0;
	}
	else if (Rflag)
	{
		EIFR = 0xF7;
		Gflag=0;
		Bflag=0;
		Lflag=0;
		Hflag=0;
		for (int j=0; j<4; j++){
			_delay_ms(500);		
			send_frame(&R); // 'R
			_delay_ms(100);
		}
		//for (int j=0; j<3; j++)
		//{
			//_delay_ms(500);
			//send_frame("Right");
		//}
		Rflag=0;
		cnt=0;
	}
	else if (Lflag)
	{
		EIFR = 0xEF;
		Gflag=0;
		Bflag=0;
		Rflag=0;
		Hflag=0;
		for (int j=0; j<4; j++){
			_delay_ms(500);		
			send_frame(&L); // 'L
			_delay_ms(100);
		}
		//for (int j=0; j<3; j++)
		//{
			//_delay_ms(300);
			//send_frame("Left");
		//}
		Lflag=0;
		cnt=0;
	}
	return;
}

int main(void)
{
	DDRE = 0b00000010; // 스위치와 연결된 핀을 입력핀으로 설정
	DDRD = 0x00; // 스위치와 연결된 핀을 입력핀으로 설정
	DDRB = 0xff; // IRED와 연결된 핀을 출력핀으로 설정
	
	PORTE = 0b00000010;
	
	// overflow interrupt, normal mode, Prescale=1 -> No Prescaling
	TCCR0 = 0x01; // (1<<CS00)
	TIMSK = 0x01; // (1<<TOIE0)
	TCNT0 = 48; // 16000000/1/(256-TCNTinit), 208count => 13us 
	
	EICRA = (1<<ISC31)|(1<<ISC21)|(1<<ISC11)|(1<<ISC01); // Falling Edge에 Interrupt를 사용
	EICRB = (1<<ISC41); // Falling Edge에 Interrupt를 사용
	EIMSK = (1<<INT4)|(1<<INT3)|(1<<INT2)|(1<<INT1)|(1<<INT0); // 0~4번 Interrupt핀 활성화
	sei(); // Interrupt를 사용하겠다는 초기설정
	
	UCSR0A = 0X00;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	
	UBRR0H = 0;
	UBRR0L = 103;
	
    while (1){
		flagSearch();
	}
}

