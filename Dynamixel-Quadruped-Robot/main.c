/*
 * Dynamixel_Quadruped_Robot.c
 *
 * Created: 2020-06-05 오후 10:43:29
 * Author : 민규
 */ 

#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#define F_CPU 16000000UL

unsigned char flag=0; // 전역변수로서 flag를 초기화해서 설정
unsigned char Gflag=0,Bflag=0,Sflag=0,Rflag=0,Lflag=0; // 적외선 감지 시 set되는 플래그들
void Uart_Init(void);
void Timer_init(void);
void Interrupt_init(void);
void Port_init(void);
//Register Initialization
void Uart_trans (unsigned char data);
void UART0_TX(unsigned char data); // 들어오는 적외선 확인용 UART
unsigned char UART0_RX(); // 들어오는 적외선 확인용 UART
void UART0_print_string(char *str); // 들어오는 적외선 확인용 UART
void PacketTX(unsigned char ID,int Pos,int Spd);
//UART 송신
void Hello();
void Stay();
inline void StayLeft();
inline void StayRight();
void MoveFront();
void MoveBack();
void MoveRight();
void MoveLeft();
//보행로봇 구동 함수
volatile unsigned int cnt;
volatile int temp_cnt;
volatile int sample_flag;
int val,size;
char buf[32];
//적외선 통신 변수


ISR(USART1_RX_vect){ // Interrupt Service Routine
	while(!(UCSR1A & (1<<RXC1))); // Receive Buffer에 data가 없을 때
	flag = UDR1; // Receiver Buffer에 새로운 data를 받을 준비가 되면 flag에 수신한 data를 저장
}

ISR(TIMER0_OVF_vect){ // every 50us
	TCNT0 = 160;
	++cnt;
}

ISR(INT4_vect){ // 4번 interrupt에서 수행할 명령
	sample_flag = 1; // 신호를 감지하면 플래그가 선다
	temp_cnt = cnt; // temp_cnt에 cnt값을 저장
	cnt=0; // cnt 초기화
}

void detect_code() {
	int i=0,j=0;
	
	while(1)
	{
		if(cnt>40000) // 2s동안 신호가 들어오지 않을 때
		{
			PORTA = 0xff;
			memset(buf,32,sizeof(buf)); // 버퍼를 space문자로 다 초기화해준다. 즉,버퍼를 비움
			break;
		}
		
		else if(cnt > 90 && sample_flag) // 적외선 신호도 받고 최소한의 데이터길이를 받았을 때
		{
			memset(buf,32,sizeof(buf)); // 먼저 버퍼를 비운다
			val = 0;
			sample_flag = 0; // detect start
			while(!sample_flag)
			{
				if(cnt>40000) // 2s동안 신호가 들어오지 않을 때
				{
					PORTA = 0xff;
					break;
				}
			}
			sample_flag=0; // count throw away
			while(!sample_flag)
			{
				if(cnt>40000) // 2s동안 신호가 들어오지 않을 때
				{ 
					PORTA = 0xff;
					break;
				}
			}
			
			while(i<8)
			{
				sample_flag=0;
				while(!sample_flag)
				{
					if(cnt>40000)
					{
						PORTA = 0xff;
						break;
					}
				}
				// 0과 1을 판별(만약 temp_cnt가 27보다 작으면, '1'이다)
				//threshold = 27, high = cnt*22, low = cnt*32
				if(temp_cnt < 27) 
					val += 1<<(7-i); // MSB 먼저 받는다
				++i;
			}
			size = val; // 윗 반복문으로 받은 data의 길이를 대입
			j=0;
			PORTA = 0x00; // 확인용 LED
			while(j<size)
			{
				val=0;
				i=0;
				while(i<8)
				{
					sample_flag=0;
					while(!sample_flag)
					{
						if(cnt>40000)
						{
							PORTA = 0xff;
							break;
						}
					}
					// 0과 1을 판별(만약 temp_cnt가 27보다 작으면, '1'이다)
					//threshold = 27, high = cnt*22, low = cnt*32
					if(temp_cnt < 27)
						val += 1<<(7-i); // MSB 먼저 받는다
					++i;					
				}
				//_delay_ms(200); //UART용 delay
				buf[j] = val; // 윗 반복문으로 받은 data를 버퍼에 넣는다
				++j;
				PORTA = 0x00; // 확인용 LED
			}
			break;
		}
	}
	if(buf[0]!=32)
	{
		_delay_ms(200);
		UART0_print_string(buf);
		//_delay_ms(200);
		UART0_TX(59); // ;
		UART0_TX(13); // \n
	}
}

void detect_flag(char msg[],char sizeMsg){
	for(int i=0; i<sizeMsg;i++){
		if(msg[i]=='G'){
			Gflag = 1; // GoFlag만 set 나머진 0
			Sflag = 0;
			Bflag = 0;
			Lflag = 0;
			Rflag = 0;
		}
		else if(msg[i]=='B'){
			Gflag = 0; 
			Sflag = 0;
			Bflag = 1; // BackFlag만 set 나머진 0
			Lflag = 0;
			Rflag = 0;
		}
		else if(msg[i]=='R')
		{
			Gflag = 0;
			Sflag = 0;
			Bflag = 0;
			Lflag = 0;
			Rflag = 1; // RightFlag만 set 나머진 0
		}
		else if(msg[i]=='L')
		{
			Gflag = 0;
			Sflag = 0;
			Bflag = 0;
			Lflag = 1; // LeftFlag만 set 나머진 0
			Rflag = 0;
		}
		else if(msg[i]=='H')
		{
			Gflag = 0;
			Sflag = 1; // StayFlag만 set 나머진 0
			Bflag = 0;
			Lflag = 0;
			Rflag = 0;
		}
	}
	return;
}

void RobotMove(){ // Flag에 따라 로봇이 할 움직임을 정함
	if(Sflag)
	{
		Hello();
		Sflag = 0;
	}
	else if(Gflag)
	{
		MoveFront();
		Stay();
		Gflag = 0;
	}	
	else if(Bflag)
	{
		MoveBack();
		Stay();
		Bflag = 0;
	}
	else if(Lflag)
	{
		MoveLeft();
		Lflag = 0;
	}
	else if(Rflag)
	{
		MoveRight();
		Rflag = 0;
	}
}

void UART0_print_string(char *str){
	while(*str) 
	{
		_delay_ms(1000);
		UART0_TX(*str++);
	}
}

int main(void)
{
	_delay_ms(1000);
	Port_init(); // PORT 초기설정
    Uart_Init(); // UART 초기설정
	Timer_init(); // 50us의 8bit타이머 생성
	Interrupt_init(); // INT4번 Interrupt enable, Falling Edge에 Interrupt발생 
	
	Stay();
	while (1) 
    {
		detect_code(); // 적외선이 들어오면 감지하는 코드
		detect_flag(buf,sizeof(buf)/sizeof(char)); // 받은 데이터에 따라 플래그를 설정해주는 코드
		RobotMove(); // 설정된 플래그에 따라 움직임을 한다
    }
	return 0;
}

void Uart_Init(void){
		UCSR1A = 0x00; //PD2~3번에 연결해서 Uart를 사용할 것이기 때문에 n=1. datasheet를 참고
		UCSR1B = (1<<TXEN1); //UART transmitter 허용
		// Transmit,Receive Interrupt를 허용안함
		UCSR1C = (1<<UCSZ11)|(1<<UCSZ10); //송수신 data의 크기를 8-bit로 설정 
		
		UBRR1H = 0;
		UBRR1L = 1;
		//통신속도를 0.5Mbps로 설정 error가 0%라서 1M가 굳이 아니여도 상관없다
		//Dynamixel의 bps를 0.5M로 설정했기 때문에 통신속도를 맞춰준다	
		
		UCSR0A = 0x00;
		UCSR0B = 0b00011000;
		UCSR0C = 0b00000110;
		
		UBRR0H = 0;
		UBRR0L = 103;
}

void Timer_init(void){
	TCCR0 = (0<<CS02)|(1<<CS01)|(0<<CS00); // normal mode, prescale=8
	TIMSK = (1<<TOIE0); // overflow interrupt
	TCNT0 =160; // 16000000/8/(256-156)=20000=(0.000050)^-1
}

void Interrupt_init(void){
	EICRB = (1<<ISC41)|(0<<ISC40); // Falling Edge에 interrupt를 사용
	EIMSK = (1<<INT4); // 4번 interrupt 핀 활성화
	sei(); // Interrupt를 사용하겠다는 초기설정	
}

void Port_init(void){
		DDRA = 0xff;
		DDRD = 0Xff;
		DDRE = 0b00000010;
		PORTA = 0xff;
		PORTD = 0X00;
}

void Uart_trans(unsigned char data){
		while(!(UCSR1A & (1<<UDRE1))); // Transmit Buffer가 새로운 data를 받을 준비가 되었다.
		//즉 Buffer가 비어있다. 그렇게 되면
		//1<<UDRE! : 0b00100000
		//  UCSR1A : 0b00100000
		//&--------------------
		//			 0b00100000
		UDR1 = data; // 반복문 탈출 후 UDR1에 수신한 data를 넣어줌.
}

void UART0_TX(unsigned char data){
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

unsigned char UART0_RX(){
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
}

void PacketTX(unsigned char ID,int Pos,int Spd){
	
	unsigned char Pos_H,Pos_L,Spd_H,Spd_L;
	
	//if Pos = 300 = 0b00000001 00101100
	//Pos_L = 0b00101100
	//Pos >> 8 = 0b00000000 00000001 = Pos_H
	Pos_L = Pos;
	Pos_H = (Pos>>8);
	Spd_L = Spd;
	Spd_H = (Spd>>8);
	
	Uart_trans(0xFF);  //ini
	Uart_trans(0xFF);  //ini
	Uart_trans(ID);  //ID, Broadcase = 0xFE
	Uart_trans(0x07);  //LENGTH = 7
	Uart_trans(0x03);  //Instruction WRITE = 3
	Uart_trans(0x1E);  //모터 회전
	Uart_trans(Pos_L);  //각도L bit 
	Uart_trans(Pos_H);  //각도H bit
	Uart_trans(Spd_L);  //속도L bit
	Uart_trans(Spd_H);  //속도L bit
	Uart_trans((unsigned char)(~(ID + 0x07 + 0x03 + 0x1E + Pos_L + Pos_H + Spd_L + Spd_H))); // CheckSum
	_delay_ms(500);
}

void Stay(){
	PacketTX(1,0x1ff,0); //중앙값 = 150º = 511
	PacketTX(4,0x1ff,0);
	PacketTX(7,0x1ff,0);
	PacketTX(10,0x1ff,0);
	// 안쪽 관절부터 정렬
	PacketTX(2,170,0); //끝값 = 50º = 170
	PacketTX(5,852,0); //반대쪽 끝값 = 250º = 852
	PacketTX(8,170,0);
	PacketTX(11,852,0);
	// 중간 관절 정렬
	PacketTX(3,170,0);
	PacketTX(6,852,0);
	PacketTX(9,170,0);
	PacketTX(12,852,0);
	PORTA = 0x00; // 확인용 LED
	// 바깥쪽 관절 정렬
}
inline void StayLeft(){ // 정면 기준 몸체 왼쪽이 정자세로 고정
	PacketTX(1,0x1ff,0);
	PacketTX(7,0x1ff,0);
	PacketTX(2,170,0);
	PacketTX(8,170,0);
	PacketTX(3,170,0);
	PacketTX(9,170,0);
}
inline void StayRight(){ // 정면 기준 몸체 오른쪽이 정자세로 고정
	PacketTX(4,0x1ff,0);
	PacketTX(10,0x1ff,0);
	PacketTX(5,852,0);
	PacketTX(11,852,0);
	PacketTX(6,852,0);
	PacketTX(12,852,0);
}
void MoveFront(){
		PORTA = 0xFE; // 확인용 LED
		StayLeft();
		PacketTX(4,681,0); // 200º = 681
		PacketTX(10,340,0); // 100º = 340
		PacketTX(5,852,0);
		PacketTX(11,852,0);
		PacketTX(6,852,0);
		PacketTX(12,852,0);
		_delay_ms(10000);
		//Step 1 : 몸체의 오른쪽 뒷다리와 앞다리를 모은다
		PORTA = 0xFD; // 확인용 LED
		StayLeft();
		PacketTX(10,340,0); // 100º = 340, 12번 모터가 출력이 안되서 소리남
		PacketTX(11,852,0);
		PacketTX(12,852,0);
		PacketTX(6,681,0); // 250->200º
		PacketTX(5,681,0);
		PacketTX(4,579,0); // 200->170º = 579
		PacketTX(6,613,0); // 200->180º = 613
		PacketTX(5,613,0); // 200->180º = 613
		PacketTX(4,477,0); // 170->140º = 477
		_delay_ms(10000);
		//Step 2 : 몸체의 오른쪽 앞다리가 앞으로 전진
		PORTA = 0xFB; // 확인용 LED
		PacketTX(1,340,0);
		PacketTX(4,511,0);
		PacketTX(2,170,0);
		PacketTX(5,852,0);
		PacketTX(3,170,0);
		PacketTX(6,852,0);
		PacketTX(7,545,0); // 150->160º = 545
		PacketTX(8,340,0); // 50->100º = 340
		PacketTX(9,340,0); // 50->100º = 340
		PacketTX(10,511,0);
		PacketTX(8,409,0); // 100->120º = 409
		PacketTX(11,852,0);
		PacketTX(9,409,0); // 100->120º = 409
		PacketTX(12,852,0);
		_delay_ms(10000);
		//Step_3 : 다리가 전진한 거리만큼 몸체도 앞으로 전진
		PORTA = 0xF7; // 확인용 LED
		StayRight();
		PacketTX(1,340,0);
		PacketTX(7,681,0);
		PacketTX(2,170,0);
		PacketTX(8,170,0);
		PacketTX(3,170,0);
		PacketTX(9,170,0);
		_delay_ms(10000);
		//Step_4 : 몸체의 왼쪽 뒷다리와 앞다리를 모은다
		PORTA = 0xEF; // 확인용 LED
		StayRight();
		PacketTX(7,681,0);
		PacketTX(8,170,0);
		PacketTX(9,170,0);
		PacketTX(3,340,0); // 50->100º
		PacketTX(2,340,0); // 50->100º
		PacketTX(1,443,0); // 100->130º = 443
		PacketTX(3,409,0); // 100->120º = 409
		PacketTX(2,409,0); // 100->120º
		PacketTX(1,545,0); // 130->160º = 545
		_delay_ms(10000);
		//Step_5 : 몸체의 왼쪽 앞다리가 앞으로 전진
		PORTA = 0xDF; // 확인용 LED
		PacketTX(1,511,0);
		PacketTX(4,681,0);
		PacketTX(2,170,0);
		PacketTX(5,852,0);
		PacketTX(3,170,0);
		PacketTX(6,852,0);
		PacketTX(10,545,0); // 150->160º = 545
		PacketTX(11,681,0); // 250->200º = 681
		PacketTX(12,681,0); // 250->200º = 681
		PacketTX(7,511,0);
		PacketTX(11,613,0); // 200->180º = 613
		PacketTX(8,170,0);
		PacketTX(12,613,0); // 200->180º = 613
		PacketTX(9,170,0);
		_delay_ms(10000);
		//Step_6 : 다리가 전진한 거리만큼 몸체도 앞으로 전진
		PORTA = 0xBF; // 확인용 LED
		StayLeft();
		PacketTX(4,681,0); // 200º = 681
		PacketTX(10,340,0); // 100º = 340
		PacketTX(5,852,0);
		PacketTX(11,852,0);
		PacketTX(6,852,0);
		PacketTX(12,852,0);
		_delay_ms(10000);
		//Step 7 : 몸체의 오른쪽 뒷다리와 앞다리를 모은다
}
void MoveBack(){
		PORTA = 0x7F; // 확인용 LED
		StayLeft();
		PacketTX(4,681,0); // 200º = 681
		PacketTX(10,340,0); // 100º = 340
		PacketTX(5,852,0);
		PacketTX(11,852,0);
		PacketTX(6,852,0);
		PacketTX(12,852,0);
		_delay_ms(10000);
		//Step 1 : 몸체의 오른쪽 뒷다리와 앞다리를 모은다
		PORTA = 0xBF; // 확인용 LED
		StayLeft();
		PacketTX(4,681,0);
		PacketTX(5,852,0);
		PacketTX(6,852,0);
		PacketTX(12,681,0); // 250->200º = 681
		PacketTX(11,681,0); // 250->200º = 681
		PacketTX(10,443,0); // 100->130º = 443
		PacketTX(12,613,0); // 200->180º = 613
		PacketTX(11,613,0); // 200->180º = 613
		PacketTX(10,545,0); // 130->160º = 545
		_delay_ms(10000);
		//Step_2 : 몸체의 오른쪽 뒷다리가 뒤로 후진
		PORTA = 0xDF; // 확인용 LED
		PacketTX(7,681,0);
		PacketTX(10,511,0);
		PacketTX(8,170,0);
		PacketTX(11,852,0);
		PacketTX(9,170,0);
		PacketTX(12,852,0);
		PacketTX(3,340,0); // 50->100º
		PacketTX(2,340,0); // 50->100º
		PacketTX(1,545,0); // 150->160º = 545
		PacketTX(4,511,0);
		PacketTX(2,409,0); // 100->120º
		PacketTX(5,852,0);
		PacketTX(3,409,0); // 100->120º = 409
		PacketTX(6,852,0);
		_delay_ms(10000);
		//Step_3 : 다리가 후진한 거리만큼 몸체도 뒤로 후진
		PORTA = 0xEF; // 확인용 LED
		StayRight();
		PacketTX(1,340,0);
		PacketTX(7,681,0);
		PacketTX(2,170,0);
		PacketTX(8,170,0);
		PacketTX(3,170,0);
		PacketTX(9,170,0);
		_delay_ms(10000);
		//Step_4 : 몸체의 왼쪽 뒷다리와 앞다리를 모은다
		PORTA = 0xF7; // 확인용 LED
		StayRight();
		PacketTX(1,340,0);
		PacketTX(2,170,0);
		PacketTX(3,170,0);
		PacketTX(9,340,0); // 50->100º = 340
		PacketTX(8,340,0); // 50->100º = 340
		PacketTX(7,579,0); // 200->170º = 579
		PacketTX(9,409,0); // 100->120º = 409
		PacketTX(8,409,0); // 100->120º = 409
		PacketTX(7,477,0); // 170->140º = 477
		_delay_ms(10000);
		//Step_5 : 몸체의 왼쪽 뒷다리가 뒤로 후진
		PORTA = 0xFB; // 확인용 LED
		PacketTX(10,340,0);
		PacketTX(7,511,0);
		PacketTX(11,852,0);
		PacketTX(8,170,0);
		PacketTX(12,852,0);
		PacketTX(9,170,0);
		PacketTX(6,681,0); // 250->200º = 681
		PacketTX(5,681,0);
		PacketTX(4,478,0); // 150->140º = 478
		PacketTX(1,511,0);
		PacketTX(5,613,0); // 200->180º = 613
		PacketTX(2,170,0);
		PacketTX(6,613,0); // 200->180º = 613
		PacketTX(3,170,0);
		_delay_ms(10000);
		//Step 6 : 다리가 후진한 거리만큼 몸체도 뒤로 후진
		PORTA = 0xFD; // 확인용 LED
		StayLeft();
		PacketTX(4,681,0); // 200º = 681
		PacketTX(10,340,0); // 100º = 340
		PacketTX(5,852,0);
		PacketTX(11,852,0);
		PacketTX(6,852,0);
		PacketTX(12,852,0);
		_delay_ms(10000);
		//Step 7 : 몸체의 오른쪽 뒷다리와 앞다리를 모은다
}
void MoveRight(){
		PORTA = 0xDF; // 확인용 LED
		StayRight();
		PacketTX(1,511,0);
		PacketTX(2,170,0);
		PacketTX(3,170,0);
		PacketTX(9,255,0); // 50->75º = 255
		PacketTX(8,170,0);
		PacketTX(7,852,0); // 150->250º = 852
		PacketTX(9,170,0); // 75->50º = 170
		_delay_ms(10000);
		//Step_1 : 몸체의 왼쪽 뒷다리를 옆으로 회전한다
		PORTA = 0xEF; // 확인용 LED
		StayRight();
		PacketTX(7,681,0); // 200º = 681
		PacketTX(8,170,0);
		PacketTX(9,170,0); // 50º = 170
		PacketTX(3,255,0); // 50->75º = 255
		PacketTX(2,170,0);
		PacketTX(1,852,0); // 150->250º = 852
		PacketTX(3,170,0); // 75->50º = 170
		_delay_ms(10000);
		//Step_2 : 몸체의 왼쪽 앞다리를 옆으로 회전한다
		PORTA = 0xF7; // 확인용 LED
		PacketTX(1,681,0); // 200º = 681
		PacketTX(10,511,0);
		PacketTX(2,170,0);
		PacketTX(11,852,0);
		PacketTX(3,170,0); // 50º = 170
		PacketTX(12,852,0);
		PacketTX(7,681,0); // 200º = 681
		PacketTX(8,170,0);
		PacketTX(9,170,0); // 50º = 170
		PacketTX(6,768,0); // 250->225º = 768
		PacketTX(5,852,0);
		PacketTX(4,852,0); // 150->250º = 852
		PacketTX(6,852,0); // 225->250º = 852
		_delay_ms(10000);
		//Step_3 : 몸체의 오른쪽 앞다리를 옆으로 회전한다
		PORTA = 0xFB; // 확인용 LED
		PacketTX(4,681,0); // 200º = 681
		PacketTX(7,681,0); // 200º = 681
		PacketTX(5,852,0);
		PacketTX(8,170,0);
		PacketTX(6,852,0); // 250º = 852
		PacketTX(9,170,0); // 50º = 170
		PacketTX(1,681,0); // 200º = 681
		PacketTX(2,170,0);
		PacketTX(3,170,0); // 50º = 170
		PacketTX(12,768,0); // 250->225º = 768
		PacketTX(11,852,0);
		PacketTX(10,852,0); // 150->250º = 852
		PacketTX(12,852,0); // 225->250º = 852
		_delay_ms(10000);
		//Step_4 : 몸체의 오른쪽 뒷다리를 옆으로 회전한다
		PORTA = 0xC3; // 확인용 LED
		Stay();
		_delay_ms(10000);
		//Step_5 : 옆으로 50º씩 회전한 다리를 다시 원상태로
		//이후 다시 Step_1으로 되돌아가서 반복수행
}
void MoveLeft(){
		PORTA = 0xFB; // 확인용  LED
		StayLeft();
		PacketTX(4,511,0); // 150º = 511
		PacketTX(5,852,0);
		PacketTX(6,852,0);
		PacketTX(12,768,0); // 250->225º = 768
		PacketTX(11,852,0);
		PacketTX(10,170,0); // 150->50º = 170
		PacketTX(12,852,0); // 225->250º = 852
		_delay_ms(10000);
		//Step_1 : 몸체의 오른쪽 뒷다리를 옆으로 회전한다
		PORTA = 0xF7;
		StayLeft();
		PacketTX(10,340,0); // 100º = 340
		PacketTX(11,852,0);
		PacketTX(12,852,0); // 225->250º = 852
		PacketTX(6,768,0); // 250->225º = 768
		PacketTX(5,852,0);
		PacketTX(4,170,0); // 150->50º = 170
		PacketTX(6,852,0); // 225->250º = 852
		_delay_ms(10000);
		//Step_2 : 몸체의 오른쪽 앞다리를 옆으로 회전한다
		PORTA = 0xEF; // 확인용 LED
		PacketTX(4,340,0);
		PacketTX(7,0x1ff,0);
		PacketTX(5,852,0); //반대쪽 끝값 = 250º = 852
		PacketTX(8,170,0);
		PacketTX(6,852,0);
		PacketTX(9,170,0);
		PacketTX(10,340,0); // 100º = 340
		PacketTX(11,852,0);
		PacketTX(12,852,0);
		PacketTX(3,255,0); // 50->75º = 255
		PacketTX(2,170,0);
		PacketTX(1,170,0); // 150->50º = 170
		PacketTX(3,170,0); // 75->50º = 170
		_delay_ms(10000);
		//Step_3 : 몸체의 왼쪽 앞다리를 옆으로 회전한다
		PORTA = 0xBF; // 확인용 LED
		PacketTX(4,340,0);
		PacketTX(10,340,0); // 100º = 340
		PacketTX(5,852,0);
		PacketTX(11,852,0);
		PacketTX(6,852,0);
		PacketTX(12,852,0);
		PacketTX(1,340,0);
		PacketTX(2,170,0);
		PacketTX(3,170,0); // 50º = 170
		PacketTX(9,255,0); // 50->75º = 255
		PacketTX(8,170,0);
		PacketTX(7,170,0); // 150->50º = 170
		PacketTX(9,170,0); // 75->50º = 170
		_delay_ms(10000);
		//Step_4 : 몸체의 오른쪽 뒷다리를 옆으로 회전한다
		PORTA = 0xC3; // 확인용 LED
		Stay();
		_delay_ms(10000);
		//Step_5 : 옆으로 50º씩 회전한 다리를 다시 원상태로
		//이후 다시 Step_1으로 되돌아가서 반복수행
}
void Hello(){
	Stay();
	_delay_ms(10000);
	//Step_1 : 몸체를 중앙에 모은다
	PORTA = 0xF7; // 확인용 LED
	PacketTX(1,579,0); // 150->170º = 579
	PacketTX(10,443,0); // 150->130º = 443 
	PacketTX(2,170,0);
	PacketTX(11,852,0);
	PacketTX(3,170,0);
	PacketTX(12,852,0);
	PacketTX(7,511,0);
	PacketTX(8,238,0); // 50->70º = 238
	PacketTX(9,238,0); // 50->70º = 238
	PacketTX(4,511,0);
	PacketTX(5,852,0);
	PacketTX(6,511,0);
	_delay_ms(2500);
	//Step_2 : 로봇 전방기준 몸을 남서쪽으로 기울이고 오른쪽 앞다리를 위로 든다
	PORTA = 0xEF; // 확인용 LED
	PacketTX(1,579,0); // 150->170º = 579
	PacketTX(10,443,0); // 150->130º = 443
	PacketTX(2,170,0);
	PacketTX(11,852,0);
	PacketTX(3,170,0);
	PacketTX(12,852,0);
	PacketTX(7,511,0);
	PacketTX(8,238,0); // 50->70º = 238
	PacketTX(9,238,0); // 50->70º = 238
	PacketTX(4,511,0);
	PacketTX(5,852,0);
	PacketTX(6,579,0); // 150->170º = 579
	_delay_ms(2500);
	//Step_3 : 오른쪽 앞다리를 살짝 위로 든다
	PORTA = 0xF7; // 확인용 LED
	PacketTX(1,579,0); // 150->170º = 579
	PacketTX(10,443,0); // 150->130º = 443
	PacketTX(2,170,0);
	PacketTX(11,852,0);
	PacketTX(3,170,0);
	PacketTX(12,852,0);
	PacketTX(7,511,0);
	PacketTX(8,238,0); // 50->70º = 238
	PacketTX(9,238,0); // 50->70º = 238
	PacketTX(4,511,0);
	PacketTX(5,852,0);
	PacketTX(6,511,0);
	_delay_ms(2500);
	//Step_4 : 오른쪽 앞다리를 살짝 아래로 내린다
	PORTA = 0xEF; // 확인용 LED
	PacketTX(1,579,0); // 150->170º = 579
	PacketTX(10,443,0); // 150->130º = 443
	PacketTX(2,170,0);
	PacketTX(11,852,0);
	PacketTX(3,170,0);
	PacketTX(12,852,0);
	PacketTX(7,511,0);
	PacketTX(8,238,0); // 50->70º = 238
	PacketTX(9,238,0); // 50->70º = 238
	PacketTX(4,511,0);
	PacketTX(5,852,0);
	PacketTX(6,579,0); // 150->170º = 579
	_delay_ms(5000);
	//Step_5 : 오른쪽 앞다리를 살짝 위로 든다
	Stay();
	_delay_ms(10000);
	//Step_6 : 다시 원상태로 복귀
}