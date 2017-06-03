#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "Serial.h"
#include "MultiWii.h"

static volatile uint8_t serialHeadRX[UART_NUMBER], serialTailRX[UART_NUMBER];
static uint8_t serialBufferRX[RX_BUFFER_SIZE][UART_NUMBER];
static volatile uint8_t serialHeadTX[UART_NUMBER], serialTailTX[UART_NUMBER];
static uint8_t serialBufferTX[TX_BUFFER_SIZE][UART_NUMBER];
//위 버퍼들은 이 문서에서만 편집할 수 있다. static(정적 변수)

// *******************************************************
// For Teensy 2.0, these function emulate the API used for ProMicro
// it cant have the same name as in the arduino API because it wont compile for the promini (eaven if it will be not compiled)
// *******************************************************
#if defined(TEENSY20)
unsigned char T_USB_Available() {
	int n = Serial.available();
	if (n > 255) n = 255;
	return n;
}
#endif

// *******************************************************
// Interrupt driven UART transmitter - using a ring buffer
// *******************************************************


#if defined(PROMINI) || defined(MEGA)
#if defined(PROMINI)
ISR(USART_UDRE_vect) {  // Serial 0 on a PROMINI
#endif
#if defined(MEGA)
ISR(USART0_UDRE_vect) { // Serial 0 on a MEGA 
								//-> UDRIEn이 High가 되면, 인터럽트 서브루틴이 실행되므로 원형 버퍼에서 TX 테일이 가리키는 데이터를 전송
	#endif
	uint8_t t = serialTailTX[0]; //TX 테일을 가져와서
	if (serialHeadTX[0] != t) { //헤드와 테일이 다르면(전송하기 위해 쌓은 데이터들이 있다면)
		if (++t >= TX_BUFFER_SIZE) t = 0; //버퍼 크기를 벗어나면 테일 초기화
		UDR0 = serialBufferTX[t][0];  // Transmit next byte in the ring -> ring: 원형 버퍼상의 데이터를 UDR0에 write함. UDR0에 write하면 전송준비됨.
		serialTailTX[0] = t; //전송하고 1칸 이동한 테일값을 저장
	}
	if (t == serialHeadTX[0]) UCSR0B &= ~(1 << UDRIE0); // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
																	//헤드와 테일이 같으면 전송이 완료된거고, 전송이완료되었다면 UDRIEn을 Clear해서 인터럽트 서브루틴에 들어가지 않도록 함.
																//UDRIEn은 플래그가 아닌듯. 그래서 수동으로 내려줘야 하는 듯
}
#endif
#if defined(MEGA) || defined(PROMICRO)
ISR(USART1_UDRE_vect) { // Serial 1 on a MEGA or on a PROMICRO
	uint8_t t = serialTailTX[1];
	if (serialHeadTX[1] != t) {
		if (++t >= TX_BUFFER_SIZE) t = 0;
		UDR1 = serialBufferTX[t][1];  // Transmit next byte in the ring
		serialTailTX[1] = t;
	}
	if (t == serialHeadTX[1]) UCSR1B &= ~(1 << UDRIE1);
}
#endif
#if defined(MEGA)
ISR(USART2_UDRE_vect) { // Serial 2 on a MEGA
	uint8_t t = serialTailTX[2];
	if (serialHeadTX[2] != t) {
		if (++t >= TX_BUFFER_SIZE) t = 0;
		UDR2 = serialBufferTX[t][2];
		serialTailTX[2] = t;
	}
	if (t == serialHeadTX[2]) UCSR2B &= ~(1 << UDRIE2);
}
ISR(USART3_UDRE_vect) { // Serial 3 on a MEGA
	uint8_t t = serialTailTX[3];
	if (serialHeadTX[3] != t) {
		if (++t >= TX_BUFFER_SIZE) t = 0;
		UDR3 = serialBufferTX[t][3];
		serialTailTX[3] = t;
	}
	if (t == serialHeadTX[3]) UCSR3B &= ~(1 << UDRIE3);
}
#endif

void UartSendData(uint8_t port) { //데이터를 전송하도록 명령(Interrupt Enable)
	#if defined(PROMINI)
	UCSR0B |= (1 << UDRIE0);
	#endif
	#if defined(PROMICRO)
	switch (port) {
	case 0:
		while (serialHeadTX[0] != serialTailTX[0]) {
			if (++serialTailTX[0] >= TX_BUFFER_SIZE) serialTailTX[0] = 0;
			#if !defined(TEENSY20)
			USB_Send(USB_CDC_TX, serialBufferTX[serialTailTX[0]], 1);
			#else
			Serial.write(serialBufferTX[serialTailTX[0]], 1);
			#endif
		}
		break;
	case 1: UCSR1B |= (1 << UDRIE1); break;
	}
	#endif
	#if defined(MEGA)
	switch (port) {
	case 0: UCSR0B |= (1 << UDRIE0); break;
	case 1: UCSR1B |= (1 << UDRIE1); break;
	case 2: UCSR2B |= (1 << UDRIE2); break;
	case 3: UCSR3B |= (1 << UDRIE3); break;
	}
	#endif
}

#if defined(GPS_SERIAL)
bool SerialTXfree(uint8_t port) {
	return (serialHeadTX[port] == serialTailTX[port]); //전송할 데이터가 있는지 없는지 확인. 전송할게 없으면 false, 있으면 true
}
#endif

void SerialOpen(uint8_t port, uint32_t baud) {
	uint8_t h = ((F_CPU / 4 / baud - 1) / 2) >> 8;
	uint8_t l = ((F_CPU / 4 / baud - 1) / 2);
	switch (port) {
		#if defined(PROMINI)
	case 0: UCSR0A = (1 << U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); break;
		#endif
		#if defined(PROMICRO)
		#if (ARDUINO >= 100) && !defined(TEENSY20)
	case 0: UDIEN &= ~(1 << SOFE); break;// disable the USB frame interrupt of arduino (it causes strong jitter and we dont need it)
		#endif
	case 1: UCSR1A = (1 << U2X1); UBRR1H = h; UBRR1L = l; UCSR1B |= (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1); break;
		#endif
		#if defined(MEGA)
	case 0: UCSR0A = (1 << U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); break;
	case 1: UCSR1A = (1 << U2X1); UBRR1H = h; UBRR1L = l; UCSR1B |= (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1); break;
	case 2: UCSR2A = (1 << U2X2); UBRR2H = h; UBRR2L = l; UCSR2B |= (1 << RXEN2) | (1 << TXEN2) | (1 << RXCIE2); break;
	case 3: UCSR3A = (1 << U2X3); UBRR3H = h; UBRR3L = l; UCSR3B |= (1 << RXEN3) | (1 << TXEN3) | (1 << RXCIE3); break;
		#endif
	}
}

void SerialEnd(uint8_t port) {
	switch (port) {
		#if defined(PROMINI)
	case 0: UCSR0B &= ~((1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) | (1 << UDRIE0)); break;
		#endif
		#if defined(PROMICRO)
	case 1: UCSR1B &= ~((1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1) | (1 << UDRIE1)); break;
		#endif
		#if defined(MEGA)
	case 0: UCSR0B &= ~((1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) | (1 << UDRIE0)); break;
	case 1: UCSR1B &= ~((1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1) | (1 << UDRIE1)); break;
	case 2: UCSR2B &= ~((1 << RXEN2) | (1 << TXEN2) | (1 << RXCIE2) | (1 << UDRIE2)); break;
	case 3: UCSR3B &= ~((1 << RXEN3) | (1 << TXEN3) | (1 << RXCIE3) | (1 << UDRIE3)); break;
		#endif
	}
}

// we don't care about ring buffer overflow (head->tail) to avoid a test condition : data is lost anyway if it happens 
//인터럽트 서브루틴에 들어갈 함수 -> overflow를 고려하지 않는다고 한다.
void store_uart_in_buf(uint8_t data, uint8_t portnum) {
	#if defined(SPEKTRUM)
	if (portnum == SPEK_SERIAL_PORT) {
	#endif
	#if defined(SBUS) 
	if (portnum == SBUS_SERIAL_PORT) {
		#endif
		#if defined(SPEKTRUM) || defined(SBUS)
		if (!spekFrameFlags) {
			sei();
			uint32_t spekTimeNow = (timer0_overflow_count << 8) * (64 / clockCyclesPerMicrosecond()); //Move timer0_overflow_count into registers so we don't touch a volatile twice
			uint32_t spekInterval = spekTimeNow - spekTimeLast;                                       //timer0_overflow_count will be slightly off because of the way the Arduino core timer interrupt handler works; that is acceptable for this use. Using the core variable avoids an expensive call to millis() or micros()
			spekTimeLast = spekTimeNow;
			if (spekInterval > 5000) {  //Potential start of a Spektrum frame, they arrive every 11 or every 22 ms. Mark it, and clear the buffer. 
				serialTailRX[portnum] = 0;
				serialHeadRX[portnum] = 0;
				spekFrameFlags = 0x01;
			}
			cli();
		}
	}
	#endif

	uint8_t h = serialHeadRX[portnum]; //버퍼 레지스터의 값을 쌓을 자리를 선택하는 indicator
	serialBufferRX[h++][portnum] = data; //버퍼 레지스터의 값을 가져와서 쌓음
	if (h >= RX_BUFFER_SIZE) h = 0; //버퍼 사이즈를 벗어나면 indicator 초기화
	serialHeadRX[portnum] = h;
}

//RX 인터럽트 서브루틴 정의
#if defined(PROMINI)
ISR(USART_RX_vect) { store_uart_in_buf(UDR0, 0); }
#endif
#if defined(PROMICRO)
ISR(USART1_RX_vect) { store_uart_in_buf(UDR1, 1); }
#endif
#if defined(MEGA)
ISR(USART0_RX_vect) { store_uart_in_buf(UDR0, 0); } //받자마자 원형 버퍼에 쌓음
ISR(USART1_RX_vect) { store_uart_in_buf(UDR1, 1); }
ISR(USART2_RX_vect) { store_uart_in_buf(UDR2, 2); }
ISR(USART3_RX_vect) { store_uart_in_buf(UDR3, 3); }
#endif

uint8_t SerialRead(uint8_t port) {
	#if defined(PROMICRO)
	#if defined(TEENSY20)
	if (port == 0) return Serial.read();
	#else
	#if (ARDUINO >= 100)
	if (port == 0) USB_Flush(USB_CDC_TX);
	#endif
	if (port == 0) return USB_Recv(USB_CDC_RX);
	#endif
	#endif
	uint8_t t = serialTailRX[port]; //헤드, 테일 -> 원형 버퍼: 헤드는 사용자가 데이터를 집어넣는 곳, 테일은 사용자가 데이터를 찾는 곳
	uint8_t c = serialBufferRX[t][port];
	if (serialHeadRX[port] != t) { //헤드가 테일과 다르면
		if (++t >= RX_BUFFER_SIZE) t = 0; //테일을 1 증가시키고 버퍼 크기를 넘어가기 직전에 초기화(사실상 헤드 초기화랑 상황은 똑같음
																						//이 말은 즉슨 헤드가 쌓은 만큼의 데이터를 테일이 읽을 수 있다는 얘기. 읽어도 읽어도 결과가 같다면
																						//그건 같은 입력이 되었던지, 테일과 헤드가 같아졌던지 한 것임.
		serialTailRX[port] = t;
	}
	return c; //테일과 헤드가 같아지면 그냥 동시에 가리키는 공간의 데이터 출력
}

#if defined(SPEKTRUM)
uint8_t SerialPeek(uint8_t port) {
	uint8_t c = serialBufferRX[serialTailRX[port]][port];
	if ((serialHeadRX[port] != serialTailRX[port])) return c; else return 0;
}
#endif

uint8_t SerialAvailable(uint8_t port) {
	#if defined(PROMICRO)
	#if !defined(TEENSY20)
	if (port == 0) return USB_Available(USB_CDC_RX);
	#else
	if (port == 0) return T_USB_Available();
	#endif
	#endif
	return ((uint8_t)(serialHeadRX[port] - serialTailRX[port])) % RX_BUFFER_SIZE;
	//테일과 헤드가 같다면 0이, 그 외엔 정수 반환 -> 출력 가능한 문자 수를 알 수 있음
}

uint8_t SerialUsedTXBuff(uint8_t port) {
	return ((uint8_t)(serialHeadTX[port] - serialTailTX[port])) % TX_BUFFER_SIZE;
	//테일과 헤드가 같다면 0이, 그 외엔 정수 반환 -> 전송 가능한 문자 수를 알 수 있음???
}

void SerialSerialize(uint8_t port, uint8_t a) {
	uint8_t t = serialHeadTX[port];
	if (++t >= TX_BUFFER_SIZE) t = 0;
	serialBufferTX[t][port] = a;
	serialHeadTX[port] = t;
}//전송은 하지 말고, TX버퍼를 편집(원형 버퍼에 순서대로 문자 a를 하나씩 써넣음)

void SerialWrite(uint8_t port, uint8_t c) {
	SerialSerialize(port, c); UartSendData(port); //TX버퍼를 편집하면서 최종적으로 전송
}

