#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "EEPROM.h"
#include "LCD.h"
#include "Output.h"
#include "GPS.h"
#include "MultiWii.h"
#include "Serial.h"
#include "Protocol.h"
#include "RX.h"

/************************************** MultiWii Serial Protocol *******************************************************/
// Multiwii Serial Protocol 0 
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
//Multiwii Serial Protocol(MSP) -> 아래에 해당하는 데이터를 시리얼로 3DR Radio에 보내면, PC가 Serial로 받는다. MultiwiiConf.pde참조
//MultiwiiConf.pde에는 Serial 데이터를 받아서 ...
#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONF           120   //out message         Servo settings

#define MSP_NAV_STATUS			 121   //out message	     Returns navigation status
#define MSP_NAV_CONFIG			 122   //out message		 Returns navigation parameters


#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed    //depreciated 
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212   //in message          Servo settings
#define MSP_SET_MOTOR            214   //in message          PropBalance function

#define MSP_SET_NAV_CONFIG       215   //in message			 Sets nav config parameters - write to the eeprom  

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

#ifdef DEBUGMSG
#define DEBUG_MSG_BUFFER_SIZE 128
static char debug_buf[DEBUG_MSG_BUFFER_SIZE];
static uint8_t head_debug;
static uint8_t tail_debug;
static uint8_t debugmsg_available();
static void debugmsg_serialize(uint8_t l);
#endif

static uint8_t CURRENTPORT = 0;

#define INBUF_SIZE 64 //입력 버퍼(INBUF)의 크기
static uint8_t inBuf[INBUF_SIZE][UART_NUMBER]; //입력 버퍼의 크기만큼, 각 포트마다 버
static uint8_t checksum[UART_NUMBER]; //패킷의 checksum값
static uint8_t indRX[UART_NUMBER]; //입력 버퍼의 indicator 정도의 역할
static uint8_t cmdMSP[UART_NUMBER]; //패킷에서 읽어들인 명령값

void evaluateOtherData(uint8_t sr);
#ifndef SUPPRESS_ALL_SERIAL_MSP
void evaluateCommand();
#endif

#define BIND_CAPABLE 0;  //Used for Spektrum today; can be used in the future for any RX type that needs a bind and has a MultiWii module. 
#if defined(SPEK_BIND)
#define BIND_CAPABLE 1;
#endif
// Capability is bit flags; next defines should be 2, 4, 8...

const uint32_t capability = 0 + BIND_CAPABLE;


uint8_t read8() { //indRX가 가리키는 곳을 8bit 데이터 읽기
	return inBuf[indRX[CURRENTPORT]++][CURRENTPORT] & 0xff; //inBuf에서 indRX가 가리키는 곳의 값을 0~255(8bit)로 깎아내서 반환(깎는건 무의미해보이긴 하는데)
	//읽어들이고 나면 indRX값 +1 -> indicator가 한 칸 다음 위치를 가리킴
}
uint16_t read16() { //indRX가 가리키는 연속된 두 개의 Byte를 읽어서 반환
	uint16_t t = read8(); //indRX가 가리키는 곳의 8bit데이터를 읽고 t에 저장
	t += (uint16_t)read8() << 8; //indRX가 가리키는 다음 inBuf를 하나 더 읽어서 t의 다음 byte에 저장
	return t;
}
uint32_t read32() {//indRX가 가리키는 연속된 네 개의 Byte를 읽어서 반환
	uint32_t t = read16();
	t += (uint32_t)read16() << 16;
	return t;
}

void serialize8(uint8_t a) { //8bit 데이터를 SerialSerialize(전송 없이 TX버퍼에 쌓음. Serial.cpp 참조)
	SerialSerialize(CURRENTPORT, a);
	checksum[CURRENTPORT] ^= a;
}
void serialize16(int16_t a) { //16bit 데이터를 전송 없이 TX버퍼에 쌓음
	serialize8((a)& 0xFF);
	serialize8((a >> 8) & 0xFF);
}
void serialize32(uint32_t a) { //32bit 데이터를 전송 없이 TX버퍼에 쌓음
	serialize8((a)& 0xFF);
	serialize8((a >> 8) & 0xFF);
	serialize8((a >> 16) & 0xFF);
	serialize8((a >> 24) & 0xFF);
}

void headSerialResponse(uint8_t err, uint8_t s) { //MSP에서 PC로 메시지를 보내는 코드 -> MSP를 그대로 전송
	serialize8('$');
	serialize8('M');
	serialize8(err ? '!' : '>');
	checksum[CURRENTPORT] = 0; // start calculating a new checksum
	serialize8(s); //s-> 메시지 크기
	serialize8(cmdMSP[CURRENTPORT]); //-> cmdMSP: 명령 데이터
} //-> 어쨋거나 조금 편집하면 writing하는데 써먹을 수 있겠음.

void headSerialReply(uint8_t s) { //"$M>"+s+cmdMSP, 메시지 앞단에 사용
	headSerialResponse(0, s);
}

void inline headSerialError(uint8_t s) { //"$M!"+s+cmdMSP, 메시지 앞단에 사용 -> '!'에 대한 내용은 MSP 문서에 나와있진 않던데?
	headSerialResponse(1, s);
}

void tailSerialReply() { //TX버퍼 말단에 checksum 채우고 완성된 메시지 전송(메시지를 만들 때 맨 마지막에 사용하는 함수
	serialize8(checksum[CURRENTPORT]); UartSendData(CURRENTPORT);
}

void serializeNames(PGM_P s) { //PGM_P: Program memorry Pointer인 것으로 보임. string 따위를 s에 받아서, strlen_P(s)로 문자열의 길이를 구한 다음,
							   //c에 s의 주소값을 저장해서 첫 번째 주소에서 읽어들인 값의 크기만큼 읽어들인 내용을 TX Buffer에 쌓는다.
							   //string따위의 첫 번째 주소에서 읽어들인 값의 크기가 string의 전체 길이를 알려주나???
	headSerialReply(strlen_P(s)); //문자열 s의 길이를 strlen_P()로 구해서, 메시지의 머리부분을 만듦
	for (PGM_P c = s; pgm_read_byte(c); c++) {
		serialize8(pgm_read_byte(c));
	}
}

void serialCom() {
	uint8_t c, n;
	static uint8_t offset[UART_NUMBER];
	static uint8_t dataSize[UART_NUMBER];
	static enum _serial_state { //열거형 -> _serial_state라는 열거형을 선언, c_state[0]~c_state[3]은 IDLE~HEADER_CMD만 대입할 수 있음
              //state machine 설계 시 매우 유용한 스킬로 보임.
		IDLE,
		HEADER_START,
		HEADER_M,
		HEADER_ARROW,
		HEADER_SIZE,
		HEADER_CMD,
	} c_state[UART_NUMBER];// = IDLE; c_state[UART_NUMBER]는 열거형 _serial_state의 c_state라는 크기 4(=UART_NUMBER)의 1차원 배열을 선언

	for (n = 0; n < UART_NUMBER; n++) {
		#if !defined(PROMINI)
		CURRENTPORT = n; //UART_NUMBER가 4이므로, 포트0부터 포트3까지 loop
		#endif
		#define GPS_COND
		#if defined(GPS_SERIAL)
		#if defined(GPS_PROMINI)
		#define GPS_COND       
		#else
		#undef GPS_COND
		#define GPS_COND  && (GPS_SERIAL != CURRENTPORT)
		#endif      
		#endif
		#define SPEK_COND
		#if defined(SPEKTRUM) && (UART_NUMBER > 1)
		#define SPEK_COND && (SPEK_SERIAL_PORT != CURRENTPORT)
		#endif
		#define SBUS_COND
		#if defined(SBUS) && (UART_NUMBER > 1)
		#define SBUS_COND && (SBUS_SERIAL_PORT != CURRENTPORT)
		#endif
		uint8_t cc = SerialAvailable(CURRENTPORT); //현재 포트(CURRENT PORT)에 들어온 문자의 수를 cc에 저장: character count 정도로 이해
		while (cc-- GPS_COND SPEK_COND SBUS_COND) { //현재 GPS_COND만 정의되어 있으므로 cc값이 참이고 CURRENTPORT값이 2가 아닐 때 loop(GPS_COND 정의 참조)
														//cc(character count)가 0이 될 때까지 loop를 돌림 -> phasing정도의 개념
			uint8_t bytesTXBuff = SerialUsedTXBuff(CURRENTPORT); // indicates the number of occupied bytes in TX buffer??? : Serial.cpp 참조, 전송 가능한 문자 수를???
																//byteTXBuff에 저장???
			if (bytesTXBuff > TX_BUFFER_SIZE - 50) return; // ensure there is enough free TX buffer to go further (50 bytes margin)
												//전송할 문자의 양이 TX_BUFFER_SIZE(128)-50(=78)보다 적어야 통과 -> 50의 전송을 위한 여분이 필요하다는 의미인 듯???
			c = SerialRead(CURRENTPORT); //Tail 문자 하나 읽기
			#ifdef SUPPRESS_ALL_SERIAL_MSP
			// no MSP handling, so go directly
			evaluateOtherData(c);
			#else
			// regular data handling to detect and handle MSP and other data 읽어들인 문자 c에 대한 state machine(switch-case문이나 마찬가지)
			if (c_state[CURRENTPORT] == IDLE) { //IDLE state일 경우
				c_state[CURRENTPORT] = (c == '$') ? HEADER_START : IDLE; //읽어들인 문자가 $이면 HEADER_START, 아니면 IDLE
				if (c_state[CURRENTPORT] == IDLE) evaluateOtherData(c); // evaluate all other incoming serial data
			}
			else if (c_state[CURRENTPORT] == HEADER_START) { //HEADER_START state일 경우
				c_state[CURRENTPORT] = (c == 'M') ? HEADER_M : IDLE; //읽어들인 문자가 M이면 HEADER_M, 아니면 IDLE
			}
			else if (c_state[CURRENTPORT] == HEADER_M) {//HEADER_M state일 경우
				c_state[CURRENTPORT] = (c == '<') ? HEADER_ARROW : IDLE; //읽어들인 문자가 <이면 HEADER_ARROW, 아니면 IDLE
			}										//MSP wiki를 참조하면, <는 MWC를 향해서, >는 MWC로부터 라고 하였으므로, <는 드론 방향, >는 반대 방향인 것으로
                          //사료된다. >는 processing 2로 작성된 MultiWiiConf.pde 참조(확인 가능)
                          
			else if (c_state[CURRENTPORT] == HEADER_ARROW) {//HEADER_ARROW state일 경우
				if (c > INBUF_SIZE) {  // now we are expecting the payload size: INBUF_SIZE(=64)보다 큰 값이 들어올 경우, 
															//정해진 자료형의 크기(dataSize[CURRENTPORT]) 값의 범위를 한정
					c_state[CURRENTPORT] = IDLE;//IDLE state로 복귀
					continue; //아래 내용 모두 무시하고 while문을 다시 실행(다시 한 문자를 읽어들임. $문자가 읽힐 때까지 다시 기다림)
				}//통과했으면, 아래로
				dataSize[CURRENTPORT] = c; //읽어들인 값을 dataSize에 대입
				offset[CURRENTPORT] = 0; //offset값 초기화
				checksum[CURRENTPORT] = 0; //chkecksum값 초기화
				indRX[CURRENTPORT] = 0; //indRX값 초기화
				checksum[CURRENTPORT] ^= c; //dataSize의 XOR값을 checksum에 대입(MSP의 checksum 규칙은 dataSize의 XOR임)
				c_state[CURRENTPORT] = HEADER_SIZE;  // the command is to follow: 대입이 끝났으면 HEADER_SIZE state로
			}
			else if (c_state[CURRENTPORT] == HEADER_SIZE) { //HEADER_SIZE state일 경우
				cmdMSP[CURRENTPORT] = c; //읽어들인 값을 cmdMSP에 대입 -> 명령 데이터를 읽어들임
				checksum[CURRENTPORT] ^= c; //cmdMSP의 XOR값을 checksum에 대입
				c_state[CURRENTPORT] = HEADER_CMD; //데이터 사이즈 파악이 끝났으면 HEADER_CMD state로
			}
			else if (c_state[CURRENTPORT] == HEADER_CMD && offset[CURRENTPORT] < dataSize[CURRENTPORT]) {
				checksum[CURRENTPORT] ^= c;
				inBuf[offset[CURRENTPORT]++][CURRENTPORT] = c;
			}
			else if (c_state[CURRENTPORT] == HEADER_CMD && offset[CURRENTPORT] >= dataSize[CURRENTPORT]) {
				if (checksum[CURRENTPORT] == c) {  // compare calculated and transferred checksum
					//crc = XOR of <size>, <command> and each data byte into a zero'ed sum -> Multiwii
					evaluateCommand();  // we got a valid packet, evaluate it 유효한 패킷임을 checksum을 통해 확인하였으므로 커맨드 연산 실행
										//checksum이 유효하지 않으면, evaluate하지 않는다!!
				}
				c_state[CURRENTPORT] = IDLE;
				cc = 0; // no more than one MSP per port and per cycle
			}
			#endif // SUPPRESS_ALL_SERIAL_MSP
		}
	}
} //들어온 데이터를 Phasing하는 정도의 기능을 수행하는 듯.

//응답하면서 요청데이터를 보내기 위해 준비
void  s_struct(uint8_t *cb, uint8_t siz) { //이 Protocol.cpp 문서의 내용은 PC에서 온 요청에 대한 응답을 보내기 위한 코드로 구성되어 있으므로,
										   //cmdMSP라는, request 메시지에서 읽어들인 명령 값을 전역 변수로 이용한다.
										   //써먹으려면 이 부분의 코드 수정이 필요함.
										   //cb는 응답으로 보낼 데이터 세트에 해당, siz는 그 데이터의 크기
										   //이 함수 s_struct는 시리얼 메시지를 '구성'(struct)하는 명령. tail(checksum)을 제외하고 TX버퍼에 쌓는다.
	headSerialReply(siz);
	while (siz--) serialize8(*cb++);
}

//응답하면서 버퍼데이터를 읽어들임
void s_struct_w(uint8_t *cb, uint8_t siz) {//s_struct와는 달리 데이터 세트 cb에 데이터를 읽어서(read8()) siz만큼 저장하고 메시지 내용의 길이가 0인 명령의 응답을 구성struct한다.
	headSerialReply(0);
	while (siz--) *cb++ = read8();
}

#ifndef SUPPRESS_ALL_SERIAL_MSP
void evaluateCommand() { //Flight Controller에 들어가는 명령은 요청이면 데이터를 전송하고, 입력이면 그에 해당하는 행동을 해야 함.
	uint32_t tmp = 0;

	switch (cmdMSP[CURRENTPORT]) {
	case MSP_SET_RAW_RC:
		s_struct_w((uint8_t*)&rcSerial, 16); //rcSerial을 저장할 장소에 크기 16만큼, 수신된 데이터를 읽어서 저장한다.
		rcSerialCount = 50; // 1s transition 
		break;
	case MSP_SET_PID:
		s_struct_w((uint8_t*)&conf.pid[0].P8, 3 * PIDITEMS);
		break;
	case MSP_SET_BOX:
		s_struct_w((uint8_t*)&conf.activate[0], CHECKBOXITEMS * 2);
		break;
	case MSP_SET_RC_TUNING:
		s_struct_w((uint8_t*)&conf.rcRate8, 7);
		break;
		#if !defined(DISABLE_SETTINGS_TAB)
	case MSP_SET_MISC:
		struct {
			uint16_t a, b, c, d, e, f;
			uint32_t g;
			uint16_t h;
			uint8_t  i, j, k, l;
		} set_misc;
		s_struct_w((uint8_t*)&set_misc, 22);
		#if defined(POWERMETER)
		conf.powerTrigger1 = set_misc.a / PLEVELSCALE;
		#endif
		conf.minthrottle = set_misc.b;
		#ifdef FAILSAFE 
		conf.failsafe_throttle = set_misc.e;
		#endif  
		#if MAG
		conf.mag_declination = set_misc.h;
		#endif
		#if defined(VBAT)
		conf.vbatscale = set_misc.i;
		conf.vbatlevel_warn1 = set_misc.j;
		conf.vbatlevel_warn2 = set_misc.k;
		conf.vbatlevel_crit = set_misc.l;
		#endif
		break;
	case MSP_MISC:
		struct {
			uint16_t a, b, c, d, e, f;
			uint32_t g;
			uint16_t h;
			uint8_t  i, j, k, l;
		} misc;
		misc.a = intPowerTrigger1;
		misc.b = conf.minthrottle;
		misc.c = MAXTHROTTLE;
		misc.d = MINCOMMAND;
		#ifdef FAILSAFE 
		misc.e = conf.failsafe_throttle;
		#else  
		misc.e = 0;
		#endif
		#ifdef LOG_PERMANENT
		misc.f = plog.arm;
		misc.g = plog.lifetime + (plog.armed_time / 1000000); // <- computation must be moved outside from serial
		#else
		misc.f = 0; misc.g = 0;
		#endif
		#if MAG
		misc.h = conf.mag_declination;
		#else
		misc.h = 0;
		#endif
		#ifdef VBAT
		misc.i = conf.vbatscale;
		misc.j = conf.vbatlevel_warn1;
		misc.k = conf.vbatlevel_warn2;
		misc.l = conf.vbatlevel_crit;
		#else
		misc.i = 0; misc.j = 0; misc.k = 0; misc.l = 0;
		#endif
		s_struct((uint8_t*)&misc, 22);
		break;
		#endif
		#if defined (DYNBALANCE)
	case MSP_SET_MOTOR:
		s_struct_w((uint8_t*)&motor, 16);
		break;
		#endif
		#ifdef MULTIPLE_CONFIGURATION_PROFILES
	case MSP_SELECT_SETTING:
		if (!f.ARMED) {
			global_conf.currentSet = read8();
			if (global_conf.currentSet > 2) global_conf.currentSet = 0;
			writeGlobalSet(0);
			readEEPROM();
		}
		headSerialReply(0);
		break;
		#endif
	case MSP_SET_HEAD:
		s_struct_w((uint8_t*)&magHold, 2);
		break;
	case MSP_IDENT:
		struct {
			uint8_t v, t, msp_v;
			uint32_t cap;
		} id;
		id.v = VERSION;
		id.t = MULTITYPE;
		id.msp_v = MSP_VERSION;
		id.cap = capability | DYNBAL << 2 | FLAP << 3 | NAVCAP << 4 | ((uint32_t)NAVI_VERSION << 28);			//Navi version is stored in the upper four bits; 
		//PC에 capability 정보를 보낸다. 0번 비트에 BINDCapability, 2번 비트에 DYNBALCapability, 3번 비트에 FLAP, 등...
		//BIND는 사용 중이지 않음
		s_struct((uint8_t*)&id, 7);
		break;
	case MSP_STATUS:
		struct {
			uint16_t cycleTime, i2c_errors_count, sensor;
			uint32_t flag;
			uint8_t set;
		} st;
		st.cycleTime = cycleTime;
		st.i2c_errors_count = i2c_errors_count;
		st.sensor = ACC | BARO << 1 | MAG << 2 | GPS << 3 | SONAR << 4;
		#if ACC
		if (f.ANGLE_MODE)   tmp |= 1 << BOXANGLE;
		if (f.HORIZON_MODE) tmp |= 1 << BOXHORIZON;
		#endif
		#if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
		if (f.BARO_MODE) tmp |= 1 << BOXBARO;
		#endif
		#if MAG
		if (f.MAG_MODE) tmp |= 1 << BOXMAG;
		#if !defined(FIXEDWING)
		if (f.HEADFREE_MODE)       tmp |= 1 << BOXHEADFREE;
		if (rcOptions[BOXHEADADJ]) tmp |= 1 << BOXHEADADJ;
		#endif
		#endif
		#if defined(SERVO_TILT) || defined(GIMBAL)|| defined(SERVO_MIX_TILT)
		if (rcOptions[BOXCAMSTAB]) tmp |= 1 << BOXCAMSTAB;
		#endif
		#if defined(CAMTRIG)
		if (rcOptions[BOXCAMTRIG]) tmp |= 1 << BOXCAMTRIG;
		#endif
		#if GPS
		switch (f.GPS_mode) {
		case GPS_MODE_HOLD:
			tmp |= 1 << BOXGPSHOLD;
			break;
		case GPS_MODE_RTH:
			tmp |= 1 << BOXGPSHOME;
			break;
		case GPS_MODE_NAV:
			tmp |= 1 << BOXGPSNAV;
			break;
		}

		#endif
		#if defined(FIXEDWING) || defined(HELICOPTER)
		if (f.PASSTHRU_MODE) tmp |= 1 << BOXPASSTHRU;
		#endif
		#if defined(BUZZER)
		if (rcOptions[BOXBEEPERON]) tmp |= 1 << BOXBEEPERON;
		#endif
		#if defined(LED_FLASHER)
		if (rcOptions[BOXLEDMAX]) tmp |= 1 << BOXLEDMAX;
		if (rcOptions[BOXLEDLOW]) tmp |= 1 << BOXLEDLOW;
		#endif
		#if defined(LANDING_LIGHTS_DDR)
		if (rcOptions[BOXLLIGHTS]) tmp |= 1 << BOXLLIGHTS;
		#endif
		#if defined(VARIOMETER)
		if (rcOptions[BOXVARIO]) tmp |= 1 << BOXVARIO;
		#endif
		#if defined(INFLIGHT_ACC_CALIBRATION)
		if (rcOptions[BOXCALIB]) tmp |= 1 << BOXCALIB;
		#endif
		#if defined(GOVERNOR_P)
		if (rcOptions[BOXGOV]) tmp |= 1 << BOXGOV;
		#endif
		#if defined(OSD_SWITCH)
		if (rcOptions[BOXOSD]) tmp |= 1 << BOXOSD;
		#endif
		if (f.ARMED) tmp |= 1 << BOXARM;
		st.flag = tmp;
		st.set = global_conf.currentSet;
		s_struct((uint8_t*)&st, 11);
		break;
	case MSP_RAW_IMU:
		#if defined(DYNBALANCE)
		for (uint8_t axis = 0; axis < 3; axis++) { imu.gyroData[axis] = imu.gyroADC[axis]; imu.accSmooth[axis] = imu.accADC[axis]; } // Send the unfiltered Gyro & Acc values to gui.
		#endif 
		s_struct((uint8_t*)&imu, 18);
		break;
	case MSP_SERVO:
		s_struct((uint8_t*)&servo, 16);
		break;
	case MSP_SERVO_CONF:
		s_struct((uint8_t*)&conf.servoConf[0].min, 56); // struct servo_conf_ is 7 bytes length: min:2 / max:2 / middle:2 / rate:1    ----     8 servo =>  8x7 = 56
		break;
	case MSP_SET_SERVO_CONF:
		s_struct_w((uint8_t*)&conf.servoConf[0].min, 56);
		break;
	case MSP_MOTOR:
		s_struct((uint8_t*)&motor, 16);
		break;
	case MSP_RC:
		s_struct((uint8_t*)&rcData, RC_CHANS * 2);
		break;
		#if GPS
	case MSP_RAW_GPS:
		headSerialReply(16);
		serialize8(f.GPS_FIX);
		serialize8(GPS_numSat);
		serialize32(GPS_coord[LAT]);
		serialize32(GPS_coord[LON]);
		serialize16(GPS_altitude);
		serialize16(GPS_speed);
		serialize16(GPS_ground_course);        // added since r1172
		break;
	case MSP_COMP_GPS:
		headSerialReply(5);
		serialize16(GPS_distanceToHome);
		serialize16(GPS_directionToHome);
		serialize8(GPS_update & 1);
		break;
		#endif
	case MSP_ATTITUDE:
		s_struct((uint8_t*)&att, 6); //Reply header 준비 + att에 저장된 6Byte짜리 데이터를 Serialize (응답하면서 요청데이터 준비)
		break;
	case MSP_ALTITUDE:
		s_struct((uint8_t*)&alt, 6); //응답하면서 요청데이터 준비
		break;
	case MSP_ANALOG://아날로그 데이터: ADC해서 읽어들인 Powermeter 따위의 데이터 -> 현재 사용중이지 않음
		s_struct((uint8_t*)&analog, 7);
		break;
	case MSP_RC_TUNING:
		s_struct((uint8_t*)&conf.rcRate8, 7);
		break;
	case MSP_PID:
		s_struct((uint8_t*)&conf.pid[0].P8, 3 * PIDITEMS);
		break;
	case MSP_PIDNAMES:
		serializeNames(pidnames);
		break;
	case MSP_BOX:
		s_struct((uint8_t*)&conf.activate[0], 2 * CHECKBOXITEMS);
		break;
	case MSP_BOXNAMES:
		serializeNames(boxnames);
		break;
	case MSP_BOXIDS:
		headSerialReply(CHECKBOXITEMS);
		for (uint8_t i = 0; i < CHECKBOXITEMS; i++) {
			serialize8(pgm_read_byte(&(boxids[i])));
		}
		break;
	case MSP_MOTOR_PINS:
		s_struct((uint8_t*)&PWM_PIN, 8);
		break;

		#if defined(USE_MSP_WP)    

	case MSP_SET_NAV_CONFIG:
		s_struct_w((uint8_t*)&GPS_conf, sizeof(GPS_conf));
		break;

	case MSP_NAV_CONFIG:
		s_struct((uint8_t*)&GPS_conf, sizeof(GPS_conf));
		break;

	case MSP_NAV_STATUS:
	{
		headSerialReply(7);
		serialize8(f.GPS_mode);
		serialize8(NAV_state);
		serialize8(mission_step.action);
		serialize8(mission_step.number);
		serialize8(NAV_error);
		serialize16((int16_t)(target_bearing / 100));
		//serialize16(magHold);
	}
	break;

	case MSP_WP:
	{
		uint8_t wp_no;
		uint8_t flag;
		bool    success;

		wp_no = read8();        //get the wp number  
		headSerialReply(21);
		if (wp_no == 0)					//Get HOME coordinates
		{
			serialize8(wp_no);
			serialize8(mission_step.action);
			serialize32(GPS_home[LAT]);
			serialize32(GPS_home[LON]);
			flag = MISSION_FLAG_HOME;
		}
		if (wp_no == 255)				//Get poshold coordinates
		{
			serialize8(wp_no);
			serialize8(mission_step.action);
			serialize32(GPS_hold[LAT]);
			serialize32(GPS_hold[LON]);
			flag = MISSION_FLAG_HOLD;
		}

		if ((wp_no > 0) && (wp_no < 255))
		{
			if (NAV_state == NAV_STATE_NONE)
			{
				success = recallWP(wp_no);
				serialize8(wp_no);
				serialize8(mission_step.action);
				serialize32(mission_step.pos[LAT]);
				serialize32(mission_step.pos[LON]);
				if (success == true) flag = mission_step.flag;
				else flag = MISSION_FLAG_CRC_ERROR;	//CRC error
			}
			else
			{
				serialize8(wp_no);
				serialize8(0);
				serialize32(GPS_home[LAT]);
				serialize32(GPS_home[LON]);
				flag = MISSION_FLAG_NAV_IN_PROG;
			}
		}
		serialize32(mission_step.altitude);
		serialize16(mission_step.parameter1);
		serialize16(mission_step.parameter2);
		serialize16(mission_step.parameter3);
		serialize8(flag);
	}
	break;

	case MSP_SET_WP:
		//TODO: add I2C_gps handling

	{
		uint8_t wp_no = read8();																   //Get the step number

		if (NAV_state == NAV_STATE_HOLD_INFINIT && wp_no == 255) {                              //Special case - during stable poshold we allow change the hold position
			mission_step.number = wp_no;
			mission_step.action = MISSION_HOLD_UNLIM;
			uint8_t temp = read8();
			mission_step.pos[LAT] = read32();
			mission_step.pos[LON] = read32();
			mission_step.altitude = read32();
			mission_step.parameter1 = read16();
			mission_step.parameter2 = read16();
			mission_step.parameter3 = read16();
			mission_step.flag = read8();
			if (mission_step.altitude != 0) set_new_altitude(mission_step.altitude);
			GPS_set_next_wp(&mission_step.pos[LAT], &mission_step.pos[LON], &GPS_coord[LAT], &GPS_coord[LON]);
			if ((wp_distance / 100) >= GPS_conf.safe_wp_distance) NAV_state = NAV_STATE_NONE;
			else NAV_state = NAV_STATE_WP_ENROUTE;
			break;
		}


		if (NAV_state == NAV_STATE_NONE) {									// The Nav state is not zero, so navigation is in progress, silently ignore SET_WP command)

			mission_step.number = wp_no;
			mission_step.action = read8();
			mission_step.pos[LAT] = read32();
			mission_step.pos[LON] = read32();
			mission_step.altitude = read32();
			mission_step.parameter1 = read16();
			mission_step.parameter2 = read16();
			mission_step.parameter3 = read16();
			mission_step.flag = read8();

			//It's not sure, that we want to do poshold change via mission planner so perhaps the next if is deletable
			/*
					   if (mission_step.number == 255)											//Set up new hold position via mission planner, It must set the action to MISSION_HOLD_INFINIT
						   {
						   if (mission_step.altitude !=0) set_new_altitude(mission_step.altitude);							//Set the altitude
						   GPS_set_next_wp(&mission_step.pos[LAT], &mission_step.pos[LON], &GPS_coord[LAT], &GPS_coord[LON]);
						   NAV_state = NAV_STATE_WP_ENROUTE;									//Go to that position, then it will switch to poshold unlimited when reached
						   }
			*/
			if (mission_step.number == 0)											//Set new Home position
			{
				GPS_home[LAT] = mission_step.pos[LAT];
				GPS_home[LON] = mission_step.pos[LON];
			}

			if (mission_step.number > 0 && mission_step.number < 255)			//Not home and not poshold, we are free to store it in the eprom
				if (mission_step.number <= getMaxWPNumber())				    // Do not thrash the EEPROM with invalid wp number
					storeWP();

			headSerialReply(0);
		}
	}
	break;

	#endif

	case MSP_RESET_CONF:
		if (!f.ARMED) LoadDefaults();
		headSerialReply(0);
		break;
	case MSP_ACC_CALIBRATION:
		if (!f.ARMED) calibratingA = 512;
		headSerialReply(0);
		break;
	case MSP_MAG_CALIBRATION:
		if (!f.ARMED) f.CALIBRATE_MAG = 1;
		headSerialReply(0);
		break;
		#if defined(SPEK_BIND)
	case MSP_BIND:
		spekBind();
		headSerialReply(0);
		break;
		#endif
	case MSP_EEPROM_WRITE:
		writeParams(0);
		headSerialReply(0);
		break;
	case MSP_DEBUG:
		s_struct((uint8_t*)&debug, 8);
		break;
		#ifdef DEBUGMSG
	case MSP_DEBUGMSG:
	{
		uint8_t size = debugmsg_available();
		if (size > 16) size = 16;
		headSerialReply(size);
		debugmsg_serialize(size);
	}
	break;
	#endif
	default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
		headSerialError(0);
		break;
	}
	tailSerialReply();
}
#endif // SUPPRESS_ALL_SERIAL_MSP

// evaluate all other incoming serial data
void evaluateOtherData(uint8_t sr) {
	#ifndef SUPPRESS_OTHER_SERIAL_COMMANDS
	switch (sr) {
		// Note: we may receive weird characters here which could trigger unwanted features during flight.
		//       this could lead to a crash easily.
		//       Please use if (!f.ARMED) where neccessary
		#ifdef LCD_CONF
	case 's':
	case 'S':
		if (!f.ARMED) configurationLoop();
		break;
		#endif
		#ifdef LOG_PERMANENT_SHOW_AT_L
	case 'L':
		if (!f.ARMED) dumpPLog(1);
		break;
		#endif
		#if defined(LCD_TELEMETRY) && defined(LCD_TEXTSTAR)
	case 'A': // button A press
		toggle_telemetry(1);
		break;
	case 'B': // button B press
		toggle_telemetry(2);
		break;
	case 'C': // button C press
		toggle_telemetry(3);
		break;
	case 'D': // button D press
		toggle_telemetry(4);
		break;
	case 'a': // button A release
	case 'b': // button B release
	case 'c': // button C release
	case 'd': // button D release
		break;
		#endif
		#ifdef LCD_TELEMETRY
	case '0':
	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
	case '7':
	case '8':
	case '9':
		#ifndef SUPPRESS_TELEMETRY_PAGE_R
	case 'R':
		#endif
		#if defined(DEBUG) || defined(DEBUG_FREE)
	case 'F':
		#endif
		toggle_telemetry(sr);
		break;
		#endif // LCD_TELEMETRY
	}
	#endif // SUPPRESS_OTHER_SERIAL_COMMANDS
}

void SerialWrite16(uint8_t port, int16_t val)
{
	CURRENTPORT = port;
	serialize16(val); UartSendData(port);
}


#ifdef DEBUGMSG //디버거 관련 시리얼 통신!
void debugmsg_append_str(const char *str) {
	while (*str) { //가리키는 문자열에 데이터가 들어있지 않을 때? 아니면 널문자가 들어 있는 공간을 가리킬 때 참인 듯 -> 확인바람!
		debug_buf[head_debug++] = *str++; //str로 문자열을 받는데 해당 문자열을 원형 버퍼에 하나씩 write함
		if (head_debug == DEBUG_MSG_BUFFER_SIZE) {
			head_debug = 0;;
		}
	}
}

static uint8_t debugmsg_available() {
	if (head_debug >= tail_debug) {
		return head_debug - tail_debug;
	}
	else {
		return head_debug + (DEBUG_MSG_BUFFER_SIZE - tail_debug);
	}
}

static void debugmsg_serialize(uint8_t l) {
	for (uint8_t i = 0; i < l; i++) {
		if (head_debug != tail_debug) {
			serialize8(debug_buf[tail_debug++]);
			if (tail_debug == DEBUG_MSG_BUFFER_SIZE) {
				tail_debug = 0;
			}
		}
		else {
			serialize8('\0');
		}
	}
}
#else
void debugmsg_append_str(const char *str) {};
#endif

