#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/poll.h>
#include <errno.h>
#include <pthread.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdbool.h>

// timer include
#include <sys/time.h>
#include <signal.h>

#define DELAY_TIME 100
// define PORT "/dev/ttyUSB0'
#define PORT "/dev/ttyAMA0"

#define STX1 0xAF
#define STX2 0xFA
#define ETX1 0xAF
#define ETX2 0xA0

#define CMD1 0x01
#define CMD2 0x02 
#define CMD3 0x03
#define CMD4 0x1F

#define BATTERY1_ADDRESS 0x60
#define BATTERY2_ADDRESS 0X61
#define RC_ADDRESS 0x30
#define C1_ADDRESS 0x40

#define _DEBUG 1
#define BUFFER_SIZE 1024
#define DATATYPE unsigned char

DATATYPE GetData[BUFFER_SIZE] = {0,};
DATATYPE CropData[BUFFER_SIZE] = {0,};
//DATATYPE Data[BUFFER_SIZE]={0,};

int Fdport;
struct pollfd poll_events;
int poll_state;
int datapos=0;

char SendData[1024]={0,};

char sendLength = 0x05;
char batteryData1 = 0x14;
char batteryData2 = 0x00;
char remoteData1 = 0x00;
char remoteData2 = 0x00;

int remote_flag=1;
int battery_charging_flag;
char commandControlFlag=0xFF;
char commandWheelFlag=0xFF;

int BATTERY;
char Battery1=0x00;
char Battery2=0x00;

int SendCommandCount=0;
int gRemoteCommunicationSend = 0;

int batteryGpio[10]={10, 22, 27, 17, 4, 9, 25, 24, 23, 18};
int remoteGpio[10]={26, 19, 13, 6, 5, 11, 20, 21, 14, 0};
int LED_Display[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Timer
timer_t SendCommandTimerID;
timer_t SerialToRingBufferTimerID;
timer_t CropandControlTimerID;

struct Buffer {
    DATATYPE data[BUFFER_SIZE];
    int head;
    int tail;
	int usednum;
};

struct Buffer RingBuffer;

enum BufferStatus {
	BUFFER_OK, BUFFER_EMPTY, BUFFER_FULL, BUFFER_FIND_OK, BUFFER_FIND_FAIL, BUFFER_OVERSIZE
};

int openSerial(char *device_name);
static int makeTimer(char *name, timer_t *timerID, int sec, int msec);
static void timer_handler(int sig, siginfo_t *si, void *uc);
int SendCommand(void);
void* sendRemoteCommand(void);
void* sendBattery1Command(void);
void* sendBattery2Command(void);
int SerialToRingBuffer(void);
int CropSetCommandFromRingBufferAndControl(void);
enum BufferStatus bufferWrite(struct Buffer *buffer, DATATYPE byte);
enum BufferStatus bufferRead(struct Buffer *buffer, DATATYPE *byte);
enum BufferStatus bufferFinder(struct Buffer *buffer, DATATYPE ch, int *length, int offset);
enum BufferStatus bufferSetFinder(struct Buffer *buffer, int *length);
DATATYPE ChkSum(DATATYPE *Crop, int length);
int CheckChkSum(DATATYPE *Crop, int length);
void* battery_status(char *CropData);
void remote_status(char *CropData);
void* battery_led_display(int BATTERY, int battery_charging_flag);
void* send_C1_Status(int C1_flag,char DATA1,char DATA2);
void* sendEmergency(void);
void debug(char *data_frame,int dataFrameCnt);

int main()
{
	Fdport=openSerial(PORT);

	poll_events.fd=Fdport;
	poll_events.events=POLLIN|POLLERR;
	poll_events.revents=0;

	makeTimer("Serial To RingBuffer Timer", &SerialToRingBufferTimerID, 0, 50); // 50 ms
	delay(20);
	makeTimer("Crop and Control Timer", &CropandControlTimerID, 0, 100); // 100ms timer
	delay(40);
	makeTimer("SendCommand Timer", &SendCommandTimerID, 0, 100); // 100ms timer
	delay(40);
	
	while(1){}
	close(Fdport);
	return 0;

}

int openSerial(char *device_name)
{
	struct termios newtio;

	Fdport=open(device_name,O_RDWR|O_NOCTTY|O_NONBLOCK);
	if(Fdport<0){
		fprintf(stderr,"Unable to open serial device: %s\n",strerror(errno));
		return -1;
	}
	
	wiringPiSetupGpio();
	if(wiringPiSetup()==-1){
		fprintf(stdout,"Unable to start wiringPi: %s\n",strerror(errno));
		return -1;
	}

	for(int i=0;i<10;i++){
		pinMode(batteryGpio[i],OUTPUT);
		pinMode(remoteGpio[i],OUTPUT);
	}

	memset(&newtio,0,sizeof(newtio));

	newtio.c_cflag=B19200|CS8|CLOCAL|CREAD;
	newtio.c_iflag=IGNPAR;
	newtio.c_oflag=0;
	newtio.c_lflag=0;
	newtio.c_cc[VTIME]=10;
	newtio.c_cc[VMIN]=32;
	
	tcflush(Fdport,TCIFLUSH);
	tcsetattr(Fdport,TCSANOW,&newtio);
	fcntl(Fdport,F_SETFL,FNDELAY);

	return Fdport;
}

static int makeTimer( char *name, timer_t *timerID, int sec, int msec ) 
{ 
	struct sigevent         te; 
	struct itimerspec       its; 
	struct sigaction        sa; 
	int                     sigNo = SIGRTMIN; 
	
	/* Set up signal handler. */ 
	sa.sa_flags = SA_SIGINFO; 
	sa.sa_sigaction = timer_handler; 
	sigemptyset(&sa.sa_mask); 
	
	if (sigaction(sigNo, &sa, NULL) == -1) 
	{ 
	    printf("sigaction error\n");
	    return -1; 
	} 
	
	// Install timer_handler as the signal handler for SIGVTALRM. 
	te.sigev_notify = SIGEV_SIGNAL; 
	te.sigev_signo = sigNo; 
	te.sigev_value.sival_ptr = timerID; 
	timer_create(CLOCK_REALTIME, &te, timerID); 
	
	// every se c& msec after that. 
	its.it_interval.tv_sec = sec;
	its.it_interval.tv_nsec = msec * 1000; 
	
	// Configure the timer to expire after sec...
	its.it_value.tv_sec = sec;
	its.it_value.tv_nsec = msec * 1000;

	timer_settime(*timerID, 0, &its, NULL); 
	

	printf("%s Timer %d sec / %d msec Created\n", name, sec, msec);

	return 0; 
}

static void timer_handler( int sig, siginfo_t *si, void *uc ) 
{ 
	timer_t *tidp;
	tidp = si->si_value.sival_ptr;
	
	if(*tidp == SendCommandTimerID)
	{
	    printf("SendCommandTimerID\n");
		SendCommand();
	}
	else if(*tidp == SerialToRingBufferTimerID) 
	{
	    printf("SerialToRingBufferTimerID\n");
		SerialToRingBuffer();
	}
	else if(*tidp == CropandControlTimerID)
	{
	    printf("CropandControlTimerID\n");
		CropSetCommandFromRingBufferAndControl();
	}
}  

int SendCommand(void)
{
	SendData[0]=STX1;
	SendData[1]=STX2;
	SendData[3]=sendLength;
	SendData[4]=CMD1;
	SendData[9]=ETX1;
	SendData[10]=ETX2;

	SendCommandCount = SendCommandCount % 10;
	switch(SendCommandCount) {
		case 0:
		case 1:
		case 2:
		case 3:
		case 5:
		case 6:
		case 7:
		case 8:
			sendRemoteCommand();
			break;
		case 4:
			sendBattery1Command();
			break;
		case 9:
			sendBattery2Command();
			break;
	}	

	SendCommandCount++;
	// SendCommandCount = SendCommandCount % 10; ???

	return SendCommandCount;
}

void* sendRemoteCommand(void)
{
	/* send remote 0x30 data frame 
	  AF FA 30 05 01 30 00 00 66 AF A0 */

	char CKSUM=(RC_ADDRESS+sendLength+CMD1+RC_ADDRESS+remoteData1+remoteData2) & 0xFF;

	SendData[2] = SendData[5] = RC_ADDRESS;
	SendData[6] = remoteData1;
	SendData[7] = remoteData2;
	SendData[8] = CKSUM;

	for(int i=0;i<11;i++){
		serialPutchar(Fdport,SendData[i]);
	}
	/*
	if(_DEBUG){
		printf("send  REMOTE  : ");
		debug(SendData, 11);
	}
	*/
	gRemoteCommunicationSend++;
		
	delay(DELAY_TIME);
	fflush(stdout);
	
	//communication error
	if(gRemoteCommunicationSend >= 4){
		sendEmergency();
		gRemoteCommunicationSend = 0;
	}

}

void* sendBattery1Command(void)
{
	/* send battery 0x60 data frame 
	  AF FA 60 05 01 60 14 00 DA AF A0 */
	char CKSUM = (BATTERY1_ADDRESS+sendLength+CMD1+BATTERY1_ADDRESS+batteryData1+batteryData2) & 0xFF;
	
	SendData[2] = SendData[5] = BATTERY1_ADDRESS;
	SendData[6] = batteryData1;
	SendData[7] = batteryData2;
	SendData[8] = CKSUM;
	
	for(int i=0; i<11; i++){
		serialPutchar(Fdport, SendData[i]);
	}
	/*
	if(_DEBUG){
		printf("send BATTERY1 : ");
		debug(SendData, 11);
	}
	*/
	delay(DELAY_TIME);
	fflush(stdout);
}

void* sendBattery2Command(void)
{
	/* send battery 0x61 data frame 
	  AF FA 61 05 01 61 14 00 DC AF A0 */
	char CKSUM = (BATTERY2_ADDRESS+sendLength+CMD1+BATTERY2_ADDRESS+batteryData1+batteryData2) & 0xFF;
	
	SendData[2] = SendData[5] = BATTERY2_ADDRESS;
	SendData[6] = batteryData1;
	SendData[7] = batteryData2;
	SendData[8] = CKSUM;
	
	for(int i=0; i<11; i++){
		serialPutchar(Fdport, SendData[i]);
	}
	/*
	if(_DEBUG){
		printf("send BATTERY2 : ");
		debug(SendData, 11);
	}
	*/
	delay(DELAY_TIME);
	fflush(stdout);
}

int SerialToRingBuffer(void)
{
	int	DataCnt = 0;
	int i;

	poll_state=poll( (struct pollfd*)&poll_events,1,0);
	if(0<poll_state) {
		if(poll_events.revents&POLLIN){
			delay(30);
			DataCnt=read(Fdport,GetData,BUFFER_SIZE);
			printf("Input Ring Data : ");
			for(i = 0;i<DataCnt; i++) {
				bufferWrite(&RingBuffer, GetData[i]);
				printf("%02X ", GetData[i]);
			}
			printf("\n");
			memset(GetData,0,sizeof(DATATYPE)*BUFFER_SIZE);
		}
		// error check
		if(poll_events.revents&POLLERR) {
			printf("poll_events Error\n");
		}
	}
}

int CropSetCommandFromRingBufferAndControl(void)
{
	int length;
	int i;
	enum BufferStatus status;
	DATATYPE received_byte;
	
	status = bufferSetFinder(&RingBuffer, &length);
	printf("%d\n", length);

	if(status == BUFFER_EMPTY) {
		printf("Buffer Empty Error\n");
		return 0;
	}

	for(i = 0;i < length;i++) {
		status = bufferRead(&RingBuffer, &received_byte);
		if (status == BUFFER_OK) {
			CropData[i] = received_byte;
			printf("%02X ", CropData[i]);
		}
	}	
	printf("\n");

	DATATYPE chksum=0;

	if(status == BUFFER_OK) {
		chksum = ChkSum(CropData, length);
		if(CheckChkSum(CropData, length)) {
			printf("CheckSum OK\n");

			if(CropData[2]==BATTERY1_ADDRESS || CropData[2]==BATTERY2_ADDRESS){
				battery_status(CropData);
			}
			if(CropData[2]==RC_ADDRESS){
				gRemoteCommunicationSend = 0;
				remote_status(CropData);
			}
			return 1;
		}
		else {
			printf("CheckSum ERROR\n");
			return 0;
		}
	}

	return 0;
}

enum BufferStatus bufferWrite(struct Buffer *buffer, DATATYPE byte)
{
	int next_index = (((buffer->head)+1) % BUFFER_SIZE);
	
	if (next_index == buffer->tail){
	    return BUFFER_FULL;
	}
	
	buffer->data[buffer->head] = byte;
	buffer->head = next_index;
	buffer->usednum++;
	return BUFFER_OK;
}
 
enum BufferStatus bufferRead(struct Buffer *buffer, DATATYPE *byte)
{
	if (buffer->head == buffer->tail){
		return BUFFER_EMPTY;
	}
	
	*byte = buffer->data[buffer->tail];
	buffer->tail = ((buffer->tail+1) % BUFFER_SIZE);
	buffer->usednum--;
	return BUFFER_OK;
}

enum BufferStatus bufferFinder(struct Buffer *buffer, DATATYPE ch, int *length, int offset)
{
	int i;
	int cnt = 0;

	if(offset < 0) {
		return BUFFER_OVERSIZE;
	}

	if(buffer->head > buffer->tail+offset) {

		for(i=buffer->tail+offset;i<buffer->head;i++, cnt++) {
			if(buffer->data[i] == ch) {
				*length = cnt+1;
				return BUFFER_FIND_OK;
			}
		}
	}
	else if(buffer->head < buffer->tail+offset) {
		for(i=(buffer->tail+offset)%BUFFER_SIZE;i<BUFFER_SIZE;i++, cnt++) {
			if(buffer->data[i] == ch) {
				*length = cnt+1;
				return BUFFER_FIND_OK;
			}
		}
		for(i=0;i<buffer->tail+offset;i++, cnt++) {
			if(buffer->data[i] == ch) {
				*length = cnt+1;
				return BUFFER_FIND_OK;
			}
		}
	}
	return BUFFER_FIND_FAIL;;
}

// 2byte finder
enum BufferStatus bufferSetFinder(struct Buffer *buffer, int *length)
{
	int lenetx2 = 0;
	int lenetx1 = 0;
	int lenstx2 = 0;
	int lenstx1 = 0;
	int i = 0;


	enum BufferStatus status;
	DATATYPE received_byte;

	*length = 0;

	int len=0;

	int FindSTXFlag = 0;
	int FindETXFlag = 0;

	int checkcountnumber = RingBuffer.usednum;
	int WhileCount = 0;

	while(!FindSTXFlag) {
 		status = bufferFinder(&RingBuffer, STX1, &lenstx1, len);
		if(status == BUFFER_FIND_FAIL) return BUFFER_FIND_FAIL;

		// delete error data before STX1 , STX1 앞부분 쓰레기 값 제거
		if(lenstx1 != 1) { 
			for(i = 0;i < lenstx1-1;i++) {
				status = bufferRead(&RingBuffer, &received_byte); 
			}
			len = 0;
		}
		else {
			len = 1;
		}

		status = bufferFinder(&RingBuffer, STX2, &lenstx2, 1);
		if(status == BUFFER_FIND_FAIL) return BUFFER_FIND_FAIL;

		if(lenstx2 != 1) {
			FindSTXFlag = false;
			if(WhileCount>BUFFER_SIZE) return BUFFER_FIND_FAIL; // 무한루프 방지를 위한 while 탈출
			WhileCount++;
		}
		else {
			FindSTXFlag = true;
			len++;
		}

	}

	WhileCount = 0;

	while(!FindETXFlag) {
		status = bufferFinder(&RingBuffer, ETX1, &lenetx1, len); // offset = len 는 STX1 STX2 제외하고찾기 시작
		if(status == BUFFER_FIND_FAIL) return BUFFER_FIND_FAIL;
		
		len+=lenetx1;
		
		status = bufferFinder(&RingBuffer, ETX2, &lenetx2, len);
		if(status == BUFFER_FIND_FAIL) return BUFFER_FIND_FAIL;

		if(lenetx2 != 1) {
			FindETXFlag = false;
			if(WhileCount>BUFFER_SIZE) return BUFFER_FIND_FAIL; // 무한루프 방지를 위한 while 탈출
			WhileCount++;
			len++;
		}
		else {
			FindETXFlag = true;
			len++;
		}
	}

	*length = len;
	return BUFFER_FIND_OK;
}

DATATYPE ChkSum(DATATYPE *Crop, int length)
{
	DATATYPE chksum = 0;

	if(length <= 5) return chksum;

	for(int i=2;i<length-3;i++) {
		chksum += Crop[i];
		
	}
	chksum = chksum & 0xFF;
	return chksum;
}

int CheckChkSum(DATATYPE *Crop, int length)
{
	DATATYPE chksum = 0;

	chksum = ChkSum(Crop, length);

	if(Crop[length-3] == chksum) return 1;
	
	return false;
}

void* battery_status(char *CropData)
{
	char Error=0x00;
	int C1_flag = 0;
	int charging=CropData[8]<<8|CropData[9];
	
	// receive battery status
	if(CropData[4]==CMD3){
		C1_flag = 1;
		if(CropData[2]==BATTERY1_ADDRESS){
			Battery1=CropData[6]<<8|CropData[7];
			if(_DEBUG){
				printf("RECEIVE BAT1 : ");
				debug(CropData, 13);
			}
		}
		else if(CropData[2]==BATTERY2_ADDRESS){
			Battery2=CropData[6]<<8|CropData[7];
			if(_DEBUG){
				printf("RECEIVE BAT2 : ");
				debug(CropData, 13);
			}
		}
		
		//if(Battery1==0) Battery1=Battery2;
		//if(Battery2==0) Battery2=Battery1;
		BATTERY=((int)Battery1+(int)Battery2)/2;
		
		printf("Battery 1 : %d\nBattery 2 : %d\n",Battery1,Battery2);

		if(charging==0){ 		// No charging
			battery_charging_flag = 0;
		}
		else{
			battery_charging_flag = 1;
		}
		send_C1_Status(C1_flag, Battery1, Battery2);
		battery_led_display(BATTERY,battery_charging_flag);
	}
	
	// receive battery error command
	else if(CropData[4]==CMD4){
		C1_flag = 2;
		if(CropData[2]==BATTERY1_ADDRESS){
			Battery1=CropData[5];
		}
		else if(CropData[2]==BATTERY2_ADDRESS){
			Battery2=CropData[5];
		}
		Error = Battery1 | Battery2;
		send_C1_Status(C1_flag, Error, 0);
	}	
}

void remote_status(char *CropData)
{
	for(int i=0;i<10;i++){
		digitalWrite(remoteGpio[i],0);
	}
	
	if(CropData[6]==0x00 && CropData[7]==0x00){
		return ;
	}
	
	if(remote_flag){
		remote_flag=0;
		
		char cropControlFlag=CropData[6]&commandControlFlag;
		if(cropControlFlag){
			if(cropControlFlag&0x01){
				commandControlFlag=CropData[6]&0x01;
			}
			else if(cropControlFlag&0x02){
				commandControlFlag=CropData[6]&0x02;
			}
			else if(cropControlFlag&0x04){
				commandControlFlag=CropData[6]&0x04;
			}
			else if(cropControlFlag&0x08){
				commandControlFlag=CropData[6]&0x08;
			}
			else if(cropControlFlag&0x10){
				commandControlFlag=CropData[6]&0x10;
			}
			send_C1_Status(3,commandControlFlag,0x00);
		}
		else{
			commandControlFlag=0xFF;
		}
		
		char cropWheelFlag=CropData[7]&commandWheelFlag;
		if(cropWheelFlag){
			if(cropWheelFlag&0x01){
				digitalWrite(remoteGpio[0],1);
				commandWheelFlag=CropData[7]&0x01;
				printf("RC_LiftUp - command 0\n");
			}
			else if(cropWheelFlag&0x02){
				digitalWrite(remoteGpio[1],1);
				commandWheelFlag=CropData[7]&0x02;
				printf("RC_LiftDn - command 1 \n");
			}
			else if(cropWheelFlag&0x04){
				digitalWrite(remoteGpio[2],1);
				commandWheelFlag=CropData[7]&0x04;
				printf("RC_LeftTurn - command 2 \n");
			}
			else if(cropWheelFlag&0x08){
				digitalWrite(remoteGpio[3],1);
				commandWheelFlag=CropData[7]&0x08;
				printf("RC_RightTurn - command 3 \n");
			}
			else if(cropWheelFlag&0x10){
				digitalWrite(remoteGpio[4],1);
				commandWheelFlag=CropData[7]&0x10;
				printf("RC_FowardGantry - command 4 \n");
			}
			else if(cropWheelFlag&0x20){
				digitalWrite(remoteGpio[5],1);
				commandWheelFlag=CropData[7]&0x20;
				printf("RC_ReverseGantry - command 5 \n");
			}
			else if(cropWheelFlag&0x40){
				digitalWrite(remoteGpio[6],1);
				commandWheelFlag=CropData[7]&0x40;
				printf("RC_LeftSlide - command 6 \n");
			}
			else if(cropWheelFlag&0x80){
				digitalWrite(remoteGpio[7],1);
				commandWheelFlag=CropData[7]&0x80;
				printf("RC_RightSlide - command 7 \n");
			}
		}
		else{
			commandWheelFlag=0xFF;
		}
		remote_flag=1;
	}
}

void* battery_led_display(int BATTERY, int battery_charging_flag)
{
	bool bBlinkStatus = true;

	while (1) {
		// LED_Display Init
		for(int i=0;i<10;i++){
			LED_Display[i] = 0;
		}
		for(int i=0;i<BATTERY/10;i++){
			LED_Display[i] = 1;
		}

		if(battery_charging_flag == 0){
			if(_DEBUG){
				printf("Battery value : %d \n",BATTERY);
			}
			// LED display
			for(int i=0;i<10;i++){
				digitalWrite(batteryGpio[i],LED_Display[i]);
			}	
		}
		else{
			if(_DEBUG){
				printf("Battery Charging ...\n");
			}
			if(bBlinkStatus) {  
				for(int i=0;i<10;i++){
					digitalWrite(batteryGpio[i],LED_Display[i]);
				}
			}
			else { 
				for(int i=0;i<10;i++){
					digitalWrite(batteryGpio[i],0);
				}
			}
		}
		delay(1000);
		
		/*
		// for debug 
		printf("datapos = %d --------------------\n", datapos);
		if(datapos != 0) {
			printf("Data = ");
			debug(data, datapos);
		}
		*/
	}
}

void* send_C1_Status(int C1_flag,char DATA1,char DATA2)
{
	char SendToC1[1024]={0,};
	int length=0;
	
	SendToC1[0] = STX1;
	SendToC1[1] = STX2;
	SendToC1[2] = SendToC1[5] = C1_ADDRESS;
	SendToC1[6] = DATA1;
	
	// send battery status
	if(C1_flag == 1){
		length = 11;
		SendToC1[3] = sendLength;
		SendToC1[4] = CMD2;
		SendToC1[7] = DATA2;
		SendToC1[8] = (2*C1_ADDRESS+sendLength+CMD2+DATA1+DATA2) & 0xFF;
		SendToC1[9] = ETX1;
		SendToC1[10] = ETX2;
	}
	// send remote status
	else if(C1_flag == 3){
		length = 11;
		SendToC1[3] = sendLength;
		SendToC1[4] = CMD3;
		SendToC1[7] = DATA2;
		SendToC1[8]=(2*C1_ADDRESS+sendLength+CMD3+DATA1+DATA2) & 0xFF;
		SendToC1[9] = ETX1;
		SendToC1[10] = ETX2;
	}
	// send battery error
	else if(C1_flag == 2){
		length = 10;
		SendToC1[3] = 0x04;
		SendToC1[4] = CMD4;
		SendToC1[7] = (2*C1_ADDRESS+0x04+CMD4+DATA1)&0xFF;
		SendToC1[8] = ETX1;
		SendToC1[9] = ETX2;
	}
	
	for(int i=0;i<length;i++){
		serialPutchar(Fdport,SendToC1[i]);
	}
	if(_DEBUG){
		printf("C3 -> C1 : ");
		debug(SendToC1,length);
	}
	
	fflush(stdout);
}

void* sendEmergency(void)
{
	printf("\nEmergency All Stop\n");
	
	// IO  off
	for(int i=0;i<10;i++){
		digitalWrite(remoteGpio[i],0);
	}
	
	// send all stop to C1
	send_C1_Status(3,0x00,0x00);
}

void debug(char *data_frame,int dataFrameCnt)
{
	for(int i=0;i<dataFrameCnt;i++){
		printf("%02X ",data_frame[i]);
	}
	printf("\n");
}
