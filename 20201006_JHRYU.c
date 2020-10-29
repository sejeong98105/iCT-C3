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

#define PORT1 "/dev/ttyUSB0"

#define DELAY_TIME 100
#define _DEBUG 1
#define DATASIZE 1024
#define MINIMAL_SIZE_OF_COMMAND_SET 11

#define STX1 0xAF
#define STX2 0xFA
#define ETX1 0xAF
#define ETX2 0xA0

#define C1_ADDRESS 0x40
#define BATTERY1_ADDRESS 0x60
#define BATTERY2_ADDRESS 0x61
#define REMOTE1_ADDRESS 0x30
#define REMOTE2_ADDRESS 0x31

#define COMMAND1 0x01
#define COMMAND2 0x02
#define COMMAND3 0x03
#define COMMAND4 0x1F

//Send Command Data 1,2
char SendData[1024]={0,};
char SendLength=0x05;
char BatteryData1=0x14;
char BatteryData2=0x00;
char RemoteData1=0x00;
char RemoteData2=0x00;
 
// Remote Flag
int remote_flag=1;
int battery_charging_flag;
char commandControlFlag=0xFF;
char commandWheelFlag=0xFF;

char Battery1=0x00;
char Battery2=0x00;

int datapos=0;
int SendCommandCount=0;
char Getdata[1024]={0,};
char Data[1024]={0,};
int LED[10]={18,23,24,25,9,4,17,27,22,30};
int Remote[10]={26,19,13,6,5,11,10,21,20,16};
int LED_Display[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int gRemoteCommunicationSend = 0;

int BATTERY;
void* sendEmergency(void* fd);

int openSerial(char *device_name);
void debug(char *data_frame,int dataFrameCnt);
void* sendCommand(void* fd);
void* sendBattery1Command(void* fd);
void* sendBattery2Command(void* fd);
void* sendRemoteCommand(void* fd);
void* send_C1_Status(void* fd,int status,char DATA1,char DATA2);

//int searchData(char *data,char searchdata);
int searchData(char *Data,char searchdata, int searchpos);

void* battery_led(void* fd);
void remote_status(void* fd,char *Cropdata);
void* battery_status(void* fd,char *Cropdata);

// timer
static void timer_handler( int sig, siginfo_t *si, void *uc );
static int makeTimer( char *name, timer_t *timerID, int sec, int msec );

// Timer
timer_t SendCommandTimerID;
timer_t SerialToRingBufferTimerID;
timer_t CropandControlTimerID;


 
#define BUFFER_SIZE 1024
#define DATATYPE unsigned char


struct Buffer {
    DATATYPE data[BUFFER_SIZE];
    int head;
    int tail;
	int usednum;
};


DATATYPE GetData[BUFFER_SIZE] = {0,};
DATATYPE CropData[BUFFER_SIZE] = {0,};
//int CropSize = 0;


enum BufferStatus {
	BUFFER_OK, BUFFER_EMPTY, BUFFER_FULL, BUFFER_FIND_OK, BUFFER_FIND_FAIL, BUFFER_OVERSIZE
};


struct Buffer RingBuffer;

int Fdport;
struct pollfd poll_events;
int poll_state;

static int makeTimer( char *name, timer_t *timerID, int sec, int msec );
static void timer_handler( int sig, siginfo_t *si, void *uc );
enum BufferStatus bufferWrite(struct Buffer *buffer, DATATYPE byte);
enum BufferStatus bufferRead(struct Buffer *buffer, DATATYPE *byte);
enum BufferStatus bufferFinder(struct Buffer *buffer, DATATYPE ch, int *length, int offset);
enum BufferStatus bufferSetFinder(struct Buffer *buffer, int *length);
DATATYPE ChkSum(DATATYPE *Crop, int length);
int CheckChkSum(DATATYPE *Crop, int length);
int CropSetCommandFromRingBufferAndControl(void);
int SerialToRingBuffer(void);
int SendCommand(void);


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
	
	if ( *tidp == SendCommandTimerID )
	{
	    printf("SendCommandTimerID\n");
		SendCommand();
	}
	else if ( *tidp == SerialToRingBufferTimerID ) 
	{
	    printf("SerialToRingBufferTimerID\n");
		SerialToRingBuffer();
	}
	else if ( *tidp == CropandControlTimerID ) 
	{
	    printf("CropandControlTimerID\n");
		CropSetCommandFromRingBufferAndControl();
	}
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
//		offset = 0;
		return BUFFER_OVERSIZE;
	}

	if(buffer->head > buffer->tail+offset) {

		for(i=buffer->tail+offset;i<buffer->head;i++, cnt++) {
			if(buffer->data[i] == ch) {
				//return i;
				//return cnt+1;
				*length = cnt+1;
				return BUFFER_FIND_OK;
			}
		}
	}
	else if(buffer->head < buffer->tail+offset) {
//		for(i=buffer->tail+offset;i<DATASIZE;i++, cnt++) {
		for(i=(buffer->tail+offset)%BUFFER_SIZE;i<DATASIZE;i++, cnt++) {
			if(buffer->data[i] == ch) {
				//return i;
				*length = cnt+1;
				return BUFFER_FIND_OK;
			}
		}
		//cnt = i;
		for(i=0;i<buffer->tail+offset;i++, cnt++) {
			if(buffer->data[i] == ch) {
				//return (cnt+i);
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
//	*length = lenstx1+lenstx2+lenetx1+lenetx2;
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
//	TRACE("%02X\n", chksum);
	return chksum;
}

// return value 1 = same, 0 = error
int CheckChkSum(DATATYPE *Crop, int length)
{
	DATATYPE chksum = 0;

	chksum = ChkSum(Crop, length);

	if(Crop[length-3] == chksum) return 1;
	
	return false;
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
				battery_status(&Fdport,CropData);
			}
			if(CropData[2]==REMOTE1_ADDRESS){
				gRemoteCommunicationSend = 0;
				remote_status(&Fdport,CropData);
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

int SerialToRingBuffer(void)
{
	int	DataCnt = 0;
	int i;

	poll_state=poll( (struct pollfd*)&poll_events,2,5000);
	if(0<poll_state) {
		if(poll_events.revents&POLLIN){
//			delay(20);
			DataCnt=read(Fdport,Getdata,BUFFER_SIZE);
			printf("Input Ring Data : ");
			for(i = 0;i<DataCnt; i++) {
				bufferWrite(&RingBuffer, GetData[i]);
				printf("%02X ", GetData[i]);
			}
			printf("\n");			

			memset(Getdata,0,sizeof(DATATYPE) * BUFFER_SIZE);
		}
		// error check
		if(poll_events.revents&POLLERR) {
			printf("poll_events Error\n");
		}
	}
}

int SendCommand(void)
{
	int Fd=Fdport;

	SendData[0]=STX1;
	SendData[1]=STX2;
	SendData[3]=SendLength;
	SendData[4]=COMMAND1;
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
			sendRemoteCommand(&Fd);
			break;
		case 4:
			sendBattery1Command(&Fd);
			break;
		case 9:
			sendBattery2Command(&Fd);
			break;
	}	

	SendCommandCount++;
	SendCommandCount = SendCommandCount % 10;

	return SendCommandCount;

}

int main()
{
	int status;

	Fdport=openSerial(PORT1);

	int getDataCnt;

	poll_events.fd=Fdport;
	poll_events.events=POLLIN|POLLERR;
	poll_events.revents=0;


	makeTimer("Serial To RingBuffer Timer", &SerialToRingBufferTimerID, 0, 50); // 50 ms
	delay(20); 
	makeTimer("SendCommand Timer", &SendCommandTimerID, 0, 100); // 100ms timer
	delay(40); 
	makeTimer("Crop and Control Timer", &CropandControlTimerID, 0, 100); // 100ms timer

	
	while(1){}
	close(Fdport);
	return 0;

}

int openSerial(char *device_name)
{
	int fd;
	struct termios newtio;

	fd=open(device_name,O_RDWR|O_NOCTTY|O_NONBLOCK);
	if(fd<0){
		fprintf(stderr,"Unable to open serial device: %s\n",strerror(errno));
		return -1;
	}
	
	wiringPiSetupGpio();
	if(wiringPiSetup()==-1){
		fprintf(stdout,"Unable to start wiringPi: %s\n",strerror(errno));
		return -1;
	}

	for(int i=0;i<10;i++){
		pinMode(LED[i],OUTPUT);
		pinMode(Remote[i],OUTPUT);
	}

	memset(&newtio,0,sizeof(newtio));

	newtio.c_cflag=B19200|CS8|CLOCAL|CREAD;
	newtio.c_iflag=IGNPAR;
	newtio.c_oflag=0;
	newtio.c_lflag=0;
	newtio.c_cc[VTIME]=10;
	newtio.c_cc[VMIN]=32;
	
	tcflush(fd,TCIFLUSH);
	tcsetattr(fd,TCSANOW,&newtio);
	fcntl(fd,F_SETFL,FNDELAY);

	return fd;
}

void* sendBattery1Command(void* fd)
{
	int Fd=*((int *)fd);

	// send battery 0x60 data frame
	char CKSUM_0x60=(2*BATTERY1_ADDRESS+SendLength+COMMAND1+BatteryData1+BatteryData2)&0xFF;

	SendData[2]=SendData[5]=BATTERY1_ADDRESS;
	SendData[6]=BatteryData1;
	SendData[7]=BatteryData2;
	SendData[8]=CKSUM_0x60;
 
	for(int i=0;i<11;i++){
		serialPutchar(Fd,SendData[i]);
	}
	printf("Send BAT1 : ");
	debug(SendData,11);
	delay(DELAY_TIME);
	fflush(stdout);
}

void* sendBattery2Command(void* fd)
{
	int Fd=*((int *)fd);

	// send battery 0x61 data frame
	char CKSUM_0x61=(2*BATTERY2_ADDRESS+SendLength+COMMAND1+BatteryData1+BatteryData2)&0xFF;

	SendData[2]=SendData[5]=BATTERY2_ADDRESS;
	SendData[6]=BatteryData1;
	SendData[7]=BatteryData2;
	SendData[8]=CKSUM_0x61;

	for(int i=0;i<11;i++){
		serialPutchar(Fd,SendData[i]);
	}
	printf("Send BAT2 : ");
	debug(SendData,11);
	delay(DELAY_TIME);
	fflush(stdout);
}

void* sendRemoteCommand(void* fd)
{
	int Fd=*((int *)fd);

	// send remote 0x30 data frame
	char CKSUM_0x30=(REMOTE1_ADDRESS+SendLength+COMMAND1+REMOTE1_ADDRESS+RemoteData1+RemoteData2)&0xFF;

	SendData[2]=SendData[5]=REMOTE1_ADDRESS;
	SendData[6]=RemoteData1;
	SendData[7]=RemoteData2;
	SendData[8]=CKSUM_0x30;

	for(int i=0;i<11;i++){
		serialPutchar(Fd,SendData[i]);
	}
	//debug(SendData,11);
		
	gRemoteCommunicationSend++;
		
	delay(DELAY_TIME);
	fflush(stdout);
	
	//communication error
	if(gRemoteCommunicationSend >= 4){
		sendEmergency(fd);
		gRemoteCommunicationSend = 0;
	}

}

void* send_C1_Status(void* fd,int status,char DATA1,char DATA2)
{
	int Fd=*((int *)fd);
	
	int length=0;
	char data_frame_0x40[1024]={0,};

	data_frame_0x40[0]=STX1;
	data_frame_0x40[1]=STX2;
	data_frame_0x40[2]=data_frame_0x40[5]=C1_ADDRESS;
	data_frame_0x40[6]=DATA1;
	
	if((status==1)||(status==3)){
		length=11;
		data_frame_0x40[3]=0x05;
		data_frame_0x40[7]=DATA2;
		data_frame_0x40[9]=ETX1;
		data_frame_0x40[10]=ETX2;

		if(status==1){	// battery status
			data_frame_0x40[4]=COMMAND2;
			data_frame_0x40[8]=(2*C1_ADDRESS+0x05+COMMAND2+DATA1+DATA2)&0xFF;
		}
		else if(status==3){	//remote controller
			data_frame_0x40[4]=COMMAND3;
			data_frame_0x40[8]=(2*C1_ADDRESS+0x05+COMMAND3+DATA1+DATA2)&0xFF;
		}
	}
	else if(status==2){	// battery error
		length=10;
		data_frame_0x40[3]=0x04;
		data_frame_0x40[4]=COMMAND4;
		data_frame_0x40[7]=(2*C1_ADDRESS+0x04+COMMAND4+DATA1)&0xFF;
		data_frame_0x40[8]=ETX1;
		data_frame_0x40[9]=ETX2;
	}
 
	for(int i=0;i<length;i++){
		serialPutchar(Fd,data_frame_0x40[i]);
	}
	printf("C3 -> C1 : ");
	debug(data_frame_0x40,length);
	fflush(stdout);
}
 
int searchData(char *Data,char searchdata, int searchpos)
{
	int searchcount=-1;
	int i;
	
	for(i=0;i<DATASIZE;i++){
		if(Data[i]==searchdata){
			searchcount=i;
			break;
		}
		
	}
	return searchcount;
}
 
void* battery_status(void* fd,char *Cropdata)
{
	//int BATTERY=0;
	char Error=0x00;
	int charging=Cropdata[8]<<8|Cropdata[9];
	
	if(Cropdata[4]==COMMAND3){
		if(Cropdata[2]==BATTERY1_ADDRESS){
			Battery1=Cropdata[6]<<8|Cropdata[7];
			printf("RECEIVE BAT1 : ");
			debug(Cropdata, 13);
		}
		else if(Cropdata[2]==BATTERY2_ADDRESS){
			Battery2=Cropdata[6]<<8|Cropdata[7];
			printf("RECEIVE BAT2 : ");
			debug(Cropdata, 13);
		}
		
//		if(Battery1==0) Battery1=Battery2;
//		if(Battery2==0) Battery2=Battery1;
		BATTERY=((int)Battery1+(int)Battery2)/2;
		//printf("Battery 1 : %d\nBattery 2 : %d\n",Battery1,Battery2);
		printf("Battery average : %d\n",BATTERY);

		
		if(charging==0){ 		// No charging
			battery_charging_flag = 0;
			//printf("battery charging flag = 0\n");
		}
		else{
			battery_charging_flag = 1;
			//printf("battery charging flag = 1\n");
		}
		//battery_led(BATTERY,charging);
		send_C1_Status(fd,1,Battery1,Battery2);
	}
	
	else if(Cropdata[4]==COMMAND4){
		if(Cropdata[2]==BATTERY1_ADDRESS){
			Battery1=Cropdata[5];
		}
		else if(Cropdata[2]==BATTERY2_ADDRESS){
			Battery2=Cropdata[5];
		}
		Error = Battery1 | Battery2;
		send_C1_Status(fd,2,Error,0);
	}	
}

void* battery_led(void* fd)
{
	int Fd=*((int *)fd);
	bool bBlinkStatus = true;


	while (1) {

		// LED_Display Init
		for(int i=0;i<10;i++){
			LED_Display[i] = 0;
		}
		for(int i=0;i<BATTERY/10;i++){
			LED_Display[i] = 1;
		}

		//if(_DEBUG) printf("battery_charging_flag : %d\n", battery_charging_flag);

		if(battery_charging_flag == 0){
			if(_DEBUG)
			{
				//printf("battery value : %d \n",BATTERY);
			}
			// LED display
			for(int i=0;i<10;i++){
				digitalWrite(LED[i],LED_Display[i]);
			}	
		}
		else{ // if(battery_charging_flag == 0){
			if(_DEBUG){
			//	printf("Battery Charging ...\n");
			}
			if(bBlinkStatus) {  
				for(int i=0;i<10;i++){
					digitalWrite(LED[i],LED_Display[i]);
				}
			}
			else { // all off for blink //if(bBlinkStatus) {
				for(int i=0;i<10;i++){
					digitalWrite(LED[i],0);
				}
			}
		} //if(battery_charging_flag == 0){
		delay(1000);
		
		
		// for debug 
		printf("datapos = %d --------------------\n", datapos);
		if(datapos != 0) {
			printf("Data = ");
			debug(Data, datapos);
		}
	}
}
 
void remote_status(void* fd,char *Cropdata)
{
	for(int i=0;i<10;i++){
		digitalWrite(Remote[i],0);
	}
	
	if(Cropdata[6]==0x00 && Cropdata[7]==0x00){
		return ;
	}
	
	if(remote_flag){
		remote_flag=0;
		
		char cropControlFlag=Cropdata[6]&commandControlFlag;
		if(cropControlFlag){
			if(cropControlFlag&0x01){
				commandControlFlag=Cropdata[6]&0x01;
			}
			else if(cropControlFlag&0x02){
				commandControlFlag=Cropdata[6]&0x02;
			}
			else if(cropControlFlag&0x04){
				commandControlFlag=Cropdata[6]&0x04;
			}
			else if(cropControlFlag&0x08){
				commandControlFlag=Cropdata[6]&0x08;
			}
			else if(cropControlFlag&0x10){
				commandControlFlag=Cropdata[6]&0x10;
			}
			send_C1_Status(fd,3,commandControlFlag,0x00);
		}
		else{
			commandControlFlag=0xFF;
		}
		
		char cropWheelFlag=Cropdata[7]&commandWheelFlag;
		if(cropWheelFlag){
			if(cropWheelFlag&0x01){
				digitalWrite(Remote[0],1);
				commandWheelFlag=Cropdata[7]&0x01;
				printf("command 0 \n");
			}
			else if(cropWheelFlag&0x02){
				digitalWrite(Remote[1],1);
				commandWheelFlag=Cropdata[7]&0x02;
				printf("command 1 \n");
			}
			else if(cropWheelFlag&0x04){
				digitalWrite(Remote[2],1);
				commandWheelFlag=Cropdata[7]&0x04;
				printf("command 2 \n");
			}
			else if(cropWheelFlag&0x08){
				digitalWrite(Remote[3],1);
				commandWheelFlag=Cropdata[7]&0x08;
				printf("command 3 \n");
			}
			else if(cropWheelFlag&0x10){
				digitalWrite(Remote[4],1);
				commandWheelFlag=Cropdata[7]&0x10;
				printf("command 4 \n");
			}
			else if(cropWheelFlag&0x20){
				digitalWrite(Remote[5],1);
				commandWheelFlag=Cropdata[7]&0x20;
				printf("command 5 \n");
			}
			else if(cropWheelFlag&0x40){
				digitalWrite(Remote[6],1);
				commandWheelFlag=Cropdata[7]&0x40;
				printf("command 6 \n");
			}
			else if(cropWheelFlag&0x80){
				digitalWrite(Remote[7],1);
				commandWheelFlag=Cropdata[7]&0x80;
				printf("command 7 \n");
			}
		}
		else{
			commandWheelFlag=0xFF;
		}
		remote_flag=1;
	}
}

void debug(char *data_frame,int dataFrameCnt)
{
	if(_DEBUG){
		for(int i=0;i<dataFrameCnt;i++){
			printf("%02X ",data_frame[i]);
		}
		printf("\n");
	}
}

void* sendEmergency(void* fd)
{
	int Fd=*((int *)fd);
	
	printf("\nEmergency All Stop\n");
	
	// IO  off
	for(int i=0;i<10;i++){
		digitalWrite(Remote[i],0);
	}
	
	// send all stop to C1
	send_C1_Status(fd,3,0x00,0x00);
}
