#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/poll.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>

#define PORT "/dev/ttyAMA0"

#define _DEBUG 1
#define DELAY_TIME 100

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
#define BATTERY_DATA1 0x14
#define BATTERY_DATA2 0x00
#define RC_DATA 0x00
#define BUFFER_SIZE 1024

int openSerial(char *device_name);
void debug(char *dataFrame, int dataFrameCnt);
void sendEmergency();
int sendCommand();
void sendRemoteCommand();
void sendBattery1Command();
void sendBattery2Command();

// Timer
int createTimer(char *name, timer_t *timerID, int sec, int mesc);
void timer_handler(int sig, siginfo_t *si, void *uc);

timer_t sendCommadTimerID;
timer_t timerID2;
timer_t timerID3;

int Fdport;
int sendCommandCount = 0;
int gRemoteCommunicationSend = 0;
char sendLength = 0x05;

char sendData[1024] = {0,};
char getData[1024] = {0,};
char cropData[1024] = {0,};

// IO port
int BatteryGPIO[10] = {10, 22, 27, 17, 4, 9, 25, 24, 23, 18};
int RemoteGPIO[10] = {26, 19, 13, 6, 5, 11, 20, 21, 0, 0};

int main()
{
	Fdport = openSerial(PORT);
	
	createTimer("First Timer", &sendCommadTimerID, 5, 0);
		
	while(1){}
	close(Fdport);
}

int openSerial(char *device_name)
{
	struct termios newtio;
	
	// Open serial port
	Fdport = open(device_name, O_RDWR|O_NOCTTY|O_NONBLOCK);
	if(Fdport < 0){
		fprintf(stderr, "Unable to open serial device : %s\n", strerror(errno));
		return -1;
	}
	
	// Set up gpio
	wiringPiSetupGpio();
	if(wiringPiSetup() == -1){
		fprintf(stdout, "Unable to start wiringPi : %s\n", strerror(errno));
		return -1;
	}
	for(int i=0; i<10; i++){
		pinMode(BatteryGPIO[i],OUTPUT);
		pinMode(RemoteGPIO[i],OUTPUT);
	}
	
	// Set up serial port
	memset(&newtio, 0, sizeof(newtio));
	
	newtio.c_cflag = B19200|CS8|CLOCAL|CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	newtio.c_cc[VTIME] = 10;
	newtio.c_cc[VMIN] = 32;
	
	tcflush(Fdport, TCIFLUSH);
	tcsetattr(Fdport, TCSANOW, &newtio);
	fcntl(Fdport, F_SETFL, FNDELAY);
	
	return Fdport;
}

int createTimer(char *name, timer_t *timerID, int sec, int mesc)
{
	struct sigevent te;
	struct itimerspec its;
	struct sigaction sa;
	int sigNo = SIGRTMIN;
	
	// Set up signal handler
	sa.sa_flags = SA_SIGINFO;
	sa.sa_sigaction = timer_handler;
	sigemptyset(&sa.sa_mask);
	
	if(sigaction(sigNo, &sa, NULL) == -1){
		printf("sigaction error\n");
		return -1;
	}
	
	// Set and enable alarm
	te.sigev_notify = SIGEV_SIGNAL;
	te.sigev_signo = sigNo;
	te.sigev_value.sival_ptr = timerID;
	timer_create(CLOCK_REALTIME, &te, timerID);
	
	its.it_interval.tv_sec = sec;
	its.it_interval.tv_nsec = mesc * 1000000;
	its.it_value.tv_sec = sec;
 	
	its.it_value.tv_nsec = mesc * 1000000;
	timer_settime(*timerID, 0, &its, NULL);
	
	return 0;
	
}

void timer_handler(int sig, siginfo_t *si, void *uc)
{
	timer_t *tidp;
	tidp = si -> si_value.sival_ptr;
	
	if(*tidp == sendCommadTimerID){
		printf("sendCommadTimerID\n");
		sendCommand();
	}
	else if(*tidp == timerID2){
		printf("getDataTimerID\n");
	}
	else if(*tidp == timerID3){
		printf("ThirdTimerID\n");
	}
}

int sendCommand()
{
	sendData[0] = STX1;
	sendData[1] = STX2;
	sendData[3] = sendLength;
	sendData[4] = CMD1;
	sendData[9] = ETX1;
	sendData[10] = ETX2;
	
	sendCommandCount = sendCommandCount % 10;
	switch(sendCommandCount){
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
	sendCommandCount++;
	// sendCommandCount = sendCommandCount % 10; ????
	
	return sendCommandCount;
}

void sendRemoteCommand()
{
	/* send remote 0x30 data frame 
	  AF FA 30 05 01 30 00 00 66 AF A0 */
	char CKSUM = (RC_ADDRESS + sendLength + CMD1 + RC_ADDRESS + RC_DATA + RC_DATA) & 0xFF;
	
	sendData[2] = sendData[5] = RC_ADDRESS;
	sendData[6] = sendData[7] = RC_DATA;
	sendData[8] = CKSUM;
	
	for(int i=0; i<11; i++){
		serialPutchar(Fdport, sendData[i]);
	}
	if(_DEBUG){
		printf("send  REMOTE  : ");
		debug(sendData, 11);
	}
	
	gRemoteCommunicationSend++;
	
	delay(DELAY_TIME);
	fflush(stdout);
	
	// communication error
	if(gRemoteCommunicationSend >= 4){
		sendEmergency();
		gRemoteCommunicationSend = 0;
	}
}

void sendBattery1Command()
{
	/* send battery 0x60 data frame 
	  AF FA 60 05 01 60 14 00 DA AF A0 */
	char CKSUM = (BATTERY1_ADDRESS + sendLength + CMD1 + BATTERY1_ADDRESS + BATTERY_DATA1 + BATTERY_DATA2) & 0xFF;
	
	sendData[2] = sendData[5] = BATTERY1_ADDRESS;
	sendData[6] = BATTERY_DATA1;
	sendData[7] = BATTERY_DATA2;
	sendData[8] = CKSUM;
	
	for(int i=0; i<11; i++){
		serialPutchar(Fdport, sendData[i]);
	}
	if(_DEBUG){
		printf("send BATTERY1 : ");
		debug(sendData, 11);
	}
	
	delay(DELAY_TIME);
	fflush(stdout);
}

void sendBattery2Command()
{
	/* send battery 0x61 data frame 
	  AF FA 61 05 01 61 14 00 DC AF A0 */
	char CKSUM = (BATTERY2_ADDRESS + sendLength + CMD1 + BATTERY2_ADDRESS + BATTERY_DATA1 + BATTERY_DATA2) & 0xFF;
	
	sendData[2] = sendData[5] = BATTERY2_ADDRESS;
	sendData[6] = BATTERY_DATA1;
	sendData[7] = BATTERY_DATA2;
	sendData[8] = CKSUM;
	
	for(int i=0; i<11; i++){
		serialPutchar(Fdport, sendData[i]);
	}
	if(_DEBUG){
		printf("send BATTERY2 : ");
		debug(sendData, 11);
	}
	
	delay(DELAY_TIME);
	fflush(stdout);
}

void debug(char *dataFrame, int dataFrameCnt)
{
	for(int i=0; i<dataFrameCnt; i++){
		printf("%02X ", dataFrame[i]);
	}
	printf("\n");
}
/*
int serialToRingbuffer()
{
	dataCnt = read(Fdport, getData, BUFFER_SIZE);
	printf("Input Ring Data : ");
	for(int i=0; i<dataCnt; i++){
		bufferWrite(&RingBuffer,getData[i]);
		printf("02x ", getData[i]);
	}
	printf("\n");
	
	memset(getData, 0, sizeof(char)*BUFFER_SIZE);
} 
*/
void sendEmergency()
{
	printf("Emergency All Stop\n");
	
	// IO off
	for(int i=0; i<10; i++){
		digitalWrite(RemoteGPIO[i],0);
	}
	
	// send all stop to C1
	//send_to_C1(3, 0x00, 0x00);
}









