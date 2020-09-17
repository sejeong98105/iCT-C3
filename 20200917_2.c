#include <stdio.h> 
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

#define PORT1 "/dev/ttyUSB0"

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

char Getdata[1024]={0,};
char Data[1024]={0,};
int LED[10]={18,23,24,25,9,4,17,27,22,10};
int Remote[10]={26,19,13,6,5,11,10,21,20,16};

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

void* getStatus(void* fd);
void* datacopy(void* fd);
int searchData(char *data,char searchdata);

void* battery_led(void* fd);
void remote_status(void* fd,char *Cropdata);
void* battery_status(void* fd,char *Cropdata);

int main()
{
	int status;
	int Fdport=openSerial(PORT1);

	pthread_t p_thread[4];

	pthread_create(&p_thread[0],NULL,sendCommand,(void *)&Fdport);
	pthread_create(&p_thread[1],NULL,getStatus,(void *)&Fdport);
	pthread_create(&p_thread[2],NULL,datacopy,(void *)&Fdport);
	pthread_create(&p_thread[3],NULL,battery_led,(void *)&Fdport);
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
	}
	for(int i=0;i<10;i++){
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

void* sendCommand(void* fd)
{
	int Fd=*((int *)fd);

	SendData[0]=STX1;
	SendData[1]=STX2;
	SendData[3]=SendLength;
	SendData[4]=COMMAND1;
	SendData[9]=ETX1;
	SendData[10]=ETX2;

	while(1){
		sendRemoteCommand(fd);
		sendBattery1Command(fd);
		sendRemoteCommand(fd);
		sendBattery2Command(fd);
	}
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
	//debug(SendData,11);
	delay(500);
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
	//debug(SendData,11);
	delay(500);
	fflush(stdout);
}

void* sendRemoteCommand(void* fd)
{
	int Fd=*((int *)fd);

	for(int cnt=0;cnt<5;cnt++){

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
		
		delay(100);
		fflush(stdout);
/*
		// send remote 0x31 data frame
		char CKSUM_0x31=(REMOTE2_ADDRESS+SendLength+COMMAND1+REMOTE2_ADDRESS+RemoteData1+RemoteData2)&0xFF;

		SendData[2]=SendData[5]=REMOTE2_ADDRESS;
		SendData[8]=CKSUM_0x31;

		for(int i=0;i<11;i++){
			serialPutchar(Fd,SendData[i]);
		}
		debug(SendData,11);
		
		gRemoteCommunicationSend++;
		
		delay(100);
		fflush(stdout);
*/		
		//communication error
		if(gRemoteCommunicationSend >= 4){
			sendEmergency(fd);
		}
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
 
void* getStatus(void* fd)
{
	int Fd=*((int *)fd);
	
	int getDataCnt;
	struct termios newtio;
	struct pollfd poll_events;
	int poll_state;

	poll_events.fd=Fd;
	poll_events.events=POLLIN|POLLERR;
	poll_events.revents=0;
	
	while(1){
		poll_state=poll(
						(struct pollfd*)&poll_events,2,5000
						);
		if(0<poll_state){
			if(poll_events.revents&POLLIN){
				delay(100);
				getDataCnt=read(Fd,Getdata,1024);
				
				memcpy(Data+datapos,Getdata,getDataCnt);
				datapos+=getDataCnt;
				getDataCnt=0;
				memset(Getdata,0,1024);
			}
			// error check
			if(poll_events.revents&POLLERR) {
				printf("Error");
			}
		}
	}
}

void* datacopy(void* fd) 
{
	int i=0;
	int searchcountstx1=0;
	int searchcountetx2=0;
	int searchlength=0;
	int cropsize=0;
	int cropflag=0;
	
	char Tempdata[1024]={0,};	
	char Cropdata[1024]={0,};
	
	while(1){
		if(datapos >= MINIMAL_SIZE_OF_COMMAND_SET){
			searchcountetx2=searchData(Data,ETX2);
			if(searchcountetx2 != -1){
				searchcountstx1=searchData(Data,STX1);
				if(searchcountstx1 != -1){
					if(searchcountetx2 > searchcountstx1){
						searchlength = searchcountetx2 - searchcountstx1 + 1;
 						if(Data[searchcountstx1]==STX1 && Data[searchcountstx1+1]==STX2){
							memcpy(Cropdata,Data+searchcountstx1,searchlength*sizeof(char));
							// data shift
							memcpy(Tempdata,Data,DATASIZE*sizeof(char));		
							memset(Data,0,DATASIZE*sizeof(char));
							memcpy(Data,Tempdata+searchlength,(DATASIZE-searchlength)*sizeof(char));
						
							datapos-=searchlength;
							cropsize=searchlength;
							cropflag=1;
							printf("cropflag=1\n");
						}
					}
				}
			}
						
			if(cropflag){
				printf("Cropdata : ");
				debug(Cropdata,cropsize);
				if(Cropdata[2]==BATTERY1_ADDRESS || Cropdata[2]==BATTERY2_ADDRESS){
					battery_status(fd,Cropdata);
				}
				if(Cropdata[2]==REMOTE1_ADDRESS){
				//if(Cropdata[2]==REMOTE1_ADDRESS || Cropdata[2]==REMOTE2_ADDRESS){
					gRemoteCommunicationSend = 0;
					remote_status(fd,Cropdata);
				}
				cropflag=0;
			}	
		}
		delay(10);
	}
}
	
int searchData(char *Data,char searchdata)
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
		}
		else if(Cropdata[2]==BATTERY2_ADDRESS){
			Battery2=Cropdata[6]<<8|Cropdata[7];
		}
		
		if(Battery1==0) Battery1=Battery2;
		if(Battery2==0) Battery2=Battery1;
		BATTERY=((int)Battery1+(int)Battery2)/2;
		printf("Battery 1 : %d\nBattery 2 : %d\n",Battery1,Battery2);
		printf("Battery average : %d\n",BATTERY);
		if(charging==0){ 		// No charging
			battery_charging_flag = 0;
			printf("battery charging flag = 0\n");
		}
		else{
			battery_charging_flag = 1;
			printf("battery charging flag = 1\n");
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
	
	printf("battery_charging_flag : %d\n", battery_charging_flag);
	if(battery_charging_flag == 0){
		if(_DEBUG)
		{
			printf("battery value : %d \n",BATTERY);
		}
		// LED display
		for(int i=0;i<BATTERY/10;i++){
			digitalWrite(LED[i],1);
		}
		for(int i=9;i>=BATTERY/10;i--){
			digitalWrite(LED[i],0);
		}		
	}
	else{
		if(_DEBUG){
			printf("Battery Charging ...\n");
		}
		for(int i=0;i<BATTERY/10;i++){
			digitalWrite(LED[i],1);
		}
		for(int i=9;i>=BATTERY/10;i--){
			digitalWrite(LED[i],0);
		}
		delay(1000);
		
		for(int i=0;i<10;i++){
			digitalWrite(LED[i],0);
		}
		delay(1000);
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
