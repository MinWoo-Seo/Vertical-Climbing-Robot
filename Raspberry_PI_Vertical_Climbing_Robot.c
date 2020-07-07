cd usart-test

gcc -o chat_serv chat_serv.c -lpthread -lwiringPi

sudo ./chat_serv 8000

gcc -o usart usart.c -lwiringPi

sudo ./usart





ID: pi
passward: 1234


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>

#include <errno.h>

#include <wiringPi.h>
#include <wiringSerial.h>
#include <softPwm.h>
#define PIN1 1
#define PIN0 0

#define BUF_SIZE 100
#define MAX_CLNT 256
#define NAME_SIZE 20

void * handle_clnt(void * arg);
void * send_msg(void * arg);
void error_handling(char * msg);

int clnt_cnt=0;
int clnt_socks[MAX_CLNT];
pthread_mutex_t mutx;
char msg[100];
char rev_msg[100];
char sel;
int fd;
int main(int argc, char *argv[])
{
	
	  
   	  //int data;
	  //char sel;
  
	int serv_sock, clnt_sock;
	struct sockaddr_in serv_adr, clnt_adr;
	void * thread_return;
	int clnt_adr_sz;
	pthread_t t_id, snd_thread;
	if(argc!=2) {
		printf("Usage : %s <port>\n", argv[0]);
		exit(1);
	}
  
	pthread_mutex_init(&mutx, NULL);
	serv_sock=socket(PF_INET, SOCK_STREAM, 0);

	memset(&serv_adr, 0, sizeof(serv_adr));
	serv_adr.sin_family=AF_INET; 
	serv_adr.sin_addr.s_addr=htonl(INADDR_ANY);
	serv_adr.sin_port=htons(atoi(argv[1]));
	
	int      option;

option = 1;          
setsockopt( serv_sock, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option) );

	if(bind(serv_sock, (struct sockaddr*) &serv_adr, sizeof(serv_adr))==-1)
		error_handling("bind() error");
	if(listen(serv_sock, 5)==-1)
		error_handling("listen() error");
	
	if ((fd = serialOpen ("/dev/ttyAMA0", 115200)) < 0)
  {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno));
    return 1;
  }

   if(wiringPiSetup()==-1)
        return 1;
		fputs(rev_msg,stdout);
 softPwmCreate(PIN1 ,0, 200);
  softPwmCreate(PIN0 ,0, 200);
        
  printf ("\nRaspberry Pi UART Test");
  
	while(1)
	{
		clnt_adr_sz=sizeof(clnt_adr);
		clnt_sock=accept(serv_sock, (struct sockaddr*)&clnt_adr,&clnt_adr_sz);
		
		pthread_mutex_lock(&mutx);
		clnt_socks[clnt_cnt++]=clnt_sock;
		pthread_mutex_unlock(&mutx);
		
		pthread_create(&t_id, NULL, handle_clnt, (void*)&clnt_sock);
		pthread_create(&snd_thread, NULL, send_msg, (void*)&clnt_sock);
		
		//pthread_detach(t_id);
		//pthread_detach(snd_thread);
		printf("Connected client IP: %s \n", inet_ntoa(clnt_adr.sin_addr));
		
		//getchar();
		
		//sel=(char)rev_msg[0];
		//printf("sel : %c \n",sel);
    /* if(sel == '0'){
       serialPutchar(fd,'0');
       printf ("\naaa", (char)sel);
       }
     else if(sel == '1'){
       serialPutchar(fd,'1');
       printf ("\nbbb", (char)sel);
       }
     else if(sel == '2'){
       serialPutchar(fd,'2');
       printf ("\nbbb", (char)sel);
       }
       else if(sel == '3'){
       serialPutchar(fd,'3');
       printf ("\nbbb", (char)sel);
       }
       else if(sel == '4'){
       serialPutchar(fd,'4');
       printf ("\nbbb", (char)sel);
       }
    
    fflush(stdout);
    */
	}
	close(serv_sock);
	return 0;
}
	
void * handle_clnt(void * arg)
{
	int clnt_sock=*((int*)arg);
	int str_len, i;

	while(1)
	{
		int i;

		str_len = read(clnt_sock, rev_msg, sizeof(rev_msg));

		pthread_mutex_lock(&mutx);
		
		for(i=0; i<clnt_cnt; i++)
			write(clnt_socks[i], rev_msg, str_len);

		pthread_mutex_unlock(&mutx);

		if(str_len==-1) 
			return (void*)-1;

		rev_msg[str_len]=0;
		printf("message from client : %s \n",rev_msg);
		
		
		  sel=(char)rev_msg[0];
		printf("sel : %c \n",sel);
		
		serialPutchar(fd,sel);
       printf ("%c%c%c\n", (char)sel, (char)sel, (char)sel);
       
		if(sel == 'u'){   // right  20
       serialPutchar(fd,'u');
       softPwmWrite(PIN0 ,20);  //+6
       softPwmWrite(PIN1 ,17);  //-3
       /*or(i=14;i>6;i--){
         softPwmWrite(PIN ,i);
         delay(80); 
         * */
        
       printf ("uuu\n", (char)sel);
       }
       if(sel == 'y'){   // 14
       serialPutchar(fd,'y');
       softPwmWrite(PIN0 ,17);  //
       softPwmWrite(PIN1 ,10);  //
       /*for(i=14;i<22;i++){
         softPwmWrite(PIN ,i);
         delay(80);
         * */
         
       printf ("yyy\n", (char)sel);
       }
       
       
       if(sel == 't'){   // left  8
       serialPutchar(fd,'t');
       softPwmWrite(PIN0 ,11);  //-3
       softPwmWrite(PIN1 ,8 );  //+6
       /*for(i=14;i<22;i++){
         softPwmWrite(PIN ,i);
         delay(80);
         * */
         
       printf ("ttt\n", (char)sel);
       }
       
			
}

	pthread_mutex_lock(&mutx);
	
	for(i=0; i<clnt_cnt; i++)   // remove disconnected client
	{
		if(clnt_sock==clnt_socks[i])
		{
			while(i++<clnt_cnt-1)
				clnt_socks[i]=clnt_socks[i+1];
			break;
		}
	}
	clnt_cnt--;
	pthread_mutex_unlock(&mutx);
	close(clnt_sock);

	return NULL;
}
void * send_msg(void * arg)   // send thread main
{
	int sock=*((int*)arg);
	int i;
	while(1) 
	{
		fgets(msg, BUF_SIZE, stdin);

		pthread_mutex_lock(&mutx);
		for(i=0; i<clnt_cnt; i++)
			write(clnt_socks[i], msg, strlen(msg));
		pthread_mutex_unlock(&mutx);
	}
}

void error_handling(char * msg)
{
	fputs(msg, stderr);
	fputc('\n', stderr);
	exit(1);
}
