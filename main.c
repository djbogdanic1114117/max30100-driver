#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <math.h>
#include <time.h>

int k=0;

#define BUF_LEN 4
#define RED_current_level 50
#define IR_current_level 46.8

int IRsamples[3000];

int HeartRate[]={0,0,0,0,0,0,0,0,0,0,0};
float spo2=0;
long int RedRms=0;
long int IrRms=0;
void clear_screen(void);

int file_desc;
volatile char done = 0x0;
char tmp[BUF_LEN];

void fun();


static uint16_t samplesIR;
static uint16_t samplesRED;
uint16_t samples[2000];
double sumOfIRSamples=0;

struct coefficients
{
	float b[2];	//numerator
	float a[2];	//denominator
};

union converter
{
  uint16_t source;
  char tgt[sizeof(uint16_t)];
};

static union converter conv;

uint16_t samplesIRconvert()
{
	conv.tgt[0] = tmp[0];
	conv.tgt[1] = tmp[1];
	return conv.source;
}

uint16_t samplesREDconvert()
{
	conv.tgt[0] = tmp[2];
	conv.tgt[1] = tmp[3];
	return conv.source;
}

int *lfilter(struct coefficients transferFunction,int N, int *x)	//N-duzina niza x
{
	float a1,a2,b1,b2;
	a1=transferFunction.a[0];
	a2=transferFunction.a[1];
	b1=transferFunction.b[0];
	b2=transferFunction.b[1];
	
	
	int *y=(int*)calloc(N,sizeof(int));
	int i;
	
	y[0]=b1*x[0];
	
	for(i=1; i<N; i++)
	{
		y[i]=b1*x[i]+b2*x[i-1]-a2*y[i-1];
		y[i]=y[i]/a1;
	}
	
	return y;
}

void *function(void *param)
{
	int counter=1;
	while(done != 'q')
	{
			
			fun();
		
			sumOfIRSamples=sumOfIRSamples/k;
	int i;
	for(i=0; i<k; i++)
	{
		IRsamples[i]=IRsamples[i]-sumOfIRSamples;
		
	}
	//Otklonjena je DC komponenta semplovima
	//filtriramo semplove sa datim koef. dobili smo ih preko
	struct coefficients transferFunction={{0.08636403, 0.08636403},{1,  -0.82727195} };
	
	int minInterval=k/11.01;
	int hr=0;
	int *y=lfilter(transferFunction, i, IRsamples);
	for(i=0; i<k; i++)
	{	
		if(y[i]<-20000)
		{	hr++;
			i+=minInterval;
		}
	
	}
	HeartRate[(counter)%10]=hr;
	counter++;
	int sum=0;
	for(i=0; i<10; i++)
		sum+=HeartRate[i];
	hr=20*sum/(counter>10 ? 10 : counter);
	free(y);
			clear_screen();
			printf("\n HEART RATE: %d \n", hr);
			printf("SPO2: %f \n",spo2);
			fflush(stdout);
	}
	return NULL;
}

void fun()
{
	
	int i=0;
	float R;
	k=0;
	RedRms=0;
	IrRms=0;
	int errnum;
	time_t t=time(NULL)+3;
   while(1)
    {
	     if(time(NULL)>=t)
	     { 
		break;
		}
	     int position;
             file_desc = open("/dev/max30100", O_RDWR);
	     if(file_desc < 0)
        {
	  printf("'/dev/max30100' device isn't open\n");
	  errnum = errno;
	  fprintf(stderr, "Value of errno: %d\n", errno);
	  perror("Error printed by perror");
          fprintf(stderr, "Error opening file: %s\n", strerror( errnum ));
	    
        }
             read(file_desc,tmp,4);
	     samplesIR=samplesIRconvert();
	     samplesRED=samplesREDconvert();
	     //clear_screen();
	     IRsamples[k]=samplesIR;
	     sumOfIRSamples=sumOfIRSamples+samplesIR;
		k++;
	
		RedRms=RedRms+(samplesRED^2);	
		IrRms=IrRms+(samplesIR^2);
		
		R=(sqrt(RedRms)/RED_current_level)/(sqrt(IrRms)/IR_current_level);
		spo2=110-18*R;
		
	
	     close(file_desc);
             //nanosleep((const struct timespec[]){{0, 50000000L}}, NULL);
    }
		
    
    
}

void* get_input(void* param)
{
	while (!done)
	{
		scanf("%c", &done);
	}
	return NULL;
}

void clear_screen(void)
{
	const char* CLEAR_SCREEN_ANSI = "\e[1;1H\e[2J";
	write(STDOUT_FILENO, CLEAR_SCREEN_ANSI, 12);
}

int main()
{
	pthread_t t;
	pthread_t quit_thread;
	//file_desc = open("/dev/max30100", O_RDWR);
        
	
	pthread_create(&t, NULL, function, NULL);
	pthread_create(&quit_thread, NULL, get_input, NULL);
	
	
	//pthread_join(t, NULL);
	
	pthread_join(quit_thread, NULL);
	
	
		
	
	
    return 0;
}
