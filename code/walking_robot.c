#include <stdio.h>  
#include <stdlib.h>
#include <bcm2835.h>
#include "camera.h"
#include <sys/time.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
// gcc -o test walking-robot.c camera.h -l bcm2835 -lpthread

FILE *fp;
int Y1_position = 0;
int Y1_velocity = 0;
int Y1_previous = 0;
int Electromagnet_Clutch_1 = 0; 

int Y2_position = 0;
int Y2_velocity = 0;
int Y2_previous = 0;
int Electromagnet_Clutch_2 = 0;

int Y3_position = 0;
int Y3_velocity = 0;
int Y3_previous = 0;
int Electromagnet_Clutch_3 = 0;

int Y4_position = 0;
int Y4_velocity = 0;
int Y4_previous = 0;
int Electromagnet_Clutch_4 = 0;

int Y5_position = 0;
int Y5_velocity = 0;
int Y5_previous = 0;
int Electromagnet_Clutch_5 = 0;

int Y6_position = 0;
int Y6_velocity = 0;
int Y6_previous = 0;
int Electromagnet_Clutch_6 = 0;

int Y7_position = 0;
int Y7_velocity = 0;
int Y7_previous = 0;
int Electromagnet_Clutch_7 = 0;

int Y8_position = 0;
int Y8_velocity = 0;
int Y8_previous = 0;
int Electromagnet_Clutch_8 = 0;

struct timeval StartTime;
struct timeval EndTime;
double TimeUse = 0;


void testingT_start()
{
    gettimeofday(&StartTime, NULL);  //measure the time
}

double testingT_end()
{
    gettimeofday(&EndTime, NULL);   //measurement ends
    TimeUse = 1000000*(EndTime.tv_sec-StartTime.tv_sec)+EndTime.tv_usec-StartTime.tv_usec;
    TimeUse /= 1000;  //the result is in the ms dimension
    return TimeUse;
}

// Sensor one
int mousecam_read_reg(int reg)
{
	bcm2835_gpio_write(PIN_MOUSECAM_CS, LOW);	
  	bcm2835_spi_transfer(reg);
	bcm2835_delayMicroseconds(75);
  	int ret = bcm2835_spi_transfer(0xff);
  	bcm2835_gpio_write(PIN_MOUSECAM_CS,HIGH);
	bcm2835_delayMicroseconds(1);
  	return ret;
}

void mousecam_write_reg(int reg, int val)
{
	bcm2835_gpio_write(PIN_MOUSECAM_CS, LOW);
  	bcm2835_spi_transfer(reg | 0x80);
  	bcm2835_spi_transfer(val);
  	bcm2835_gpio_write(PIN_MOUSECAM_CS, HIGH);
	bcm2835_delayMicroseconds(50);
}

int mousecam_init()
{
  	bcm2835_gpio_fsel(PIN_MOUSECAM_CS, 0b001);
  	bcm2835_gpio_write(PIN_MOUSECAM_CS,HIGH);
  	int pid = mousecam_read_reg(ADNS3080_PRODUCT_ID);
  	if(pid != ADNS3080_PRODUCT_ID_VAL)
    	return -1;
  	mousecam_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19);
  	return 0;
}

void mousecam_read_motion(struct MD *p)
{
	bcm2835_gpio_write(PIN_MOUSECAM_CS, LOW);
  	bcm2835_spi_transfer(ADNS3080_MOTION_BURST);
	bcm2835_delayMicroseconds(75);
  	p->motion = bcm2835_spi_transfer(0xff);
  	p->dx =  bcm2835_spi_transfer(0xff);
  	p->dy =  bcm2835_spi_transfer(0xff);
  	bcm2835_gpio_write(PIN_MOUSECAM_CS, HIGH);
  	bcm2835_delayMicroseconds(5);
}

//Sensor two
int mousecam1_read_reg(int reg)
{
	bcm2835_gpio_write(PIN1_MOUSECAM_CS, LOW);
  	bcm2835_spi_transfer(reg);
	bcm2835_delayMicroseconds(75);
  	int ret = bcm2835_spi_transfer(0xff);
  	bcm2835_gpio_write(PIN1_MOUSECAM_CS,HIGH);
	bcm2835_delayMicroseconds(1);
  	return ret;
}

void mousecam1_write_reg(int reg, int val)
{
	bcm2835_gpio_write(PIN1_MOUSECAM_CS, LOW);
  	bcm2835_spi_transfer(reg | 0x80);
  	bcm2835_spi_transfer(val);
  	bcm2835_gpio_write(PIN1_MOUSECAM_CS, HIGH);
	bcm2835_delayMicroseconds(50);
}

int mousecam1_init()
{
  	bcm2835_gpio_fsel(PIN1_MOUSECAM_CS, 0b001);
  	bcm2835_gpio_write(PIN1_MOUSECAM_CS,HIGH);
  	int pid = mousecam1_read_reg(ADNS3080_PRODUCT_ID);
  	if(pid != ADNS3080_PRODUCT_ID_VAL)
    	return -1;
  	mousecam1_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19);
  	return 0;
}

void mousecam1_read_motion(struct MD *p)
{
	bcm2835_gpio_write(PIN1_MOUSECAM_CS, LOW);	
  	bcm2835_spi_transfer(ADNS3080_MOTION_BURST);
	bcm2835_delayMicroseconds(75);
  	p->motion = bcm2835_spi_transfer(0xff);
  	p->dx =  bcm2835_spi_transfer(0xff);
  	p->dy =  bcm2835_spi_transfer(0xff);
  	bcm2835_gpio_write(PIN1_MOUSECAM_CS, HIGH);
  	bcm2835_delayMicroseconds(5);
}

// Sensor three
int mousecam2_read_reg(int reg)
{
	bcm2835_gpio_write(PIN2_MOUSECAM_CS, LOW);	
  	bcm2835_spi_transfer(reg);
	bcm2835_delayMicroseconds(75);
  	int ret = bcm2835_spi_transfer(0xff);
  	bcm2835_gpio_write(PIN2_MOUSECAM_CS,HIGH);	
	bcm2835_delayMicroseconds(1);
  	return ret;
}

void mousecam2_write_reg(int reg, int val)
{
	bcm2835_gpio_write(PIN2_MOUSECAM_CS, LOW);	
  	bcm2835_spi_transfer(reg | 0x80);
  	bcm2835_spi_transfer(val);
  	bcm2835_gpio_write(PIN2_MOUSECAM_CS, HIGH);	
	bcm2835_delayMicroseconds(50);
}

int mousecam2_init()
{
  	bcm2835_gpio_fsel(PIN2_MOUSECAM_CS, 0b001);
  	bcm2835_gpio_write(PIN2_MOUSECAM_CS,HIGH);
  	int pid = mousecam2_read_reg(ADNS3080_PRODUCT_ID);
  	if(pid != ADNS3080_PRODUCT_ID_VAL)
    	return -1;
  	mousecam2_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19);
  	return 0;
}

void mousecam2_read_motion(struct MD *p)
{
	bcm2835_gpio_write(PIN2_MOUSECAM_CS, LOW);	
  	bcm2835_spi_transfer(ADNS3080_MOTION_BURST);
	bcm2835_delayMicroseconds(75);
  	p->motion = bcm2835_spi_transfer(0xff);
  	p->dx =  bcm2835_spi_transfer(0xff);
  	p->dy =  bcm2835_spi_transfer(0xff);
  	bcm2835_gpio_write(PIN2_MOUSECAM_CS, HIGH);
  	bcm2835_delayMicroseconds(5);
}

// Sensor four
int mousecam3_read_reg(int reg)
{
	bcm2835_gpio_write(PIN3_MOUSECAM_CS, LOW);	
  	bcm2835_spi_transfer(reg);
	bcm2835_delayMicroseconds(75);
  	int ret = bcm2835_spi_transfer(0xff);
  	bcm2835_gpio_write(PIN3_MOUSECAM_CS,HIGH);	
	bcm2835_delayMicroseconds(1);
  	return ret;
}

void mousecam3_write_reg(int reg, int val)
{
	bcm2835_gpio_write(PIN3_MOUSECAM_CS, LOW);	
  	bcm2835_spi_transfer(reg | 0x80);	
  	bcm2835_spi_transfer(val);
  	bcm2835_gpio_write(PIN3_MOUSECAM_CS, HIGH);	
	bcm2835_delayMicroseconds(50);
}

int mousecam3_init()
{
  	bcm2835_gpio_fsel(PIN3_MOUSECAM_CS, 0b001);
  	bcm2835_gpio_write(PIN3_MOUSECAM_CS,HIGH);	
  	int pid = mousecam3_read_reg(ADNS3080_PRODUCT_ID);
  	if(pid != ADNS3080_PRODUCT_ID_VAL)
    	return -1;
  	mousecam3_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19);
  	return 0;
}

void mousecam3_read_motion(struct MD *p)
{
	bcm2835_gpio_write(PIN3_MOUSECAM_CS, LOW);	
  	bcm2835_spi_transfer(ADNS3080_MOTION_BURST);
	bcm2835_delayMicroseconds(75);
  	p->motion = bcm2835_spi_transfer(0xff);
  	p->dx =  bcm2835_spi_transfer(0xff);
  	p->dy =  bcm2835_spi_transfer(0xff);
  	bcm2835_gpio_write(PIN3_MOUSECAM_CS, HIGH);
  	bcm2835_delayMicroseconds(5);
}

// Sensor five
int mousecam4_read_reg(int reg)
{
	bcm2835_gpio_write(PIN4_MOUSECAM_CS, LOW);	
  	bcm2835_spi_transfer(reg);
	bcm2835_delayMicroseconds(75);
  	int ret = bcm2835_spi_transfer(0xff);
  	bcm2835_gpio_write(PIN4_MOUSECAM_CS,HIGH);
	bcm2835_delayMicroseconds(1);
  	return ret;
}

void mousecam4_write_reg(int reg, int val)
{
	bcm2835_gpio_write(PIN4_MOUSECAM_CS, LOW);
  	bcm2835_spi_transfer(reg | 0x80);
  	bcm2835_spi_transfer(val);
  	bcm2835_gpio_write(PIN4_MOUSECAM_CS, HIGH);	
	bcm2835_delayMicroseconds(50);
}

int mousecam4_init()
{
  	bcm2835_gpio_fsel(PIN4_MOUSECAM_CS, 0b001);
  	bcm2835_gpio_write(PIN4_MOUSECAM_CS,HIGH);
  	int pid = mousecam4_read_reg(ADNS3080_PRODUCT_ID);
  	if(pid != ADNS3080_PRODUCT_ID_VAL)
    	return -1;
  	mousecam4_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19);
  	return 0;
}

void mousecam4_read_motion(struct MD *p)
{
	bcm2835_gpio_write(PIN4_MOUSECAM_CS, LOW);
  	bcm2835_spi_transfer(ADNS3080_MOTION_BURST);
	bcm2835_delayMicroseconds(75);
  	p->motion = bcm2835_spi_transfer(0xff);
  	p->dx =  bcm2835_spi_transfer(0xff);
  	p->dy =  bcm2835_spi_transfer(0xff);
  	bcm2835_gpio_write(PIN4_MOUSECAM_CS, HIGH);
  	bcm2835_delayMicroseconds(5);
}

// Sensor six
int mousecam5_read_reg(int reg)
{
	bcm2835_gpio_write(PIN5_MOUSECAM_CS, LOW);
  	bcm2835_spi_transfer(reg);
	bcm2835_delayMicroseconds(75);
  	int ret = bcm2835_spi_transfer(0xff);
  	bcm2835_gpio_write(PIN5_MOUSECAM_CS,HIGH);	
	bcm2835_delayMicroseconds(1);
  	return ret;
}

void mousecam5_write_reg(int reg, int val)
{
	bcm2835_gpio_write(PIN5_MOUSECAM_CS, LOW);	
  	bcm2835_spi_transfer(reg | 0x80);
  	bcm2835_spi_transfer(val);
  	bcm2835_gpio_write(PIN5_MOUSECAM_CS, HIGH);	
	bcm2835_delayMicroseconds(50);
}

int mousecam5_init()
{
  	bcm2835_gpio_fsel(PIN5_MOUSECAM_CS, 0b001);
  	bcm2835_gpio_write(PIN5_MOUSECAM_CS,HIGH);	
  	int pid = mousecam5_read_reg(ADNS3080_PRODUCT_ID);
  	if(pid != ADNS3080_PRODUCT_ID_VAL)
    	return -1;
  	mousecam5_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19);
  	return 0;
}

void mousecam5_read_motion(struct MD *p)
{
	bcm2835_gpio_write(PIN5_MOUSECAM_CS, LOW);	
  	bcm2835_spi_transfer(ADNS3080_MOTION_BURST);
	bcm2835_delayMicroseconds(75);
  	p->motion = bcm2835_spi_transfer(0xff);
  	p->dx =  bcm2835_spi_transfer(0xff);
  	p->dy =  bcm2835_spi_transfer(0xff);
  	bcm2835_gpio_write(PIN5_MOUSECAM_CS, HIGH);
  	bcm2835_delayMicroseconds(5);
}

// Sensor seven
int mousecam6_read_reg(int reg)
{
	bcm2835_gpio_write(PIN6_MOUSECAM_CS, LOW);	
  	bcm2835_spi_transfer(reg);
	bcm2835_delayMicroseconds(75);
  	int ret = bcm2835_spi_transfer(0xff);
  	bcm2835_gpio_write(PIN6_MOUSECAM_CS,HIGH);	
	bcm2835_delayMicroseconds(1);
  	return ret;
}

void mousecam6_write_reg(int reg, int val)
{
	bcm2835_gpio_write(PIN6_MOUSECAM_CS, LOW);	
  	bcm2835_spi_transfer(reg | 0x80);
  	bcm2835_spi_transfer(val);
  	bcm2835_gpio_write(PIN6_MOUSECAM_CS, HIGH);	
	bcm2835_delayMicroseconds(50);
}

int mousecam6_init()
{
  	bcm2835_gpio_fsel(PIN6_MOUSECAM_CS, 0b001);
  	bcm2835_gpio_write(PIN6_MOUSECAM_CS,HIGH);	
  	int pid = mousecam6_read_reg(ADNS3080_PRODUCT_ID);
  	if(pid != ADNS3080_PRODUCT_ID_VAL)
    	return -1;
  	mousecam6_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19);
  	return 0;
}

void mousecam6_read_motion(struct MD *p)
{
	bcm2835_gpio_write(PIN6_MOUSECAM_CS, LOW);
  	bcm2835_spi_transfer(ADNS3080_MOTION_BURST);
	bcm2835_delayMicroseconds(75);
  	p->motion = bcm2835_spi_transfer(0xff);
  	p->dx =  bcm2835_spi_transfer(0xff);
  	p->dy =  bcm2835_spi_transfer(0xff);
  	bcm2835_gpio_write(PIN6_MOUSECAM_CS, HIGH);
  	bcm2835_delayMicroseconds(5);
}

// Sensor eight
int mousecam7_read_reg(int reg)
{
	bcm2835_gpio_write(PIN7_MOUSECAM_CS, LOW);	
  	bcm2835_spi_transfer(reg);
	bcm2835_delayMicroseconds(75);
  	int ret = bcm2835_spi_transfer(0xff);
  	bcm2835_gpio_write(PIN7_MOUSECAM_CS,HIGH);
	bcm2835_delayMicroseconds(1);
  	return ret;
}

void mousecam7_write_reg(int reg, int val)
{
	bcm2835_gpio_write(PIN7_MOUSECAM_CS, LOW);	
  	bcm2835_spi_transfer(reg | 0x80);	
  	bcm2835_spi_transfer(val);
  	bcm2835_gpio_write(PIN7_MOUSECAM_CS, HIGH);	
	bcm2835_delayMicroseconds(50);
}

int mousecam7_init()
{
  	bcm2835_gpio_fsel(PIN7_MOUSECAM_CS, 0b001);
  	bcm2835_gpio_write(PIN7_MOUSECAM_CS,HIGH);	
  	int pid = mousecam7_read_reg(ADNS3080_PRODUCT_ID);
  	if(pid != ADNS3080_PRODUCT_ID_VAL)
    	return -1;
  	mousecam7_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19);
  	return 0;
}

void mousecam7_read_motion(struct MD *p)
{
	bcm2835_gpio_write(PIN7_MOUSECAM_CS, LOW);	
  	bcm2835_spi_transfer(ADNS3080_MOTION_BURST);
	bcm2835_delayMicroseconds(75);
  	p->motion = bcm2835_spi_transfer(0xff);
  	p->dx =  bcm2835_spi_transfer(0xff);
  	p->dy =  bcm2835_spi_transfer(0xff);
  	bcm2835_gpio_write(PIN7_MOUSECAM_CS, HIGH);
  	bcm2835_delayMicroseconds(5);
}


void get_info()  
{   static int a = 0;
    mousecam_read_motion(&ma);
    Y1_velocity = (int)ma.dy;
    if (!a && Y1_position !=0){
		Y1_position = 0;
		a = 1;
	}
    Y1_position += Y1_velocity;

	static int b = 0;
	mousecam1_read_motion(&mb);
	Y2_velocity = (int)mb.dy;
	if (!b && Y2_position !=0){
		Y2_position = 0;
		b = 1;
	}
    Y2_position += Y2_velocity;

    static int c = 0;
	mousecam2_read_motion(&mc);
	Y3_velocity = (int)mc.dy;
	if (!c && Y3_position !=0){
		Y3_position = 0;
		c = 1;
	}
    Y3_position += Y3_velocity;
	
	static int d = 0;
	mousecam3_read_motion(&md);
	Y4_velocity = (int)md.dy;
	if (!d && Y4_position !=0){
		Y4_position = 0;
		d = 1;
	}
    Y4_position += Y4_velocity;
	
	static int e = 0;
	mousecam4_read_motion(&me);
	Y5_velocity = (int)me.dy;
	if (!e && Y5_position !=0){
		Y5_position = 0;
		e = 1;
	}
    Y5_position += Y5_velocity;
	
	static int f = 0;
	mousecam5_read_motion(&mf);
	Y6_velocity = (int)mf.dy;
	if (!f && Y6_position !=0){
		Y6_position = 0;
		f = 1;
	}
    Y6_position += Y6_velocity;
	
	static int g = 0;
	mousecam6_read_motion(&mg);
	Y7_velocity = (int)mg.dy;
	if (!g && Y7_position !=0){
		Y7_position = 0;
		g = 1;
	}
    Y7_position += Y7_velocity;
	
	static int h = 0;
	mousecam7_read_motion(&mh);
	Y8_velocity = (int)mh.dy;
	if (!h && Y8_position !=0){
		Y8_position = 0;
		h = 1;
	}
    Y8_position += Y8_velocity;
	
}


void speed_clutch_method()
{
	if(Y1_previous > 0 && Y1_velocity <= 0 && Y1_position > -50){
		Y1_position = 0;
    }
    
	
	if(Y2_previous > 0 && Y2_velocity <= 0 && Y2_position > -50){
        Y2_position = 0;
    }
	
	if(Y3_previous > 0 && Y3_velocity <= 0 && Y3_position > -50){
        Y3_position = 0;
    }

	if(Y4_previous > 0 && Y4_velocity <= 0 && Y4_position > -50){
        Y4_position = 0;
    }

	if(Y5_previous > 0 && Y5_velocity <= 0 && Y5_position > -50){
        Y5_position = 0;
    }

	if(Y6_previous > 0 && Y6_velocity <= 0 && Y6_position > -50){
        Y6_position = 0;
    }

    if(Y7_previous > 0 && Y7_velocity <= 0 && Y7_position > -50){
        Y7_position = 0;
    }

	if(Y8_previous > 0 && Y8_velocity <= 0 && Y8_position > -50){
        Y8_position = 0;
    }

   
    if(Y1_velocity < 0 && Y1_velocity >= -6 && Y1_velocity > Y1_previous && Y1_position < -60 && Electromagnet_Clutch_1 == 0){
        Electromagnet_Clutch_1 = 1;
    }
    Y1_previous = Y1_velocity;

    if(Y2_velocity < 0 && Y2_velocity >= -6 && Y2_velocity > Y2_previous && Y2_position < -60 && Electromagnet_Clutch_2 == 0){
        Electromagnet_Clutch_2 = 1;
    }
    Y2_previous = Y2_velocity;

    if(Y3_velocity < 0 && Y3_velocity >= -6 && Y3_velocity > Y3_previous && Y3_position < -60 && Electromagnet_Clutch_3 == 0){
        Electromagnet_Clutch_3 = 1;
    }
    Y3_previous = Y3_velocity;

    if(Y4_velocity < 0 && Y4_velocity >= -6 && Y4_velocity > Y4_previous && Y4_position < -60 && Electromagnet_Clutch_4 == 0){
        Electromagnet_Clutch_4 = 1;
    }
    Y4_previous = Y4_velocity;

    if(Y5_velocity < 0 && Y5_velocity >= -6 && Y5_velocity > Y5_previous && Y5_position < -60 && Electromagnet_Clutch_5 == 0){
        Electromagnet_Clutch_5 = 1;
    }
    Y5_previous = Y5_velocity;

    if(Y6_velocity < 0 && Y6_velocity >= -6 && Y6_velocity > Y6_previous && Y6_position < -60 && Electromagnet_Clutch_6 == 0){
        Electromagnet_Clutch_6 = 1;
    }
    Y6_previous = Y6_velocity;

    if(Y7_velocity < 0 && Y7_velocity >= -6 && Y7_velocity > Y7_previous && Y7_position < -60 && Electromagnet_Clutch_7 == 0){
        Electromagnet_Clutch_7 = 1;
    }
    Y7_previous = Y7_velocity;

    if(Y8_velocity < 0 && Y8_velocity >= -6 && Y8_velocity > Y8_previous && Y8_position < -60 && Electromagnet_Clutch_8 == 0){
        Electromagnet_Clutch_8 = 1;
    }
    Y8_previous = Y8_velocity;
}

static volatile int file_state = 1;
void intHandler(int i){
    file_state = 0;
    bcm2835_delay(10);
    bcm2835_gpio_clr(electromagnet_1);
	bcm2835_gpio_clr(electromagnet_2);
	bcm2835_gpio_clr(electromagnet_3);
	bcm2835_gpio_clr(electromagnet_4);
	bcm2835_gpio_clr(electromagnet_5);
	bcm2835_gpio_clr(electromagnet_6);
	bcm2835_gpio_clr(electromagnet_7);
	bcm2835_gpio_clr(electromagnet_8);
}

timer_t timerid;

void timer_handler(int signo) 
{
    get_info();
	printf("\nThe velocity is: %d mm/s, The position is: %d mm", Y3_velocity, Y3_position);  
    fflush(stdout);
    if(file_state == 1){
	fprintf(fp,"%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",testingT_end(), Y1_velocity, Y1_position, Electromagnet_Clutch_1, 
		                                                               Y2_velocity, Y2_position, Electromagnet_Clutch_2,
																	   Y3_velocity, Y3_position, Electromagnet_Clutch_3,
																	   Y4_velocity, Y4_position, Electromagnet_Clutch_4,
																	   Y5_velocity, Y5_position, Electromagnet_Clutch_5,
																	   Y6_velocity, Y6_position, Electromagnet_Clutch_6,
																	   Y7_velocity, Y7_position, Electromagnet_Clutch_7,
																	   Y8_velocity, Y8_position, Electromagnet_Clutch_8);
	speed_clutch_method();

// 	if(Electromagnet_Clutch_1 == 1 ){
// 		bcm2835_gpio_set (electromagnet_1);
// 		if(Y2_velocity < -5){
// 			Electromagnet_Clutch_1 = 0;
// 			bcm2835_gpio_clr(electromagnet_1);
// 		}
//    }
   
//    	if(Electromagnet_Clutch_2 == 1 ){
// 		bcm2835_gpio_set (electromagnet_2);
// 		if(Y3_velocity < -5){
// 			 Electromagnet_Clutch_2 = 0;
// 			bcm2835_gpio_clr(electromagnet_2);
			
// 		}
//    }
  
   if(Electromagnet_Clutch_3 == 1 ){
		bcm2835_gpio_set (electromagnet_3);
        if(Y4_velocity < -5){
			Electromagnet_Clutch_3 = 0;
			bcm2835_gpio_clr(electromagnet_3);
			
		}
   }
   
   if(Electromagnet_Clutch_4 == 1 ){
		bcm2835_gpio_set (electromagnet_4);
        if(Y5_velocity < -5){
			Electromagnet_Clutch_4 = 0;
			bcm2835_gpio_clr(electromagnet_4);
			
		}
   }
   
   if(Electromagnet_Clutch_5 == 1 ){
		bcm2835_gpio_set (electromagnet_5);
        if(Y6_velocity < -5){
			Electromagnet_Clutch_5 = 0;
			bcm2835_gpio_clr(electromagnet_5);
			
		}
   }
   
    if(Electromagnet_Clutch_6 == 1 ){
		bcm2835_gpio_set (electromagnet_6);
        if(Y7_velocity < -5){
			Electromagnet_Clutch_6 = 0;
			bcm2835_gpio_clr(electromagnet_6);
			
		}
   }
   
     if(Electromagnet_Clutch_7 == 1 ){
		bcm2835_gpio_set (electromagnet_7);
        if(Y8_velocity < -5){
			Electromagnet_Clutch_7 = 0;
			bcm2835_gpio_clr(electromagnet_7);
			 
		}
   }
  
    if(Electromagnet_Clutch_8 == 1 ){
		bcm2835_gpio_set (electromagnet_8);
      if(Y1_velocity < -5){
		    Electromagnet_Clutch_8 = 0;
			bcm2835_gpio_clr(electromagnet_8);
			 
		}
   }
 }
        else
            fclose(fp); 
}


void setup()
{
	printf("Start setup\n");
  	if(!bcm2835_init())
  	{
    	printf("bcm2835_init failed. Are you running as root??\n");
    	return;
  	}

  	if (!bcm2835_spi_begin())
    {
      	printf("bcm2835_spi_begin failed. Are you running as root??\n");
      	return;
    }
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  	bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);
  	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_512);
  	bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
	bcm2835_gpio_fsel(electromagnet_1, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(electromagnet_2, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(electromagnet_3, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(electromagnet_4, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(electromagnet_5, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(electromagnet_6, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(electromagnet_7, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(electromagnet_8, BCM2835_GPIO_FSEL_OUTP);
	if(mousecam_init()==-1)
  	{
		printf("Mousecam1_init failed.\n");
    		while(1);
  	}

	if(mousecam1_init()==-1)
  	{
		printf("Mousecam2_init failed.\n");
    		while(1);
  	}

	if(mousecam2_init()==-1)
  	{
		printf("Mousecam3_init failed.\n");
    		while(1);
  	}

	if(mousecam3_init()==-1)
  	{
		printf("Mousecam4_init failed.\n");
    		while(1);
  	}

	if(mousecam4_init()==-1)
  	{
		printf("Mousecam5_init failed.\n");
    		while(1);
  	}

	if(mousecam5_init()==-1)
  	{
		printf("Mousecam6_init failed.\n");
    		while(1);
  	}

	if(mousecam6_init()==-1)
  	{
		printf("Mousecam7_init failed.\n");
    		while(1);
  	}

	if(mousecam7_init()==-1)
  	{
		printf("Mousecam8_init failed.\n");
    		while(1);
  	}
}

void* timer_thread_function(void *arg) {
    struct sigevent sev;
    struct itimerspec its;
    timer_t timerid;
    signal(SIGALRM, timer_handler);
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGALRM;
    sev.sigev_value.sival_ptr = &timerid;
    timer_create(CLOCK_REALTIME, &sev, &timerid);
    its.it_value.tv_sec = 0;     
    its.it_value.tv_nsec = 5000000; 
    its.it_interval.tv_sec = 0;  
    its.it_interval.tv_nsec = 5000000; 
    timer_settime(timerid, 0, &its, NULL);
    while (1) {
        pause();  
    }
    return NULL;
}


int main() {
    time_t now;
    struct tm *current_time;
    char date_str[20];
    time(&now);
    current_time = localtime(&now);
    strftime(date_str, sizeof(date_str), "%m%d", current_time);
    char new_filename[50]; 
    snprintf(new_filename, sizeof(new_filename), "./%s_sensors.csv", date_str);

    pthread_t thread_id;
    testingT_start(); 
    fp = fopen(new_filename, "w+");
    if (fp == NULL){
        fprintf(stderr, "fopen() failed.\n");
        printf("Failed\n");
        exit(EXIT_FAILURE);
    }
    fprintf(fp, "Time,Y1_velocity,Y1_position,Clutch_1,Y2_velocity,Y2_position,Clutch_2,Y3_velocity,Y3_position,Clutch_3,Y4_velocity,Y4_position,Clutch_4,Y5_velocity,Y5_position,Clutch_5,Y6_velocity,Y6_position,Clutch_6,Y7_velocity,Y7_position,Clutch_7,Y8_velocity,Y8_position,Clutch_8\n");
    signal(SIGINT, intHandler);
    
    if (pthread_create(&thread_id, NULL, timer_thread_function, NULL) != 0) {
        fprintf(stderr, "Failed to create thread\n");
        exit(EXIT_FAILURE);
    }

    setup();  

    while (1) {
       
    }
    pthread_join(thread_id, NULL);  
    return 0;
}