/*
 * controller.h
 *
 * Created: 8/14/2015 12:18:19 PM
 *  Author: QWA
 */ 



#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <math.h>
#include <asf.h>
//#include "functions.h"
//geometric constants

#define a1 56.31/180*M_PI    // 0.9827949017980069  rad
#define a2 135/180*M_PI      // 2.356194490192345   rad
#define a3 225/180*M_PI      // 3.926990816987241   rad
#define a4 303.69/180*M_PI   // 5.300390405381579   rad

//

#define sina1 0.8321   
#define sina2 0.7071   
#define sina3 -0.7071  
#define sina4 -0.8321  

#define cosa1 0.5547  
#define cosa2 -0.7071 
#define cosa3 -0.7071 
#define cosa4 0.5547   


//robot constants
#define N	3.8              //         %76/20
#define res 1.2              //       %ohm
#define km	25.5/1000        //        %Nm/A
#define kn	374.0            //        %rpm/V
#define kf	0.0001           //        %unknown
#define ks	0.01             //       %unknown
#define r	28.5/1000        //         %m
#define J	0.0192           //         %kg*m2%           >>modeling needed
#define Jm	92.5/1000/10000  //        %kg*m2								//          0.00000925
#define Jw	0.0000233        //        %kg*m2        >>modeling needed		//          0.00000642  obtained from SOLIDWORKS's model
#define d	0.084            //         %m
#define M	1.5	             //          %kg         >>need measuring

#define b	60/(2*M_PI*r)


#define landa 0.99999

//run time : 4694 clk 
void setpoint_generator ( void ) ;

//run time : 12407 clk
void state_feed_back ( void ) ;

void state_generator ( void );

void camera_data ( void );

void observer ( void ) ;

double sign ( double number ) ;

void ocr_change(void) ;


typedef union High_Low{
	uint8_t byte[2] ;
	int16_t full ;
} HL;

typedef union Float_High_Low{
	uint8_t byte[2] ;
	float full ;
} FHL;

struct Robot_Data
{
	//! Wireless data
	uint8_t RID;
	HL Vx_sp ;
	HL Vy_sp ;
	HL Wr_sp ;
	HL Vx ;
	HL Vy ;
	HL Wr ;
	HL alpha ;
	uint8_t KICK;
	uint8_t CHIP;
	uint8_t SPIN;
	uint8_t ASK;
	
	//! GYRO data
	uint8_t GVxh;
	uint8_t GVxl;
	uint8_t GVyh;
	uint8_t GVyl;
	uint8_t GWh;
	uint8_t GWl;
	
	//! Wheels' speed setpoint
	HL W0_sp	;
	HL W1_sp	;
	HL W2_sp	;
	HL W3_sp	;
	
	uint8_t orc_length;
	
	//Wheels' speed
	HL W0	;
	HL W1	;
	HL W2	;
	HL W3	;
	
	//motors fault
	uint16_t W0_warning	;
	uint16_t W1_warning	;
	uint16_t W2_warning	;
	uint16_t W3_warning	;
	
	//! Motors' current
	FHL I0;
	FHL I1;
	FHL I2;
	FHL I3;
	
	//! MCU's temperature
	HL MCU_temperature;
	
	//! Battery voltage
	FHL bat_v;
	HL  batx1000;
	
	//! Spin_back's speed setpoint
	int8_t SB_sp	;
	
	//! Spin_back's speed
	HL SB	;
	
	//! wireless signal_strength
	uint8_t ss;
	//! wireless_reset_counter
	uint8_t wrc;
	
	//! SPARTAN3 & Atxmega64 signal_strength
	//! Number of sent packet from Atxmega64 to SPARTAN3
	uint8_t nsp;
	//! Number of received packet from SPARTAN3
	uint8_t nrp;
	
	// Charging time of boost circuit
	uint16_t ct;
	
};

extern struct Robot_Data Robot;

extern double Vx , Vy , Wr ;

extern double x[7][1] , x_OB[7][1] , dx[7][1] ,xd[7][1] , du[4][1] , ud[4][1] , u[4][1] ;

extern double camera_d[3][1];

extern double Yd[7] ;

extern double A [7][7] ;

extern double B [7][4] ;

extern double C [7][7] ;

//  -inv(B'*B)*B'*A	= uFx					
extern double uFx[4][7] ;
	
//k:state feed back	
extern double k_sf[4][7] ;

extern double max_ocr ;

extern float cycle_time_s, cycle_time_us;

#endif /* CONTROLLER_H_ */