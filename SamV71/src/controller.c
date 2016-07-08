/*
* controller.c
*
* Created: 8/14/2015 12:19:25 PM
*  Author: QWA
*/

#include "controller.h"

struct Robot_Data Robot={.bat_v.full=12.60, .orc_length=0b00010000,.alpha.full=2,.Vx_sp.full=23.2343e+1,.Vy_sp.full=23.2343e+1};
double Vx , Vy , Wr ;

double x[7][1] , dx[7][1] ,xd[7][1] , du[4][1] , ud[4][1] , u[4][1] ,x_OB[7][1] , x_temp_1[7][1] , x_temp_2[7][1] ,x_temp_3[7][1] ;

double camera_d[3][1];

//Xdot = A*X + B*U
//Y = C*X + D*U

double Yd[7] ;// Y desired


double max_ocr = 16383 ;


double A [7][7] =	 {{    -4.2613  ,  2.976e-16  ,-3.3331e-17 ,  -0.0044376 , -0.0037712,  0.0037712 ,   0.0044376 },
{  2.976e-16  ,    -2.8867  ,   0.045755 ,   0.0029584 , -0.0037712, -0.0037712 ,   0.0029584	},
{ -2.604e-15  ,     3.5746  ,    -3.9403 ,       0.035 ,      0.035,      0.035 ,       0.035	},
{    -405.2   ,    270.13   ,    40.907  ,    -7.3211  ,         0 ,         0  ,          0	},
{    -344.36  ,    -344.36  ,     40.907 ,           0 ,    -7.3211,          0 ,           0	},
{    344.36   ,   -344.36   ,    40.907  ,          0  ,         0 ,   -7.3211  ,          0	},
{    405.2    ,   270.13    ,   40.907   ,         0   ,        0  ,        0   ,   -7.3211	}};
	  

double B [7][4] =	{	{	0			,0			,0			,0			}	,
{	0			,0			,0			,0			}	,
{	0			,0			,0			,0			}	,
{	514.7574	,0			,0			,0			}	,
{	0			,514.7574	,0			,0			}	,
{	0			,0			,514.7574	,0			}	,
{	0			,0			,0			,514.7574	}	}	;


//-inv(B'*B)*B'*A	= uFx
double uFx[4][7] =	{{   0.78717  ,   -0.52478   , -0.079469   ,  0.014222   ,         0  ,          0   ,         0  },
{   0.66897  ,    0.66897   , -0.079469   ,         0   ,  0.014222  ,          0   ,         0	 },
{   -0.66897 ,     0.66897  ,  -0.079469  ,          0  ,          0 ,    0.014222  ,          0 },
{   -0.78717 ,    -0.52478  ,  -0.079469  ,          0  ,          0 ,           0  ,   0.014222 }};

//! discretized
// double uFx[4][7] =
//    {{  0.97826  ,  -0.65388  ,  -0.098844 ,    0.014915 ,-6.4163e-06  , 3.6846e-06 , -1.4417e-06  },
//    {  0.83136  ,   0.83244  ,  -0.098751 , -6.4163e-06 ,   0.014915  ,-4.7394e-06 ,  3.6846e-06	 },
//    {  -0.83136 ,    0.83244 ,   -0.098751,   3.6846e-06, -4.7394e-06 ,    0.014915,  -6.4163e-06 },
//    {  -0.97826 ,   -0.65388 ,   -0.098844,  -1.4417e-06,  3.6846e-06 , -6.4163e-06,     0.014915 }};             


//k:state feed back
// double k_sf[4][7] =    {{	-1.2008  ,    1.6935  ,    0.73135 ,   0.0052338,   0.0014925 ,  0.00088455 ,   0.0015616  },
// {	-1.0205  ,   -1.1201  ,    0.66353 ,   0.0014925,   0.0049425 ,    0.001469 ,  0.00088455  },
// {	1.0205   ,  -1.1201   ,   0.66353  , 0.00088455 ,   0.001469  ,  0.0049425  ,  0.0014925   },
// {	1.2008   ,   1.6935   ,   0.73135  ,  0.0015616 , 0.00088455  ,  0.0014925  ,  0.0052338   }};
	
// double k_sf[4][7] =  {{  -0.66013  ,    0.99275   ,   0.46945 ,   0.0044049 ,  0.00098755 ,  0.00065014 ,    0.001051 },
// 	   {    -0.561  ,   -0.56714   ,   0.42425 ,  0.00098755 ,    0.004199 ,  0.00095543 ,  0.00065014 },
// 	   {    0.561   ,  -0.56714    ,  0.42425  , 0.00065014  , 0.00095543  ,   0.004199  , 0.00098755  },
// 	   {  0.66013   ,   0.99275    ,  0.46945  ,   0.001051  , 0.00065014  , 0.00098755  ,  0.0044049  }};
		   
double k_sf[4][7] =	      	  { {  -2.6997   ,    2.9813  ,    0.27636 ,   0.0076231 ,  0.00080656 ,  -0.0010846 ,  0.00061916 } ,
{  -2.2943   ,   -3.0659  ,    0.21198 ,  0.00080656 ,   0.0073562 ,  0.00097043 ,  -0.0010846 } ,
{  2.2943    ,  -3.0659   ,   0.21198  , -0.0010846  , 0.00097043  ,  0.0073562  , 0.00080656  } ,
{  2.6997    ,   2.9813   ,   0.27636  , 0.00061916  , -0.0010846  , 0.00080656  ,  0.0076231  } };

	
double G[7][7]	= {     {  6.6109      ,6.9631e-16  ,  5.583e-16,   -0.0087235 ,  -0.0074135,    0.0074135,    0.0087235},
{  6.9631e-16  ,   7.4575   ,    1.1174 ,   0.0059986  ,  -0.008557 ,   -0.008557 ,   0.0059986 },
{  5.583e-16   ,  1.1174    ,   8.7986  ,   0.021189   ,  0.018921  ,   0.018921  ,   0.021189	 },
{  -87.235     ,  59.986    ,   211.89  ,     25.236   , -0.013695  ,   -0.17802  ,  -0.095042	 },
{  -74.135     ,  -85.57    ,   189.21  ,  -0.013695   ,    25.256  ,   -0.02198  ,   -0.17802	 },
{  74.135      , -85.57     ,  189.21   ,  -0.17802    , -0.02198   ,    25.256   , -0.013695	 },
{  87.235      , 59.986     ,  211.89   , -0.095042    , -0.17802   , -0.013695   ,    25.236	 }};

	
float cycle_time_s, cycle_time_us;
	


void setpoint_generator ( void )
{
	//! Transferring vectors from camera's coordinate system to Robot coordinate system
	float cos_alpha = cos(Robot.alpha.full/1000.0);
	float sin_alpha = sin(Robot.alpha.full/1000.0);
	Vx = (  Robot.Vx_sp.full * cos_alpha + Robot.Vy_sp.full * sin_alpha ) / 1000.0;
	Vy = ( -Robot.Vx_sp.full * sin_alpha + Robot.Vy_sp.full * cos_alpha ) / 1000.0;
	Wr = Robot.Wr_sp.full / 1000.0;
	// 7 set points for system
	//kinematics rules that should be considered for specifying desired output
	
	// 	xd=(C'*C)\C'*Yd;%inv(C'*C)*C'*Yd; & C = I7x7		=>		xd = Yd
	xd[0][0] = Vx ;
	xd[1][0] = Vy ;
	xd[2][0] = Wr ;
	xd[3][0] = (-Vx*sina1+Vy*cosa1+Wr*d)*b ;
	xd[4][0] = (-Vx*sina2+Vy*cosa2+Wr*d)*b ;
	xd[5][0] = (-Vx*sina3+Vy*cosa3+Wr*d)*b ;
	xd[6][0] = (-Vx*sina4+Vy*cosa4+Wr*d)*b ;
	
	// data checking 1 : data is produced correctly (checked with model in MATLAB)
}

void state_generator ( void )
{
	//! Transferring vectors from camera's coordinate system to Robot coordinate system
	float cos_alpha = cos(Robot.alpha.full/1000.0);
	float sin_alpha = sin(Robot.alpha.full/1000.0);
	Vx = (  Robot.Vx.full * cos_alpha + Robot.Vy.full * sin_alpha ) / 1000.0;
	Vy = ( -Robot.Vx.full * sin_alpha + Robot.Vy.full * cos_alpha ) / 1000.0;
	Wr = Robot.Wr.full / 100.0;
	// 7 set points for system
	//kinematics rules that should be considered for specifying desired output
	
	// 	xd=(C'*C)\C'*Yd;%inv(C'*C)*C'*Yd; & C = I7x7		=>		xd = Yd
	x[0][0] = Vx ;
	x[1][0] = Vy ;
	x[2][0] = Wr ;
	x[3][0] = Robot.W0.full ;
	x[4][0] = Robot.W1.full ;
	x[5][0] = Robot.W2.full ;
	x[6][0] = Robot.W3.full ;
	
	// data checking 1 : data is produced correctly (checked with model in MATLAB)
}

void camera_data ( void )
{
		float cos_alpha = cos(Robot.alpha.full/1000.0);
		float sin_alpha = sin(Robot.alpha.full/1000.0);
		camera_d[0][0] = (x_OB[0][0]*1000 / sin_alpha - x_OB[1][0]*1000/ cos_alpha ) / (cos_alpha / sin_alpha  + sin_alpha/ cos_alpha )  ;
	
}

void state_feed_back ( void )
{
	// 	ud=-inv(B'*B)*B'*A*xd;
	
	for (int i=0 ; i < 4 ; i ++)
	{
		for (int j = 0 ; j < 1 ; j ++)
		{
			ud [i][j] = 0 ;
			for (int k = 0 ; k < 7 ; k ++)
			{
				ud [i][j] += uFx [i][k] * xd [k][j];
			}
		}
	}
	
	// data checking 2 : data is produced correctly (checked with model in MATLAB)

	// 	du=-K*(x-xd);%for simulating controller with unnoisy data
	// 	%du=-K*(xl-xd);%for simulating controller and observer:(x-xd)|(xh-xd)|(xl-xd)
	
	for (int i=0 ; i < 7 ; i ++)
	{
		for (int j = 0 ; j < 1 ; j ++)
		{
			
			dx [i][j] = xd [i][j] - x_OB [i][j] ;  // minus is here <<<<<=

		}
	}
	
	
	for (int i=0 ; i < 4 ; i ++)
	{
		for (int j = 0 ; j < 1 ; j ++)
		{
			du [i][j] = 0 ;
			for (int k = 0 ; k < 7 ; k ++)
			{
				du [i][j] += k_sf [i][k] * dx [k][j];
			}
		}
	}
	
	
	
	// 	u=ud+du;
	
	for (int i=0 ; i < 4 ; i ++)
	{
		for (int j = 0 ; j < 1 ; j ++)
		{
			u [i][j] = (du [i][j] + ud [i][j]);
			// 2.8 v is gained with experiment : it should be applied to confront resistive forces
			u [i][j] += sign(u [i][j]) * 2.9 ;
			
			//emitting saturation
			if (fabs(u[i][j]) > Robot.bat_v.full)
			{
				u[i][j] = sign(u[i][j]) * (Robot.bat_v.full *.997) ;// 0.997 since in transfer function of volt to OCR 100% occurs in 99.7% of full voltage
			}
		}
	}
	
}

void observer ( void )
{
	//xhp=Ah*xh+Bh*uh+G*(y-Ch*xh);
	
		//(y-Ch*xh)
		for (int i=0 ; i < 7 ; i ++)
		{
// 			for (int j = 0 ; j < 1 ; j ++)
// 			{
			int j = 0;
				x_temp_1 [i][j] = (x [i][j] - x_OB [i][j]) ;
/*			}*/
		}
		//xhp+=G*(y-Ch*xh)
		for (int i=0 ; i < 7 ; i ++)
		{
// 			for (int j = 0 ; j < 1 ; j ++)
// 			{
				int j = 0;
				x_temp_2 [i][j] = 0;
				for (int k = 0 ; k < 7 ; k ++)
				{
					x_temp_2 [i][j] += G [i][k] * x_temp_1 [k][j]*cycle_time_s;
				}
/*			}*/
		}
	
	//xhp+=Ah*xh
	for (int i=0 ; i < 7 ; i ++)
	{
// 		for (int j = 0 ; j < 1 ; j ++)
// 		{
			int j = 0;
			x_temp_3 [i][j] = 0;
			for (int k = 0 ; k < 7 ; k ++)
			{
				x_temp_3 [i][j] += A [i][k] * x_OB [k][j]*cycle_time_s;
			}
/*		}*/
	}
	
	//xhp+=Bh*uh
	for (int i=0 ; i < 7 ; i ++)
	{
// 		for (int j = 0 ; j < 1 ; j ++)
// 		{
			int j = 0;
			for (int k = 0 ; k < 4 ; k ++)
			{
				x_OB [i][j] += B [i][k] * ud [k][j]*cycle_time_s;
			}
			x_OB [i][j] += x_temp_2[i][j] + x_temp_3[i][j];
/*		}*/
	}
	


}



//void RLS (void)
//{
//
//p=p/landa-(p*(xk'*xk)*p)/(landa+xk*p*xk')/landa
//q=q+xk'*yk;
////         %x1
////         %x2
////         %x3
////         %x4
////         %x5
////    q=q+[%x6]*[y1 y2 y3 y4 y5 y6 y7 y8 y9 y10 y11]
////         %x7
////         %x8
////         %x9
////         %x10
////         %x11
//
//theta(:,:)=p*q;
//}


double sign (double number)
{
	if (number > 0)
	{
		return 1 ;
	}
	else if (number < 0)
	{
		return -1 ;
	}
	else
	{
		return 0 ;
	}
}

inline void ocr_change(void)
{
	// TODO correct it
	Robot.orc_length=127;
	max_ocr = (Robot.orc_length << 8) - 1 ;
}