//****************************************************************************
//ACCwithStopandGoSimulator.c
//---------------
//We created this program in C and tested with Lego mindstrom EV3 
//****************************************************************************
//Linux Includes
//--------------
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <stdbool.h>
// ACC-Mode 
#define accoff (1)
#define standby (2)
#define resume (3)
#define cruise (4)
#define follow (5)
#define stop (6)
 
 
 
//****************************************************************************
// GLOBAL VARIABLES

double frontspeed = 15.458; //20.3194;		
double currentspeed = 0;
int safedistance = 20;
int frontdistance = -1;
int accmode = accoff;
double decelerationratio = 2;
double accelerationratio = 4;
double initialspeed = 1;
double desiredspeed = 20.416; //25.1805;		// Deg/s  (40% of max speed)
double deltaX;
double minimumSpeed = 1.22;
double timeGap = 0;
double temp;
double safetyTimeGap = 0.9;

//PID Controller
double epsilon = 0.01;
//double proportionalgain;
double integralgain = 0;
double derivativegain;
double deltatime = 0.01;
double maxvalue = 200;
double minvalue = 0;
double error;
double preverror = 0;
double output;
double kp = 0.1; //2000
double ki = 0.1; //80
double kd = 0.1; //80

//****************************************************************************

double calPID(double diff) { 
    error = diff;  

	if (( error) > epsilon) {
		integralgain += error * deltatime;
	}

	derivativegain += (error - preverror) / deltatime;
	
	output = (kp * error)
			+ (ki * integralgain)
			+ (kd * derivativegain);

	if (output > maxvalue) {
		output = maxvalue;
	} else if (output < minvalue) {
		output = minvalue;
	}
	//output= transforfunction(output);
	preverror = error;
	temp = output;
	return output;
}

void calcTimeGap() {
	if (currentspeed > 0) {
		timeGap = (frontdistance/currentspeed);
	} else {
		timeGap = 0;
	}
}

void accelerate() {
	if (accmode != cruise && accmode != resume){
		currentspeed += calPID(sqrt(fabs(
			(currentspeed * currentspeed) + 2 * fabs((deltaX + safetyTimeGap) - timeGap)))-currentspeed);
	} else {
		currentspeed = currentspeed;
	} 
	
	if (accmode == resume) {
		currentspeed += accelerationratio;
	}
	
	if (currentspeed > desiredspeed) {
		currentspeed = desiredspeed;
	}
}

void decelerate() {
	currentspeed -= calPID(sqrt(fabs(
		(currentspeed * currentspeed) + 2.7 * fabs(deltaX + safetyTimeGap - timeGap)))-currentspeed);
	if (currentspeed <= 0) {
		currentspeed = 0;
	} else if (currentspeed > desiredspeed) {
		currentspeed = desiredspeed;
	}
}

void goMove(){
	if (accmode != follow) {
		accelerate();
		if (accmode == stop) {
			currentspeed = 0;
		}
	} else if (accmode == follow) {
		decelerate();
	}
	//currentspeed= (int)((currentspeed/17.593)*360);
		//SetSpeed_WithSpeedCtrl(BC, (int)((currentspeed/17.593)*360));				
}

int GetSonarRawValue()
{
	srand(time(NULL));
int r = (rand() % 250);	
return r;
}

//****************************************************************************
// MAIN PROGRAM
//-------------
int main()
{
	
	accmode = standby;
	
	//****************************************************************************
	 //DECLARE VARIABLES
		//-----------------
				
		currentspeed = initialspeed;		
	    int i = 1;
		while(i < 300) 
			{	
				frontdistance = GetSonarRawValue();
				
				if (frontdistance <= -1) {
					frontdistance = 250;
				}
				
				calcTimeGap();
				//deltaX=abs(((318*318)-(currentspeed*currentspeed))/(2*(- accelerationratio)));
				//if(safedistance <= frontdistance) {
					//deltaX=sqrt(abs(((timeGap*timeGap)-(0.5*0.5))/(2*accelerationratio)));
					deltaX = 0.5+sqrt(timeGap);
					if (deltaX < 0) {
						deltaX = 0;
					} else if (deltaX > 10) {
						deltaX = 10;
					}
				//}
				

				if (accmode == standby) {
					if (currentspeed > minimumSpeed) {
						accmode = resume; 
					}  
				} else if (accmode == resume) {
					if (timeGap < (deltaX + safetyTimeGap) && timeGap != 0) {
				    	accmode = follow;
					} else if (currentspeed == desiredspeed && timeGap > safetyTimeGap) {
						accmode = cruise;
					} else if (timeGap == 0) {
						accmode = stop;
					} else if (currentspeed < desiredspeed && timeGap > safetyTimeGap) {
						accmode = resume;
					}
				} else if (accmode == cruise) {
					if (timeGap > (deltaX + safetyTimeGap) && currentspeed == desiredspeed) {
						accmode = cruise;
					} else if (timeGap < (deltaX + safetyTimeGap)) {
						accmode = follow;
					} else if (currentspeed < desiredspeed && timeGap > (deltaX + safetyTimeGap)) {
						accmode = resume;
						
					}
				} else if (accmode == follow) {
					if (timeGap > (deltaX + safetyTimeGap) && frontdistance > 10) {
						accmode = resume;
					} else if (timeGap <= safetyTimeGap && frontdistance < 10) {
						accmode = stop;
					}
				} else if (accmode == stop) {
					if (timeGap > safetyTimeGap || frontdistance > 10) {
						accmode = resume;
					}
				}
				
				goMove();	
			    printf("===Controlspeed====%d======\n", i);
                printf("Frontdistance: %d\n", frontdistance);
			    printf("DeltaX: %lg\n", deltaX);
				printf("Speed: %lg\n", ((currentspeed/17.593)*360));
				printf("SpeedInt: %lg\n", currentspeed);
				printf("AccMode: %d\n", accmode);
				printf("TimeGap: %lg\n", timeGap);
			    printf("Difference value of Speed: %lg\n", temp);
				
				//sleep 0.1s
				nanosleep((struct timespec[]){{0, 100000000}}, NULL);
				
				i = i + 1;
				
				
			}
		 
		
		 
	 
		return 0;
}
