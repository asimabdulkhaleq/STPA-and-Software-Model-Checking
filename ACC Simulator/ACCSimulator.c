/*
 This program is free software: you can redistribute it and/or modify
    it under the terms of the General Public License. 
This is a simulator of software controller in the Adaptive cruise control system 
	The ACC simulator runs three threads : move_car, run_Sumilator and monitor RadarData.
	The simulator is built based on the existing ACC simulator : AutoPlug-3.0 project 
	  Authors: Asim Abdulkahleq, Stefan Wagner 
	 Institute of Software Technology, University of Stuttgart, Germany
*/

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <pthread.h>
// ACC-Operation 
#define accelerate (1)
#define deccelerate  (2)
#define resume (3)
   // ACC-Mode 

 #define off (1)
#define cruise  (2)
#define standby (3)
 
//Radar Sensor
#define Received (1)
#define NotReceived (0)
//ACC Software controller coefficients
 volatile double timegap=0.3; // secound
  volatile double accelerationRatio = 2.5;
  volatile double deccelerateRatio =3.0;
  volatile double minSpeed = 10.0; // mph.
  volatile double maxSpeed =240;
//ACC opeartion;
volatile int acclerateSignal;
volatile int declerateSignal;
volatile int accOperation;
volatile int accMode; // 0- off , 1- cruise, 3-standby  
volatile int radarData;  //1 detected 0- not detected 
//driver console 
double desiredSpeed=65.0;
double safeDistance=20.0; 
int enable =1; //1 = on, 0 off; 

// Car 

typedef struct car car;
 
struct car
{
	char *name ;
	volatile int accmode;
	int engineOff;
	int engineOn;
	int changesGear;
	double gearRatio[8];
	double wheelRadius[4];
	int pressbrakepedal ;
	volatile double currentspeed;
	volatile double maxSpeed;
	volatile double maxThrottle;
	volatile double brake;
	int ignition ;     
    volatile double throttle;        
    volatile double speed;                
    volatile double distance;             
    volatile double brakepedal;           
    volatile double maxBrake;
	double enginerpmRedLine;
	double shift; 
    double shiftmargin;
	signed int currentGear;
	double airResistance; 
    int ticksPerSecond;

};

  car accvehicle;
  car targetvehicle;

void 
initACCVehicle ()
{
	accOperation =-1; 
        accvehicle.accmode=standby;
	accvehicle.engineOff=0;
	accvehicle.engineOn=1;
	accvehicle.changesGear=0;
	accvehicle.brakepedal=1;
	accvehicle.currentspeed=0;
	accvehicle.maxSpeed=240.0;
	accvehicle.ignition=0;
	accvehicle.gearRatio [0]=0.0;
	accvehicle.gearRatio [1]=3.9*4.5;
	accvehicle.gearRatio [2]=2.9*4.5;
	accvehicle.gearRatio [3]= 2.3*4.5;
	accvehicle.gearRatio [4]=1.87*4.5;
	accvehicle.gearRatio [5]=1.68*4.5;
	accvehicle.gearRatio [6]=1.54*4.5;
	accvehicle.gearRatio [7]=1.46*4.5;
	accvehicle.enginerpmRedLine=1958.26;
	accvehicle.shift=0.95;
	accvehicle.shiftmargin=0.95;
	accvehicle.wheelRadius [0] = 0.3024;
	accvehicle.wheelRadius [1] = 0.3024;
	accvehicle.wheelRadius [2] = 0.3151;
	accvehicle.wheelRadius [3] = 0.3151;
	accvehicle.currentGear=0;
	accvehicle.maxThrottle=10.0;
	accvehicle. airResistance = accvehicle.maxSpeed/accvehicle.maxThrottle; 
    accvehicle. ticksPerSecond = 5;
	accvehicle.distance=15.0; 

}

void 
initTargetVehicle ()
{
	targetvehicle.accmode=off;
	targetvehicle.engineOff=0;
	targetvehicle.engineOn=1;
	targetvehicle.changesGear=0;
	targetvehicle.brakepedal=1;
	targetvehicle.currentspeed=0;
	targetvehicle.maxSpeed=240.0;
	targetvehicle.ignition=0;
	targetvehicle.gearRatio [0]=0.0;
	targetvehicle.gearRatio [1]=3.3*2.5;
	targetvehicle.gearRatio [2]=1.9*5.5;
	targetvehicle.gearRatio [3]= 2.6*2.5;
	targetvehicle.gearRatio [4]=2.0*1.5;
	targetvehicle.gearRatio [5]=1.0*1.5;
	targetvehicle.gearRatio [6]=1.0*2.5;
	targetvehicle.gearRatio [7]=1.0*2.5;
	targetvehicle.enginerpmRedLine=2318.26;
	targetvehicle.shift=0.75;
	targetvehicle.shiftmargin=0.95;
	targetvehicle.wheelRadius [0] = 0.1023;
	targetvehicle.wheelRadius [1] = 0.2023;
	targetvehicle.wheelRadius [2] = 0.2121;
	targetvehicle.wheelRadius [3] = 0.2121;
	targetvehicle.currentGear=0;
	targetvehicle.maxThrottle=10.0;
	targetvehicle. airResistance = accvehicle.maxSpeed/accvehicle.maxThrottle; 
    targetvehicle. ticksPerSecond = 5;
	targetvehicle.distance=15.0; 
       

}

double transforfunction(double s) {
	int m = 1000;  
	int u = 500;  
	int b = 50;  

	return (1/((1000*s) +50));
}

double calPID(double diff) {

	//PID Controller 
  double epsilon ;
  double proportionalgain;
  double integralgain;
double derivativegain;
double preverror;
double   deltatime ;
double   maxvalue ;
double   minvalue ;
double error;
double output;
double   kp ;
double   ki  ;
double   kd ;
epsilon=0.01;
deltatime=0.01;
maxvalue=200;
minvalue=0;
kp=2000;
ki =80;
kd=80;
          preverror = 0;
	 integralgain = 0;
         error = diff;  

	 

	if (( error) > epsilon)  
			{
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
	output = transforfunction(output);
	preverror = error; 
	return output;
}


  
double adjustcarspeed(double vehicleSpeed,  double deltaX, double distance ) {
	double  tspeed ;
      tspeed=0.0;
	if (accOperation == accelerate) {
		tspeed = sqrt(
				(vehicleSpeed * vehicleSpeed)	+ abs(2 * (-accelerationRatio) *  abs((deltaX +   safeDistance) - distance)));
		printf("\n The accvehicle is starting to accelerate speed");

	} else if (accOperation == deccelerate) {
		tspeed = sqrt((vehicleSpeed * vehicleSpeed)		+ abs(2 * (-accelerationRatio) *  abs(  safeDistance - distance)));
		printf("\n The accvehicle is starting to deccelerate the speed");
		
	} else if (accOperation == resume) {
		tspeed = vehicleSpeed + (accelerationRatio * timegap); 
		printf("\n The accvehicle is starting to resume");
	}
	return tspeed;

}

double controlSpeed(double currentSpeed, double frontSpeed, double frontDistance) {
 
 double deltaX;
 double targetSpeed;
 double setSpeed;
 double speed ;
  deltaX=0;
  targetSpeed=0;
  setSpeed=0;
  speed =0;

if (currentSpeed ==0)
	currentSpeed=minSpeed;

 deltaX=abs(((frontSpeed*frontSpeed)-(currentSpeed*currentSpeed))/(2*(- accelerationRatio)));
 
 if(frontDistance<=(safeDistance + deltaX)) {
    if ((frontDistance <= (deltaX + safeDistance)) && (frontDistance  >= safeDistance)) {
                
                accOperation=accelerate;
		speed = adjustcarspeed (currentSpeed,  deltaX,  frontDistance);
	         if (frontSpeed >=speed)
                    {
                  setSpeed=frontSpeed;
                    }else 
                     {
                      setSpeed=speed;

                      }
 
		acclerateSignal=accelerate;
		
		 
		accvehicle.accmode=cruise;
	}
    else if(frontDistance <safeDistance) {
                  accOperation=deccelerate;   
                  speed =adjustcarspeed (currentSpeed,  deltaX,  frontDistance );  
		 if (minSpeed >=speed)
                    {
                  setSpeed=minSpeed;
                    }else 
                     {
                      setSpeed=speed;

                      }
 
 
		   accvehicle.accmode=cruise;
	}
	}
 else if(frontDistance>(safeDistance + deltaX)&&(frontDistance >=safeDistance))
{          accOperation=resume;
	   setSpeed = currentSpeed + (accelerationRatio*timegap); 
       
       
	   accvehicle.accmode=cruise;
    
	}
 
         if (desiredSpeed <=speed)
                    {
                  setSpeed=desiredSpeed;
                    }else 
                     {
                      setSpeed=setSpeed;

                      }
 
 
 
    
  targetSpeed = calPID(setSpeed - currentSpeed);
 
    
printf("\n crruentSpeed =%f frontSpeed=%f", currentSpeed, frontSpeed);
printf("\n frontDistance=%f safeDistance=%f", frontDistance,	safeDistance);
  
 
  
   return targetSpeed;
}

 void * moveCar(void * ptr) {
	  
	if (accvehicle.brakepedal > 0)
		accvehicle.brakepedal = 0;
	else {
		if (accvehicle.throttle < (accvehicle.maxThrottle - 1))
			accvehicle.throttle += 1.0;
		else
			accvehicle.throttle = accvehicle.maxThrottle;
	}

	accMode= accvehicle.accmode;
	 
	if (accMode ==cruise)
 	{       
                 if (radarData ==NotReveived)
                    radarData=Received;
		printf("\n cruise mode");
		accvehicle.speed += controlSpeed(accvehicle.currentspeed, targetvehicle.speed, targetvehicle.distance);
		if (accvehicle.speed > accvehicle.maxSpeed) {
			accvehicle.speed= accvehicle.maxSpeed;

		} else if (accvehicle.speed < minSpeed) {
			accvehicle.speed = minSpeed;
		}
		accvehicle.currentspeed = accvehicle.speed;
		switch (accOperation) {
		 case accelerate:
			   accvehicle.speed = accvehicle.speed+ accelerationRatio;
		       accvehicle.currentspeed = accvehicle.speed;
			   accvehicle.speed += controlSpeed(accvehicle.currentspeed, targetvehicle.speed, targetvehicle.distance);
		      printf("\n accelerate operation");
		 	 break;
			case deccelerate:
				accvehicle.speed = accvehicle.speed- deccelerateRatio;
				accvehicle.currentspeed = accvehicle.speed;
		        accvehicle.speed -= controlSpeed(accvehicle.currentspeed, targetvehicle.speed, targetvehicle.distance);
		        printf("\n deccelerate operation");
		   	break;
			
			case resume:
				printf("\n resume operation");
				accvehicle.currentspeed = desiredSpeed;
				accvehicle.speed +=controlSpeed(accvehicle.currentspeed, targetvehicle.speed, targetvehicle.distance);
		 	 break;

		


	}
		
		
		if (accvehicle.speed > desiredSpeed)
		{
			accvehicle.speed =desiredSpeed;
		}else if (accvehicle.speed<0.0)
		{
			accvehicle.speed=minSpeed;
		}


	}else 
	{
		 accvehicle.speed = (accvehicle.speed+((accvehicle.throttle - accvehicle.speed/accvehicle.airResistance - 2*accvehicle.brakepedal))/accvehicle.ticksPerSecond)*2;
		if (accvehicle.speed > desiredSpeed)
		{
			accvehicle.speed =desiredSpeed;
		}else if (accvehicle.speed<0.0)
		{
			accvehicle.speed=minSpeed;
		}
		  accvehicle.currentspeed = accvehicle.speed;
        if (accvehicle.throttle>0.0)
           accvehicle.throttle-=0.5/accvehicle.ticksPerSecond; //throttle decays
		
		}

	   if (accvehicle.currentspeed <25.0)
		{
			accvehicle.accmode= off;
			accMode=off;
		}
 
 }

void * radarSensorUnit (void *ptr)
{
  targetvehicle.speed=rand()%(240-20+1) + 20;;
  targetvehicle.distance = targetvehicle.distance + (accvehicle.speed/36.0)/targetvehicle.ticksPerSecond;
}



void * runSimulator (void * ptr)
 {
      pthread_t t_radarSensor;
      pthread_t t_moveCar;
         int step=0;  
	 minSpeed = 10.0;
         desiredSpeed=65.0;
         safeDistance=20.0; 

        initACCVehicle ();
		initTargetVehicle();
		accvehicle.name="accVehilce";
		targetvehicle.name="targetvehicle";
                 printf("\n The simulator is started...");  
		printf("\n targetvehicle is moved...");
		accvehicle.speed=minSpeed;
		accvehicle.currentspeed=minSpeed; 
                 radarData=NotReceived;

                                 
                 
                while (1)
			{
				 pthread_create( &t_radarSensor, NULL, &radarSensorUnit, NULL);
                                 pthread_create( &t_moveCar, NULL, &moveCar, NULL);
                                
			if(accvehicle.currentspeed< desiredSpeed)
				 {
					 accvehicle.accmode=standby;
					 
					 accMode=standby;
					  			
					  

					 
			     }else if (accvehicle.currentspeed== desiredSpeed)
				 {
					 radarData=Received;
					 accvehicle.accmode=cruise ;
					 accMode=cruise;				
					  
					 
					  ;
		 
			      }else if (accvehicle.currentspeed > desiredSpeed)
				 {
				  accvehicle.currentspeed=desiredSpeed;
				  
                                 
 
				 }

                             printf("\n =================");
		          step++;
                     pthread_join(t_radarSensor, NULL);
                    pthread_join(t_moveCar, NULL);
			}
          return NULL;

 }

 int 
main(void) {

     pthread_t t_simulator;

      pthread_create( &t_simulator, NULL, &runSimulator, NULL);
 pthread_exit((void*) 0);
	
			

       getchar();


return 0;
} 
