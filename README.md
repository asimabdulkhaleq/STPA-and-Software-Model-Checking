# STPA-and-Software-Model-Checking
This project is an illustrative example of integrated STPA and software modeling (research paper)
Authors: Asim Abdulkhaleq, Stefan Wagner
Software Engineering Group, Institute of Software Technology
University of Stuttgart, Germany 
March, 2015

Prerequisites:

-Download SPIN (http://spinroot.com/spin/whatispin.html) and Modex (http://spinroot.com/modex/) 
- If your operation system is windows: You need to install Cygwin (https://www.cygwin.com/)with gcc

To generate the verification model :
1- run modex - Y ACCsimulator.c 
2- run Spin with command line
 $home/asim/Spin/Scr6.4.3/spin -a model
3- compile the pan files with command 

 gcc -DMEMLIM=1024 -O2 -DXUSAFE -DSAFETY -DNOCLAIM -DSAFETY   -DMA=1520  -DCOLLAPSE  -DVECTORSZ=2048 -w -o pan pan.c -lm
 
-pthread to compile the c code which includes threads.
-lm option to avoid the warning message of using mathematical library such as sqart and abs.
  pan.m:5952:42: warning: incompatible implicit declaration of built-in function ‘sqrt’ [enabled by default]
-DVECTORSZ to avoid the small number of memory. 
The spin will generate the files : pan.m, pan.h and pan


To verify the safety requriements which are written in LTL: 
1- convert LTL into never claim by using SPIN command line 
$ spin -f 'LTL'
 example: spin -f ' [] (p-> q)
 
 The result of this command will be generated such as :
 never  {    /* [] (p-> q) */
	do
	:: (((! ((p))) || ((q)))) -> accept_init
	od;
accept_init:
}
 
 2. Copy the resutl of command into new file neverpq.pml
 3. open the model.xml file and define the properties p and q such as
 #define p c_expr{ Pp_controlSpeed->frontDistance <= now.safeDistance && now.accvehicle.currentspeed >= now.desiredSpeed &&                        now.accMode==cruise}
 #define q c_expr{now.accOperation==accelerate}
 
 4. run the spin with command line : 
 
$ ./pan -m100000  

5. save the results in separate log file: ./pan -m100000 &>logfile.txt
 

 
