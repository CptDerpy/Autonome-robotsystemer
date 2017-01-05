/*
 * An example SMR program.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct {
   double x, y, z, omega, phi, kappa, code,id,crc;
} gmk;
double visionpar[10];
double laserpar[10];

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

componentservertype lmssrv,camsrv;

 symTableElement * 
     getinputref (const char *sym_name, symTableElement * tab)
     {
       int i;
       for (i=0; i< getSymbolTableSize('r'); i++)
         if (strcmp (tab[i].name,sym_name) == 0)
           return &tab[i];
       return 0;
     }

    symTableElement * 
     getoutputref (const char *sym_name, symTableElement * tab)
     {
       int i;
       for (i=0; i< getSymbolTableSize('w'); i++)
         if (strcmp (tab[i].name,sym_name) == 0)
           return &tab[i];
       return 0;
     }
/*****************************************
* odometry
*/
#define WHEEL_DIAMETER   0.067	/* m */
#define WHEEL_SEPARATION 0.252	/* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT	24902


// We have in this struct added 'right_pos_old' and 'left_pos_old' in 
// order to be able to calculate the x- and y-positions and theta (these needs to be logged). 
typedef struct{ //input 
		int left_enc,right_enc; // encoderticks
		// parameters
		double w;	// wheel separation
		double cr,cl;   // meters per encodertick
	        //output signals
		double right_pos,left_pos, right_pos_old, left_pos_old;
		double delta_U, x, y, theta;
		// internal variables
		int left_enc_old, right_enc_old;
		} odotype;

void reset_odo(odotype *p);
void update_odo(odotype *p);




/********************************************
* Motion control
*/

// In this struct we have added 'Vmax' which is used in the acceleration and 
// deceleration of the robot.
typedef struct{//input
                int cmd;
		int curcmd;
		double speedcmd;
		double Vmax;
		double dist;
		double angle;
		double left_pos,right_pos;
		// parameters
		double w;
		//output
		double motorspeed_l,motorspeed_r; 
		int finished;
		// internal variables
		double startpos;
	       }motiontype;
	       
enum {mot_stop=1,mot_move,mot_turn};

void update_motcon(motiontype *p);	       


int fwd(double dist, double speed,int time);
int turn(double angle, double speed,int time);



typedef struct{
                int state,oldstate;
		int time;
	       }smtype;

void sm_update(smtype *p);

// SMR input/output data

symTableElement *  inputtable,*outputtable;
symTableElement *lenc,*renc,*linesensor,*irsensor, *speedl,*speedr,*resetmotorr,*resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

enum {ms_init,ms_fwd,ms_turn,ms_end};


// Here we create the array, which we use to log certain values (explained later).
// It has a capacity if 10000, but we only log the incomming data, which 'data_count' keeps track of.
// 'createDat' is  boolean value, which tells the program when to stop logging and close the file.
double data_log[10000][6];
int i, data_count = 0, createDat = 0;

int main()
{
  
  // Here we create the files that holds the data, which we are logging.
  // One for the x- and y-position, angle etc. and one for the laser configuration.
  FILE *f = fopen("data_log.dat", "w");
  if (f == NULL)
  {
      printf("Error opening file!\n");
      exit(1);
  }
  
  FILE *f2 = fopen("laser_log.dat", "w");
  if (f2 == NULL)
  {
      printf("Error opening file!\n");
      exit(1);
  }

  int running,n=0,arg,time=0;
  double dist=0,angle=0;

  /* Establish connection to robot sensors and actuators.
   */
     if (rhdConnect('w',"localhost",ROBOTPORT)!='w'){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      } 
      
      printf("connected to robot \n");
      if ((inputtable=getSymbolTable('r'))== NULL){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      }
      if ((outputtable=getSymbolTable('w'))== NULL){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      }
      // connect to robot I/O variables
      lenc=getinputref("encl",inputtable);
      renc=getinputref("encr",inputtable);
      linesensor=getinputref("linesensor",inputtable);
      irsensor=getinputref("irsensor",inputtable);
           
      speedl=getoutputref("speedl",outputtable);
      speedr=getoutputref("speedr",outputtable);
      resetmotorr=getoutputref("resetmotorr",outputtable);
      resetmotorl=getoutputref("resetmotorl",outputtable);
     // **************************************************
//  Camera server code initialization
//

/* Create endpoint */
   lmssrv.port=24919;
   strcpy(lmssrv.host,"127.0.0.1");
   strcpy(lmssrv.name,"laserserver");
   lmssrv.status=1;
   camsrv.port=24920;
   strcpy(camsrv.host,"127.0.0.1");
   camsrv.config=1;
   strcpy(camsrv.name,"cameraserver");
   camsrv.status=1;

   if (camsrv.config) {
      int errno = 0; 
      camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( camsrv.sockfd < 0 )
   {
    perror(strerror(errno));
    fprintf(stderr," Can not make  socket\n");
    exit(errno);
   }

   serverconnect(&camsrv);

   xmldata=xml_in_init(4096,32);
   printf(" camera server xml initialized \n");

}   
 
   
   
   
// **************************************************
//  LMS server code initialization
//

/* Create endpoint */
   lmssrv.config=1;
   if (lmssrv.config) {
       char buf[256];
      int errno = 0,len; 
      lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( lmssrv.sockfd < 0 )
   {
    perror(strerror(errno));
    fprintf(stderr," Can not make  socket\n");
    exit(errno);
   }

   serverconnect(&lmssrv);
   if (lmssrv.connected){
     xmllaser=xml_in_init(4096,32);
     printf(" laserserver xml initialized \n");
//     len=sprintf(buf,"push  t=0.2 cmd='mrcobst width=0.4'\n");
     len=sprintf(buf,"scanpush cmd='zoneobst'\n");
     send(lmssrv.sockfd,buf,len,0);
   }

}   
   
 
  /* Read sensors and zero our position.
   */
  rhdSync();
  
  odo.w=0.256;
  odo.cr=DELTA_M;
  odo.cl=odo.cr;
  odo.left_enc=lenc->data[0];
  odo.right_enc=renc->data[0];
  reset_odo(&odo);
  printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
  mot.w=odo.w;
running=1; 
mission.state=ms_init;
mission.oldstate=-1;
while (running){ 
   if (lmssrv.config && lmssrv.status && lmssrv.connected){
           while ( (xml_in_fd(xmllaser,lmssrv.sockfd) >0))
             xml_proca(xmllaser);
      }
      
      if (camsrv.config && camsrv.status && camsrv.connected){
          while ( (xml_in_fd(xmldata,camsrv.sockfd) >0))
             xml_proc(xmldata);
      }
       

  rhdSync();
// These outcommented lines of code was used in a previous assignment, where we had to log
// the motorspeeds and time, but was not used later on.
//Input motorspeeds and time to data_log array
//   if (data_count < 500) {
//     data_log[data_count][0] = mission.time;
//     data_log[data_count][1] = mot.motorspeed_l;
//     data_log[data_count++][2] = mot.motorspeed_r;
//   } else if (createDat == 0) {
//     fprintf(f, "time:\tspeed_l:\tspeed_r:\n");
//     for (i = 0; i < 500; i++) {
//       fprintf(f, "%.0f\t%.2f\t%.2f\n", data_log[i][0], data_log[i][1], data_log[i][2]);
//     }
//     fclose(f);
//     createDat++;
    
  // Here we log the data in the array which was created above. The array logs
  // 'time', 'x-position', 'y-position', 'angle (theta)' and the left- and right-position of the wheels.
  // Input odometry values to data_log array and file
    data_log[data_count][0] = mission.time;
    data_log[data_count][1] = odo.x;
    data_log[data_count][2] = odo.y;
    data_log[data_count][3] = odo.theta;
    data_log[data_count][4] = odo.left_pos;
    data_log[data_count++][5] = odo.right_pos;
  
  
    
  
  odo.left_enc=lenc->data[0];
  odo.right_enc=renc->data[0];
  update_odo(&odo);
  
/****************************************
/ mission statemachine   
*/
   sm_update(&mission);
   switch (mission.state) {
     case ms_init:
       n=4; dist=1;angle=90.0/180*M_PI;
       mission.state= ms_fwd;      
     break;
  
     case ms_fwd:
       if (fwd(dist,0.6,mission.time))  mission.state=ms_turn;
     break;
  
     case ms_turn:
       if (turn(angle,0.3,mission.time)){
         n=n-1;
	 if (n==0) 
	   mission.state=ms_end;
	 else
	   mission.state=ms_fwd;
	}
     break;    
     
     case ms_end:
       mot.cmd=mot_stop;
       running=0;
     break;
   }  
/*  end of mission  */
 
  mot.left_pos=odo.left_pos;
  mot.right_pos=odo.right_pos;
  update_motcon(&mot);
  speedl->data[0]=100*mot.motorspeed_l;
  speedl->updated=1;
  speedr->data[0]=100*mot.motorspeed_r;
  speedr->updated=1;
  if (time  % 100 ==0)
    //    printf(" laser %f \n",laserpar[3]);
  
  // Here is where we log the laser data.
  fprintf(f2, "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t\n", laserpar[0], laserpar[1], laserpar[2], laserpar[3], laserpar[4], laserpar[5], laserpar[6], laserpar[7], laserpar[8]);
  time++;
/* stop if keyboard is activated
*
*/
  ioctl(0, FIONREAD, &arg);
  if (arg!=0)  running=0;
    
}/* end of main control loop */
  speedl->data[0]=0;
  speedl->updated=1;
  speedr->data[0]=0;
  speedr->updated=1;
  rhdSync();
  rhdDisconnect();
	
  // This loop logs the position data mentioned earlier into a file called 'data_log.dat'.
  if (createDat == 0) {
    for (i = 0; i < data_count; i++) {
      fprintf(f, "%.0f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n", data_log[i][0], data_log[i][1], data_log[i][2], data_log[i][3], data_log[i][4], data_log[i][5]);
    }
    fclose(f);
    createDat++;
  }
  fclose(f2);
  exit(0);
}


/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */


void reset_odo(odotype * p)
{
  p->theta = p->x = p->y = 0.0;
  p->right_pos = p->left_pos = p->right_pos_old = p->left_pos_old = 0.0; // Here we initialize the right and left old positions.
  p->right_enc_old = p->right_enc;
  p->left_enc_old = p->left_enc;
}

void update_odo(odotype *p)
{
  int delta;

  delta = p->right_enc - p->right_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->right_enc_old = p->right_enc;
  p->right_pos += delta * p->cr;
  
  delta = p->left_enc - p->left_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->left_enc_old = p->left_enc;
  p->left_pos += delta * p->cl;
  
  // Here we calculate 'delta_U' to be able to further calculate the angle 'theta' and the x- and y-positions.
  p->delta_U = (((p->right_pos - p->right_pos_old) + (p->left_pos - p->left_pos_old))/2);
  p->theta += ((p->right_pos - p->right_pos_old) - (p->left_pos - p->left_pos_old)) / p->w;
  p->x += p->delta_U * cos(p->theta);
  p->y += p->delta_U * sin(p->theta);
  p->right_pos_old = p->right_pos;
  p->left_pos_old = p->left_pos;
}


void update_motcon(motiontype *p){ 

if (p->cmd !=0){
     
     p->finished=0;
     switch (p->cmd){
     case mot_stop:
       p->curcmd=mot_stop;
       break;
       case mot_move:
       p->startpos=(p->left_pos+p->right_pos)/2;
       p->curcmd=mot_move;
       break;
       
       case mot_turn:
         if (p->angle > 0) 
	    p->startpos=p->right_pos;
	 else
	    p->startpos=p->left_pos;
         p->curcmd=mot_turn;
       break;
       
       
     }
     
     p->cmd=0;
   }
   
   switch (p->curcmd){
     case mot_stop:
       p->motorspeed_l=0;
       p->motorspeed_r=0;
     break;
     case mot_move:
		   // Vmax is calculated, which calculate the max speed we can have during deceleration.
		   // The distance is calculated currectly, by always using the relative position (startpos), which is
		   // updated each 'turn' plus the remaining distance and substracting the robots current position.
       p->Vmax = sqrt(2*0.5*(p->startpos+p->dist - (p->left_pos + p->right_pos)/2));
       if ((p->right_pos+p->left_pos)/2- p->startpos > p->dist){
          p->finished=1;
	  p->motorspeed_l=0;
          p->motorspeed_r=0;
       }	  
       else {
	       // This code make the robot accelerate untill it reaches the maximum velocity givin by 'speedcmd'.
	       // This speed is then kept constant untill the 'Vmax' is below 'speedcmd', this means
	       // that the robot needs to decelerate and the velocity is then set equal to Vmax.
	  if (p->motorspeed_l < p->speedcmd && p->motorspeed_r < p->speedcmd && p->motorspeed_l < p->Vmax) {
	    p->motorspeed_l=0.005*mission.time;
	    p->motorspeed_r=0.005*mission.time;
	  } else if (p->speedcmd > p->Vmax) {
	    p->motorspeed_l=p->Vmax;
	    p->motorspeed_r=p->Vmax;
	  } else {
	    p->motorspeed_l=p->speedcmd;
	    p->motorspeed_r=p->speedcmd;
	  }
       }
     break;
     
     case mot_turn:
       if (p->angle>0){
	       
	       // This code makes the robot accelerate and decelerate while turning as well. Vmax has to be calculated
	       // differently due to the fact, that the distance is now curved and not linear. The curved distance
	       // is calculated by multyplying pi/2 (angle) with the wheelbase/2. The wheel that is backing up
	       // has a negative velocity and the wheel going forward keeps the positive value.
	       // The if-statements have small arithmatic changes for the wheel with the negative velocity.
	 p->Vmax = sqrt(2*0.5*(p->startpos+(p->angle*p->w/2) - p->right_pos));
	  if (p->right_pos-p->startpos < p->angle*p->w/2){
	    if (p->motorspeed_l > -p->speedcmd && p->motorspeed_r < p->speedcmd && p->motorspeed_l > -p->Vmax) {
	      p->motorspeed_l=-0.005*mission.time;
	      p->motorspeed_r=0.005*mission.time;
	    } else if (p->speedcmd > p->Vmax) {
	      p->motorspeed_l=-p->Vmax;
	      p->motorspeed_r=p->Vmax;
	    } else {
	      p->motorspeed_l=-p->speedcmd;
	      p->motorspeed_r=p->speedcmd;
	    }
	  }
	  else {	     
            p->motorspeed_l=0;
	    p->motorspeed_r=0;
            p->finished=1;
	  }
	}
	else {
		// See the comment for doing a left turn. The idea is completely the same ith only small changes.
	  p->Vmax = sqrt(2*0.5*(p->startpos-(p->angle*p->w/2) - p->left_pos));
	  if (p->left_pos-p->startpos < fabs(p->angle)*p->w/2){
	      if (p->motorspeed_l < p->speedcmd && p->motorspeed_r > -p->speedcmd && p->motorspeed_r > -p->Vmax) {
	    p->motorspeed_l=0.005*mission.time;
	    p->motorspeed_r=-0.005*mission.time;
	  } else if (p->speedcmd > p->Vmax) {
	    p->motorspeed_l=p->Vmax;
	    p->motorspeed_r=-p->Vmax;
	  } else {
	    p->motorspeed_l=p->speedcmd;
	    p->motorspeed_r=-p->speedcmd;
	  }
	  }
	  else {	     
            p->motorspeed_l=0;
	    p->motorspeed_r=0;
            p->finished=1;
	  }
	}

     break;
   }   
}   


int fwd(double dist, double speed,int time){
   if (time==0){ 
     mot.cmd=mot_move;
     mot.speedcmd=speed;
     mot.dist=dist;
     return 0;
   }
   else
     return mot.finished;
}

int turn(double angle, double speed,int time){
   if (time==0){ 
     mot.cmd=mot_turn;
     mot.speedcmd=speed;
     mot.angle=angle;
     return 0;
   }
   else
     return mot.finished;
}


void sm_update(smtype *p){
  if (p->state!=p->oldstate){
       p->time=0;
       p->oldstate=p->state;
   }
   else {
     p->time++;
   }
}



