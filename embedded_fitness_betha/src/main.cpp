#include <mbed.h>
#include <math.h>
#include <USBSerial.h>
#include "LIS3DSH.h"

//Initializing PINs
LIS3DSH acc(SPI_MOSI,SPI_MISO, SPI_SCK,PE_3);
USBSerial serial;
DigitalOut led3(LED3); // orange for Jumping Jacks
DigitalOut led4(LED4); // green for Squats
DigitalOut led5(LED5); // red for Situps
DigitalOut led6(LED6); // blue for Pushups

//Variables declaration 
int16_t X,Y,Z = 0; //raw acceleration data in X,Y,Z
float roll, pitch; //float angle variables
volatile float g_z_filt = 0;//filtered Z acceleration data
volatile float g_y_filt = 0;//filtered Y acceleration data
volatile float g_x_filt = 0;//filtered X acceleration data
volatile float g_z_filt_old = 0;//transient variable to keep the old value
volatile float g_y_filt_old = 0;//transient variable to keep the old value
volatile float g_x_filt_old = 0;//transient variable to keep the old value 

//Data inditcation variables
volatile float max_z=0,min_z=0,max_y=0,min_y=0,max_x=0,min_x=0,var_z=0,var_y=0,var_x=0,mean_z=0,mean_y=0,mean_x = 0;
volatile float total_z=0,total_y=0,total_x = 0;
volatile float ten_max_z=0,ten_min_z=0,ten_max_y=0,ten_min_y=0,ten_max_x=0,ten_min_x=0,ten_mean_var_z=0,ten_mean_var_y=0,ten_mean_var_x=0,ten_mean_z=0,ten_mean_y=0,ten_mean_x = 0;
volatile float ten_total_z=0,ten_total_y=0,ten_total_x=0,ten_tot_var_z=0,ten_tot_var_y=0,ten_tot_var_x = 0;
 
//movement decision variable
float movement_det_idx = 0;

//flags and states
volatile int calculateflag = 0;
volatile char movementflag = 0;
volatile  uint8_t n_situp_state = 0;
volatile  uint8_t n_squad_state = 0;
volatile  uint8_t n_pushup_state = 0;
volatile  uint8_t jj_flip=0,push_flip=0,sit_flip=0,squad_flip=0;

//counters
volatile  uint8_t n_squad_done = 0;
volatile  uint8_t n_jj_done = 0;
volatile  uint8_t n_pushup_done = 0;
volatile  uint8_t n_situp_done = 0;
volatile uint8_t ten_mv_idx = 0;
uint8_t mv_idx = 0; 
uint8_t ringbuf_index = 0;

//constant array sizes
const uint8_t N = 20; //array length for filtering and finding out indication variables(mean, variance, max, min)

//arrays
volatile float ten_mean_bufz[N];//buffer for filtered 100ms Z's mean values
volatile float ten_mean_bufy[N];//buffer for filtered 100ms Y's mean values
volatile float ten_mean_bufx[N];//buffer for filtered 100ms X's mean values
volatile float ten_var_bufz[N];//buffer for filtered 100ms Z's variance values
volatile float ten_var_bufy[N];//buffer for filtered 100ms Y's variance values
volatile float ten_var_bufx[N];//buffer for filtered 100ms X's variance values
float ringbufz[N]; //raw sample buffer
float ringbufy[N]; //raw sample buffer
float ringbufx[N]; //raw sample buffer
volatile float mv_bufz[N]; //filtered sample buffer
volatile float mv_bufy[N]; //filtered sample buffer
volatile float mv_bufx[N]; //filtered sample buffer

//Timers
Ticker decisiontime_flipper;//ticker timer; calls one_hundred_ms() function every 100ms
Ticker half_situp_reject;//ticker timer;calls half_situp() to prevent wrong decisions
Ticker half_squad_reject;//ticker timer;calls half_suquad() to prevent wrong decisions
Ticker half_pushup_reject;//ticker timer;calls half_pushup() to prevent wrong decisions

//Functions
char isitmoving();//decides wheter the user is moving or not
void data_processing();////calculates movement detection variables - last 10 mean vales X,Y,Z and means of their variances filtered samples' variables
void detectmovement();//Decides which type of exercise is on progress or done by user
void half_situp();//When situp state is 1(user is lying down), expects user to finish a proper situp in 2 seconds or deletes the half progress  
void half_suquad();//When squat state is 1(user squatted), expects user to finish a proper squat in 2 seconds or deletes the half progress
void half_pushup();//When pushup state is 1(user is down), expects user to finish a proper pushup in 2 seconds or deletes the half progress  
void one_hundred_ms();//this function is called every 100ms if device is moving it will start calculation and decisions, it also flips the LEDs of in progress exercises
void filter_raw_data();//Reads 20 raw X,Y,Z values and calculates their means gives proper data for movement detection
void mean_var_max_values();//Calculates mean, variance and finds maximum and minimum values of filtered X,Y,Z approximately for last 1 second / 10 values of each
void initializer();//Initializes neccesary variables after existance and type of movement decisions are made

int main() {
  while(acc.Detect() != 1){
    printf("Could not detect accelerometer\n\r");
    wait_ms(200);
  }
  while(1) {
    initializer();
    filter_raw_data();
    if(isitmoving())
    {
      decisiontime_flipper.attach(&one_hundred_ms, 0.1);
      calculateflag = 1;
      while(calculateflag)
      {
        filter_raw_data();
        mean_var_max_values();
      }
      data_processing();
      detectmovement();
    }
  }
}

void filter_raw_data(){
  ringbuf_index = 0; //counter   
  while(ringbuf_index < N)
  {
    //wait_ms(2);
    acc.ReadData(&X, &Y, &Z);//reads raw X,Y,Z values
    acc.ReadAngles(&roll, &pitch);//reads roll and pitch angles

  
    X = (float)X/16; //when the card is stable and in the sitting position x is 1000
    Y = (float)Y/17; //when the card is stable and in the sitting position y is 1000
    Z = (float)Z/18; //when the card is stable and in the lying position z is 1000
    
    ringbufx[ringbuf_index] = X;//fill the ring buffer with raw X samples
    ringbufy[ringbuf_index] = Y;//fill the ring buffer with raw Y samples
    ringbufz[ringbuf_index++] = Z;//fill the ring buffer with raw Z samples
  }

  g_z_filt_old = g_z_filt;//transient variable to keep the old value
  g_y_filt_old = g_y_filt;//transient variable to keep the old value
  g_x_filt_old = g_x_filt;//transient variable to keep the old value  

  g_z_filt = 0;//needed to initialize before find out the new value
  g_y_filt = 0;//needed to initialize before find out the new value
  g_x_filt = 0;//needed to initialize before find out the new value

  for (uint8_t i = 0; i < N; i++){
    g_z_filt += ringbufz[i];
    g_y_filt += ringbufy[i];
    g_x_filt += ringbufx[i];
  } 
   
  g_z_filt /= (float)N;//filtered
  g_y_filt /= (float)N;//filtered
  g_x_filt /= (float)N;//filtered

  //keep the values to calculate last 100 ms's mean and variance of filtered X,Y,Z values 
  mv_bufz[mv_idx] = g_z_filt;
  mv_bufy[mv_idx] = g_y_filt;
  mv_bufx[mv_idx++] = g_x_filt;  
  total_z += g_z_filt;
  total_y += g_y_filt;
  total_x += g_x_filt;
}

char isitmoving(){ 
 movement_det_idx = (ringbufz[19] - ringbufz[0])*(ringbufz[19] - ringbufz[0]) +(ringbufy[19] - ringbufy[0])*(ringbufy[19] - ringbufy[0]) +(ringbufx[19] - ringbufx[0])*(ringbufx[19] - ringbufx[0]);
 
 if(movement_det_idx > 1200 ){
    serial.printf("Movement detected!\n");
    movementflag = 1;
    return true;
  }
  else
  {
    movementflag = 0;
    wait_ms(10);
    serial.printf("No Movement\n");
    return false;
  } 
}

void mean_var_max_values(){//Calculates mean, variance and finds maximum and minimum values of filtered X,Y,Z approximately for last 100ms / 20 values of each
  mean_z = total_z/(float) (mv_idx);
  mean_y = total_y/(float) (mv_idx);
  mean_x = total_x/(float) (mv_idx);

  //not zero because max value may be smaller than zero      
  max_z = -5000;
  max_y = -5000;
  max_x = -5000;

  //not zero because min value may be bigger than zero
  min_z = 5000;
  min_y = 5000;
  min_x = 5000;

  total_x = 0,total_y= 0 ,total_z = 0;

  for (uint8_t i = 0; i < mv_idx; i++){
    if(mv_bufz[i] > max_z){max_z = mv_bufz[i];}
    if(mv_bufz[i] < min_z){min_z = mv_bufz[i];}
    if(mv_bufy[i] > max_y){max_y = mv_bufy[i];}
    if(mv_bufy[i] < min_y){min_y = mv_bufy[i];}
    if(mv_bufx[i] > max_x){max_x = mv_bufx[i];}
    if(mv_bufx[i] < min_x){min_x = mv_bufx[i];}

    var_z += (mv_bufz[i] - mean_z)*(mv_bufz[i] - mean_z);
    var_y += (mv_bufy[i] - mean_y)*(mv_bufy[i] - mean_y);
    var_x += (mv_bufx[i] - mean_x)*(mv_bufx[i] - mean_x);

    mv_bufz[i] = 0;
    mv_bufy[i] = 0;
    mv_bufx[i] = 0;
  }

  var_z = var_z/(mv_idx);
  var_y = var_y/(mv_idx);
  var_x = var_x/(mv_idx);

  mv_idx = 0;

  if(max_z > ten_max_z){ten_max_z = max_z;}
  if(ten_min_z > min_z){ten_min_z = min_z;}
  if(max_y > ten_max_y){ten_max_y = max_y;}
  if(ten_min_y > min_y){ten_min_y = min_y;}
  if(max_x > ten_max_x){ten_max_x = max_x;}
  if(ten_min_x > min_x){ten_min_x = min_x;}

  ten_mean_bufz[ten_mv_idx]=mean_z; 
  ten_mean_bufy[ten_mv_idx]=mean_y; 
  ten_mean_bufx[ten_mv_idx]=mean_x; 
  ten_var_bufz[ten_mv_idx]=var_z; 
  ten_var_bufy[ten_mv_idx]=var_y; 
  ten_var_bufx[ten_mv_idx++]=var_x;

  if(ten_mv_idx > 18){ 
    calculateflag = 0;
    serial.printf("--------------unsufficient buffer size-------------\n");
 }
}

void data_processing(){ 
//calculates movement detection variables - last 1 second's mean X,Y,Z and means of their variances via 100ms filtered samples - and their variables
//Uses mean, variance, maximum and minimum values of filtered X,Y,Z approximately for last 100ms / 10 values of each
  for (uint8_t i = 0; i < ten_mv_idx; i++)
    {
        ten_total_z += ten_mean_bufz[i];
        ten_total_y += ten_mean_bufy[i];
        ten_total_x += ten_mean_bufx[i];
        ten_tot_var_z += ten_var_bufz[i];
        ten_tot_var_y += ten_var_bufy[i];
        ten_tot_var_x += ten_var_bufx[i];
    }

  ten_mean_var_z =  ten_tot_var_z/(float)ten_mv_idx;
  ten_mean_var_y =  ten_tot_var_y/(float)ten_mv_idx;
  ten_mean_var_x =  ten_tot_var_x/(float)ten_mv_idx;
  ten_mean_z     =  ten_total_z/(float)ten_mv_idx;
  ten_mean_y     =  ten_total_y/(float)ten_mv_idx;
  ten_mean_x     =  ten_total_x/(float)ten_mv_idx;

  ten_mv_idx = 0;
  ten_tot_var_z = 0;
  ten_tot_var_y = 0;
  ten_tot_var_x = 0;
  ten_total_z = 0;
  ten_total_y = 0;
  ten_total_x = 0;
}

void detectmovement(){
  if(100000 > ten_mean_var_x && ten_mean_var_x > 5000 && ten_max_y > -200 && ten_max_z > -400 && movementflag)//During Jumping Jacks variance of x is extremely high, so this one is the easiest to detect and count
    {
      n_jj_done++;//increments the counter for number of jumping jacks completed
      n_jj_done >= 6 ? jj_flip = 0, led3 = 1 : jj_flip = 1;//decide wheter orange LED should blink or be on
      serial.printf("--------------JJ--------------%d----------------JJ-------------\n",n_jj_done);
      wait_ms(50);//Prevents multiple detection counts, waits until user lands
    }
  else if ((ten_min_z<-650 && ten_mean_z<-400) || (ten_max_y < 150 &&ten_max_y > -50 && ten_max_z> 0 && ten_max_z<600) && movementflag)//PUSHUP
    {//Since the device is on user's chest, z values are very low(almost all negative), so it is not hard to distinguish from others and even one step is enough     
      if (ten_max_z < -1200  && ten_mean_var_z < 150 && ten_mean_var_z > -100 && n_pushup_state == 0)//z reaches it's maximum point when user is pushing up
      {
        half_pushup_reject.attach(&half_pushup, 2.0);//gives 2 seconds to complete the move or it will update the state as not started
        n_pushup_state = 1;
      }
      else if (ten_max_z > -300 && ten_mean_var_z < 100 && ten_mean_var_z > -50)
      {
        half_pushup_reject.detach();//when the move is completed in 2 secs. it is a proper finish of the movement
        n_pushup_state = 0;
        n_pushup_done++;//increments the counter for number of pushups completed
        n_pushup_done >= 6 ? push_flip = 0, led6 = 1 : push_flip = 1;//decide wheter blue LED should blink(pushups in progress) or be on (pushups are done)
        wait_ms(50);
      }
      serial.printf("------------PUSHUP----------------%d--------------PUSHUP-----------\n",n_pushup_done);
    }
  else if (ten_max_x < 0 && ten_mean_y > -100 &&  (ten_mean_var_z + ten_mean_var_y) < 300 && ten_mean_z < 500 && movementflag)
    {//SQUAT
      if(n_squad_state == 0)//detection of going down during squat exercises
        {
          for (uint8_t i = 0; i < ten_mv_idx; i++)
            {
              if(-1200 <  ten_mean_bufx[i])//checks last 10 mean filtered x values to see wheter user squatted or not
                { 
                  n_squad_state = 1;
                  half_squad_reject.attach(&half_suquad, 2.0);//gives 2 seconds to complete the move or it will update the state as not started
                  break;                 
                }
            }
        }
      else//detection of going up during squat exercises
        {
          for (uint8_t i = 0; i < ten_mv_idx; i++)//checks last 10 mean filtered x values to see wheter user rised up or not
            {
              if(-800 > ten_mean_bufx[i] ){
                  half_squad_reject.detach();//when the move is completed in 2 secs. it is a proper finish of the movement
                  n_squad_state = 0;
                  n_squad_done++;//increment the counter for number of Squats detected
                  n_squad_done >= 6 ? squad_flip = 0,led4 = 1 : squad_flip = 1;//decide wheter green LED should blink or be on 
                  serial.printf("--------SQUAT----------%d---------SQUAT---------\n",n_squad_done);
                  break;
                } 
            }
        }    
    }        
  else if (ten_mean_y < 100 && ten_min_y < 0 && ten_max_z > 800 || ten_max_z < 0 && movementflag)
    {//situp
          serial.printf("----------------------------In situp----------------------------------------------\n");
          if(n_situp_state == 0)
            {
              for (uint8_t i = 0; i < ten_mv_idx; i++)
              {
                 if(1050 <  ten_mean_bufz[i])//checks last 10 mean filtered z values to see wheter went down or not
                  {
                    if (ten_mean_bufy[i] > 50  && ten_mean_bufy[i] < 300)
                    {               
                      n_situp_state = 1;//now user is lying down
                      half_situp_reject.attach(&half_situp, 2.0);//when the move is completed in 2 secs. it is a proper finish of the movement
                      break;
                    }
                  }
              }
            }
          else
            {
              for (uint8_t i = 0; i < ten_mv_idx; i++)
              {
                if(0 > ten_mean_bufz[i]){//checks last 10 mean filtered z values to see wheter rised up or not
                      half_situp_reject.detach();//When it completes the move in 2 secs. it is a proper finish of the movement
                      n_situp_state = 0;//now user is sitting up
                      n_situp_done++;//increment the counter for number of situps detected
                      n_situp_done >= 6 ? sit_flip = 0,led5 = 1 : sit_flip = 1;//decide wheter red LED should blink or be on 
                      serial.printf("----------------SIT-UP--------------%d-------------SIT-UP------------------\n",n_situp_done);
                      break;
                }
              }
            }
    }
  else{
          serial.printf("----------------------------No proper exercise detected----------------------------------------------\n");         
        }
}//end of function: detectmovement()

void half_situp()
{
    n_situp_state = 0;//Half progress of a situp is  timed-out
    //serial.printf("--------------half situp timeout-------------------------------\n");
    half_situp_reject.detach();//Detach the timer 
}

void half_suquad()
{
    n_squad_state = 0;//Half progress of a squat is  timed-out
    //serial.printf("--------------half squad timeout-------------------------------\n");
    half_situp_reject.detach();//When it completes the move in 2 secs. it is a proper finish of the movement
}

void half_pushup()
{
    n_pushup_state = 0;//Half progress of a squat is  timed-out
    //serial.printf("--------------half squad timeout-------------------------------\n");
    half_pushup_reject.detach();//When it completes the move in 2 secs. it is a proper finish of the movement
}

void one_hundred_ms() //this function is called every 100ms if device is moving it will start calculation and decisions, it also flips the LEDs of in progress exercises
{
    if (jj_flip){led3 = !led3;} //Orange LED blinks if Jumping Jacks are in progress
    if (push_flip){led6 = !led6;} //Blue LED blinks if Pushups are in progress
    if (sit_flip){led5 = !led5;} //Red LED blinks if Situps are in progress
    if (squad_flip){led4 = !led4;} //Green LED blinks if Squats are in progress

    calculateflag = 0;
}

void initializer(){
  ten_max_z = -20000;//not zero because max value may be smaller than zero
  ten_max_y = -20000;//not zero because max value may be smaller than zero
  ten_max_x = -20000;//not zero because max value may be smaller than zero
  ten_min_z = 20000;//not zero because min value may be smaller than zero
  ten_min_y = 20000;//not zero because min value may be smaller than zero
  ten_min_x = 20000;//not zero because min value may be smaller than zero

  ten_total_z = 0;
  ten_total_y = 0;
  ten_total_x = 0;
  ten_tot_var_z = 0;
  ten_tot_var_y = 0;
  ten_tot_var_x = 0;   
  ten_mv_idx=0;       

  mv_idx = 0;
  var_z = 0;
  var_y = 0;
  var_x = 0;
  mean_z = 0;
  mean_y = 0;
  mean_x = 0;

  g_z_filt = 0;
  g_y_filt = 0;
  g_x_filt = 0;
}