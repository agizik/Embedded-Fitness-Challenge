#include <mbed.h>
#include <math.h>
#include <USBSerial.h>
#include "LIS3DSH.h"
volatile int printerflag = 1;
volatile int movementflag = 0;
volatile int calculateflag = 0;
volatile  int n_squad_done = 0;
volatile  int n_jj_done = 0;
volatile  int n_pushup_done = 0;
volatile  int n_situp_done = 0;
volatile  int n_situp_state = 0;
volatile  int n_squad_state = 0;
volatile  char jj_flip=0,push_flip=0,sit_flip=0,squad_flip=0;


DigitalOut led3(LED3); // orange - jj
DigitalOut led4(LED4); // green - squat
DigitalOut led5(LED5); // red - situp
DigitalOut led6(LED6); // blue - pushup


USBSerial serial;
Ticker printer;

Ticker half_situp_reject;
void half_situp()
{
    n_situp_state = 0;
    //serial.printf("--------------half situp timeout-------------------------------\n");
    half_situp_reject.detach();//When it completes the move in 2 secs. it is a proper finish of the movement
}

Ticker half_squad_reject;
void half_suquad()
{
    n_squad_state = 0;
    //serial.printf("--------------half squad timeout-------------------------------\n");
    half_situp_reject.detach();//When it completes the move in 2 secs. it is a proper finish of the movement
}



LIS3DSH acc(SPI_MOSI,SPI_MISO, SPI_SCK,PE_3);

void print()
{
    printerflag = 1;
    if (jj_flip){led3 = !led3;}
    if (push_flip){led6 = !led6;}
    if (sit_flip){led5 = !led5;}
    if (squad_flip){led4 = !led4;}
}


int main() {
  int16_t X,Y, Z = 0; //raw acceleration data in X,Y,Z
  float g_z_filt = 0;
  float g_y_filt = 0;
  float g_x_filt = 0;


  float g_z_filt_old = 0;
  float g_y_filt_old = 0;
  float g_x_filt_old = 0;

  float movement_det_idx = 0;
  int movement_cnt = 0;




  printer.attach(&print, 0.1);

  const float PI = 3.1415926; //pi

  const uint8_t N = 20; //filter length
  float ringbufz[N]; //sample buffer
  float ringbufy[N]; //sample buffer
  float ringbufx[N]; //sample buffer

  volatile float mv_bufz[N]; //sample buffer
  volatile float mv_bufy[N]; //sample buffer
  volatile float mv_bufx[N]; //sample buffer


  volatile float max_z=0,min_z=0,max_y=0,min_y=0,max_x=0,min_x=0,var_z=0,var_y=0,var_x=0,mean_z=0,mean_y=0,mean_x = 0;
  volatile float total_z=0,total_y=0,total_x = 0;

  volatile float ten_mean_bufz[10]; //sample buffer
  volatile float ten_mean_bufy[10]; //sample buffer
  volatile float ten_mean_bufx[10]; //sample buffer
  volatile float ten_var_bufz[10]; //sample buffer
  volatile float ten_var_bufy[10]; //sample buffer
  volatile float ten_var_bufx[10]; //sample buffer

  volatile float ten_max_z=0,ten_min_z=0,ten_max_y=0,ten_min_y=0,ten_max_x=0,ten_min_x=0,ten_mean_var_z=0,ten_mean_var_y=0,ten_mean_var_x=0,ten_mean_z=0,ten_mean_y=0,ten_mean_x = 0;
  volatile float ten_total_z=0,ten_total_y=0,ten_total_x=0,ten_tot_var_z=0,ten_tot_var_y=0,ten_tot_var_x = 0;
  float roll, pitch;      // float variables for angle

  uint8_t ten_mv_idx = 0;
  uint8_t mv_idx = 0; //filter length


  uint8_t ringbuf_index = 0; //index to insert sample

  while(acc.Detect() != 1){
    printf("Could not detect accelerometer\n\r");
    wait_ms(200);
  }

  // put your setup code here, to run once:
  int count = 0;
  while(1) {
    acc.ReadData(&X, &Y, &Z);
    acc.ReadAngles(&roll, &pitch);      // reads roll and pitch angles

    X = (float)X/16;
    Y = (float)Y/17;
    Z = (float)Z/18;
    
    ringbufx[ringbuf_index] = X;
    ringbufy[ringbuf_index] = Y;
    ringbufz[ringbuf_index++] = Z;


    if (ringbuf_index >= N)
    {
      g_z_filt_old = g_z_filt;
      g_y_filt_old = g_y_filt;
      g_x_filt_old = g_x_filt;

      g_z_filt = 0;
      g_y_filt = 0;
      g_x_filt = 0;

      for (uint8_t i = 0; i < N; i++){
          g_z_filt += ringbufz[i];
          g_y_filt += ringbufy[i];
          g_x_filt += ringbufx[i];
      }
      ringbuf_index = 0;         

      g_z_filt /= (float)N;
      g_y_filt /= (float)N;
      g_x_filt /= (float)N;

      mv_bufz[mv_idx] = g_z_filt;
      mv_bufy[mv_idx] = g_y_filt;
      mv_bufx[mv_idx++] = g_x_filt;

      total_z += g_z_filt;
      total_y += g_y_filt;
      total_x += g_x_filt;     
      
      movement_det_idx = (g_z_filt_old - g_z_filt)*(g_z_filt_old - g_z_filt) +(g_y_filt_old - g_y_filt)*(g_y_filt_old - g_y_filt) +(g_x_filt_old - g_x_filt)*(g_x_filt_old - g_x_filt);
      if(movement_det_idx > 1200 ){
        //serial.printf("Movement detected!!!\n");
        movement_cnt= 0;
        movementflag = 1;
      }
      else
      {
        if(movement_cnt++ >= N){
          movement_cnt= 0;
          movementflag = 0;
          //serial.printf("No Movement\n");

        }
      }
    }
    if(printerflag == 1 ){
      //serial.printf("max_z:  %f   max_y:  %f max_x:  %f\n", max_z, max_y,max_x);
      //serial.printf("min_z:  %f   min_y:  %f min_x:  %f\n", min_z, min_y,min_x);
      //serial.printf("%d mean_z: %f   mean_y: %f mean_x: %f\n",mv_idx, mean_z, mean_y,mean_x);
      //serial.printf("var_z:  %f   var_y:  %f var_x:  %f\n", max_z, max_y,max_x);

      printerflag = 0;
      calculateflag = 1;
    }

    if(calculateflag == 1){
        mean_z = total_z/(float) (mv_idx);
        mean_y = total_y/(float) (mv_idx);
        mean_x = total_x/(float) (mv_idx);
        
        max_z = -5000;
        max_y = -5000;
        max_x = -5000;
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

        calculateflag = 0;

        //serial.printf("max_z:  %.2f   max_y:  %.2f max_x:  %.2f\n", max_z, max_y,max_x);
        //serial.printf("min_z:  %.2f   min_y:  %.2f min_x:  %.2f\n", min_z, min_y,min_x);
        //serial.printf("mean_z: %.2f   mean_y: %.2f mean_x: %.2f\n", mean_z, mean_y,mean_x);
        //serial.printf("var_z:  %.2f   var_y:  %.2f var_x:  %.2f\n", var_z, var_y,var_x);
        
        if(max_z > ten_max_z){ten_max_z = max_z;}
        if(ten_min_z > min_z){ten_min_z = min_z;}

        if(max_y > ten_max_y){ten_max_y = max_y;}
        if(ten_min_y > min_y){ten_min_y = min_y;}

        if(max_x > ten_max_x){ten_max_x = max_x;}
        if(ten_min_x > min_x){ten_min_x = min_x;}


        ten_mean_bufz[ten_mv_idx]=mean_z; //sample buffer
        ten_mean_bufy[ten_mv_idx]=mean_y; //sample buffer
        ten_mean_bufx[ten_mv_idx]=mean_x; //sample buffer
        ten_var_bufz[ten_mv_idx]=var_z; //sample buffer
        ten_var_bufy[ten_mv_idx]=var_y; //sample buffer
        ten_var_bufx[ten_mv_idx++]=var_x;

        if (ten_mv_idx == 10)
        {
          for (uint8_t i = 0; i < ten_mv_idx; i++)
          {
            ten_total_z += ten_mean_bufz[i];
            ten_total_y += ten_mean_bufy[i];
            ten_total_x += ten_mean_bufx[i];
            ten_tot_var_z += ten_var_bufz[i];
            ten_tot_var_y += ten_var_bufy[i];
            ten_tot_var_x += ten_var_bufx[i];
          }
          ten_mean_var_z=ten_tot_var_z/10;
          ten_mean_var_y=ten_tot_var_y/10;
          ten_mean_var_x=ten_tot_var_x/10;
          ten_mean_z= ten_total_z/10;
          ten_mean_y=ten_total_y/10;
          ten_mean_x = ten_total_x/10;

        //Before the condition values are decided variables observed via serial printfs then analyzed via Matlab/Simulink
        if(ten_mean_var_x>120000 && movementflag)
        {//JJ
          //if (ten_max_y>200 && 1200 > ten_max_z && ten_min_x < -1500)
          //{
            n_jj_done++;
            n_jj_done >= 5 ? jj_flip = 0, led3 = 1 : jj_flip = 1;
          //}
          //serial.printf("-ten_mean_var_x  %f----ten_max_y  %f------ten_max_z %f----------ten_min_x %f------------\n",ten_mean_var_x,ten_max_y,ten_max_z,ten_min_x);          
          serial.printf("--------------JJ--------------%d----------------JJ-------------\n",n_jj_done);
          wait_ms(50);
        }
        else if (ten_min_z<-650 && ten_mean_z<-400 && movementflag)
        {//PUSHUP
          if (ten_max_z > -150 && ten_max_z < 450 )
          {
            n_pushup_done++;
            n_pushup_done >= 5 ? push_flip = 0, led6 = 1 : push_flip = 1;

          }
          //serial.printf("-ten_max_z  %.2f------ten_min_z %.2f---------ten_mean_z  %.2f----------\n",ten_max_z,ten_min_z,ten_mean_z);
          serial.printf("------------PUSHUP----------------%d--------------PUSHUP-----------\n",n_pushup_done);
          //serial.printf("----ten_max_x %.2f---ten_mean_y %.2f----ten_mean_z %.2f------\n",ten_max_x,ten_mean_y,ten_mean_z);

        }
        else if (ten_max_x > -500 && ten_mean_y > -100 &&  ten_mean_y < 100 && ten_mean_z < 500 && movementflag)
        {//SQUAD    ten_max_x<0 &&  /*ten_max_x<-300 && */
          n_situp_state = 0;
          //serial.printf("----squad case------------\n");

          //if(n_squad_state == 0)
            //{
              for (uint8_t i = 0; i < ten_mv_idx; i++)
              {
                if(-900 <  ten_mean_bufx[i])
                  {
                    
                    if (n_squad_state == 0)
                    {
                      n_squad_state = 1;
                      //serial.printf("-----------half squad--------ten_mean_bufx[i] %.2f-------------\n",ten_mean_bufx[i]);
                      //serial.printf("----ten_max_x %.2f---ten_mean_y %.2f----ten_mean_z %.2f------\n",ten_max_x,ten_mean_y,ten_mean_z);
                    }
                    else
                    {
                      n_squad_state = 0;
                      n_squad_done++;
                      n_squad_done >= 5 ? squad_flip = 0,led4 = 1 : squad_flip = 1;
                      serial.printf("--------SQUAD----------%d---------SQUAD---------\n",n_squad_done);
                    }
                    break;                 
                    //half_squad_reject.attach(&half_suquad, 10.0);//gives to second to complete the move or it will update the state as not started
                  }
              }
            //}
          //else if(n_squad_state == 1)
            //{
              /*for (uint8_t i = 0; i < ten_mv_idx; i++)
              {
                if(-1200 > ten_mean_bufx[i] )
                  {
                    n_squad_done++;
                    n_squad_done >= 8 ? squad_flip = 0,led4 = 1 : squad_flip = 1;
                    serial.printf("--------SQUAD----------%d---------SQUAD---------\n",n_squad_done);
                    //serial.printf("-------------------ten_mean_bufx[i] %.2f-------------\n",ten_mean_bufx[i]);
                    //serial.printf("----ten_max_x %.2f---ten_mean_y %.2f----ten_mean_z %.2f------\n",ten_max_x,ten_mean_y,ten_mean_z);

                    n_squad_state = 0;

                    //half_squad_reject.detach();//When it completes the move in 2 secs. it is a proper finish of the movement
                    break;
                  }
              }*/
            //}
        }        
        else if (ten_mean_y < 100 && ten_min_y < 0 && ten_max_z > 1300 && movementflag)
        {//situp && ten_mean_y < 100
          n_squad_state = 0;
          float situp_mean_y = 700;
          //serial.printf("----situp_ case------------\n");

          /*
          if(n_situp_state == 0)
            {
              for (uint8_t i = 0; i < ten_mv_idx; i++)
              {
                 if(1050 >  ten_mean_bufz[i])
                  {
                    situp_mean_y = ten_mean_bufy[i];
                    if (situp_mean_y > 50  && situp_mean_y < 300)
                    {               
                      n_situp_state = 1;
                      serial.printf("---------------------------half situp--------------------\n");
                      serial.printf("----ten_mean_y   %.2f-------ten_max_z %.2f------------\n",ten_mean_y,ten_max_z);
                      serial.printf("----situp_mean_y   %.2f-------ten_mean_bufz %.2f------------\n",situp_mean_y,ten_mean_bufz[i]);
                      //half_situp_reject.attach(&half_situp, 10.0);//gives to second to complete the move or it will update the state as not started
                      break;
                    }
                  }
              }
            }
            */
  /*else*/ //if(n_situp_state == 1)
            //{
              for (uint8_t i = 0; i < ten_mv_idx; i++)
              {
                if(1150 <  ten_mean_bufz[i])                                     //350 < ten_mean_bufz[i])
                  {
                    //serial.printf("----situp_mean_y   %.2f-------ten_mean_bufz %.2f------------\n",situp_mean_y,ten_mean_bufz[i]);

                    //situp_mean_y = ten_mean_bufy[i];
                    //if (situp_mean_y > -10  && situp_mean_y < 150)                                             //situp_mean_y > -300  && situp_mean_y < -50)
                    //{
                      //half_situp_reject.detach();//When it completes the move in 2 secs. it is a proper finish of the movement
                      n_situp_state = 0;
                      n_situp_done++;
                      n_situp_done >= 5 ? sit_flip = 0,led5 = 1 : sit_flip = 1;
                      serial.printf("----------------SIT-UP--------------%d-------------SIT-UP------------------\n",n_situp_done);
                      //serial.printf("----ten_mean_y   %.2f-------ten_max_z %.2f------------\n",ten_mean_y,ten_max_z);
                      //serial.printf("----situp_mean_y   %.2f-------ten_mean_bufz %.2f------------\n",situp_mean_y,ten_mean_bufz[i]);

                      break;
                    //}
                  }
              }
            //}
        }
        else
        {
          serial.printf("-----------------------------------------------------------------------------------\n");
          //serial.printf("----ten_mean_y   %.2f-------ten_max_z %.2f----ten_min_y %.2f--------\n",ten_mean_y,ten_max_z,ten_min_y);
          //serial.printf("----ten_max_x %.2f---ten_mean_y %.2f----ten_mean_z %.2f------\n",ten_max_x,ten_mean_y,ten_mean_z);

          //serial.printf("----ten_mean_y %.2f---ten_min_y %.2f----ten_max_z %.2f------\n",ten_mean_y,ten_min_y,ten_max_z);
          //serial.printf("ten_max_x   %.2f ten_mean_y: %.2f   ten_mean_z:   %.2f ten_mean_x:   %.2f\n", ten_max_x,ten_mean_y, ten_mean_z,ten_mean_x);
          //serial.printf("max_mean_x:      %.2f   \n", max_mean_x);
        }
        
        


        //serial.printf("------------------------------------------------------------------\n");
        //serial.printf("ten_max_z:      %.2f   ten_max_y:      %.2f ten_max_x:      %.2f\n", ten_max_z, ten_max_y,ten_max_x);
        //serial.printf("ten_min_z:      %.2f   ten_min_y:      %.2f ten_min_x:      %.2f\n", ten_min_z, ten_min_y,ten_min_x);
        //serial.printf("ten_mean_z:     %.2f   ten_mean_y:     %.2f ten_mean_x:     %.2f\n", ten_mean_z, ten_mean_y,ten_mean_x);
        //serial.printf("ten_mean_var_z: %.2f   ten_mean_var_y: %.2f ten_mean_var_x: %.2f\n", ten_mean_var_z, ten_mean_var_y,ten_mean_var_x);
        //serial.printf("ten_mean_var_z: %.2f   ten_mean_var_y: %.2f ten_mean_var_x: %.2f\n", ten_mean_var_z, ten_mean_var_y,ten_mean_var_x);

        //serial.printf("------------------------------------------------------------------\n");
        
        ten_max_z=-2000,ten_max_y=-2000,ten_max_x=-2000,ten_min_z=2000,ten_min_y=2000,ten_min_x=2000;
        ten_total_z = 0;
        ten_total_y = 0;
        ten_total_x = 0;
        ten_tot_var_z = 0;
        ten_tot_var_y = 0;
        ten_tot_var_x = 0;   
        ten_mv_idx=0;       
        }


        mv_idx = 0;
        var_z = 0;
        var_y = 0;
        var_x = 0;
        mean_z = 0;
        mean_y = 0;
        mean_x = 0;
      }
  }
}