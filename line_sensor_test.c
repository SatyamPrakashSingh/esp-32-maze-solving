//C Headers
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>


//Components
#include "MPU.h"
#include "SRA18.h"
#include "TUNING.h"


adc1_channel_t channel[6] = {ADC_CHANNEL_7,ADC_CHANNEL_6,ADC_CHANNEL_0,ADC_CHANNEL_3,ADC_CHANNEL_5,ADC_CHANNEL_4};
adc2_channel_t channel2[2] = {ADC2_CHANNEL_4,ADC2_CHANNEL_3};
int weights[4] = {3,1,-1,-3};

#define kP 1.5
#define kI 0
#define kD 1.5

float weighted_sum=0;
float opt = 70;
float lower_pwm_constrain = 60;
float higher_pwm_constrain = 90;
float left_pwm = 0, right_pwm = 0;
double pos =0;
float error=0, prev_error, difference, cumulative_error, correction;
//int all_black_flag[4];
int adc_reading[6];
int adc2_reading[2];
int sensor_value[6];
int sensor_value2[2];
int all_black_flag[6];
int k;
int mode;
int q;
int r;
int s;
char path[50]={};
int pathLength;


static void read_sensors()
{
  for(int i = 0; i < 6; i++)
    {
        adc_reading[i] = adc1_get_raw(channel[i]);
        // printf("RAW %d: %d\t",i, sensor_value[i]);
    }
   for(int i=0; i<2 ; i++)
   {
   		adc2_config_channel_atten(channel2[i],ADC_ATTEN_DB_6);
   		adc2_get_raw(channel2[i],ADC_WIDTH_BIT_12,&adc2_reading[i]);
   		
   }
   // printf("\n");
}
static void calc_sensor_values()
{
		for(int i =0;i<6;i++)
		{
			adc_reading[i] = adc1_get_raw(channel[i]);
			sensor_value[i] = constrain(map(adc_reading[i], 1736, 4180, 0, 1000),0,1000);
			printf("%d\t",sensor_value[i]);
		}
		for(int i=0;i<2;i++)
		{
			adc2_config_channel_atten(channel2[i],ADC_ATTEN_DB_6);
   			adc2_get_raw(channel2[i],ADC_WIDTH_BIT_12,&adc2_reading[i]);
   			sensor_value2[i] = constrain(map(adc2_reading[i], 1736, 4180, 1000,0),0,1000);
   			printf("%d\t",sensor_value2[i]);
		}
}

static void calculate_error()
{
//int all_black_flag[4];
float weighted_sum=0;
float sum = 0;
int i;

 for(int i = 0; i < 4; i++)
 {

         if(sensor_value[i] > 400)
         {
             all_black_flag[i] = 0;
          }
       else
       {
       	all_black_flag[i]=1;
         
       }
      
        weighted_sum += ((sensor_value[i]) * (weights[i]));

        sum += sensor_value[i];
    }    
    
     //printf("sum %.2f: \t",sum);
      //printf("weighted_sum %.2f: \t",weighted_sum);
   if(sum != 0)
    {
        pos = weighted_sum/sum;
         //printf("pos : %.2f\t",pos);
    }

     if(all_black_flag[0]==1 && all_black_flag[1]==1 && all_black_flag[2]==1 && all_black_flag[3]==1)
    {
        if(error > 0)
        {
             pos = 2.5;
         }
         else
            { 
              pos = -2.5;
         }

    }
    error=pos;
    
}


static void calculate_correction()
{
error *= 10;
    difference = (error - prev_error);
    cumulative_error += error;
    
    if(cumulative_error > 30)
    {
        cumulative_error = 30;
    }
    
    else if(cumulative_error < -30)
    {
        cumulative_error = -30;
    }

    correction = kP*error + kI*cumulative_error + kD*difference;

    prev_error = error;
}
 
 int sensor()
 {
  for(int i = 0; i < 6; i++)
 {


         if(sensor_value[i] > 400)
         {
             all_black_flag[i] = 0;
          }



          else
         {
              all_black_flag[i]=1;
         
          }
  }
  k=all_black_flag[3]*1+all_black_flag[2]*2+all_black_flag[1]*4+all_black_flag[0]*8;
  return(k);  
  
  //printf("\n") ; 
  }  
void recIntersection(char direction)
{
	path[pathLength]=direction;
	pathLength++;
	//simplifyPath();

}  
static void maze_solve()
{
   //mode=sensor();

   if(sensor_value[0]>400 && sensor_value[1]>400 && sensor_value[2]>400 && sensor_value[3]>400 && sensor_value2[0]<400)
   {
    //printf("\n");
    printf("%s\n","T or +" );
    vTaskDelay(5);
    bot_stop(MCPWM_UNIT_0,MCPWM_TIMER_0);
    vTaskDelay(20);
    if(sensor_value[4]<400 && sensor_value[5]<400 )
      {
      	printf("%s\n","T" );
        q=1;
        recIntersection('L');
		}

	if(sensor_value[4]>200 || sensor_value[5]>200)
		{
		printf("%s\n","+" );
			r=1;
			recIntersection('R');
		}

    while(q==1)
    {
      printf("Turn Loop left\n");
      read_sensors();
      calc_sensor_values();
      if(sensor_value[4]>300 && sensor_value[5]>300 && sensor_value2[0]<400 && sensor_value2[1]<400)
      {
      	bot_stop(MCPWM_UNIT_0,MCPWM_TIMER_0);
        vTaskDelay(50);
        q=0;
        break;
      }
      bot_spot_left(MCPWM_UNIT_0,MCPWM_TIMER_0,68,68);
    }
    while(r==1)
    {
      printf("Turn Loop left\n");
      printf("%d\n",r);
      read_sensors();
      calc_sensor_values();
      if(sensor_value[4]<400 && sensor_value[5]<400 )
      {
        r=0;
        s=1;
        break;
      }
      bot_spot_left(MCPWM_UNIT_0,MCPWM_TIMER_0,68,68);
    }
    while(s==1)
    {
      printf("Turn Loop left\n");
      read_sensors();
      calc_sensor_values();
      if(sensor_value[4]>300 && sensor_value[5]>300 && sensor_value2[0]<400 && sensor_value2[1]<400)
      {
      	bot_stop(MCPWM_UNIT_0,MCPWM_TIMER_0);
        vTaskDelay(50);
        s=0;
       break;
      }
      bot_spot_left(MCPWM_UNIT_0,MCPWM_TIMER_0,65,65);
    }
    
     }
     if(sensor_value[0]<400 && sensor_value[3]>700 && sensor_value[1]>700 && sensor_value[2]>700)
   {
    printf("spot right\n");
    printf("%s\n","stay" );
    vTaskDelay(5);
    bot_stop(MCPWM_UNIT_0,MCPWM_TIMER_0);
    vTaskDelay(30);
    if(sensor_value[4]<400 && sensor_value[5]<400 )
      {
        q=1;
        //break;
        recIntersection('R');
		}
	if(sensor_value[4]>200 || sensor_value[5]>200 )
      {
        bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
       // break;
        recIntersection('S');
		}

   while(q==1)
    {
      printf("Turn Loop right\n");
      read_sensors();
      calc_sensor_values();
      if(sensor_value[4]>400 && sensor_value[5]>400 )
      {
      	bot_stop(MCPWM_UNIT_0,MCPWM_TIMER_0);
        vTaskDelay(50);
        q=0;
        break;
      }
      bot_spot_right(MCPWM_UNIT_0,MCPWM_TIMER_0,65,65);
    }
}
    if(sensor_value[0]>700 && sensor_value[3]<400 && sensor_value[1]>700 &&sensor_value[2]>600)
   {
    printf("spot left\n");
    printf("%s\n","stay" );
    vTaskDelay(5);
    bot_stop(MCPWM_UNIT_0,MCPWM_TIMER_0);
    vTaskDelay(50);
    if(sensor_value[4]<400 && sensor_value[5]<400 )
      {
        q=1;
       // break;
        recIntersection('L');
		}
	if(sensor_value[4]>400 || sensor_value[5]>400 )
      {
        r=1;
       // break;
        recIntersection('L');
		}
    while(q==1)
    {
      printf("Turn Loop left\n");
      read_sensors();
      calc_sensor_values();
      if(sensor_value[4]>400 && sensor_value[5]>400 )
      {
      	bot_stop(MCPWM_UNIT_0,MCPWM_TIMER_0);
        vTaskDelay(50);
        q=0;
        break;
      }
      bot_spot_left(MCPWM_UNIT_0,MCPWM_TIMER_0,65,65);
     
    }
        while(r==1)
    {
      printf("Turn Loop left\n");
      read_sensors();
      calc_sensor_values();
      if(sensor_value[4]<400 && sensor_value[5]<400 )
      {
      	bot_stop(MCPWM_UNIT_0,MCPWM_TIMER_0);
        vTaskDelay(30);
        r=0;
        s=1;
        break;
      }
      bot_spot_left(MCPWM_UNIT_0,MCPWM_TIMER_0,65,65);
      
    }
     while(s==1)
    {
      printf("Turn Loop left\n");
      read_sensors();
      calc_sensor_values();
      if(sensor_value[4]>400 && sensor_value[5]>400 )
      {
      	bot_stop(MCPWM_UNIT_0,MCPWM_TIMER_0);
        vTaskDelay(30);
        s=0;
        break;
      }
      bot_spot_left(MCPWM_UNIT_0,MCPWM_TIMER_0,65,65);
      
    }
    }
    if(sensor_value[0]<400 && sensor_value[1]<400 && sensor_value[2]<400 && sensor_value[3]<400)
    {
    	vTaskDelay(5);
    bot_stop(MCPWM_UNIT_0,MCPWM_TIMER_0);
    vTaskDelay(30);
    if(sensor_value[4]<400 && sensor_value[5]<400 )
      {
        q=1;
        recIntersection('B');
      // break;
		}
    while(q==1)
    {
      printf("Turn Loop left\n");
      read_sensors();
      calc_sensor_values();
      if(sensor_value[4]>400 && sensor_value[5]>400 )
      {
      	bot_stop(MCPWM_UNIT_0,MCPWM_TIMER_0);
        vTaskDelay(50);
        q=0;
        break;
      }
      bot_spot_left(MCPWM_UNIT_0,MCPWM_TIMER_0,65,65);
      
    }

    }
    if(sensor_value[0]>400 && sensor_value[1]>400 && sensor_value[2]>400 && sensor_value[3]>400 && sensor_value2[0]>400 && sensor_value2>400)
    {
    	bot_stop(MCPWM_UNIT_0,MCPWM_TIMER_0);
    	gpio_set_direction(LED_1,GPIO_MODE_OUTPUT);
	    gpio_set_direction(LED_2,GPIO_MODE_OUTPUT);	
	    gpio_set_level(LED_1,0);	
		gpio_set_level(LED_2,0);
		for(int i=0;i<pathLength;i++)
    {
    	printf("%c\t",path[i]);
    	printf(" ");
    }
    }
}
static void control()
{
	if(sensor_value[0]<600 && sensor_value[3]<600 && sensor_value[1]>100)
	{
		left_pwm = constrain((opt - correction), lower_pwm_constrain, higher_pwm_constrain);
    //printf(" %2f\t",left_pwm);

     right_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
    //printf(" %2f\t",right_pwm);
     bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
    }
    else{

        printf("%s\n","maze_solve" );
    	maze_solve();
    }
    
}
   
// void simplifyPath()
// {

// }
void line_follow_task(void *arg)
{
 mcpwm_initialize();
// move_extra();
//read_sensors();
  while(1)
  {
   read_sensors();
   calc_sensor_values();
   calculate_error();
   sensor();
   calculate_correction();
     left_pwm = constrain((opt - correction), lower_pwm_constrain, higher_pwm_constrain);
    //printf(" %2f\t",left_pwm);

     right_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
    //printf(" %2f\t",right_pwm);
    bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
   //maze_solve();
    control();
  // printf(" %d\t",k);
    
    printf("\n");
   }
}
  
 void app_main()
{
    xTaskCreate(&line_follow_task,"line_follow_task",4096,NULL,1,NULL);
	
}

