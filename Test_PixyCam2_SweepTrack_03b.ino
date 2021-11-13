/*** Header Block ***
   Code file name:
      Test_PixyCam2_SweepTrack_##
   Code description:
      -Unit test program to have the turret sweep across light sources
      annd save their position via the PiixyCam and save their heat level
      via the flame sensor
      -"Light source" attributes are handled via the pixymon application
      by connecting the PixyCam2 to a PC. Briefly: The PixyCam2 will only
      respond to very bright light sources, like a flame, flashlight, or lightbulb.
      -Struct Arrays [1]
      -V00: Non-funnctional
      -V01: Test case
      -V02: Partially Functional
      -V03: changing baud rate to 56k from 19k
      -V03b: Removing serial monitor prints to inc stability
   Hardware components required with this code:
      -Arduino (My model is the Nano Every)
      -Servomotors
      -A buck converter (to supply a steady 5v and amperage to motors)
      -Wires
      -PixyCam2
      -Flame sensor
   IDE version used to test code:
      Arduino IDE v1.8.16
   Programmer(s) name: 
   Date when code is created/modified: Cr:11/06/21 Up:11/12/21
   Code version: 3b

  Based on:
    Test_Flame_Sensor_01 , Test_PixyCam2_Tracking_04f , Test_Servo_CalibrationSweep_01
    by  ***(See individual code files for credit to other's work)

    [1]https://forum.arduino.cc/t/defining-a-struct-array/43699
    []"The problem is that once you an array is passed to a function, it becomes a pointer. So sizeof() is meaningless."
    https://forum.arduino.cc/t/how-to-compare-if-two-arrays-share-at-least-one-element/629406/6
    [2]Arduino Helpers
    https://github.com/tttapa/Arduino-Helpers
    https://forum.arduino.cc/t/possible-to-sort-an-array-of-structs-solved/617954/8
  
  ***/
/****************************************/

// Pre-processor Directives
#include <Servo.h>
#include <Pixy2SPI_SS.h>
//The Arduino Helpers Library [2]
#include <Arduino_Helpers.h>
#include <AH/STL/algorithm>
#include <AH/STL/iterator>
#include "pitches.h"    //To create tones with active buzzer

// Class/object declarations
Servo recoil_servo; //Create servo objects.
Servo pan_servo;
Servo tilt_servo;
Pixy2SPI_SS pixy; //Create PixyCam2 object using the SPI/Arduino interface.

struct light_type{
    int x1; //frame data: the location of the object on the x axis
    int y1; //frame data: the location of the object on the x axis
    int frame_angle_x; //The angle at which this servo took the frame
    int frame_angle_y;
    int angle_x; //The angle of the servo used to aim at the object, using the map function
    int angle_y;
    int temp; //the temperature of the object
  };

light_type light[3];

/***
    Limits  Angles  MicroS  Note
    Recoil  0               is closest to dc motors    
            70      1200    "full stroke"
            120     ????    "at rest" (primed)
    Pan     90      1500    "rest" (centered)
            30      1100    To the "right" (towards mainboard)
            150     1900    To the "left"
    Tilt    5      ????    "Down" (towards ground)
            43       900    "at rest" (parallel to earth)
            75      ????    "Up" (towards sky)
            ??    
***/
//-----General Servo Variables
int recoil_rest = 120, recoil_pushed = 70;  
int pan_rest = 90; 
int pan_limit_2 = 140, pan_limit_1 = 40;
int tilt_rest = 45; 
int tilt_limit_2 = 65, tilt_limit_1 = 25; 

int speed1 = 10;            // increase speed variable to decrease speed actual
int steps = 2;              // increase step variable to increase speed actual

//-----Flame Sensor
const int a0Pin = A0;


//-----Related to SweepTrack
int identifier, buffer3;
bool newVal = false;
int isEmpty;
int j,k;

void setup(){
  //Servos
  //-----Attaches servo to pins
  pan_servo.attach(6);
  pan_servo.write(pan_rest);
  tilt_servo.attach(9);
  tilt_servo.write(tilt_rest);

  //Flame Sensor
  pinMode(a0Pin, INPUT);

  pixy.init();
}

void loop(){
  pixy.ccc.getBlocks(); 
  if(pixy.ccc.numBlocks){// Need to wait to properly initialize Pixy.
    //Zero out (return to center)
    zero_out();
    
    //First, take in the blocks that are present/visible on boot up.
    record_on_boot();
    
    //Sweep in a circle, capturing new data from blocks that may have been missed by the initial view
    pan_motion();
  
    //record_xy_location();
  
    take_temps();
  
    //Sort temps
    //sort_temps();
    
    //Then aim at the largest temp, beep agressively, fire darts twice. aim at the next targets in terms of temp, fire
    //firefighter();
    
    //Zero out
    //zero_out();
    
    //Delay here just to present findings. the let the loop restart.
     delay(1000);
    //Clear array of structs
    clear_a_of_s();
  }
}

//User defined functions
//-----Time-dependent Functions

//-----Data-collecting/Data-processing functions
/*
 * 
 */
void record_on_boot(){
  for (int i = 0; i <pixy.ccc.numBlocks; i++ ){
    light[i].x1 =  pixy.ccc.blocks[i].m_x;
    light[i].y1 =  pixy.ccc.blocks[i].m_y;
    light[i].frame_angle_x =  pan_servo.read();
    light[i].frame_angle_y =  tilt_servo.read();
    light[i].angle_x =  map(light[i].x1,0,315,light[i].frame_angle_x+50,light[i].frame_angle_x-50);
    light[i].angle_y =  map(light[i].y1,0,207,light[i].frame_angle_y+20,light[i].frame_angle_y-20);
  }
}

/*  alpha2 alpha3 alpha4
 *  Initialize j. j is the pan (x) angle of any block currently detected by the pixy mapped 
 *  to where the pan servo is currently at +- 60 (60 being half of delta(pan_liimit1 & 2). If j 
 *  is an angle outisde of pan limit 1 & 2, break the function and ignore the light source. While the  
 *  Piixy can see the light source out of "the corner of iits eye" iit can turn towardds the light source
 *  and take a temp. For the number of structs in the array light [10], if there is a value for the x 
 *  angle location of that light, compare j to that x angle location wwithiin +- 10 degrees. If this
 *  value is > +- 10 deg., buffer1 = j;. If there wwas no value for the x angle location of that light,
 *  it means that that struct has unitialized values. We therefor use buffer1 to create an x angle location 
 *  for a "neww" (previiously underdefined) struct.
*/
void record_xy_location(){
  if(pixy.ccc.numBlocks == 0){
    return;
  }
  
  for(int l = 0;l<pixy.ccc.numBlocks;l++){
    j = map(pixy.ccc.blocks[l].m_x,0,315,pan_servo.read()+50,pan_servo.read()-50);
    k = map(pixy.ccc.blocks[l].m_y,0,207,tilt_servo.read()+20,tilt_servo.read()-20);
    //beta3
    if(j < pan_limit_1 || j > pan_limit_2 ){
      //https://stackoverflow.com/questions/6302227/how-to-break-out-of-a-function/6302235
      return;
    }
    if(k < tilt_limit_1 || k > tilt_limit_2 ){
      //https://stackoverflow.com/questions/6302227/how-to-break-out-of-a-function/6302235
      return;
    }
    
    //beta2
    for (int i = 0; i<3; i++){
      if(light[i].angle_x!=0){
        //If its between these two values...
        newVal = compare_to_array(j);
      }
      else{
        isEmpty = i;

        if (newVal==true){
          light[isEmpty].x1=pixy.ccc.blocks[l].m_x;
          light[isEmpty].y1=pixy.ccc.blocks[l].m_y;
          light[isEmpty].frame_angle_x=pan_servo.read();
          light[isEmpty].angle_x=j;
          light[isEmpty].angle_y=k;
          newVal=false;
        }
        break; //this is an empty spot (equal to 0), theres no more need to compare. We can assume all values after this spot are empty too. break the for loop.
      }
      
    }
  }
  
  //Debugging prints
}

/*  alpha6
 *  Sort array lights[10] from greatest temp to lowest temp.  
 */
void sort_temps(){
  std::sort(std::begin(light), std::end(light), cmpfunc);
  for(light_type e : light){
 }
}

/*
 * 
 */
bool cmpfunc(light_type a, light_type b) { 
  return a.temp > b.temp; 
}

bool compare_to_array(int m){
  int n, o;
  for(int i = 0;i<3;i++){
    n = light[i].angle_x - 20; o = light[i].angle_x + 20;
    if(light[i].angle_x != 0){
      if(m  > n && m < o){
        newVal =false;
        return newVal;
      }
      else{
        newVal =true;
      }
    }
  }
  return newVal;
}

/*
 * 
 */
void clear_a_of_s(){
  for( int i = 0; i < 3;  i++ ){
   light[i].x1 = 0;
   light[i].y1 = 0;
   light[i].frame_angle_x =0;
   light[i].frame_angle_y =0;
   light[i].angle_x = 0;
   light[i].angle_y = 0;
   light[i].temp = 0;
  }
}

void zero_out(){  
  pan_servo.write(pan_rest);
  delay(250);
  tilt_servo.write(tilt_rest);
  delay(250);
}
//Moves field of view in a circle
void pan_motion(){
  for(int i =pan_rest;i<=pan_limit_2;i+=25){
   if(i!=pan_rest){//need to skip the first data-taking to not get duplicates
    pan_servo.write(i);
    delay(500); //Wait to actually get there or the blocks "slide" across the screen and the data pickup is off from 
    //the actual static lightsource.
   if(pixy.ccc.numBlocks){
      record_xy_location();
   }
   }
  }
  for(int i =pan_limit_2;i>=pan_limit_1;i-=25){
    pan_servo.write(i);
    delay(500);
    if(pixy.ccc.numBlocks){
      record_xy_location();
    }
  }
  for(int i =pan_limit_1;i<=pan_rest;i+=25){
    pan_servo.write(i);
    delay(250);
    //dont need to take new data on this pass
  }
}

/*  alpha5 beta5
 *  For num of lights (10), go to each light in no particular order via pan servo. Block should be
 *  at center of frame. Raise pixy
 *  [][][][] Use PID control to move tilt servo up, PID on the tempp sensor, to center the tilt where temp 
 *  is highest?
 *  taise pixy to center of block. Take temp. Set some arbiitrary delay pperhaps just to make it look more
 *  presentable. Also, map this y angle to the struct.
 */
void take_temps(){
  Serial.println("In take_temps()");
  for(int i = 0; i<3; i++){
    if(light[i].angle_x!=0){
      pan_servo.write(light[i].angle_x);
      delay(100);
      tilt_servo.write(light[i].angle_y);
      delay(100);
      light[i].temp = analogRead(a0Pin);
      delay(750);
    }
  }
}

/*  alpha7
 *   Move from highest temp to lowest temp. Fire dart. Beep. Delay for effect. 
 */
void firefighter(){
  for(int i=0; i<3;i++)
    if(light[i].angle_x >0 && light[i].angle_y >0){
      pan_servo.write(light[i].angle_x);
      tilt_servo.write(light[i].angle_y);
    delay(250);
    //Fire recoil servo and beep
   } 
}
