// VolBot simulator using the jave-processing ported version of box2D by shiffman
/*
This prorgam should automatically execute simulations for 5 different line angles at the same speed of vibration. 
 The program will same the position data for the blue ball in the files with names specified in the filenames stringlist
 The number of datapoints recorded for each file will be given by the variable max_record. 
 The initial positions of the robots will be as defined in the initial_positions.txt file. The same positions will be used every time. 
 */
import shiffman.box2d.*;
import org.jbox2d.collision.shapes.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.joints.*;

import controlP5.*; // This library creates the sliders to control frequency and so forth

Box2DProcessing box2d;

Box box;
final float stepsPerSec = 200.0;
final float dataCollectionRate = 15.0; // Real world seconds between data points
int steps = 0; // keeps track of the number of times the world has stepped
final int DELAY = 200;
final String mode = "SEG"; // The mode can be SEG for segregation or FLOW for flow. zone lines not drawn for SEG and drawn for FLOW
// A list for all of our rectangles
ArrayList<Robot> robots;
ArrayList<Vec2> path;
Ground ground;
int no_of_robots = 100;
int count = 0;
char flag = 'n';
float velConst = 1;
Vec2 vel = new Vec2();
boolean box_pause = true;
float big = 60;
float small = 30;

float freq;
float freqStep = 10.0;
float maxFreq = 15.0;
float initialFreq = 15.0;
Slider freqSlider;

int amplitude;
int ampStep = 1;
int maxAmp = 8;
int initialAmp = 8;
Slider ampSlider;

float bigDensity;
float densityStep = 1;
float initialDensity = 5.0;
float maxDensity = 5.0;
Slider densitySlider;

int delay = 0;
int record = 0;


float box_bottom = 800;
int mean_box_height;
Vec2 center_pos, center_velo;
int velocityDirection;

Vec2 mouse1, mouse2;
Vec2 temp_mouse = new Vec2();
boolean mouseActive;
int maxRecord = 20000;
StringList LinePositionNumbers; // The array to hold the names of the files for different simulations
StringList LinePositionGroup;
String extention; // The extention for the files 
int currentUpPosition;
int currentDownPosition;
Vec2 COM = new Vec2();
Vec2 box_pos = new Vec2();
final int vertical_divisions = 15;
final int horizontal_divisions = 15; // These are the divisions to be used for the velocity distribution graph. 
String in = "initial_positions";

boolean blueBallSelected = false;
BufferedReader reader;
int fileCounter;
float probability = 0.1; // The initial value of the probability
final float probIncrease = 0.1; // The amount the probability will increase per loop

boolean dataCollectMode = false;
Vec2 lastBoxPos = new Vec2(0, 0);
boolean boxOnGround = false;
boolean dataCollected = false;
float SavedBoxHeight = 0.;

int ampNumber = 0;

ControlP5 cp5;
Slider abc;

void setup()
{
  size(1000, 900);
  smooth();

  box2d = new Box2DProcessing(this, 100); /// This is the ratio of pixels to m.

  box2d.createWorld();
  mean_box_height = 9*height/10;

  mouse1 = new Vec2(width-(box_bottom/2), 0);
  mouse2 = new Vec2(width-(box_bottom/2), 600); // The two points we click at
  mouseActive = false; // If the mouse has been clicked once

  box2d.setGravity(0, -9.8);

  robots = new ArrayList<Robot>();
  path = new ArrayList<Vec2>();
  center_velo = new Vec2();
  center_pos = new Vec2();


  int box_height = height*2;

  ground = new Ground();
  vel.x = 0.0;
  vel.y = velConst;

  amplitude = initialAmp - ampStep;
  freq = initialFreq;
  bigDensity = initialDensity;
  
  extention = ".txt";
  fileCounter = 0;
  
  cp5 = new ControlP5(this);
  
    freqSlider = cp5.addSlider("Frequency")
     .setPosition(width/2 - 400, 20)
     .setColorCaptionLabel(0)
     .setRange(1, 50)
     .setSize(200,50)
     ;
     
  ampSlider = cp5.addSlider("Amplitude")
     .setPosition(width/2 + 100, 20)
     .setColorCaptionLabel(0)
     .setSize(200,20)
     .setRange(1, 20)
     .setNumberOfTickMarks(20)
     ;
     
   densitySlider = cp5.addSlider("Density")
     .setPosition(width/2 - 400, 100 )
     .setColorCaptionLabel(0)
     .setSize(200,20)
     .setRange(-10, 10)
     .setNumberOfTickMarks(20)
     ;     
     
   /*
   cp5.addSlider("Diameter Rato")
     .setPosition(width/2 + 100, 100 )
     .setColorCaptionLabel(0)
     .setSize(200,20)
     .setRange(1, 3)
     ;     
     */
     
     box = new Box(width/2, height - 50, 'k');
     createRobots(in + extention);
}

void draw() {
  //background(255);
  /*
    The following if statement is an initialiser.
   It should run every time the record count for a particular file is complete. 
   It has to do the following
   1) Set the next file to be written. 
   2) Initialise all the robots. The robots will be destroyed in a seperate if block
   3) Reset the delay to zero, so the next batch of robots can settle down before being shaken
   4) Set the position of the next line. 
   */

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // We must always step through time!

    background(255);
    box2d.step(1/stepsPerSec, 10, 10);
   // box2d.step();
    steps++;
    
    freq = freqSlider.getValue();
    amplitude = (int)ampSlider.getValue();
    
    for (int i = robots.size()-1; i >= 0; i--) {
        Robot p = robots.get(i);
        if(p.bigProb)
        {
           if(p.density != densitySlider.getValue())
             {
               Robot g = new Robot(p.checkPos().x, p.checkPos().y, big, 'g', true, densitySlider.getValue());
               robots.add(g);
              p.killBody();
              robots.remove(i);
              
              
             }
        }

    }
    
    // Display all the boundaries
    box.display();
    ground.display();
    displayRobots();

    for (int i = robots.size()-1; i >= 0; i--) {
      Robot p = robots.get(i);
      if (p.done()) {
        robots.remove(i);
      }
    }

    delay++;
    if (delay > DELAY)
    {
      record++;
      box_pause = false;
      float total_height = 0;



      for (int i = robots.size()-1; i >= 0; i--) {
        Robot p = robots.get(i);

        if (p.bigProb)  // If the robot is a designated large robot
        {
          p.increasing = true;
        } else 
        {
          p.decreasing = true;
        }
      }
      for (int i = robots.size()-1; i >= 0; i--) 
      {
        Robot p = robots.get(i);
        if (p.increasing) // If the robot is small and close to the bottom of the box
        {
          p.changeRadius(big);
        } else if (p.decreasing)
        {
          p.changeRadius(small);
        }
      }


      if (!box_pause)
      {
        box.applyVelocity(calcVelocity(box.checkPos()));
        //println("The position of the box is = " + box.checkPos().y);
      } 
      
      else if (box_pause)
      {
        vel.x = 0.;
        vel.y = 0.;
        box.applyVelocity(vel);
      }

      
      
     
    }
  
}
///////////////////////////////////////////////
void displayRobots()
{
  for (Robot p : robots)
  {
    p.display();
  }
}


void setZoneLine()
{
  mouse1.x = width/2-(box_bottom/2) + (currentUpPosition-1)*(box_bottom/LinePositionNumbers.size()-1); 
  mouse2.x = width/2-(box_bottom/2) + (currentDownPosition-1)*(box_bottom/LinePositionNumbers.size()-1);

  //println(mouse1 + " " + mouse2);
  probability = probability + probIncrease;
  if (probability > 1.0)
  {
    probability = 0.05;
    currentDownPosition--;
    if (currentDownPosition <= 1)
    {
      currentUpPosition--;
      currentDownPosition = currentUpPosition-1;
      if (currentUpPosition <= 4)
      {
        println("This is the end of the program.");
        println("Number of files written = " + fileCounter);
        exit(); // Fail-safe
      }
    }
  }
}
///////////////////////////////////////////////
void createRobots(String input)
{
  reader = createReader(input);
  String line = null;

  try {
    while ((line = reader.readLine()) != null) 
    {
      //println(line);
      int[] nums = int(split(line, " "));
      char colour = 'g';
    
    
 
      
      if(nums[2] == big)
      {
         Robot p = new Robot(nums[0], nums[1], big, 'g', true, densitySlider.getValue());
         robots.add(p);
      }
      
      else if(nums[2] == small)
      {
          Robot p = new Robot(nums[0], nums[1], small, 'b', false, 1.0);
          robots.add(p);
      }
      
      else
      {
         print("There is something wrong with the initial size setting. Exiting.");
         exit();
      }
    

    }
    reader.close();
  } 
  catch (IOException e) 
  {
    e.printStackTrace();
  }
}

Vec2 calcVelocity(Vec2 box_pos)
{
  Vec2 new_velocity = new Vec2();
  float constVeloMargin = 0.1*amplitude;
  float constVelo = 0.1;
  float maxVelo;

  maxVelo = PI*freq*amplitude/100.0; // This is not multiplied by 2 because the amplitude is actually peak to peak amplitude. However, the calculation expects peak amplitude
  constVelo = 0.1*maxVelo;
  
  if(constVelo < 0.5)
  constVelo = 0.5;
  
  if (box_pos.y > mean_box_height + amplitude/2)
  {
    new_velocity.y = 0.5;
    velocityDirection = 1; // The box will move upward from now on
  } else if (box_pos.y < mean_box_height - amplitude/2)
  {
    new_velocity.y = -0.5; 
    velocityDirection = -1;
  }

  /*
    The next if else statement deals with the case that the box is in the section just before the maximum distance from the mean height
   The velocity will be a constant 
   */
  else if ((box_pos.y  > (float)(mean_box_height + amplitude/2 - constVeloMargin)) || (box_pos.y  < (float)(mean_box_height - amplitude/2 + constVeloMargin)))
  {
    if (velocityDirection == 1)
    {
      new_velocity.y = constVelo;
      //println(constVelo);
      
    } else if (velocityDirection == -1)
    {
      new_velocity.y = -constVelo;
    }
  } else
  {
    new_velocity.y = maxVelo*sqrt(1-(fmakePositive(box_pos.y-mean_box_height)/((float)amplitude/2.0)));
    
    if (velocityDirection == -1)
    {
      new_velocity.y = -1*new_velocity.y;
    }
    //println("Displacement = " + fmakePositive(box_pos.y - mean_box_height));
    //println(new_velocity.y);
  }

  //println(new_velocity.y + "    " + (box_pos.y - mean_box_height));
  return new_velocity;
}



float fmakePositive(float a)
{
  
  if(a < 0.0)
  {
    a = a*(-1.0);
  }
  
  return a;
}

int imakePositive(int a)
{
    if(a < 0)
  a = a*-1;
  
  return a;
}
