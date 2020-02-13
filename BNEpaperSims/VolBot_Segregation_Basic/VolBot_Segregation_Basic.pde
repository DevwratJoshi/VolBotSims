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

Box2DProcessing box2d;

Box box;
final float stepsPerSec = 200.0;
final float dataCollectionRate = 20.0; // Real world seconds between data points
final int readings_per_sim = 5; // The number of times to take readings before changing sim conditions
int readings_taken = 0; // The number of readings actually taken
//Worth noting here that dataCollectionRate seconds between sims and readings_per_sim readings means dataCollectionRate*readings_per_sim seconds for a group of conditions 

int steps = 0; // keeps track of the number of times the world has stepped
final int DELAY = 100;
// A list for all of our robots
ArrayList<Robot> robots;
ArrayList<Vec2> path;
Ground ground;
int no_of_robots = 120;
int large_robots = 0;
int small_robots = 0; 
int count = 0;
char flag = 'n';
float velConst = 1;
Vec2 vel = new Vec2();
boolean box_pause = true;
float big = 60;
float small = 30;
float freq;
float freqStep = 5.0;
float maxFreq = 30.0;
float initialFreq = 10.0;

int amplitude;
int ampStep = 1;
int maxAmp = 10;
int initialAmp = 2;

int delay = 0;
int record = 0;
PrintWriter outputx, outputy, output;

float box_bottom = 800;
int mean_box_height;
Vec2 center_pos, center_velo;
int velocityDirection;

Vec2 mouse1, mouse2;
Vec2 temp_mouse = new Vec2();
boolean mouseActive;

StringList LinePositionNumbers; // The array to hold the names of the files for different simulations
StringList LinePositionGroup;
String extention; // The extention for the files 
int currentUpPosition;
int currentDownPosition;
Vec2 COM = new Vec2();
Vec2 box_pos = new Vec2();

 
String in = "initial_positions";

boolean blueBallSelected = false;
BufferedReader reader;
int fileCounter;
float probability = 0.1; // The initial value of the probability
final float probIncrease = 0.1; // The amount the probability will increase per loop

boolean dataCollectMode = false;
Vec2 lastBoxPos = new Vec2(0, 0);
float SavedBoxHeight = 0.;

int ampNumber = 0;


boolean exitFlag = false; // exit() does not let the program exit immidiately. It will execute draw one last time, which might be troublesome. 
// Do not let anything else run if exitFlag is true
void setup()
{
  size(1000, 900);
  smooth();

  box2d = new Box2DProcessing(this, 100); /// This is the ratio of pixels to m.

  box2d.createWorld();
  box2d.setGravity(0, -9.8);
  mean_box_height = 9*height/10;

  mouse1 = new Vec2(width-(box_bottom/2), 0);
  mouse2 = new Vec2(width-(box_bottom/2), 600); // The two points we click at
  mouseActive = false; // If the mouse has been clicked once

  

  robots = new ArrayList<Robot>();
  path = new ArrayList<Vec2>();
  center_velo = new Vec2();
  center_pos = new Vec2();

  ground = new Ground();
  vel.x = 0.0;
  vel.y = velConst;

  amplitude = initialAmp - ampStep;
  freq = initialFreq;  
  
  extention = ".txt";

  
  output = createWriter("Segregation_data" + str(day()) + "_" + str(month()) + "_" + str(year()) + "_" + str(hour()) + ":" + str(minute())  + extention);
   output.println("no_of_robots" + "," + "large_robots" + "," + "small_robots" + "," + "freq" + "," + "amplitude" + "," + "height" + ","  + "steps");
   float g = amplitude*4*PI*PI*freq*freq/(9.8*100);
   

   println("Segregation_data" + str(day()) + "_" + str(month()) + "_" + str(year()) + "_" + str(hour()) + ":" + str(minute())  + extention);


}

void draw() {   
  if (steps == 0)
  {
    setSimulationConditions();
    if(!exitFlag)
    {
      // Creating the box
      box = new Box(width/2, height - 50);
      /// End creating the box
  
      //////// Creating new robots using data from initial_positions.txt
      createRobots(in + extention);
      /////// End of creating robots
  
      //////// Setting delay to zero
      delay = 0;
      ///////// End setting delay to zero
      
      write_to_file();       ///////////// Robots must all be the correct size initially. Check the initial lambda before shaking begins
    }
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  /*
    If the dataCollectMode is made true, the following sequence will be initiated. 
    The following steps are taken
    1) The box is stopped by making boxPause = true
    2) The function waits for all the robots to stop moving. Set the initial positions of the robots and check that they do not move. 
    N.B. Check that the integer values of the postions are the same as the last timestep
  */
  if(!exitFlag)
  {
    if (dataCollectMode)
    {
      background(150, 150, 150);
      box2d.step(5/stepsPerSec, 10, 10);
      box_pause = true; // Stop moving the box
      
   // lambdaCollected will be false while the robots have not come to rest
   // Wait for robots to come to rest in the following section

        robots_store_last_positions();
        box2d.step(2/stepsPerSec, 10, 10);
        box.display();
        displayRobots();
        ground.display();
        // The following function checks the change in the positions of the robots by comparing their current positions to the positions stored in the robot.last_position variable
        if(!robots_moving()) 
        { //<>//
           write_to_file();
           for(Robot r : robots)
           {
            r.last_position = new Vec2(0,0); 
           }
           dataCollectMode = false; // End data collection mode
        }
        
        
        
      

    }
    
    else if(!dataCollectMode)
    {
      background(255);
      box2d.step(1/stepsPerSec, 10, 10);

      steps++;
      box_pause = false;
      // Display all the boundaries
      box.display();
      ground.display();
  
      displayRobots();
  
      
  
      delay++;
      if (delay > DELAY)
      {
  
        record++;
        box_pause = false;
        display_sim_conditions();
  //////////////////////////// Shake box block  
  
        if (!box_pause)
        {
          box.applyVelocity(calcVelocity(box.checkPos()));
        } else if (box_pause)
        {
          vel.x = 0.;
          vel.y = 0.;
          box.applyVelocity(vel);
        }
  //////////////////////////// End shake box block
  
        if (steps%(stepsPerSec*dataCollectionRate) == 0 && steps != 0)
        {
  
          dataCollectMode = true;
          SavedBoxHeight = box.checkPos().y;
         
        }
  
    
    
      }
    }
    // This is to delete any robots that may have jumped over the wall and out of range. They must be deleted to let the robots_moving() function return false 
    for (int i = robots.size()-1; i >= 0; i--) {
        Robot p = robots.get(i);
        if (p.done(box.checkPos().y - box.w/2)) {
          robots.remove(i);
        }
      }
  }
  
  else
  {
   exit(); 
  }
}
///////////////////////////////////////////////

void display_sim_conditions()
{
  pushMatrix();
  fill(0, 102, 153);
  textSize(32);
  text("Frequency = " + str(freq), 120, 30); 
  text("Amplitude = " + str(amplitude), 120, 60);
  textSize(20);
  fill(0);
  text("Press 's' to exit the program and save data", 100, height - 30);
  popMatrix();
  
}


void write_to_file()
{  
      output.println(no_of_robots + "," + large_robots + "," + small_robots + "," + freq + "," + amplitude + "," + collectLambda('l') + ","+  steps);
      output.flush();
      readings_taken += 1;
      println(readings_taken);
      if(readings_taken >= readings_per_sim) 
      {
         steps = 0;
         readings_taken = 0;

     for (int i = robots.size()-1; i >= 0; i--)
      {
        Robot p = robots.get(i);
        p.killBody();
        robots.remove(i);
    
      }
      
      box.killBody();
      }
}

boolean robotRadiusIncrease(Robot r)
{
  
  if(r.bigProb && ((mouse1.y - mouse2.y)/(mouse1.x - mouse2.x))*(r.checkPos().x - mouse1.x) + mouse1.y < r.checkPos().y)
  {
    return true;
  }
  return false;
  
}
void displayRobots()
{
  for (Robot p : robots)
  {
    p.display();
  }
}

/////////////////////////////////////////////// 

/*
The following function collects the data we need and returns the value. 
Parameter: 
mode: if mode = 'l', will calculate lambda as smallHeight-bigHeight/bedHeight
      if mode = 'h', will reutrn the com of the large modules
*/
float collectLambda(char mode)
{
  // To write the current value of lambda to file

  // println("Collected some data");

  float lambda = 0.0;
  
  float bedHeight = 0.0; // Height of the bed. Needed as lambda calculated as a percent of the bed height.  
  float smallHeight = 0.0;
  float bigHeight = 0.0;
  int bigRobots = 0;
  int smallRobots = 0;
  switch(mode)
  {
    
    case 'l':
    {
      for(Robot p : robots)
        {
            float robotHeight = box.checkPos().y - p.checkPos().y;
            if(robotHeight + p.r > bedHeight)
            {
              bedHeight = robotHeight + p.r; 
            }
              
            if(big - p.r < p.r - small) // The robot is big
            {
                bigHeight += box.checkPos().y - p.checkPos().y;
                bigRobots += 1;
            }
            
            else
            {
               smallHeight += box.checkPos().y - p.checkPos().y;
               smallRobots += 1;
            }
            
        }
        
        bigHeight = bigHeight/bigRobots;
        smallHeight = smallHeight/smallRobots;
        
        lambda = (smallHeight-bigHeight)/bedHeight;
    }
    
    case 'h':
    {
       for(Robot p : robots)
        {
            float robotHeight = box.checkPos().y - p.checkPos().y;
           
            if(big - p.r < p.r - small) // The robot is big
            {
                bigHeight += box.checkPos().y - p.checkPos().y;
            }

            
        }   
        lambda = bigHeight;
    }
    
  }
   //output.println();
  

  
  return lambda;
}

///////////////////////////////////////////////

void keyPressed()
{
 if(key == 's')
 {
    output.flush();
    output.close();
    exit();
 }
}
void createRobots(String input)
{
  reader = createReader(input);
  String line = null;
  large_robots = 0;
  small_robots = 0;
  try {
    while ((line = reader.readLine()) != null) 
    {
      //println(line);
      int[] nums = int(split(line, " "));
      char colour = 'g';

      if(nums[2] == 60)
      {
         Robot p = new Robot(nums[0], nums[1], big, 'b', true);
         robots.add(p);
         large_robots += 1;
      }
      
      else if(nums[2] == 30)
      {
          Robot p = new Robot(nums[0], nums[1], small, 'g', false);
          robots.add(p);
          small_robots += 1;
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

  maxVelo = 2*PI*freq*amplitude/100.0;
  constVelo = 0.1*maxVelo;
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
    } else if (velocityDirection == -1)
    {
      new_velocity.y = -constVelo;
    }
  } else
  {
    new_velocity.y = maxVelo*sin(map(box_pos.y, mean_box_height - amplitude/2, mean_box_height + amplitude/2, 0.0, 3.1415926535897));
    if (velocityDirection == -1)
    {
      new_velocity.y = -1*new_velocity.y;
    }
  }

  //println(new_velocity.y + "    " + (box_pos.y - mean_box_height));
  return new_velocity;
}

void setSimulationConditions()
{
   amplitude += ampStep;
   
   if(amplitude > maxAmp)
     {
       amplitude = initialAmp;
       freq += freqStep;
      
       if(freq > maxFreq)
       {
         println("This is the end of the program");
         output.flush();
         output.close();
         exitFlag = true;
       }      
     }     
       
   
}

void robots_store_last_positions()
{
  for(Robot r : robots)
  {  
    Vec2 pos = new Vec2();
    pos = r.checkPos();
    // The values are stored after getting rid of the numbers after the decimal point. Convert to int and then back to float
     r.last_position.x = float(int(pos.x));
     r.last_position.y = float(int(pos.y));

  }
  
}

boolean robots_moving()
{
  for(Robot r : robots)
  {
   Vec2 pos = new Vec2();
   pos = r.checkPos();
   if(r.last_position.x != float(int(pos.x)) || r.last_position.y != float(int(pos.y)))
   {
     
     pushMatrix();
     translate(pos.x, pos.y);
     fill(255, 0,0);
     ellipse(0, 0, small, small);
     popMatrix();
     return true;
   }
  }
  return false;
}
