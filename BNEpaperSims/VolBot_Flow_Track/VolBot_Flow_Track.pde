// VolBot simulator using the jave-processing ported version of box2D by shiffman
/*
This program should automatically execute simulations for 5 different line angles at the same speed of vibration. 
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
final float dataCollectionRate = 15.0; // Real world seconds between data points
int steps = 0; // keeps track of the number of times the world has stepped
final int DELAY1 = 100; // Delay before box starts shaking & robots become big
final int DELAY2 = 400; // Delay before PIV data starts getting collected
// A list for all of our robots
ArrayList<Robot> robots;
ArrayList<Grid> grids; // The list to store "filterWidth" number of grids to use median filter on 
ArrayList<Vec2> path;
Ground ground;
int no_of_robots = 280;
int count = 0;
char flag = 'n';
float velConst = 1;
Vec2 vel = new Vec2();
boolean box_pause = true;
float big = 40;
float small = 20;
float freq;
float freqStep = 10.0;
float maxFreq = 20.0;
float initialFreq = 20.0;

int amplitude;
int ampStep = 2;
int maxAmp = 8;
int initialAmp = 8;

int delay = 0;
int record = 0;
PrintWriter outputx, outputy, output;

float box_bottom = 960;
float box_height = box_bottom;
int mean_box_height;
Vec2 center_pos, center_velo;
int velocityDirection;

Vec2 mouse1, mouse2;
Vec2 temp_mouse = new Vec2();
boolean mouseActive;
int maxRecord = 50000;
StringList LinePositionNumbers; // The array to hold the names of the files for different simulations
StringList LinePositionGroup;
String extention; // The extention for the files 
String folder_begin = "PIV_data_constant_top_2/";
//String folder_begin = "test_data/";
int currentUpPosition;
int currentDownPosition;
Vec2 COM = new Vec2();
Vec2 box_pos = new Vec2();
int b_line_top = 5;   // The intersection of the boundary line and the top of the box
int b_line_bottom = 6; // The intersection of the boundary line and the bottom of the box
int b_line_section_max = 10; // The last section number. Assuming the same number of intersects for both top and bottom
final int verticalDivisions = 20;
final int horizontalDivisions = 20;
final int filterWidth = 9; // The width of the median filter  
//The arrays defined in this section are used to hold the actual filtered values. The culmunative velocity will simply be added and any data processing will be done later
float xVelocities[][] = new float[horizontalDivisions][verticalDivisions];
float yVelocities[][] = new float[horizontalDivisions][verticalDivisions]; // These are the divisions to be used for the velocity distribution graph. 
int PIV_collected = 0;
String in = "initial_positions_280_3";

boolean blueBallSelected = false;
BufferedReader reader;
int fileCounter;
float probability = 0.1; // The initial value of the probability
final float probIncrease = 0.1; // The amount the probability will increase per loop

boolean dataCollectMode = false;
Vec2 lastBoxPos = new Vec2(0, 0);
boolean boxOnGround = false;
boolean lambdaCollected = false;
float SavedBoxHeight = 0.;

int ampNumber = 0;
boolean exitFlag = false;

char colour = 'b'; // This is the colour of the tracked ball
void setup()
{
  size(1200, 1100);
  smooth();

  box2d = new Box2DProcessing(this, 100); /// This is the ratio of pixels to m.

  box2d.createWorld();
  box2d.setGravity(0, -9.8);
  mean_box_height = 10*height/11;

  mouse1 = new Vec2(width-(box_bottom/2), 0);
  mouse2 = new Vec2(width-(box_bottom/2), 600); // The two points we click at
  mouseActive = false; // If the mouse has been clicked once

  robots = new ArrayList<Robot>();
  path = new ArrayList<Vec2>();
  grids = new ArrayList<Grid>();
  center_velo = new Vec2();
  center_pos = new Vec2();

  ground = new Ground();
  vel.x = 0.0;
  vel.y = velConst;

  amplitude = initialAmp - ampStep;
  freq = initialFreq;  
  
  extention = ".txt";
  fileCounter = 0;
  

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
   
  if (steps == 0)
  {
    box = new Box(width/2, height - 50);
    setSimulationConditions();
    
    if(!exitFlag)
    {
    // Creating the box
    
    /// End creating the box

    //////// Creating new robots using data from initial_positions.txt
    createRobots(in + extention);
    /////// End of creating robots

    //////// Setting delay to zero
    delay = 0;
    ///////// End setting delay to zero
    
    }
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // We must always step through time!
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
        {
          // write_to_file();
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
    if(delay < DELAY1)
    steps++;
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
    if (delay > DELAY1)
    {
      steps++;
      pushMatrix();
      strokeWeight(5);
      stroke(112, 48, 160);
      line(mouse1.x, mouse1.y, mouse2.x, mouse2.y);
      popMatrix();
      
      record++;
      box_pause = false;
/////////////////////////Reset zone line block

setZoneLine(b_line_top, b_line_bottom);

//////////////////////// End reset zone line block

///////////////////////// Change robot size block
      for (int i = robots.size()-1; i >= 0; i--) 
      {
      Robot p = robots.get(i);

      if (robotRadiusIncrease(p))  
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
//////////////////////////// End Change robot size block

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
/*
      if (steps%(stepsPerSec*dataCollectionRate) == 0)
      {

        dataCollectMode = true;
        SavedBoxHeight = box.checkPos().y;
      }

*/      
if(delay > DELAY2)
{
      if(steps%5 == 0 )
      {
        for(Robot r: robots)
        {
          if(r.colour == 'b')
          {
             Vec2 pos = new Vec2();
             pos.x = r.checkPos().x;
             pos.y = r.checkPos().y;
           
             path.add(pos); 
             output.println(pos.x + ","  + (box.checkPos().y - pos.y));
             output.flush();
             break;
          }
        } 
    }
        
        //displayPath(colour);
}
    }
  }
  }

  
}
///////////////////////////////////////////////
/*
void mouseClicked()
{
  Vec2 mpos = new Vec2(mouseX, mouseY);
  for (Robot r : robots)
  {
     Vec2 pos = r.checkPos();
     
     if(sqrt((pos.x - mpos.x)*(pos.x - mpos.x) + (pos.y - mpos.y)*(pos.y - mpos.y)) < r.r)
     {
       r.colour = 'b';
       r.bigProb = true;
       blueBallSelected = true;
       return;
     }
    
  }
}
*/
void keyPressed()
{
  if(key =='s')
  {
   output.flush();
   output.close();
   exit();
  }
}

void display_sim_conditions()
{
  pushMatrix();
  fill(0, 102, 153);
  textSize(32);
  text("Frequency = " + str(freq), 120, 90); 
  text("Amplitude = " + str(amplitude), 120, 120);
  popMatrix();
  
}

void displayPath(char c)
{
  for(Vec2 p : path)
  {
    pushMatrix();
    switch(c)
    {
      case 'b':
      stroke(0,0,255);
      break;
      
      case 'g':
      stroke(0,255,0);
      break;
      
      default:
      stroke(255,0,0);      
    }
    
    point(p.x, p.y); 
    popMatrix(); 
  }
}
boolean robotRadiusIncrease(Robot r)
{
  if(r.bigProb)
  {
      if(((mouse1.y - mouse2.y)/(mouse1.x - mouse2.x))*(r.checkPos().x - mouse1.x) + mouse1.y < r.checkPos().y)
      {
        return true;
      }
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

void writePIVtoFile()
{
for(int y = verticalDivisions-1; y >= 0; y--)
   {
      for(int x = 0; x < horizontalDivisions; x++)
      {
         outputx.print(xVelocities[x][y] + ",");
         outputy.print(yVelocities[x][y] +  ",");
      }
      outputx.println();
      outputy.println();
   }
   
   outputx.flush();
   outputy.flush();
   
   outputx.close();
   outputy.close();         
}

/////////////////////////////////////////////// 
// The following function will collect the necessary data and write it to file

void collectPIV()
{
  if(grids.size() >= filterWidth)
  { 

    for(int x = 0; x < horizontalDivisions; x++)
    {
      for(int y = 0; y < verticalDivisions; y++)
      {
        float xVelo = 0.0;
        float yVelo = 0.0;
        for(Grid g : grids)
        {
          xVelo += g.checkXVelocity(x, y);
          yVelo += g.checkYVelocity(x, y);
        }
        xVelo = xVelo/filterWidth;
        yVelo = yVelo/filterWidth;
       
        xVelocities[x][y] += xVelo;
        yVelocities[x][y] += yVelo;
        PIV_collected += 1;
      }
    }
    // remove the first element of the grid and add another 
    grids.remove(0);     
  }
  
/////////////// Add a new grid element here. 
Grid g = new Grid();
   Vec2 boxp = box.checkPos();
  
   for(Robot r : robots)
  {
   Vec2 pos = r.checkPos();
   int x = 0;
   int y = 0;
   int i = 0;
   
     while(i < horizontalDivisions)
     {
       if(pos.x > ((boxp.x - box_bottom/2) + i*(box_bottom)/horizontalDivisions) && pos.x < ((boxp.x - box_bottom/2) + ((i+1)*box_bottom)/horizontalDivisions))
       {
         x = i;
         break;
       }
       i++;
     }
     
     i = 0;
    while(i < verticalDivisions)
     {
       if(pos.y < ((boxp.y - box.w/2) - i*box_height/verticalDivisions) && pos.y > ((boxp.y - box.w/2) - (i+1)*box_height/verticalDivisions))
       {
         y = i;
         break;
       }
       i++;
     }

            g.addXVelocity(x, y, r.checkVelocity().x);
            g.addYVelocity(x, y, r.checkVelocity().y);
  }
         
    grids.add(g);   
}
//////////////////////////////////////////////
// Changing this function to take the position of d
void setZoneLine(int top, int bottom) 
{
  //mouse2.x = width/2 - (box_bottom/2) + (top * box_bottom)/(b_line_section_max + 1); 
  //mouse1.x = width/2 - (box_bottom/2) + (bottom * box_bottom)/(b_line_section_max+1);
  //The below is to fix the top of the boundary line and move the bottom from one end of the bounding box to the other. The angles will be calculated seperately. 
  // The point selection strategy still involves selecting intercepts from the bottom of the box
  mouse2.x = width/2;
  mouse1.x = width/2 - (box_bottom/2) + (bottom * box_bottom)/(b_line_section_max); 
  
  Vec2 boxp = box.checkPos();
  
  mouse2.y = boxp.y - box_height - box.w/2;
  mouse1.y = boxp.y - box.w/2;
}
///////////////////////////////////////////////


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
         freq = initialFreq;
         b_line_bottom += 1;
         
           if(b_line_bottom > b_line_section_max)
             {
               b_line_bottom = 0;
        
                 println("This is the end of the program"); 
                 exit();
                 exitFlag = true;
               
             }
       }      
     }
     
     if(!exitFlag)
    {
       setZoneLine(b_line_top, b_line_bottom);
       grids.clear();
       
       for(int x = 0; x < horizontalDivisions; x++)
       {
         for(int y = 0; y < horizontalDivisions; y++)
         {
           xVelocities[x][y] = 0.0;
           yVelocities[x][y] = 0.0;
         }
       }
       
       output = createWriter(folder_begin + "/"  + "position_track_counter_clockwise");
 
       println("Set sim conditions for " + "f = " + str(freq) + " amp = " + str(amplitude) + " up = " + str(b_line_top)+ " bottom = " + str(b_line_bottom));
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

      float r = random(1);
       if(robots.size() == 140)
        {
          Robot p = new Robot(nums[0], nums[1], small, 'b', true);
          robots.add(p);
          continue;
        }
        
      if(r < 0.5)
      {

         Robot p = new Robot(nums[0], nums[1], small, 'g', true);
         robots.add(p);
        
         
      }
      
      else if(r >= 0.5)
      {
          Robot p = new Robot(nums[0], nums[1], small, 'g', false);
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
