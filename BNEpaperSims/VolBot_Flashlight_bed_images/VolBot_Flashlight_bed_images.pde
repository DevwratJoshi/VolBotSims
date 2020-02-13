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
LightHouse light;
int steps = 0; // keeps track of the number of times the world has stepped
final int DELAY = 100;

// A list for all of our rectangles
ArrayList<Robot> robots;
ArrayList<Vec2> path;
Ground ground;
int no_of_robots = 300;
int count = 0;

float velConst = 1;
Vec2 vel = new Vec2();
boolean box_pause = true;
float big = 60;
float mid = 30;
float small = 20;

int delay = 0;
int record = 0;
PrintWriter output;

float box_bottom = 1800;
float box_height = box_bottom;
int mean_box_height;
Vec2 center_pos, center_velo;
int velocityDirection;

Vec2 mouse1, mouse2;
Vec2 temp_mouse = new Vec2();
boolean mouseActive;
int max_record = 10000;
float goal_sections[] = {2, 3, 4, 5}; // The number of goals for each simulation. The simulation will be reset for each of these. 
float goalCounter = 0; // The variable to keep track of the current position in the goal_sections array

String data_fold[] = {"data1", "data2", "data3", "data4", "data5", "data6", "data7", "data8", "data9", "data10"};
String R_fold = "R"; // Beginning of the folder for values of R (The distance to which the flashlight's influence extends)
String beta_fold = "Beta"; // Beginning of the folder for values of beta (The flashlight width)
String alpha_fold = "Alpha";

Vec2 box_pos = new Vec2();
String in = "initial_positions";
String extention = ".txt";
boolean blueBallSelected = false;
 BufferedReader reader;
int fileCounter = 0;

float freq;
float freqStep = 20.0;
float maxFreq = 20.0;
float initialFreq = 20.0;

int amplitude;
int ampStep = 1;
int maxAmp = 12;
int initialAmp = 12;


float lightAngle;
float initialAngle = 80;
float angleStep = 10;
float maxAngle = 90;

float lightWidth;  // the angle of the flashlight
float lightWidth_initial = 40;
float lightWidth_max = 180;
float lightWidth_step = 20;

float R_initial = big + mid*2*3; // The distance of influence is expressed in module lengths (ML).  The initial value is 2.5 ML. 
//Includes the radius of a mid module holdng the flashlight and 1 additional module lengths. (mid = radius. mid*2= diameter of one module)  
float R_max = big + mid*2*4; // The final distance of influence is 4 module lengths.
float R_step = mid*2; // The length will increase by 2 module lengths per simulation
float R;

int in_counter; // The counter for the input file (will use initial_positionsX.txt, X=in_counter)
int in_counter_initial = 2;
int in_counter_max = 10;

boolean lightOn = true; // This is true if you want the light to shine and false if not

boolean endProgram = false; //Processing does not end the program when exit is called. If exit is called, the program goes to the end of the draw function first, and then exits. So will make it here so that the draw function will only execute if endProgram is false 

void setup()
{
  size(2000, 1000);
  smooth();

  box2d = new Box2DProcessing(this, 100);
  box2d.createWorld();
  mean_box_height = 10*height/11;

  
  box2d.setGravity(0, -9.8);

  robots = new ArrayList<Robot>();
  path = new ArrayList<Vec2>();
  center_velo = new Vec2();
  center_pos = new Vec2();
 path = new ArrayList<Vec2>();
  
  ground = new Ground();


 
  amplitude = initialAmp - ampStep;
  freq = initialFreq;
  lightAngle = initialAngle;
  lightWidth = lightWidth_initial;
  R = R_initial;
  in_counter = in_counter_initial;
  
  output = createWriter("data_R_alpha_beta" + "/" + "README" + extention);
  
  output.println("frequency = " + str(initialFreq) + " amp = " + str(initialAmp));
  output.println("big diameter = " + str(int(big)) + "\n small diameter = " + str(int(small)));
  output.println("number of robots = " + str(no_of_robots));
  output.println("box_bottom = " + str(int(box_bottom)));
  output.println("R range = " + str(int(R_initial)) + " to " + str(int(R_max)) + " with step " + str(int(R_step)));
  output.println("Beta (Angle width of flashlight) range = " + str(int(lightWidth_initial)) + " to " + str(int(lightWidth_max)) + " with step " + str(int(lightWidth_step)));
  output.println("Alpha (angle with negative vertical axis) range = " + str(int(initialAngle)) + " to " + str(int(maxAngle)) + " with step " + str(int(angleStep)));
  output.flush();
  output.close();
  
}

void draw() {
  background(255);

if(steps == 0)
{
  box = new Box(width/2, height - 50);
  setSimulationConditions();
 
  if(!endProgram)
  {
      createRobots(in + str(int(in_counter)) + extention);
      
     // Creating the box

     /// End creating the box
    
      //End Setting the highest robot as the blue one
      
      //////// Setting delay to zero
      delay = 0;
     ///////// End setting delay to zero
     
     //Start setting new file to write to


 }
}
// End simulation initialization

  // We must always step through time!
  if(!endProgram)
  {
          box2d.step(1/200.0, 10, 10);
          steps++;
          
          // Display all the boundaries
          box.display();
          ground.display();
        
          for (Robot p : robots) {
            p.display();
           
          }
       if(lightOn) 
       {
        for(Robot p : robots)
        {
            if(p.checkColour() == 'b')
            {
         
              light.shineAngle(p.checkPos(), 90 - lightAngle);
              light.display(); // The second parameter of this function is the point where you want to go on the screen. 
            }
        }
       }
        
          for (int i = robots.size()-1; i >= 0; i--) {
            Robot p = robots.get(i);
            if (p.done()) {
              robots.remove(i);
            }
          }
        
         delay++;
      if(delay > DELAY)
        {
            
            box_pause = false;
           
            
            for (int i = robots.size()-1; i >= 0; i--) {
              Robot p = robots.get(i);
              
              if (p.checkColour() != 'b')  // If the robot is a surrounding module
              {
                 if(light.isIlluminated(p.checkPos()) == 'y')
                 {
                   p.decreasing = true;
                   p.colour = 'r';
                 }
                 
                 else
                 {
                  p.increasing = true; 
                  p.colour = 'g';
                 }
              } 
              
              else // For the big robot
              {
               p.increasing = true;
       
              }
            
        
            }
            for (int i = robots.size()-1; i >= 0; i--) 
            {
              Robot p = robots.get(i);
              if (p.increasing) // If the robot is small and close to the bottom of the box
              {
                if(p.checkColour() != 'b')
                  p.changeRadius(mid);
                  
                  else
                  p.changeRadius(big);
              } 
              
              else if(p.decreasing)
              {
                if(p.checkColour() != 'b')
                p.changeRadius(small);
                
                else
                p.changeRadius(mid);
              }
            }

          if(!box_pause)
          {
            box.applyVelocity(calcVelocity(box.checkPos()));
          }
          else if (box_pause)
          {
            vel.x = 0.;
            vel.y = 0.;
            box.applyVelocity(vel);
          }
          
          /*
          Now to get the velocity and position of the center of mass and subtract it from 
          the velocity of the individual ball
          */
           record++;
           if(record % 5 == 0)
           {
             for(Robot r : robots)
             {
                if(r.colour == 'b') 
                {
                  Vec2 pos = r.checkPos();
                  
                    output.println(str(pos.x) + "," + str(pos.y));
                }
             }
           }
            
           
           
        }


 //////////// End sim if the record count exceeds the number of points we want (max_record)
 //////////// End sim if the blue robot emerges at the top of the swarm
 float max_height = 0;
 float blue_height = 0;
 boolean done = false;
      for(Robot r : robots)
      {
          if(height - r.checkPos().y > max_height)
          {
           max_height = height - r.checkPos().y; 
          }
        
          if(r.colour == 'b')
          {
             blue_height = height - r.checkPos().y; 
          }
      }
      
      if(blue_height > max_height * 0.9)
        done = true;
 
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
    
      if(record % max_record == 0 && delay > DELAY || done)
      {
             steps = 0;
             record = 0;
             output.flush();
             output.close();
             
         for (int i = robots.size()-1; i >= 0; i--)
          {
            Robot p = robots.get(i);
            p.killBody();
            robots.remove(i);
        
          }
          
          box.killBody();
      }
   displayPath('b');   
  } ////////// End of checking if the program is supposed to end (if endProgram is false)
}///////////// End of draw
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
    strokeWeight(5);
    point(p.x, p.y); 
    popMatrix(); 
  }
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

      if(nums[2] == big)
      {
         Robot p = new Robot(nums[0], nums[1], big, 'b', true);
         robots.add(p);
         light = new LightHouse(p.checkPos(), lightWidth, R, 'y');
      }
      
      else if(nums[2] == mid)
      {
          Robot p = new Robot(nums[0], nums[1], mid, 'g', false);
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

  maxVelo = 2*PI*freq*amplitude/100.0;
  constVelo = 0.1*maxVelo;
  if (box_pos.y > mean_box_height + amplitude/2)
  {
    new_velocity.y = constVelo;
    velocityDirection = 1; // The box will move upward from now on
  } else if (box_pos.y < mean_box_height - amplitude/2)
  {
    new_velocity.y = -constVelo; 
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
  }
  
  else
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
         in_counter+=1;
         if(in_counter > in_counter_max)
          {
               in_counter = in_counter_initial;
               lightAngle = angleStep + lightAngle;

               if(lightAngle > maxAngle)
               {
                 lightAngle = initialAngle;
                 lightWidth += lightWidth_step;
                  if(lightWidth > lightWidth_max)
                   {
                     lightWidth = lightWidth_initial;
                     R += R_step;
                       if(R > R_max)
                       {
                         R = R_initial;
                         
                         println("This is the end of the program");
                         endProgram = true;
                         exit();
                       }
                  }
               }
         }  
       }
       
     }
   if(!endProgram)
   {
     output = createWriter("data_R_alpha_beta/" + R_fold + str(int(R))  + "/" + beta_fold + str(int(lightWidth)) + "/" + "alpha_fold"  + str(int(lightAngle)) + "/" + data_fold[in_counter-1] + "/" +  "Flash_" + str(R)  +"_" + str(int(lightWidth))+ "_"+ str(int(lightAngle)));
   }
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
