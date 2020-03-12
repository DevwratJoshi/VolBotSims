// VolBot simulator using the jave-processing ported version of box2D by shiffman
// In this simulation, the box shakes side to side. This is to give us a wider range to take videos with
import shiffman.box2d.*;
import org.jbox2d.collision.shapes.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.joints.*;

Box2DProcessing box2d;

Box box;
final float stepsPerSec = 100.0;
final float big_diameter = 1.73; // The diameter ratio of actual robot_large to actual robot_small
final float segregator_frac = 0.1; // fraction of modules that are segregators
final int maxSteps = 5000;
float small = 20;
float big = small*big_diameter;
float segregator_size = big; // Fixed segregator size
final int amplitude = int(small*8); //Fixed amplitude
final float freq = 0.3; //Fixed frequency
final float zoneWidth = 5.0;
//Worth noting here that dataCollectionRate seconds between sims and readings_per_sim readings means dataCollectionRate*readings_per_sim seconds for a group of conditions 

int steps = 0; // keeps track of the number of times the world has stepped

final int DELAY = 100;
// A list for all of our robots
ArrayList<Robot> robots;
ArrayList<Vec2> path;
//Ground ground;
int no_of_robots = 120;
int large_robots = 0;
int small_robots = 0; 
float packing_fraction;
int count = 0;
char flag = 'n';
float velConst = 1;
Vec2 vel = new Vec2();
boolean box_pause = true;

float mover_small_frac = 0.1; // This is the percentage of the mover robots that are small. Used to control the total packing fraction
float mover_frac_step = 0.1;
float mover_frac_initial = 0.9;
float mover_frac_final = 1.0;


float density_small = 13.15; // Actual density of the small robot
float density_large = 4.38; // Actual density of the large robot
float density_mid = 1.0;

float fric_low = 0.1;
float fric_high = 0.8;



int delay = 0;
int record = 0;


float box_bottom = small*2*40.0;
float box_height = small*2*10.0;
float box_edge_width = 40;
float mean_box_height;
Vec2 center_pos, center_velo;
int velocityDirection = 1;

Vec2 mouse1, mouse2;
Vec2 temp_mouse = new Vec2();
boolean mouseActive;

float robot_initials[];
int r_i_counter;

StringList LinePositionNumbers; // The array to hold the names of the files for different simulations
StringList LinePositionGroup;
 
int currentUpPosition;
int currentDownPosition;
Vec2 COM = new Vec2();



String sep = "seperation_18"; // The seperation folder
String in_folder = "initial_positions/" + sep + "/";
String in = "initial_positions";
int in_counter = 1;
int initial_in_counter = 1;
int in_counter_step = 1;
int in_counter_final = 10;
String data_folder = "data/" + sep + "/";
PrintWriter output_data, output; // The output file for initial positions, final positions

String extention; // The extention for the files

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

boolean to_collect_data = false;

void setup()
{
  size(2000, 1000);
  smooth();

  box2d = new Box2DProcessing(this, 100); /// This is the ratio of pixels to m.

  box2d.createWorld();
  box2d.setGravity(0, 0.0);
  mean_box_height = height/2 + box_height/2 + box_edge_width/2;

  robots = new ArrayList<Robot>();
  path = new ArrayList<Vec2>();
  center_velo = new Vec2();
  center_pos = new Vec2();
  robot_initials = new float[4];
  r_i_counter = 0;
  //ground = new Ground();
   
  in_counter = initial_in_counter - in_counter_step; // Initial setSimulationConditions will add in_counter_step to in_counter
  extention = ".txt";
  mover_small_frac = mover_frac_initial;
  mouse1 = new Vec2(0,0);
  mouse2 = new Vec2(0,0);
   
  output = createWriter(data_folder + "README");
  output.println("frequency = " + str(freq) + " amp = " + str(amplitude));
  output.println("big diameter = " + str(int(big)) + "\n small diameter = " + str(int(small)));
  output.println("number of robots = " + str(no_of_robots));
  output.println("box_bottom = " + str(int(box_bottom)));
  output.println("friction_low = " + str(fric_low) + " friction_high = " + str(fric_high));
  output.flush();
  output.close();
  
 

}

void draw() {   
  if (steps == 0)
  {
    setSimulationConditions();
    if(!exitFlag)
    {
      // Creating the box
      box = new Box(width/2-amplitude, mean_box_height, 's');
      /// End creating the box
  
      //////// Creating new robots using data from initial_positions.txt
      createRobots(in_folder + in + str(in_counter) +  extention);
      /////// End of creating robots
      output_data.println(segregatorPos(false).x + "," + segregatorPos(false).y + "," + robotCOM().x + "," + robotCOM().y + "," + record);
      //////// Setting delay to zero
      delay = 0;
      ///////// End setting delay to zero
           ///////////// Robots must all be the correct size initially. Check the initial lambda before shaking begins
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
      background(255);
      box2d.step(1/stepsPerSec, 10, 10);

      steps++;
      //box_pause = false;
      // Display all the boundaries
      box.display();
      //ground.display();
  
      displayRobots();
  
    if(delay == DELAY)
      {
        Vec2 bpos = new Vec2();
        bpos = box.checkPos();
        box.killBody();
        box = new Box(bpos.x, bpos.y, 'd');
        box_pause = false;
        
      }
  
      delay++;
      if (delay > DELAY)
      {
        record++;
        display_sim_conditions();
  //////////////////////////// Shake box block  
  
        if (!box_pause)
        {
          box.applyVelocity(calcVelocity(box.checkPos()));
        } 
        
        else if (box_pause)
        {
          vel.x = 0.;
          vel.y = 0.;
          box.applyVelocity(vel);
        }
  //////////////////////////// End shake box block
  
///////////////////////// Change robot size block
      for(int i = robots.size()-1; i >=0; i--)
      {
       Robot r = robots.get(i);
       if(robotInZone(r))
       {
         if(r.isSeed)
         r.changeRadius(big);
         r.changeFriction(fric_high);
         
         if(r.type == 'm')
         r.colour = 'y';
       }
       
       else
       {
        if(r.type == 'm')
        {
         r.changeRadius(small);
         r.changeFriction(fric_low);
         r.colour = 'g';
        }
        
       }
       
      }
//////////////////////////// End Change robot size block
  
        Vec2 box_vel = box.checkLinearVelocity();
       for (int i = robots.size()-1; i >= 0; i--) 
       {
        Robot p = robots.get(i);
        //println(p.r);
        p.applyFrictionForce(box_vel);
        
      }
    
      }////////// End of delay
    ///////////Start of zone calc
     setZones();
    displayZones();
    
    //////////End of zone calc
    
    
    // This is to delete any robots that may have jumped over the wall and out of range. They must be deleted to let the robots_moving() function return false 
    for (int i = robots.size()-1; i >= 0; i--) {
        Robot p = robots.get(i);
        if (p.done(box.checkPos().y - box.w/2)) {
          robots.remove(i);
        }
      }
      
        if(record > maxSteps - (stepsPerSec/freq) && record < maxSteps)
        {
          output_data.println(segregatorPos(false).x + "," + segregatorPos(false).y + "," + robotCOM().x + "," + robotCOM().y + "," + record);
          output_data.flush();
      }
      if(record >= maxSteps || segregatorInZone())
      {
        output_data.println(segregatorPos(false).x + "," + segregatorPos(false).y + "," + robotCOM().x + "," + robotCOM().y + "," + record);
        output_data.flush();
        output_data.close();
       for (int i = robots.size()-1; i >= 0; i--)
       {
        Robot p = robots.get(i);
        p.killBody();
        robots.remove(i);
       }
       
        steps = 0;
       record = 0;
       box.killBody();
      }
      
      packing_fraction = 0.;
      for (Robot r:robots)
      {
       packing_fraction += PI*r.r*r.r; 
      }
      packing_fraction = packing_fraction/(box_bottom*box_height);

  }
  
  else // If exitFLag is true
  {
   exit(); 
  }

pushMatrix();
noStroke();
fill(100, 0, 0);
ellipse(width/2, height, amplitude*2, amplitude*2);
Vec2 meh = new Vec2();
meh = box.checkPos();
stroke(0, 0, 100);
line(meh.x, meh.y, meh.x, height);
popMatrix();

}


/////////////////////////////////////////////// End of draw
boolean robotInZone(Robot r)
{
  Vec2 pos = r.checkPos();
  if(pos.x > mouse1.x && pos.x < mouse2.x)
    {
      
     if(r.type == 'm')
      {
       for(Robot p: robots)
       {
        if(p.type == 's')
        {
         if(vec_diff(p.checkPos(), r.checkPos()) <= (r.r+p.r))
         {
 
           return false;
         }
        }
       }
      }
  
      return true; 
    }
    
   else 
   return false;
}

void setZones() // zoneWidth is the zone width. zoneNumber is 1 for the center zone and zoneNumber_max for the zone at the edge of the box 
{
  //mouse1 corresponds to the left zone line's intercept with the bottom of the box, and mouse2 corresponds to the right line's intercept with the bottom of the box
  // The zone lines here are vertical, so 2 lines can be drawn with these points alone
  Vec2 boxp = box.checkPos();
  
  float zoneWidth_pixels = zoneWidth*small*2;
  println(zoneWidth_pixels);
  float zone_MidPoint = boxp.x; //x cordinate of the mid point of this zone
   println(zone_MidPoint);
  mouse1.x = zone_MidPoint - zoneWidth_pixels/2;
  mouse2.x = zone_MidPoint + zoneWidth_pixels/2;
  mouse1.y = boxp.y - box.w/2;
  mouse2.y = boxp.y - box.w/2;
}

void displayZones()
{
  float zoneWidth_pixels = zoneWidth*2*small;
 pushMatrix();
 fill(200, 200, 0, 180);
 noStroke();
 rectMode(CORNER);
  rect(mouse1.x, mouse1.y - box_height, zoneWidth_pixels, box_height);
 popMatrix();
}


boolean segregatorInZone()
{
 for(int i = robots.size()-1; i >=0; i--)
 {
   Robot p = robots.get(i);
   if(p.type == 's')
   {
     if(robotInZone(p))
     return true;
   }
 }
 
 return false;
}

void display_sim_conditions()
{
  pushMatrix();
  fill(0, 102, 153);
  textSize(32);
  text("Frequency = " + str(freq) + "    Friction_red = " + str(fric_low), 120, 30); 
  text("Amplitude = " + str(amplitude) + "     Friction_green = " + str(fric_low), 120, 60);
  float robot_area = 0.;

  text("Packing fraction = " + packing_fraction, 120, 100);
  textSize(20);
  fill(0);
  text("Press 's' to exit the program and save data", 100, height - 30);
  popMatrix();
  
}
float vec_diff(Vec2 a, Vec2 b) //A helper function to calculate the euclidean distance between a and b
{
  float diff = 0.;
  
  diff = sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
  
  return diff;
}

void write_to_file(PrintWriter f)
{  
  Vec2 b_pos = box.checkPos();
  for(Robot r : robots)
  {
    Vec2 pos = r.checkPos();
    f.println(robot_initials[0] + "," + robot_initials[1] + "," + robot_initials[2] + "," + robot_initials[3] + "," + (pos.x - b_pos.x + box_bottom/2)/(small*2) + "," + (b_pos.y - pos.y)/(small*2) + "," + r.type + "," + r.friction_with_bed + "," + r.r + "," + packing_fraction); // Storing the radius and the friction of the robot
  }
  f.flush();
  f.close();

}

void mousePressed()
{
 for (Robot r : robots)
 {

  if(vec_diff(new Vec2(mouseX, mouseY), r.checkPos()) <= r.r)
  {
    if(r.type == 's')
    {
     r.colour = 'b'; 
     r.isSeed = true; // change this to a seed
     r.changeFriction(1.0);
    }

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

void keyPressed()
{
 if(key == 's')
 {
    output.flush();
    output.close();
    exit();
 }
 
 if(key == 'd')
 {
    //box2d.setGravity(0.0, -5);
    box_pause = !box_pause;
 }
}

Vec2 segregatorPos(boolean seed) //true if we want seed position, false if we want other seg position
{
  Vec2 loc = new Vec2(0,0);
  Vec2 boxp = box.checkPos();
 for(Robot r : robots)
 {
  if(r.isSeed == seed && r.type =='s')
  {
    loc = r.checkPos();
    loc.x = (loc.x - (boxp.x - box_bottom/2))/(small*2); 
    loc.y = (boxp.y - r.checkPos().y - box_edge_width/2)/(small*2);
    return loc;
  }
 }
 return null;
 }
void createRobots(String input)
{
  reader = createReader(input);
  String line = null;
  large_robots = 0;
  small_robots = 0;
  int segregators = 0;
  int movers = 0;
 float r = 0.;
 char col;

 char t;
  try {
   while ((line = reader.readLine()) != null) 
    {
      //println(line);
      int[] nums = int(split(line, " "));
      char colour = 'g';
         float d;
          float f;
         
         if(nums[3] == 1)
        {
          d = density_small;
          col = 'r';
          f = fric_high;
          t = 's';
         
          
          //output_data.flush();
        }  
        else
        {
          d = density_small;
          col = 'g';
          f= fric_low;
          t = 'm';
        }
        if(nums[2] == 1) // 1  for big, 2 for small
        {
          r = big; 
        }
        else
          r = small;
          
          Robot p = new Robot(box.checkPos().x - box_bottom/2 + nums[0], box.checkPos().y - nums[1], r, col, true, d, f, t); // Segregator
            
          if(p.type=='s'  && (box.checkPos().x - box_bottom/2 + nums[0])/(small*2) == 21.0)
          {
            //p.isSeed = true;
            //p.colour = 'b';
            p.type = 'm';
            p.friction_with_bed = fric_low;
            p.colour = 'g';
          }
          robots.add(p);
    }
    reader.close();
  } 
  catch (IOException e) 
  {
    e.printStackTrace();
  }
}

Vec2 robotCOM() // returns the position of the COM of the swarm 
{
 Vec2 loc = new Vec2(0,0);
 Vec2 boxp = box.checkPos();
 for(Robot r:robots)
 {
     loc.x += (r.checkPos().x - (boxp.x - box_bottom/2))/(small*2); 
     loc.y += (boxp.y - r.checkPos().y - box_edge_width/2)/(small*2);
 }
 
 loc.x = loc.x/robots.size();
 loc.y = loc.y/robots.size();
  return loc;
}

Vec2 calcVelocity(Vec2 box_pos)
{
  Vec2 new_velocity = new Vec2();
  float constVelo = 0.1;
  float maxVelo;
  int section = 0;
  float mean_box_xpos = width/2;
  maxVelo = 2*PI*freq*amplitude/100.0;
  //println("maxvelo = " + maxVelo);
  constVelo = 0.1*maxVelo;
  if(constVelo < 0.1)
  constVelo = 0.1;
  if (box_pos.x >= width/2 + amplitude)
  {
    new_velocity.x = -constVelo;
    velocityDirection = -1; // The box will move upward from now on
    section = 1;
  } 
  else if (box_pos.x <= width/2 - amplitude)
  {
    new_velocity.x = constVelo; 
    velocityDirection = 1;
    section = 2;
  }

  else
  {
    new_velocity.x = maxVelo*sin(map(box_pos.x, mean_box_xpos - amplitude, mean_box_xpos + amplitude, radians(6), PI-radians(6)));
    if (velocityDirection == -1)
    {
      new_velocity.x = -1*new_velocity.x;
    }
    section = 5;
  }

  //println(freq);
   
  return new_velocity;
}

void setSimulationConditions()
{
   in_counter += 1;
   r_i_counter = 0;
   if(in_counter > in_counter_final)
     {
       output_data.flush();
       output_data.close();
       exitFlag = true;
     }  

    if(!exitFlag)
    {
       output_data = createWriter(data_folder + "data" + str(in_counter) + extention); 
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
