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
final float freq = 0.6; //Fixed frequency

//Worth noting here that dataCollectionRate seconds between sims and readings_per_sim readings means dataCollectionRate*readings_per_sim seconds for a group of conditions 

int steps = 0; // keeps track of the number of times the world has stepped

final int DELAY = 100;
// A list for all of our robots
ArrayList<Robot> robots;
ArrayList<Vec2> path;
//Ground ground;
int no_of_robots = 180;
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

float fric_low = 0.2;
float fric_high = 0.6;

int delay = 0;
int record = 0;


float box_bottom = small*2*30.0;
float box_height = small*2*10.0;
float box_edge_width = 40;
float mean_box_height;
Vec2 center_pos, center_velo;
int velocityDirection = 1;

Vec2 mouse1, mouse2;
Vec2 temp_mouse = new Vec2();
boolean mouseActive;

StringList LinePositionNumbers; // The array to hold the names of the files for different simulations
StringList LinePositionGroup;
 
int currentUpPosition;
int currentDownPosition;
Vec2 COM = new Vec2();


String in_folder = "initial_positions_8/mover_frac_0.9/";
String in = "initial_positions";
int in_counter = 1;
int initial_in_counter = 1;
int in_counter_step = 1;
int in_counter_final = 5;
String data_folder = "test/";
PrintWriter output_init, output_final, output; // The output file for initial positions, final positions

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

  //ground = new Ground();
  vel.x = 0.0;
  vel.y = velConst;
  in_counter = initial_in_counter - in_counter_step; // Initial setSimulationConditions will add in_counter_step to in_counter
  extention = ".txt";
  mover_small_frac = mover_frac_initial;

   
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
      createRobots(in_folder + in + str(in_counter)+  extention);
      /////// End of creating robots
  
      //////// Setting delay to zero
      delay = 0;
      ///////// End setting delay to zero
      
      write_to_file(output_init);       ///////////// Robots must all be the correct size initially. Check the initial lambda before shaking begins
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
  

  
        Vec2 box_vel = box.checkLinearVelocity();
       for (int i = robots.size()-1; i >= 0; i--) 
       {
        Robot p = robots.get(i);
        //println(p.r);
        p.applyFrictionForce(box_vel);
        
      }
    
      }
    
    // This is to delete any robots that may have jumped over the wall and out of range. They must be deleted to let the robots_moving() function return false 
    for (int i = robots.size()-1; i >= 0; i--) {
        Robot p = robots.get(i);
        if (p.done(box.checkPos().y - box.w/2)) {
          robots.remove(i);
        }
      }
      
      if(steps > maxSteps)
      {
       steps = 0;
       write_to_file(output_final);
       box.killBody();
       for (int i = robots.size()-1; i >= 0; i--)
       {
        Robot p = robots.get(i);
        p.killBody();
        robots.remove(i);
       }
      }
        
      packing_fraction = 0.;
      for(Robot r :robots)
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


void write_to_file(PrintWriter f)
{  
  Vec2 b_pos = box.checkPos();
  for(Robot r : robots)
  {
    Vec2 pos = r.checkPos();
    f.println((pos.x - b_pos.x + box_bottom/2)/(small*2) + "," + (b_pos.y - pos.y)/(small*2) + "," + r.type + "," + r.friction_with_bed + "," + r.r + "," + packing_fraction); // Storing the radius and the friction of the robot
  }
  f.flush();
  f.close();

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
 float f;
 char t;
  try {
   while ((line = reader.readLine()) != null) 
    {
      //println(line);
      int[] nums = int(split(line, " "));
      char colour = 'g';
         float d;
 
         if(nums[3] == 1)
        {
          d = density_large;
          col = 'r';
          f = fric_high;
          t = 's';
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
          
          Robot p = new Robot(nums[0], nums[1], r, col, true, d, f, t); // Segregator
          robots.add(p);	
    }
    reader.close();
  } 
  catch (IOException e) 
  {
    e.printStackTrace();
  }
  
  
}

void correct_robot_distribution(String quantity, int current_number, int actual_number, char type)
{
  // Quantity is either 'size', 'friction' or 'type'. This is the characteristic to check. type determines whether the robot is a segregator or a mover
  // If there are too many robots of a certain 'quantity' or some and not enough of the other, will take care of that here
  // current_number is the current number of robots we have  
  // actual number is the actual number we want
  // #####TODO######### type is the group of robots among which we are doing the correction. 's' for segregators and 'm' for movers
  
  String to_check = quantity;
  int new_number = current_number; // The number of robots with quantity 'small'. Either small radius or small friction

  int change_dir = 1;
  char col = 'r';
  float size;
  float friction;
  float density;
  if(quantity == "size")
  {     
     float change_from;
     float change_to;
    if(current_number > actual_number)
     {
       change_from = small;
       change_to = big;
       change_dir = -1;
       col = 'r';
       density = density_large;
       
     }
     
     else if(current_number < actual_number)
     {
       change_from = big;
       change_to = small;
       change_dir = 1;
       col = 'g';
       density = density_small;
     }
     
     else
     return;                                                                                
     while(new_number != actual_number)
     {
        for(int i = robots.size()-1; i>=0; i--)
        {
           Robot r = robots.get(i);
          if(r.r == change_from && random(1) < 0.1 && r.type == type)
          {
           Robot n_r = new Robot(r.checkPos().x, r.checkPos().y, change_to, r.colour, !r.bigProb, density, r.friction_with_bed, r.type);
           
            robots.remove(i);
            r.killBody();
            robots.add(n_r);
           new_number += change_dir;
           
          }
          
          if(new_number == actual_number)
            break;
        }
     }
     
  }
  
  if(quantity == "friction")
  {
    
      float change_from;
      float change_to;
   if(current_number > actual_number)
     {
       change_from = fric_low;
       change_to = fric_high;
       change_dir = -1;
     }
     
     else if(current_number < actual_number)
     {
       change_from = fric_high;
       change_to = fric_low;
       change_dir = 1;
     }
     
     else
     return;
     
     while(new_number != actual_number)
     {
        for(int i = robots.size()-1; i>=0; i--)
        {
          Robot r = robots.get(i);
          if(r.friction == change_from && random(1) < 0.1 && r.type == type)
          {
           Robot n_r = new Robot(r.checkPos().x, r.checkPos().y, r.r, r.colour, r.bigProb, r.density, change_to, r.type);
           
            robots.remove(i);
            r.killBody();
            robots.add(n_r);
           new_number += change_dir;
           
          }
          
          if(new_number == actual_number)
            break;
        }
     }
  }
  
  if(quantity == "type")
  {  
   char change_from;
   char change_to;
    if(current_number > actual_number)
     {
       change_from = 's';
       change_to = 'm';
       change_dir = -1;
       col = 'g';
       size = small; // small by default, but will change this when correcting mover size ratio
       friction = fric_low;
     }
     
     else if(current_number < actual_number)
     {
       change_from = 'm';
       change_to = 's';
       change_dir = 1;
       col = 'r';
       size = segregator_size;
       friction = fric_high;
     }
     
     else
     return;                                                                                
     while(new_number != actual_number)
     {
        for(int i = robots.size()-1; i>=0; i--)
        {
           Robot r = robots.get(i);
          if(r.type == change_from && random(1) < 0.1)
          {
           Robot n_r = new Robot(r.checkPos().x, r.checkPos().y, size, col, r.bigProb, r.density, friction, change_to);
           
            robots.remove(i);
            r.killBody();
            robots.add(n_r);
           new_number += change_dir;
           
          }
          
          if(new_number == actual_number)
            break;
        }
     }
     
  }
  

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
   float temp_packing = 10.0; //Check if the packing fraction is feasible. Initially set to infeasible value
       while(true) // Keep increasing if the packing fraction is not possible
       {
         temp_packing = PI*no_of_robots*(segregator_size*segregator_size *segregator_frac + (1.0-segregator_frac)*(small*small*mover_small_frac + big*big*(1.0-mover_small_frac)))/(box_bottom*box_height);
         if(temp_packing < 0.85)
           break;
         else
           mover_small_frac += mover_frac_step;
           
          if(mover_small_frac > mover_frac_final+mover_frac_step)
          break;
          
           //println(temp_packing);
       }
   if(in_counter > in_counter_final)
     {
       in_counter = initial_in_counter;
       mover_small_frac += mover_frac_step;
       
       if(mover_small_frac > mover_frac_final)
         exitFlag = true;
             
     }  
     else
     {


      output_init = createWriter(data_folder + "/Segregator_Size" + str(int(segregator_size)) + "/mover_small_frac/" + nf(mover_small_frac, 0, 1)+ "/Initial/" + str(in_counter) + extention);
      output_final = createWriter(data_folder + "/Segregator_Size" + str(int(segregator_size)) + "/mover_small_frac/" + nf(mover_small_frac, 0, 1)+ "/Final/" + str(in_counter) + extention);
     
       
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
