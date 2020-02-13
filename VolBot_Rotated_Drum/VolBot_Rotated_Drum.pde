import shiffman.box2d.*;
import org.jbox2d.collision.shapes.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.joints.*;

float stepsPerSec = 200.0;
int steps = 0;
Box2DProcessing box2d;
Drum drum;
String in = "initial_positions";
int in_counter = 3;
String extention = ".txt";
BufferedReader reader;
int large_robots = 0;
int small_robots = 0;
ArrayList<Robot> robots;

float density_low = 0.5;
float density_high = 4.0;
float density_mid = 1.0;

float large_robot_density = density_high;
float big = 30;
float small = 20;
void setup()
{
  size(1000, 1000);
  box2d = new Box2DProcessing(this, 100);
  box2d.createWorld();
  box2d.setGravity(0, -9.8);
  drum = new Drum(new Vec2(width/2, height/2), 450, 30, 144);
  robots = new ArrayList<Robot>();
  
  createRobots(in + str(in_counter)+ extention);
   
}

void draw()
{
  background(255);
  box2d.step(1/stepsPerSec, 10, 10);
  steps++;
  drum.applyRPM(11.6);
  drum.display();
  displayRobots();
}

void displayRobots()
{
  for (Robot p : robots)
  {
    p.display();
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

      if(nums[2] == big)
      {
         Robot p = new Robot(nums[0], nums[1], big, 'b', true, density_mid);
         robots.add(p);
         large_robots += 1;
      }
      
      else if(nums[2] == small)
      {
          Robot p = new Robot(nums[0], nums[1], small, 'g', false, density_mid);
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
