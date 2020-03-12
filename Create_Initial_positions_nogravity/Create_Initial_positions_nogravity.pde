// VolBot simulator using the jave-processing ported version of box2D by shiffman

import shiffman.box2d.*;
import org.jbox2d.collision.shapes.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.joints.*;

Box2DProcessing box2d;

Box box;
int steps = 0; // keeps track of the number of times the world has stepped
final int DELAY = 300;
// A list for all of our rectangles
ArrayList<Robot> robots;
Ground ground;
int no_of_robots = 150;
int blueRobots = 150;
int greenRobots = 0;
int count = 0;
char flag = 'n';
float velConst = 1;
Vec2 vel = new Vec2();
boolean box_pause = true;
float big = 30;
float mid = 25;
int small = 20;
int freq = 60;
int delay = 0;
int record = 0;
PrintWriter output;
char b = 'n';
float box_bottom = small*2*15.0;
float box_height = small*2*15.0;
float box_edge_width = 40;
int mean_box_height;
Vec2 center_pos, center_velo;
int velocityDirection;
int amplitude = int(small*4);
Vec2 mouse1, mouse2;
Vec2 temp_mouse = new Vec2();
boolean mouseActive;
int max_record = 10000;
String blah;
void setup()
{
  size(1000, 1000);
  smooth();
 
  box2d = new Box2DProcessing(this, 100);
  box2d.createWorld();
  mean_box_height = 11*height/12;

  box2d.setGravity(0, 0);
box = new Box(width/2, height/2 + box_height/2 + box_edge_width/2, 'k');
  robots = new ArrayList<Robot>();
  
  center_velo = new Vec2();
  center_pos = new Vec2();

  int box_width = 2*width/3;
  int box_height = height*2;
  
  
  amplitude = (int)(small*4);
  ground = new Ground();
  vel.x = 0.0;
  vel.y = velConst;
  blah = "initial_positionsBlue" + str(blueRobots) + "Green" + str(greenRobots) +"(1)"+ ".txt";
    output = createWriter(blah);
}
void draw() {
  background(255);
/*
  if (count < no_of_robots && random(1) < 0.4)
  {
   float ran = random(1);
    if (ran > 0.8 && greenRobots > 0)
    {
      Robot p = new Robot(width/2 + random(-5, 5), 30, big, 'g');
      greenRobots--;
      
      robots.add(p);
      count++;
    } 
    else if(ran < 0.9 && blueRobots > 0)
    {
      Robot p = new Robot(width/2 + random(-5, 5), 30, small, 'b');
      blueRobots--;
      robots.add(p);
      count++;
    } 
    */
    
    /////////////////// A single large robot in the middle of the stack
    
  if (count < no_of_robots && random(1) < 0.8)
  {
   float ran = random(1);
    
    Vec2 new_pos = new Vec2();
    
    if (ran < 0.8 && greenRobots > 0)
    {
      
    while(true)
    {
      new_pos.x = random(0, width);
      new_pos.y = random(0, height);
      
     if(checkPosAvailable(new_pos, big))
     break;
    }
    
    Robot p = new Robot(new_pos.x, new_pos.y, big, 'g');
      greenRobots--;
      robots.add(p);
      count++;
    } 
    else 
    {
      
      while(true)
    {
      new_pos.x = random(0, width);
      new_pos.y = random(0, height);
      
     if(checkPosAvailable(new_pos, small))
     break;
    }
      Robot p = new Robot(new_pos.x, new_pos.y, small, 'b');
      blueRobots--;
      robots.add(p);
      count++;
    }
    
  for(Robot r: robots)
  {
   r.applyVelocity(2); 
  }
   }


    
  

  // We must always step through time!
  box2d.step();
  steps++;

  // Display all the boundaries
  box.display();
  ground.display();

  for (Robot p : robots) {
    p.display();
  }

  for (int i = robots.size()-1; i >= 0; i--) {
    Robot p = robots.get(i);
    if (p.done()) {
      robots.remove(i);
    }
  }

if(count >= no_of_robots)
{
  pushMatrix();
  fill(0, 102, 153);
  textSize(32);
  text("Done", width/2, 90); 
  popMatrix();
  
 for(Robot r: robots)
  {
   r.applyVelocity(0.0); 
  }
}

if(mousePressed && count >= no_of_robots)
{
  println(greenRobots + " " + blueRobots + " " + count);
  for (int i = robots.size()-1; i >= 0; i--) {
    Robot p = robots.get(i);
    int colour = 0;
    switch(p.checkColour())
    {
     case 'r':
     {
     colour = 1;
     break;
     }
     
     case 'g':
     {
     colour = 2;
     break;
     }
     
     case 'b':
     {
       colour = 3;
       break;
     }
     
     default:
     {
      colour = 1;
      break;
     }
    }
    //output.println((int)p.checkPos().x + " " + (int)p.checkPos().y + " " + colour);
    output.println((int)p.checkPos().x + " " + (int)p.checkPos().y + " " + (int)p.r);
  }
  output.flush();
  output.close();
  println("File created" + " " + blah);
  exit();
}

}

/*
This function will check if placing a robot of radius rad at position indicated by pos will overlap with any robots already created 
*/
boolean checkPosAvailable(Vec2 pos, float rad)
{
  if(!robot_in_box(pos, rad))
  return false;
  
  for(Robot r : robots)
  {
   Vec2 r_pos = new Vec2();
   r_pos = r.checkPos();
   if(sqrt(square(r_pos.x-pos.x) + square(r_pos.y-pos.y)) < rad + r.r)
   {
    return false; 
   }
  }
  return true;
}

// This function checks if the robot is within the box
boolean robot_in_box(Vec2 pos, float rad)
{
  Vec2 b_pos = box.checkPos();
  if(b_pos.x - box_bottom/2 + rad <= pos.x && b_pos.x + box_bottom/2 - rad >= pos.x && b_pos.y - box.checkWidth()/2 - rad >= pos.y && b_pos.y - box.checkWidth()/2 - box.checkHeight() + rad <= pos.y)
  return true;
  
  return false;
  
}

float square(float a)
{
 return a*a; 
}
