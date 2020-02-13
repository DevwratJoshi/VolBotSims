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
int no_of_robots = 120;
int blueRobots = 119;
int greenRobots = 1;
int count = 0;
char flag = 'n';
float velConst = 1;
Vec2 vel = new Vec2();
boolean box_pause = true;
float big = 60;
float mid = 30;
int small = 30;
int freq = 60;
int delay = 0;
int record = 0;
PrintWriter output;
char b = 'n';
float box_bottom = 800;
float box_height = box_bottom*3;
int mean_box_height;
Vec2 center_pos, center_velo;
int velocityDirection;
int amplitude;
Vec2 mouse1, mouse2;
Vec2 temp_mouse = new Vec2();
boolean mouseActive;
int max_record = 5000;
String blah;
void setup()
{
  size(1000, 1000);
  smooth();
 
  box2d = new Box2DProcessing(this, 100);
  box2d.createWorld();
  mean_box_height = 9*height/10;

  box2d.setGravity(0, -9.8);
  box = new Box(width/2, mean_box_height); 
  robots = new ArrayList<Robot>();
  
  center_velo = new Vec2();
  center_pos = new Vec2();

  
  
  amplitude = (int)(small);
  ground = new Ground();
  vel.x = 0.0;
  vel.y = velConst;
  blah = "initial_positionsBlue" + str(blueRobots) + "Green" + str(greenRobots) +"(1)"+ ".txt";
    output = createWriter(blah);
}

void draw() {
  background(255);

  if(greenRobots > 0)
  {

       Robot p = new Robot(box.checkPos().x, box.checkPos().y - small*10, big, 'b', 's'); //This is for the large one at the bottom
     // Robot p = new Robot(box.checkPos().x + box_bottom/2 - small*8, 200, big, 'b', 's'); //This is for the large one at the top
      robots.add(p);
      count++;
      greenRobots--;
  }

  if (blueRobots > 0 && random(1) < 0.8)
  {
   float ran = random(1);
 
    if(blueRobots > 0)
      {
        Robot p = new Robot(width/2 + random(-box_bottom/2 + small*3, box_bottom/2 - small*5),200, small, 'g', 'd');
        blueRobots--;
        robots.add(p);
        count++;
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

if(keyPressed && count >= no_of_robots)
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
void mousePressed()
{
 for (int i = robots.size()-1; i >= 0; i--)
  {
    Robot r = robots.get(i);
    if(r.colour == 'b')
    {
      Robot g = new Robot(r.checkPos().x, r.checkPos().y, r.r, 'b', 'd');
      r.killBody();
      robots.remove(i);
      robots.add(g);
    }
  }
}

float square(float a)
{
 return a*a; 
}
