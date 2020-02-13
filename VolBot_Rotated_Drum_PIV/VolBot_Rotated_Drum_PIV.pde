import shiffman.box2d.*;
import org.jbox2d.collision.shapes.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.joints.*;

float stepsPerSec = 200.0;
int steps = 0;
Box2DProcessing box2d;
Drum drum;

int maxSteps = 50000;

String in = "initial_positions";
int in_counter = 3;
String extention = ".txt";
BufferedReader reader;
int large_robots = 0;
int small_robots = 0;
ArrayList<Robot> robots;

ArrayList<Grid> grids;
int horizontalDivisions = 20;
int verticalDivisions = 20;
float xVelocities[][] = new float[horizontalDivisions][verticalDivisions];
float yVelocities[][] = new float[horizontalDivisions][verticalDivisions]; // These are the divisions to be used for the velocity distribution graph.

 int small_positions[][] = new int[horizontalDivisions][verticalDivisions]; // To check whether a small robot is near this grid square 
int big_positions[][] = new int[horizontalDivisions][verticalDivisions]; // To check whether a large robot is near this grid square


final int filterWidth = 9; // The width of the median filter 
PrintWriter outputx, outputy, output, output_big, output_small;
String folder_begin = "Data/";
String PIV_data_prefix = "PIV_data";
String Position_data_prefix = "Position_data";
float density_low = 0.5;
float density_high = 4.0;
float density_mid = 1.0;

float large_robot_density = density_high;
float big = 30;
float small = 20;

float drumRadius  = 450;
float drumThickness = 30;
int drumSections = 144;

float RPM = 11.6; // Read this value in a paper

void setup()
{
  size(1000, 1000);
  box2d = new Box2DProcessing(this, 100);
  box2d.createWorld();
  box2d.setGravity(0, -9.8);
  drum = new Drum(new Vec2(width/2, height/2), drumRadius, drumThickness, drumSections);
  robots = new ArrayList<Robot>();
  grids = new ArrayList<Grid>();
  createRobots(in + str(in_counter)+ extention);
  
  outputx = createWriter(folder_begin + "/"  + "sections_max" + str(horizontalDivisions)  + "/"+ PIV_data_prefix + "/" + "X/" + "PIV_data");
  outputy = createWriter(folder_begin + "/"  + "sections_max" + str(horizontalDivisions)  + "/"+ PIV_data_prefix + "/" + "Y/" + "PIV_data");
  output_big = createWriter(folder_begin + "/"  + "sections_max" + str(horizontalDivisions)  + "/"+ Position_data_prefix + "/" + "big_position");
  output_small = createWriter(folder_begin + "/"  + "sections_max" + str(horizontalDivisions)  + "/"+ Position_data_prefix + "/" + "small_position");
  
  output = createWriter(folder_begin + "/" + "README" + extention);
  
 
  output.println("big diameter = " + str(int(big)) + "\n small diameter = " + str(int(small)));
  output.println("Vertical divisions = " + str(verticalDivisions) + "\n Horizontal divisions = " + str(horizontalDivisions));
  output.println("number of robots = " + str(large_robots+small_robots));
  output.println("Drum diameter = " + str(int(drumRadius*2)));
  output.println("RPM = " + str(RPM));
  output.flush();
  output.close(); 
  
  for(int i = 0; i < horizontalDivisions; i++)
  {
    for(int j = 0; j < horizontalDivisions; j++)
      {
        xVelocities[i][j] = 0.;
        yVelocities[i][j] = 0.;
        big_positions[i][j] = 0;
        small_positions[i][j] = 0;
      }
  
  }
   
}

void draw()
{
  background(255);
  box2d.step(1/stepsPerSec, 10, 10);
  steps++;
  drum.applyRPM(RPM);
  drum.display();
  displayRobots();
  
  collectPIV();
  collectPositions();
  if(steps > maxSteps)
  {
   writePIVtoFile(); 
   writePositionstoFile();
   exit();
  }
}

void keyPressed()
{
 if(key == 's')
 {
   print("Steps = " + str(steps));
  writePIVtoFile();
  writePositionstoFile();
  exit();
 }
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



void collectPositions() // To check where small and large balls are, using the same resolution of discretization as the grid squares
{
   Vec2 drumPos = drum.checkPos();
   float drumR = drum.radius;
   float drumT = drum.thickness;

 for(Robot r : robots)
  {
   Vec2 pos = r.checkPos();
   int x = 0;
   int y = 0;
   int i = 0;
   
     while(i < horizontalDivisions)
     {
       if(pos.x > ((drumPos.x - drumR + drumT ) + i*(2*drumR)/horizontalDivisions) && pos.x < ((drumPos.x - drumR + drumT) + ((i+1)*(2*drumR))/horizontalDivisions))
       {
         x = i;
         break;
       }
       i++;
     }
     
     i = 0;
    while(i < verticalDivisions)
     {
       if(pos.y < ((drumPos.y + drumR - drumT) - i*drumR*2/verticalDivisions) && pos.y > ((drumPos.y + drumR - drumT) - (i+1)*(drumR*2)/verticalDivisions))
       {
         y = i;
         break;
       }
       i++;
     }

    if(r.bigProb)
    {
      big_positions[x][y] += 1;       
    }
    
    else
    {
      small_positions[x][y] += 1;
    }
  }
  
}

void writePositionstoFile()
{
for(int y = verticalDivisions-1; y >= 0; y--)
   {
      for(int x = 0; x < horizontalDivisions; x++)
      {
         output_big.print(big_positions[x][y] + ",");
         output_small.print(small_positions[x][y] +  ",");
      }
      output_big.println();
      output_small.println();
   }
   
   output_big.flush();
   output_small.flush();
   
   output_big.close();
   output_small.close();         
}

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
      }
    }
    // remove the first element of the grid and add another 
    grids.remove(0);     
  }
  
/////////////// Add a new grid element here. 
Grid g = new Grid();
   Vec2 drumPos = drum.checkPos();
   float drumR = drum.radius;
   float drumT = drum.thickness;
   for(Robot r : robots)
  {
   Vec2 pos = r.checkPos();
   int x = 0;
   int y = 0;
   int i = 0;
   
     while(i < horizontalDivisions)
     {
       if(pos.x > ((drumPos.x - drumR + drumT ) + i*(2*drumR)/horizontalDivisions) && pos.x < ((drumPos.x - drumR + drumT) + ((i+1)*(2*drumR))/horizontalDivisions))
       {
         x = i;
         break;
       }
       i++;
     }
     
     i = 0;
    while(i < verticalDivisions)
     {
       if(pos.y < ((drumPos.y + drumR - drumT) - i*drumR*2/verticalDivisions) && pos.y > ((drumPos.y + drumR - drumT) - (i+1)*(drumR*2)/verticalDivisions))
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
