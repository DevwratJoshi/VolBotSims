/*
 An isoceles triangle of light that shines from the center of a selected robot. 
 The robots caught in the light become small or big depending on the colour of the light
*/
class LightHouse  
{
 Vec2 center_pos; // The positon of the center of a module
 Vec2 goal_pos; // The position of the goal. The arc will be drawn "facing" this position
 Vec2 diff;
 float theta_prime;
 float alpha; // The angular coordinate of the counter-clockwise limit in the blue module frame. (Origin at the center of the blue module)
 float beta; // The angular coordnate of the clockwise limit in the blue module frame

 float ang_width; // The width of the arc
 float start_ang = 0.;
 char colour;
 float diameter; // The diameter of the light beam
 
 
 LightHouse(Vec2 c_p, float ang, float d, char c)
 {
   center_pos = c_p;

   ang_width = ang;
   colour = c;
   diameter = d*2;
   diff = new Vec2(0, 0);
   alpha = 0;
   beta = 0.;
   theta_prime = 0.;
 }
 
 void shine(Vec2 c_pos, Vec2 g_pos) // This function calculates the orientation and position of the light from the lighthouse
 {
   center_pos = c_pos;
   goal_pos = g_pos;
   diff.x = (g_pos.x - c_pos.x); 
    diff.y = (g_pos.y - c_pos.y); 
    theta_prime = atan2(diff.y, diff.x);
    println(theta_prime);
     alpha = theta_prime + radians(ang_width/2);
    beta = 2*PI - radians(ang_width/2) + theta_prime;
    //println(degrees(theta_prime));
 }
 
  void shineAngle(Vec2 c_pos, float angle) // This function shines the light at a given angle from the center of the blue ball
 {
   center_pos = c_pos;
    theta_prime = radians(angle);
    alpha = theta_prime + radians(ang_width/2);
    beta = 2*PI - radians(ang_width/2) + theta_prime;
    //println(degrees(theta_prime));
 }
 
 void display() // The position of the module the ligthouse is on
 {
    if(lightOn)
    {
        pushMatrix();
        translate(center_pos.x, center_pos.y);
        
        stroke(255, 255, 0);
        fill(200, 200, 0, 180);
        
        arc(0, 0, diameter, diameter, theta_prime - radians(ang_width/2), theta_prime + radians(ang_width/2));
        //arc(0, 0, diameter, diameter, 0, radians(360)); // Uncomment this line to draw a full circle
        popMatrix();
    }
 }
 
 /*
  This function takes the position of a module as an argument.
  It outputs whether the robot is "illuminated" by the "lighthouse"
  The output is a char rather than a bool to allow for different colours of light
 */
 
 char isIlluminated(Vec2 mod_pos) 
 {
   if(lightOn)
   {
     if(sqrt((mod_pos.x - center_pos.x)*(mod_pos.x - center_pos.x) + (mod_pos.y - center_pos.y)*(mod_pos.y - center_pos.y)) < diameter/2)
     {
       float theta = atan2(mod_pos.y - center_pos.y, mod_pos.x - center_pos.x);
       
       if(degrees(theta) < degrees(theta_prime) + ang_width/2 && degrees(theta) > degrees(theta_prime) - ang_width/2)
       {
        // println("theta = " + degrees(theta) + " Theta_prime = " + degrees(theta_prime)); 
        return 'y'; // The y stands for yes. Default is a module not being illuminated 
       }
       
     }
   }
   return 'n'; // The n stands for no. Default is a module not being illuminated
 }
 
 float checktheta_prime()
 {
  return degrees(theta_prime); 
 }
  
}
