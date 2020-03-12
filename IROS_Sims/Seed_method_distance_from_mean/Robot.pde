class Robot
{
  Body body;
  float r;
  char colour;
  float density; // This will determine inter-particle segregation phenomenon
  float mass; //Setting this to 
  float friction;
  float friction_with_bed;
  float e;
  char type; // 's' for segregator and 'm' for mover
  public boolean bigProb; // true if the module will become big under the line, else false
  public boolean increasing;
  public boolean decreasing;
  public boolean isSeed;
  char orig_colour;
  Vec2 last_position;
  // Constructor
  Robot(float x, float y, float radius, char c, boolean prob, float d, float bed_fric, char t)
  {
    r = radius;
    bigProb = prob;
    density = d;
    e = 0.5;
    friction = 0.1;
    friction_with_bed = bed_fric;
    mass = PI*(radius*radius/(10000)) * d; // Divided by 10000 cause to resolve the pixel to meter ratio
    type = t;
    makeBody(new Vec2(x, y));
    increasing = false;
    decreasing = false;
    last_position = new Vec2(0,0);
    isSeed = false;
    switch(c)
    {
     case 'r':
     colour = 'r';
     orig_colour = 'r';
     break;
     
     case 'g':
     colour = 'g';
     orig_colour = 'g';
     break;
     
     case 'b':
     colour = 'b';
     orig_colour = 'b';
     break;
     
     case 'y':
     colour = 'y';
     orig_colour = 'y';
     break;
     default:
     colour = 'r';
     break;
    }
  }
  
  void killBody()
  {
     box2d.destroyBody(body); 
  }
  
  boolean done(float value)
  {
     Vec2 pos = box2d.getBodyPixelCoord(body); 
     if(pos.y > value)
     {
        killBody();
        return true;
     }
     
     return false;
  }
  
  void display() 
  {
    // We look at each body and get its screen position
    Vec2 pos = box2d.getBodyPixelCoord(body);
    // Get its angle of rotation
    float a = body.getAngle();
    
    rectMode(CENTER);
    pushMatrix();
    translate(pos.x, pos.y);
   // rotate(-a);
    switch(colour)
    {
     case 'r':
     fill(255, 0, 0);
     break;
     
     case 'g':
     fill(0, 255, 0);
     break;
     
     case 'b':
     fill(0, 0, 255);
     break;
     
     case 'y':
     fill(255, 255, 0);
     break;
     
     default:
     break;
    }

    stroke(0);
    strokeWeight(1);
    //rect(0,0,w,h);
    ellipse(0, 0, r*2, r*2);
    fill(0);
    textSize(20);
    text(str(friction_with_bed), -15,9); 
    popMatrix();
  }
  
 void makeBody(Vec2 center)
 {
  BodyDef bd = new BodyDef();
  bd.type = BodyType.DYNAMIC;
  //bd.bullet = true;
  bd.position.set(box2d.coordPixelsToWorld(center));
  body = box2d.createBody(bd);
  
  CircleShape circle = new CircleShape();
  circle.m_radius = box2d.scalarPixelsToWorld(r);
  circle.m_p.set(0, 0);
  
  FixtureDef fd = new FixtureDef();
    fd.shape = circle;
    // Parameters that affect physics
    fd.density = density;
    
    fd.friction = friction;
    fd.restitution = e;
  
  body.createFixture(fd);
  
 }
  // the following function would change the value of the radius slowly to get the goal radius eventually 
 boolean changeRadius(float goal_rad)
 {
   
    float rate = 0.2; // The rate of increase/decrease of the radius
    
    if(r <= goal_rad)
    {
      increasing = true;
      density = density_large;
      if(goal_rad - r < rate)
      {
        r = goal_rad;
        increasing = false;
        return true;
      }

    }
    
    
    else if(r > goal_rad)
    {
      density = density_small;
      decreasing = true;
      if(r - goal_rad < rate) // Return if the difference between the two radii is less than the rate, no point editing it anymore
      {
        r = goal_rad;
      decreasing = false; 
      return true;
      }
      rate = -rate;
      
    }
    
    r = r + rate;
    
    
    Fixture fixture = body.getFixtureList();

     body.destroyFixture(fixture);
     CircleShape circle = new CircleShape();
     circle.m_radius = box2d.scalarPixelsToWorld(r);
     circle.m_p.set(0, 0);
  
      FixtureDef fd = new FixtureDef();
      fd.shape = circle;
      // Parameters that affect physics
      fd.density = density;
      fd.friction = friction;
      fd.restitution = e;
      
  body.createFixture(fd);
     
     return false;
 }
 
  void changeFriction(float f)
 {
   friction_with_bed = f;
 }
 
 char checkColour()
 {
    return colour; 
 }
 
 float checkRadius()
 {
   return r;
 }
 
 float checkHeight()
 {
   Vec2 pos = box2d.getBodyPixelCoord(body);
   
   return pos.y;
 }
 
   Vec2 checkPos()
  {
   Vec2 pos = box2d.getBodyPixelCoord(body);
   return pos;
  }
  
  Vec2 checkVelocity()
  {
    return body.getLinearVelocity();  
  }
  
  void applyImpulse(Vec2 dir)
  {
   body.applyLinearImpulse(dir, body.getWorldCenter(), true); 
  }
  
  Vec2 checkLinearVelocity()
  {
   return body.getLinearVelocity(); 
  }
  
  void applyFrictionForce(Vec2 box_vel)
  {
    Vec2 force = new Vec2();
    float friction_mag = friction_with_bed*9.8*mass; // Assuming acc of gravity is 9.8 m/s^2

    Vec2 r_vel = checkLinearVelocity();
    force.x = r_vel.x - box_vel.x;
    force.y = r_vel.y - box_vel.y;
    float temp;
    temp = sqrt(force.x*force.x + force.y*force.y);
    force.x = -friction_mag*force.x/temp;
    force.y = -friction_mag*force.y/temp;
    //body.applyLinearImpulse(force, body.getWorldCenter(), false);
   body.applyForce(force, body.getWorldCenter());
  }
   
}
