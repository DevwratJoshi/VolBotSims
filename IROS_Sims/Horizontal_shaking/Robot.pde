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
  public boolean bigProb; // true if the module will become big under the line, else false
  public boolean increasing;
  public boolean decreasing;
  Vec2 last_position;
  // Constructor
  Robot(float x, float y, float radius, char c, boolean prob, float d, float bed_fric)
  {
    r = radius;
    bigProb = prob;
    density = d;
    e = 0.5;
    friction = 0.1;
    friction_with_bed = bed_fric;
    mass = 0.250;
    makeBody(new Vec2(x, y));
    increasing = false;
    decreasing = false;
    last_position = new Vec2(0,0);
    
    switch(c)
    {
     case 'r':
     colour = 'r';
     break;
     
     case 'g':
     colour = 'g';
     break;
     
     case 'b':
     colour = 'b';
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
     
     default:
     break;
    }

    stroke(0);
    strokeWeight(1);
    //rect(0,0,w,h);
    ellipse(0, 0, r*2, r*2);
    
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
      if(goal_rad - r < rate)
      {
        increasing = false;
        return true;
      }

    }
    
    
    else if(r > goal_rad)
    {
      
      if(r - goal_rad < rate) // Return if the difference between the two radii is less than the rate, no point editing it anymore
      {
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
    float friction_mag = friction_with_bed*mass*9.8; // Assuming acc of gravity is 9.8 m/s^2
    Vec2 r_vel = checkLinearVelocity();
    force.x = r_vel.x - box_vel.x;
    force.y = r_vel.y - box_vel.y;
    float temp;
    temp = sqrt(force.x*force.x + force.y*force.y);
    force.x = -friction_mag*force.x/temp;
    force.y = -friction_mag*force.y/temp;
    
    body.applyForce(force, body.getWorldCenter());
  }
   
}
