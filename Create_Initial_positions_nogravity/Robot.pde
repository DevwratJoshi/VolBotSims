class Robot
{
  Body body;
  float r;
  char colour;
  public boolean increasing;
  public boolean decreasing;
  // Constructor
  Robot(float x, float y, float radius, char c)
  {
    r = radius;
    makeBody(new Vec2(x, y));
    increasing = false;
    decreasing = false;
    
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
  
  boolean done()
  {
     Vec2 pos = box2d.getBodyPixelCoord(body); 
     if(pos.y > height)
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
    rotate(-a);
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

    //rect(0,0,w,h);
    ellipse(0, 0, r*2, r*2);
    popMatrix();
  }
  
 void makeBody(Vec2 center)
 {
  BodyDef bd = new BodyDef();
  bd.type = BodyType.DYNAMIC;
  bd.position.set(box2d.coordPixelsToWorld(center));
  body = box2d.createBody(bd);
  
  CircleShape circle = new CircleShape();
  circle.m_radius = box2d.scalarPixelsToWorld(r);
  circle.m_p.set(0, 0);
  
  FixtureDef fd = new FixtureDef();
    fd.shape = circle;
    // Parameters that affect physics
    fd.density = 2/(3.1415*r*r);
    fd.friction = 0.3;
    fd.restitution = 0.3;
  
  body.createFixture(fd);
  
 }
  // the following function would change the value of the radius slowly to get the goal radius eventually 
 boolean changeRadius(float goal_rad)
 {
   
    float rate = 0.1; // The rate of increase/decrease of the radius
    
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
      fd.density = 2/(3.1415*r*r);
      fd.friction = 0.2;
      fd.restitution = 0.2;
      
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
  
  void applyVelocity(float vel)
  {    
      Vec2 n_vel = new Vec2(0,0);
      int ran2 = int(random(10));
      
      switch(ran2)
      {
       case 1:
       {
       n_vel.x = vel;
       break;
       }
       
       case 2:
       {
       n_vel.x = -vel;
       break;
       }
       
       case 3:
       {
       n_vel.y = vel;
       break;
       }
       
       case 4:
       {
       n_vel.y = -vel;
       break;
       }
      } 
   body.setLinearVelocity(n_vel); 
  }
  
  
}
