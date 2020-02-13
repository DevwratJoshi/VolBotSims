class Box
{
  float x;
  float y;  // Position of the center point of the bottom rectangle
  float w;
  float h;
  int vel_direction; // The direction of the velocity 
  char boxType; // This variable will decide whether the box iskinematic or dynamic. This is because the box shakes best as a kinematic body, but needs to be a dynamic body to touch the ground and collect data. 
  
  Body body;
  Body anchor1, anchor2;
  
  Box(float x_, float y_, char type)
  {
     x = x_;
     y = y_; 
     w = 40;
     h = height*5;
     vel_direction = -1;
     boxType = type;
    makeBody(new Vec2(x, y)); // THis will be the middle of the bottom section of the box
  }
  
  void display()
  {  
    if(boxType == 'k')
    {
     fill(0, 150, 150);
    }
    
    else
    {
       fill(150, 150, 0);
    } 
    noStroke();
     rectMode(CENTER);
     
     //float a = b.getAngle();
     Vec2 pos = box2d.getBodyPixelCoord(body);
     
     pushMatrix();
     translate(pos.x, pos.y);
     rect(0, 0, box_bottom, w);
     popMatrix();
     
     pushMatrix();
     translate(pos.x - box_bottom/2 - w/2, pos.y - h/2 - w/2);
     rect(0, 0, w, h);
     popMatrix();
     
     pushMatrix();
     translate(pos.x  + box_bottom/2 + w/2, pos.y - h/2 - w/2);
     rect(0, 0, w, h);
     popMatrix();
  }
  
   void makeBody(Vec2 center) {
    
    PolygonShape bs = new PolygonShape();
    float box2dW = box2d.scalarPixelsToWorld(box_bottom/2);
    float box2dH = box2d.scalarPixelsToWorld(w/2);
    bs.setAsBox(box2dW, box2dH);
   

    Vec2[] verticesl = new Vec2[4];  // An array of 4 vectors
    verticesl[0] = box2d.vectorPixelsToWorld(new Vec2((-box_bottom/2)-w, -h - w/2));
    verticesl[1] =box2d.vectorPixelsToWorld(new Vec2(-box_bottom/2, -h - w/2));
    verticesl[2] = box2d.vectorPixelsToWorld(new Vec2(-box_bottom/2, -w/2));
    verticesl[3] = box2d.vectorPixelsToWorld(new Vec2((-box_bottom/2)-w, -w/2));
    //[full] Making a polygon from that array
  //  print(verticesl[0] + " " + verticesl[1] + " " + verticesl[2] + " " + verticesl[3]);
    PolygonShape ls = new PolygonShape();
    ls.set(verticesl, verticesl.length);
    
     Vec2[] verticesr = new Vec2[4];  // An array of 4 vectors
    verticesr[0] = box2d.vectorPixelsToWorld(new Vec2((box_bottom/2)+w, -h- w/2));
    verticesr[1] = box2d.vectorPixelsToWorld(new Vec2((box_bottom/2) + w, - w/2 ));
    verticesr[2] = box2d.vectorPixelsToWorld(new Vec2(box_bottom/2, -w/2));
    verticesr[3] = box2d.vectorPixelsToWorld(new Vec2((box_bottom/2), -h -w/2));
    //[full] Making a polygon from that array
    PolygonShape rs = new PolygonShape();
    rs.set(verticesr, verticesr.length);
    
    
    ////// The last fixture is attached to the bottom side of the box so that it does not bounce against the ground
     
     // the ne in verticesne stande for non-elastic
     Vec2[] verticesne = new Vec2[4];  // An array of 4 vectors
    verticesne[0] = box2d.vectorPixelsToWorld(new Vec2(-(box_bottom/2), w/2));
    verticesne[1] = box2d.vectorPixelsToWorld(new Vec2((box_bottom/2), w/2 ));
    verticesne[2] = box2d.vectorPixelsToWorld(new Vec2(box_bottom/2, w));
    verticesne[3] = box2d.vectorPixelsToWorld(new Vec2(-(box_bottom/2), w));
    //[full] Making a polygon from that array
    PolygonShape neb = new PolygonShape(); // Non elestic bottom
    neb.set(verticesne, verticesne.length);
    
   // rs.setAsBox(box2dW, box2dH);

    BodyDef bd = new BodyDef();
    if(boxType == 'k')
    {
       bd.type = BodyType.KINEMATIC;
    }
    
    else
    {
      bd.type = BodyType.DYNAMIC;
    }
    bd.position.set(box2d.coordPixelsToWorld(center));
    body = box2d.createBody(bd);
    
    BodyDef anc1= new BodyDef();
    anc1.type = BodyType.STATIC;
    anc1.position.set(box2d.coordPixelsToWorld(width/2, height));
    anchor1 = box2d.createBody(anc1);
  /*
    BodyDef anc2 = new BodyDef();
    anc2.type = BodyType.STATIC;
    anc2.position.set(box2d.coordPixelsToWorld(2*width/3, height));
    anchor2 = box2d.createBody(anc2);
    */
   FixtureDef bs_fd = new FixtureDef();
   bs_fd.shape = bs;
    bs_fd.density = 1;
    bs_fd.friction = 0.01;
    bs_fd.restitution = 0.9;
    
    FixtureDef ls_fd = new FixtureDef();
    ls_fd.shape = ls;
    ls_fd.density = 1;
    ls_fd.friction = 0.5;
    ls_fd.restitution = 0.9;
    
    FixtureDef rs_fd = new FixtureDef();
    rs_fd.shape = rs;
    rs_fd.density = 1;
    rs_fd.friction = 0.5;
    rs_fd.restitution = 0.9;
    
    FixtureDef neb_fd = new FixtureDef();
    neb_fd.shape = neb;
    neb_fd.density = 1;
    neb_fd.friction = 0.5;
    neb_fd.restitution = 0.01;
    // Define the body and make it from the shape
    body.createFixture(bs_fd);
    body.createFixture(ls_fd);
    body.createFixture(rs_fd);
    body.createFixture(neb_fd);
    // Give it some initial random velocity
    /*
    body.setLinearVelocity(new Vec2(random(-5, 5), random(2, 5)));
    body.setAngularVelocity(random(-5, 5));
    */
    
    PrismaticJointDef prj1  = new PrismaticJointDef();
    prj1.bodyA = anchor1;
    prj1.bodyB = body;
    prj1.localAxisA.set(0, 1);
    
    PrismaticJoint prj = (PrismaticJoint) box2d.world.createJoint(prj1);
    /*
    PrismaticJointDef prj2  = new PrismaticJointDef();
    prj2.bodyA = anchor2;
    prj2.bodyB = body;
    prj2.localAxisA.set(0, 1);
    */
  }
  
  void applyForce(Vec2 force) {
    Vec2 pos = body.getWorldCenter();
    body.applyForce(force, pos);
  }
  
    void applyVelocity(Vec2 vel)
  {
     Vec2 new_vel = new Vec2();
     new_vel.x = vel.x;
     new_vel.y = vel.y;
     
    // println(new_vel);
     body.setLinearVelocity(new_vel); 
  }
  
  int checkPositiony()
  {
   Vec2 pos = box2d.getBodyPixelCoord(body);
   
   if(pos.y > (mean_box_height + amplitude/2))
   {
       return -1;
   }
   
   else if (pos.y < (mean_box_height - amplitude/2))
   {
     return 1;  
   }
   
   else 
   return 0;
   
  }
  
  int checkPositionx()
  {
   Vec2 pos = box2d.getBodyPixelCoord(body);
   
   if(pos.x > (width/2 + width/100))
   {
       return 1;
   }
   
   else if (pos.x < (width/2 - width/100))
   {
     return -1;  
   }
   
   else 
   return 0;
   
  }
  
  float returnMeanHeight()
  {
   return mean_box_height; 
  }
  
    Vec2 checkPos()
  {
   Vec2 pos = box2d.getBodyPixelCoord(body);
   return pos;
  }
  
    void killBody()
  {
     box2d.destroyBody(body); 
  }
  
}
