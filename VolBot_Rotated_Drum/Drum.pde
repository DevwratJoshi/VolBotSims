class Drum
{
 float radius; // Inner radius of drum
 float thickness; // Thickness of drum wall
 int sections; // Number of convex sections making up the drum (Trapezoids)
 Body body;
 Drum(Vec2 center, float r, float t, int s)
 {
   radius = r;
   sections = s;
   thickness = t;
   makeBody(center);

 }
 
 void makeBody(Vec2 center)
 {
   BodyDef bd = new BodyDef();
   bd.position.set(box2d.coordPixelsToWorld(center));
   bd.type = BodyType.KINEMATIC;
   body = box2d.createBody(bd);
   for(int i = 0; i < sections; i++)
   {
     float theta1 = PApplet.map(i, 0, sections, 0, TWO_PI);
     float theta2 = PApplet.map(i+1, 0, sections, 0, TWO_PI);
     
     Vec2[] vertices = new Vec2[4];
     vertices[0] = box2d.vectorPixelsToWorld(new Vec2(radius*cos(theta1), radius*sin(theta1)));
     vertices[1] = box2d.vectorPixelsToWorld(new Vec2((radius+thickness)*cos(theta1), (radius+thickness)*sin(theta1)));
     vertices[2] = box2d.vectorPixelsToWorld(new Vec2((radius+thickness)*cos(theta2), (radius+thickness)*sin(theta2)));
     vertices[3] = box2d.vectorPixelsToWorld(new Vec2(radius*cos(theta2), radius*sin(theta2)));
     
     PolygonShape sec = new PolygonShape();
     sec.set(vertices, vertices.length);
     
     FixtureDef sec_fd = new FixtureDef();
     sec_fd.shape = sec;
     sec_fd.density = 1;
     sec_fd.friction = 0.2;
     sec_fd.restitution = 0.1;
     
     body.createFixture(sec_fd);
    
   }
 } 
 void display()
 {
     Vec2 pos = box2d.getBodyPixelCoord(body);
    // Get its angle of rotation
    float a = body.getAngle(); 
    rectMode(CENTER);
    pushMatrix();
    translate(pos.x, pos.y);
    rotate(-a);  
    fill(0, 150, 0);
    for(int i = 0; i < sections; i++)
    {
     beginShape();
     vertex(radius*cos(radians(180.0/sections)), radius*sin(radians(180.0/sections)));
     vertex((radius+thickness)*cos(radians(180.0/sections)), (radius+thickness)*sin(radians(180.0/sections)));
     vertex((radius+thickness)*cos(radians(180.0/sections)), -(radius+thickness)*sin(radians(180.0/sections)));
     vertex(radius*cos(radians(180.0/sections)), -radius*sin(radians(180.0/sections)));
     endShape(CLOSE);
     
     rotate(radians(360.0/sections));
    }
    
    popMatrix();
    
    
 }
    
 void applyRPM(float omega)
 {
   body.setAngularVelocity(2*PI*omega/60.0);
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
