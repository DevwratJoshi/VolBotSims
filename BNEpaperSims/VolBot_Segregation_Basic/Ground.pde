class Ground
{
  float x;
  float y;
  float w;
  float h;
  
  Body b;
  
  
  Ground()
  {
     x = width/2;
     y = height- 10;
     w = width - 10;
     h = 6;
     PolygonShape sd = new PolygonShape();
     
     float box2dw = box2d.scalarPixelsToWorld(w/2);
     float box2dh = box2d.scalarPixelsToWorld(h/2);
     
     sd.setAsBox(box2dw, box2dh);
    FixtureDef g_fd = new FixtureDef();
    g_fd.shape = sd;
    g_fd.friction = 0.9;
    g_fd.restitution = 0.01;
     BodyDef bd = new BodyDef();
     bd.type = BodyType.STATIC;
     
     bd.position.set(box2d.coordPixelsToWorld(x, y));
     b = box2d.createBody(bd);
     
     b.createFixture(g_fd);
  }
  
  void display()
  {
     fill(50, 50, 50);
     noStroke();
     rectMode(CENTER);
     
     //float a = b.getAngle();
     
     pushMatrix();
     translate(x, y);
     rect(0, 0, w, h);
     popMatrix();
  }
  
}
