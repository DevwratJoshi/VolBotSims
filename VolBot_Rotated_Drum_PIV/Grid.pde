class Grid
{
    float xVelocities[][] = new float[horizontalDivisions][verticalDivisions];
    float yVelocities[][] = new float[horizontalDivisions][verticalDivisions];
     
    Grid()
    {

    }
    
    void setXVelocity(int x, int y, float value)
    {
       xVelocities[x][y] = value;
    }
    
    
   void setYVelocity(int x, int y, float value)
    {
       yVelocities[x][y] = value;
    }
    
    void addXVelocity(int x, int y, float value)
    {
       xVelocities[x][y] += value;
    }
    
    
   void addYVelocity(int x, int y, float value)
    {
       yVelocities[x][y] += value;
    }
    
    float checkXVelocity(int x, int y)
    {
      return xVelocities[x][y];
    }
    
    float checkYVelocity(int x, int y)
    {
      return yVelocities[x][y];
    }
}
