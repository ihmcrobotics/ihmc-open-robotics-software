package us.ihmc.controlFlow;

public class DataTypeOne
{
   private double x, y;

   
   public double getX()
   {
      return x;
   }

   public void setX(double x)
   {
      this.x = x;
   }

   public double getY()
   {
      return y;
   }

   public void setY(double y)
   {
      this.y = y;
   }

   public void set(DataTypeOne inputData)
   {
      this.x = inputData.getX();
      this.y = inputData.getY();
   }
   
   
}
