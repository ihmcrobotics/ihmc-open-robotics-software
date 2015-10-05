package us.ihmc.controlFlow;

public class DataTypeTwo
{
   private double z;

   
   public double getZ()
   {
      return z;
   }

   public void setZ(double z)
   {
      this.z = z;
   }

   public void set(DataTypeTwo dataTypeTwo)
   {
      this.z = dataTypeTwo.getZ();
   }
   
   
}
