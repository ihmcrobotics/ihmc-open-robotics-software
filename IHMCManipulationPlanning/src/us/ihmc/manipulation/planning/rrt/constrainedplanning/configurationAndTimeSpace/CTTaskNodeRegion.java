package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

public class CTTaskNodeRegion
{
   private double[] upperLimit;
   private double[] lowerLimit;
   private boolean[] isEnable;
   
   private double trajectoryTime;
   private static double intentionalTimeRatio = 1.2; 
   
   public CTTaskNodeRegion(int size)
   {
      upperLimit = new double[size];
      lowerLimit = new double[size];
      isEnable = new boolean[size];
   }
   
   public double getIntentionalTimeRatio()
   {
      return intentionalTimeRatio;
   }
   
   public void setRandomRegion(int index, double lowerValue, double upperValue)
   {
      if(index == 0)
      {
         upperLimit[index] = upperValue*intentionalTimeRatio;
         trajectoryTime = upperValue;
      }
      else
      { 
         upperLimit[index] =  upperValue;
      }      
      
      lowerLimit[index] = lowerValue;
      
      if(upperValue == lowerValue)
      {
         isEnable[index] = false;
      }
      else
      {
         isEnable[index] = true;
      }
   }
   
   public boolean isEnable(int index)
   {
      return isEnable[index];
   }
   
   public double getUpperLimit(int index)
   {
      return upperLimit[index];
   }
   
   public double getLowerLimit(int index)
   {
      return lowerLimit[index];
   }
   
   public double sizeOfRegion(int index)
   {
      return Math.abs(getUpperLimit(index) - getLowerLimit(index));
   }
      
   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }
}
