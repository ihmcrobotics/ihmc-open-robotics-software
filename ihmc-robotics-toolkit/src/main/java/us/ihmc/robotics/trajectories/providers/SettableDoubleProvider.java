package us.ihmc.robotics.trajectories.providers;


public class SettableDoubleProvider implements DoubleProvider
{
   private double value = 0.0;

   public SettableDoubleProvider()
   {
   }

   public SettableDoubleProvider(double value)
   {
      this.value = value;
   }

   public void setValue(double value)
   {
      this.value = value;
   }

   public double getValue()
   {
      return value;
   }

   public void add(double amountToAdd)
   {
      value = value + amountToAdd;
   }
}