package us.ihmc.avatar.ros.messages;

public class Float64Message
{
   /**
    * General Float64 message wrapper, as defined in the std_msgs/Float64 rosmsg type.
    */

   private double value;

   public void setValue(double value)
   {
      this.value = value;
   }

   public double getValue()
   {
      return value;
   }

   @Override
   public String toString()
   {
      return "[Float64Message] Value: " + value;
   }
}
