package us.ihmc.utilities.ros.publisher;

import std_msgs.Float64;

public class RosDoublePublisher extends RosTopicPublisher<std_msgs.Float64>
{
   private final double initialValue;
   
   public RosDoublePublisher(boolean latched)
   {
      this(latched, Double.NaN);
   }
   
   public RosDoublePublisher(boolean latched, double initialValue)
   {
      super(std_msgs.Float64._TYPE, latched);
      this.initialValue = initialValue;
   }
   
   @Override
   public void connected()
   {
      if(!Double.isNaN(initialValue))
      {
         publish(initialValue);
      }
   }
   

   public void publish(double value)
   {
      Float64 message = getMessage();
      message.setData(value);
      publish(message);
   }
}
