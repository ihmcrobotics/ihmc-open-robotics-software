package us.ihmc.avatar.ros.messages;

public class Int8Message
{
   /**
    * General Int8 message wrapper, as defined in the std_msgs/Int8 rosmsg type.
    */

   private byte value;

   public void setValue(int value)
   {
      if(value >= 127 || value <= -128)
         throw new RuntimeException("Value out of range for an 8-bit Integer");

      this.value = (byte) value;
   }

   public int getValue()
   {
      return (int) value;
   }

   @Override
   public String toString()
   {
      return "[Int8Message] Value: " + (int) value;
   }
}
