package us.ihmc.aware.input;

public class InputEvent
{
   private final InputChannel channel;
   private final double value;

   public InputEvent(InputChannel channel, double value)
   {
      this.channel = channel;
      this.value = value;
   }

   public InputChannel getChannel()
   {
      return channel;
   }

   public double getValue()
   {
      return value;
   }
}
