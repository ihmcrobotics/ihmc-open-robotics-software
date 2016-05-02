package us.ihmc.quadrupedRobotics.input;

/**
 * A single event from an input device. Each event is a tuple of the source channel and the new channel value,.
 */
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
