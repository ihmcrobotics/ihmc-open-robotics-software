package us.ihmc.quadrupedRobotics.input;

public class InputChannelConfig
{
   private final InputChannel channel;
   private final boolean invert;

   /**
    * The percentage about zero within which the input value should be ignored.
    */
   private final double deadzone;
   private final int exponent;

   public InputChannelConfig(InputChannel channel, boolean invert)
   {
      this(channel, invert, 0.0);
   }

   public InputChannelConfig(InputChannel channel, boolean invert, double deadzone)
   {
      this(channel, invert, deadzone, 1);
   }

   public InputChannelConfig(InputChannel channel, boolean invert, double deadzone, int exponent)
   {
      if (exponent % 2 == 0)
      {
         throw new IllegalArgumentException("Only odd functions are allowed");
      }

      this.channel = channel;
      this.invert = invert;
      this.deadzone = deadzone;
      this.exponent = exponent;
   }

   public double apply(double value)
   {
      // If the value is within the deadzone, ignore it. Otherwise, offset the value to the deadzone limit and rescale
      // it to full scale.
      if (Math.abs(value) < deadzone)
      {
         value = 0.0;
      }
      else
      {
         value = (value - (Math.signum(value) * deadzone)) / (1.0 - deadzone);
      }

      return Math.pow((invert ? -1 : 1) * value, exponent);
   }

   public InputChannel getChannel()
   {
      return channel;
   }
}
