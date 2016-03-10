package us.ihmc.aware.input;

public class InputChannelConfig
{
   private final InputChannel channel;
   private final boolean invert;

   public InputChannelConfig(InputChannel channel, boolean invert)
   {
      this.channel = channel;
      this.invert = invert;
   }

   public InputChannel getChannel()
   {
      return channel;
   }

   public boolean isInverted()
   {
      return invert;
   }
}
