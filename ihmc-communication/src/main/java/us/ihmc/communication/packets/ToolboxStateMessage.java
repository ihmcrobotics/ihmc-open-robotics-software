package us.ihmc.communication.packets;

public class ToolboxStateMessage extends Packet<ToolboxStateMessage>
{
   public static final byte WAKE_UP = 0;
   public static final byte REINITIALIZE = 1;
   public static final byte SLEEP = 2;

   public byte requestedToolboxState;

   public ToolboxStateMessage()
   {
      // TODO Auto-generated constructor stub
   }

   @Override
   public void set(ToolboxStateMessage other)
   {
      requestedToolboxState = other.requestedToolboxState;
      set(other);
   }

   public void setRequestedState(byte requestedState)
   {
      this.requestedToolboxState = requestedState;
   }

   public byte getRequestedState()
   {
      return requestedToolboxState;
   }

   @Override
   public boolean epsilonEquals(ToolboxStateMessage other, double epsilon)
   {
      return requestedToolboxState == other.requestedToolboxState;
   }
}
