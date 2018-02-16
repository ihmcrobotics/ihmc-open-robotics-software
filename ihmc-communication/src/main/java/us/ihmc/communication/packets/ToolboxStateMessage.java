package us.ihmc.communication.packets;

public class ToolboxStateMessage extends Packet<ToolboxStateMessage>
{
   public byte requestedState;

   public ToolboxStateMessage()
   {
      // TODO Auto-generated constructor stub
   }

   @Override
   public void set(ToolboxStateMessage other)
   {
      requestedState = other.requestedState;
      set(other);
   }

   public void setRequestedState(byte requestedState)
   {
      this.requestedState = requestedState;
   }

   public byte getRequestedState()
   {
      return requestedState;
   }

   @Override
   public boolean epsilonEquals(ToolboxStateMessage other, double epsilon)
   {
      return requestedState == other.requestedState;
   }
}
