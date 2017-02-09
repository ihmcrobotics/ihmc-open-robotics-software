package us.ihmc.communication.packets;

public class ToolboxStateMessage extends TrackablePacket<ToolboxStateMessage>
{
   public ToolboxState requestedState;

   public enum ToolboxState {WAKE_UP, REINITIALIZE, SLEEP};

   public ToolboxStateMessage()
   {
      // TODO Auto-generated constructor stub
   }

   public ToolboxStateMessage(ToolboxState requestedState)
   {
      this.requestedState = requestedState;
      
   }

   public void setRequestedState(ToolboxState requestedState)
   {
      this.requestedState = requestedState;
   }

   public ToolboxState getRequestedState()
   {
      return requestedState;
   }

   @Override
   public boolean epsilonEquals(ToolboxStateMessage other, double epsilon)
   {
      return requestedState == other.requestedState;
   }
}
