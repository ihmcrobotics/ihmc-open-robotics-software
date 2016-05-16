package us.ihmc.communication.packets;

public class KinematicsToolboxStateMessage extends TrackablePacket<KinematicsToolboxStateMessage>
{
   public KinematicsToolboxState requestedState;

   public enum KinematicsToolboxState {WAKE_UP, REINITIALIZE, SLEEP};

   public KinematicsToolboxStateMessage()
   {
      // TODO Auto-generated constructor stub
   }

   public KinematicsToolboxStateMessage(KinematicsToolboxState requestedState)
   {
      this.requestedState = requestedState;
      
   }

   public void setRequestedState(KinematicsToolboxState requestedState)
   {
      this.requestedState = requestedState;
   }

   public KinematicsToolboxState getRequestedState()
   {
      return requestedState;
   }

   @Override
   public boolean epsilonEquals(KinematicsToolboxStateMessage other, double epsilon)
   {
      return requestedState == other.requestedState;
   }
}
