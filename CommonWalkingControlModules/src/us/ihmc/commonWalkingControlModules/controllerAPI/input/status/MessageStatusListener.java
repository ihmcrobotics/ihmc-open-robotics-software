package us.ihmc.commonWalkingControlModules.controllerAPI.input.status;

import us.ihmc.communication.packets.IHMCRosApiMessage;

public interface MessageStatusListener
{
   public enum Status
   {
      STARTED, COMPLETED, ABORTED;
   }

   public abstract void receivedNewMessageStatus(Status status, Class<? extends IHMCRosApiMessage<?>> messageClass, long messageId);
}
