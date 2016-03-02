package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;

public class ModifiableStopAllTrajectoryMessage implements ControllerMessage<ModifiableStopAllTrajectoryMessage, StopAllTrajectoryMessage>
{
   private boolean stopAllTrajectory = false;

   @Override
   public void set(ModifiableStopAllTrajectoryMessage other)
   {
      stopAllTrajectory = other.stopAllTrajectory;
   }

   @Override
   public void set(StopAllTrajectoryMessage message)
   {
      stopAllTrajectory = true;
   }

   public boolean isStopAllTrajectory()
   {
      return stopAllTrajectory;
   }

   public void setStopAllTrajectory(boolean stopAllTrajectory)
   {
      this.stopAllTrajectory = stopAllTrajectory;
   }

   @Override
   public Class<StopAllTrajectoryMessage> getMessageClass()
   {
      return StopAllTrajectoryMessage.class;
   }
}
