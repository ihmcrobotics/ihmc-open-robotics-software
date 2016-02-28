package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import org.apache.commons.lang3.mutable.MutableBoolean;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;

public class ModifiableStopAllTrajectoryMessage
{
   private final MutableBoolean stopRequested = new MutableBoolean(false);

   public void set(StopAllTrajectoryMessage stopAllTrajectoryMessage)
   {
      stopRequested.setTrue();
   }

   public void reset()
   {
      stopRequested.setFalse();
   }
}
