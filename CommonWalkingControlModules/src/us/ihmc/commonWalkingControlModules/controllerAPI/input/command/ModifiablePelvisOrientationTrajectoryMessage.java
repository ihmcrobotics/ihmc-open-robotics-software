package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;

public class ModifiablePelvisOrientationTrajectoryMessage
      extends ModifiableSO3TrajectoryMessage<ModifiablePelvisOrientationTrajectoryMessage, PelvisOrientationTrajectoryMessage>
{
   public ModifiablePelvisOrientationTrajectoryMessage()
   {
   }

   @Override
   public Class<PelvisOrientationTrajectoryMessage> getMessageClass()
   {
      return PelvisOrientationTrajectoryMessage.class;
   }
}
