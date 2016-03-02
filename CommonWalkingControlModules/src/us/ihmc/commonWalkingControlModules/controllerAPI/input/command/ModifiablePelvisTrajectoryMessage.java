package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;

public class ModifiablePelvisTrajectoryMessage extends ModifiableSE3TrajectoryMessage<ModifiablePelvisTrajectoryMessage, PelvisTrajectoryMessage>
{
   public ModifiablePelvisTrajectoryMessage()
   {
   }

   @Override
   public Class<PelvisTrajectoryMessage> getMessageClass()
   {
      return PelvisTrajectoryMessage.class;
   }
}
