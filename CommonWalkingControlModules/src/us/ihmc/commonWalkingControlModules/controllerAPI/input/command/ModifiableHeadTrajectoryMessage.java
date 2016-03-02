package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;

public class ModifiableHeadTrajectoryMessage extends ModifiableSO3TrajectoryMessage<ModifiableHeadTrajectoryMessage, HeadTrajectoryMessage>
{
   public ModifiableHeadTrajectoryMessage()
   {
   }

   @Override
   public Class<HeadTrajectoryMessage> getMessageClass()
   {
      return HeadTrajectoryMessage.class;
   }
}
