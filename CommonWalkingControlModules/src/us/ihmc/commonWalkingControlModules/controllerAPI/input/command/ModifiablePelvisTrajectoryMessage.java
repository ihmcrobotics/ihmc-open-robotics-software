package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.concurrent.Builder;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;

public class ModifiablePelvisTrajectoryMessage extends ModifiableAbstractSE3TrajectoryMessage<PelvisTrajectoryMessage>
{
   public ModifiablePelvisTrajectoryMessage()
   {
   }

   public static Builder<ModifiablePelvisTrajectoryMessage> createBuilder()
   {
      Builder<ModifiablePelvisTrajectoryMessage> builder = new Builder<ModifiablePelvisTrajectoryMessage>()
      {
         @Override
         public ModifiablePelvisTrajectoryMessage newInstance()
         {
            return new ModifiablePelvisTrajectoryMessage();
         }
      };
      return builder;
   }
}
