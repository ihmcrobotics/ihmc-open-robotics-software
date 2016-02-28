package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.concurrent.Builder;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;

public class ModifiablePelvisOrientationTrajectoryMessage extends ModifiableSO3TrajectoryMessage<PelvisOrientationTrajectoryMessage>
{
   public ModifiablePelvisOrientationTrajectoryMessage()
   {
   }

   public static Builder<ModifiablePelvisOrientationTrajectoryMessage> createBuilder()
   {
      Builder<ModifiablePelvisOrientationTrajectoryMessage> builder = new Builder<ModifiablePelvisOrientationTrajectoryMessage>()
      {
         @Override
         public ModifiablePelvisOrientationTrajectoryMessage newInstance()
         {
            return new ModifiablePelvisOrientationTrajectoryMessage();
         }
      };
      return builder;
   }
}
