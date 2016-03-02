package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;

public class ModifiablePelvisHeightTrajectoryMessage extends SimpleTrajectoryPoint1DList implements ControllerMessage<ModifiablePelvisHeightTrajectoryMessage, PelvisHeightTrajectoryMessage>
{
   public ModifiablePelvisHeightTrajectoryMessage()
   {
   }

   @Override
   public void set(ModifiablePelvisHeightTrajectoryMessage other)
   {
      super.set(other);
   }

   @Override
   public void set(PelvisHeightTrajectoryMessage message)
   {
      super.set(message);
   }

   @Override
   public Class<PelvisHeightTrajectoryMessage> getMessageClass()
   {
      return PelvisHeightTrajectoryMessage.class;
   }
}
