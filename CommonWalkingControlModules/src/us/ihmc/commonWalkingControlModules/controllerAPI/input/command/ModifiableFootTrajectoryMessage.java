package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class ModifiableFootTrajectoryMessage extends ModifiableSE3TrajectoryMessage<FootTrajectoryMessage>
{
   private RobotSide robotSide;

   public ModifiableFootTrajectoryMessage()
   {
   }

   @Override
   public void set(FootTrajectoryMessage trajectoryMessage)
   {
      super.set(trajectoryMessage);
      robotSide = trajectoryMessage.getRobotSide();
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }
}
