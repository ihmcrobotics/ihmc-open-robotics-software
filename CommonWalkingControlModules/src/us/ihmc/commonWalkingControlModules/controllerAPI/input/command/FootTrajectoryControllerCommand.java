package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootTrajectoryControllerCommand extends SE3TrajectoryControllerCommand<FootTrajectoryControllerCommand, FootTrajectoryMessage>
{
   private RobotSide robotSide;

   public FootTrajectoryControllerCommand()
   {
   }

   @Override
   public void clear()
   {
      robotSide = null;
      super.clear();
   }

   @Override
   public void set(FootTrajectoryMessage message)
   {
      super.set(message);
      robotSide = message.getRobotSide();
   }

   @Override
   public void set(FootTrajectoryControllerCommand other)
   {
      super.set(other);
      robotSide = other.robotSide;
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   @Override
   public Class<FootTrajectoryMessage> getMessageClass()
   {
      return FootTrajectoryMessage.class;
   }
}
