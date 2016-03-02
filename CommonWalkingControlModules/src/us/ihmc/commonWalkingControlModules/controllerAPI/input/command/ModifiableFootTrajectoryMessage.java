package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class ModifiableFootTrajectoryMessage extends ModifiableSE3TrajectoryMessage<ModifiableFootTrajectoryMessage, FootTrajectoryMessage>
{
   private RobotSide robotSide;

   public ModifiableFootTrajectoryMessage()
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
   public void set(ModifiableFootTrajectoryMessage other)
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
