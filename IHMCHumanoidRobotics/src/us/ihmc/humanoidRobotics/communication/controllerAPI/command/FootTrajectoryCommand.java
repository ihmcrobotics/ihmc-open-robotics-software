package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Random;

public class FootTrajectoryCommand extends SE3TrajectoryControllerCommand<FootTrajectoryCommand, FootTrajectoryMessage>
{
   private RobotSide robotSide;

   public FootTrajectoryCommand()
   {
      super(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
   }

   public FootTrajectoryCommand(Random random)
   {
      super(ReferenceFrame.generateRandomReferenceFrame("dataFrame", random, ReferenceFrame.getWorldFrame()),
            ReferenceFrame.generateRandomReferenceFrame("trajectoryFrame", random, ReferenceFrame.getWorldFrame()));
   }

   @Override
   public void clear()
   {
      super.clear();
      robotSide = null;
   }

   @Override
   public void set(FootTrajectoryMessage message)
   {
      super.set(message);
      robotSide = message.getRobotSide();
   }

   @Override
   public void set(FootTrajectoryCommand other)
   {
      super.set(other);
      setPropertiesOnly(other);
   }

   /**
    * Same as {@link #set(FootTrajectoryCommand)} but does not change the trajectory points.
    *
    * @param other
    */
   @Override
   public void setPropertiesOnly(FootTrajectoryCommand other)
   {
      super.setPropertiesOnly(other);
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

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && super.isCommandValid();
   }
}
