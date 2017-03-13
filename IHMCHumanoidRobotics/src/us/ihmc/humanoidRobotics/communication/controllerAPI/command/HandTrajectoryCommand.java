package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.BaseForControl;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandTrajectoryCommand extends SE3TrajectoryControllerCommand<HandTrajectoryCommand, HandTrajectoryMessage>
{
   private RobotSide robotSide;

   public HandTrajectoryCommand()
   {
   }

   public HandTrajectoryCommand(ReferenceFrame referenceFrame, RobotSide robotSide)
   {
      super.clear(referenceFrame);
      this.robotSide = robotSide;
   }

   public HandTrajectoryCommand(ReferenceFrame referenceFrame, RobotSide robotSide, BaseForControl baseForControl)
   {
      // TODO: nuke this constructor once BaseForControl is no more.

      super.clear(referenceFrame);
      this.robotSide = robotSide;
   }

   @Override
   public void clear()
   {
      super.clear();
      robotSide = null;
   }

   @Override
   public void clear(ReferenceFrame referenceFrame)
   {
      super.clear(referenceFrame);
      robotSide = null;
   }

   @Override
   public void set(HandTrajectoryCommand other)
   {
      super.set(other);
      setPropertiesOnly(other);
   }

   /**
    * Same as {@link #set(HandTrajectoryCommand)} but does not change the trajectory points.
    * @param other
    */
   @Override
   public void setPropertiesOnly(HandTrajectoryCommand other)
   {
      super.setPropertiesOnly(other);
      robotSide = other.robotSide;
   }

   @Override
   public void set(HandTrajectoryMessage message)
   {
      super.set(message);
      this.robotSide = message.getRobotSide();
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
   public Class<HandTrajectoryMessage> getMessageClass()
   {
      return HandTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && super.isCommandValid();
   }

   public BaseForControl getBase()
   {
      // TODO: nuke this once BaseForControl is no more.

      return BaseForControl.WORLD;
   }
}
