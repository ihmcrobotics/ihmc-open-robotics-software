package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandTrajectoryCommand extends SE3TrajectoryControllerCommand<HandTrajectoryCommand, HandTrajectoryMessage>
{
   private RobotSide robotSide;
   private BaseForControl baseForControl;

   public HandTrajectoryCommand()
   {
   }

   public HandTrajectoryCommand(ReferenceFrame referenceFrame, RobotSide robotSide, BaseForControl baseForControl)
   {
      super.clear(referenceFrame);
      this.robotSide = robotSide;
      this.baseForControl = baseForControl;
   }

   @Override
   public void clear()
   {
      super.clear();
      robotSide = null;
      baseForControl = null;
   }

   @Override
   public void clear(ReferenceFrame referenceFrame)
   {
      super.clear(referenceFrame);
      robotSide = null;
      baseForControl = null;
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
      baseForControl = other.baseForControl;
   }

   @Override
   public void set(HandTrajectoryMessage message)
   {
      super.set(message);
      this.robotSide = message.getRobotSide();
      this.baseForControl = message.getBase();
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void setBase(BaseForControl baseForControl)
   {
      this.baseForControl = baseForControl;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public BaseForControl getBase()
   {
      return baseForControl;
   }

   @Override
   public Class<HandTrajectoryMessage> getMessageClass()
   {
      return HandTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && baseForControl != null && super.isCommandValid();
   }
}
