package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandTrajectoryControllerCommand extends SE3TrajectoryControllerCommand<HandTrajectoryControllerCommand, HandTrajectoryMessage>
{
   private RobotSide robotSide;
   private BaseForControl baseForControl;

   public HandTrajectoryControllerCommand()
   {
   }

   public HandTrajectoryControllerCommand(ReferenceFrame referenceFrame, RobotSide robotSide, BaseForControl baseForControl)
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
   public void set(HandTrajectoryControllerCommand other)
   {
      super.set(other);
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
