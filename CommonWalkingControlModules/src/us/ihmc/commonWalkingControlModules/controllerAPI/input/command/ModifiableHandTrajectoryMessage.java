package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class ModifiableHandTrajectoryMessage extends ModifiableSE3TrajectoryMessage<HandTrajectoryMessage>
{
   private RobotSide robotSide;
   private BaseForControl baseForControl;

   public ModifiableHandTrajectoryMessage()
   {
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

   public void set(ModifiableHandTrajectoryMessage other)
   {
      super.set(other);
      robotSide = other.robotSide;
      baseForControl = other.baseForControl;
   }

   public void setIncludingFrame(ModifiableHandTrajectoryMessage other)
   {
      super.setIncludingFrame(other);
      robotSide = other.robotSide;
      baseForControl = other.baseForControl;
   }

   @Override
   public void set(HandTrajectoryMessage trajectoryMessage)
   {
      super.set(trajectoryMessage);
      this.robotSide = trajectoryMessage.getRobotSide();
      this.baseForControl = trajectoryMessage.getBase();
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
}
