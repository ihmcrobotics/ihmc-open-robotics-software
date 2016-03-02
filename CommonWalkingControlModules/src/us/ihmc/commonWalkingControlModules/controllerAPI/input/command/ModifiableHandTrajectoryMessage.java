package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class ModifiableHandTrajectoryMessage extends ModifiableSE3TrajectoryMessage<ModifiableHandTrajectoryMessage, HandTrajectoryMessage>
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

   @Override
   public void set(ModifiableHandTrajectoryMessage other)
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
}
