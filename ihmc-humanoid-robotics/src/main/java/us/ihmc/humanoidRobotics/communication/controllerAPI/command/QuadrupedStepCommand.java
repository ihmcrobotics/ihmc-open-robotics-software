package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.QuadrupedStepMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedStepCommand implements Command<QuadrupedStepCommand, QuadrupedStepMessage>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private RobotQuadrant robotQuadrant;
   private FramePoint3D goalPosition = new FramePoint3D();
   private double groundClearance = 0.0;

   public QuadrupedStepCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      robotQuadrant = null;
      groundClearance = 0.0;
      goalPosition.setToNaN();
   }

   @Override
   public void set(QuadrupedStepMessage message)
   {
      robotQuadrant = RobotQuadrant.fromByte(message.getRobotQuadrant());
      groundClearance = message.getGroundClearance();
      goalPosition.setIncludingFrame(worldFrame, message.getGoalPosition());
   }

   @Override
   public void set(QuadrupedStepCommand other)
   {
      robotQuadrant = other.robotQuadrant;
      groundClearance = other.groundClearance;
      goalPosition.setIncludingFrame(other.goalPosition);
   }

   public RobotQuadrant getRobotQuadrant()
   {
      return robotQuadrant;
   }

   public double getGroundClearance()
   {
      return groundClearance;
   }

   public FramePoint3D getGoalPosition()
   {
      return goalPosition;
   }

   @Override
   public Class<QuadrupedStepMessage> getMessageClass()
   {
      return QuadrupedStepMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotQuadrant != null;
   }
}