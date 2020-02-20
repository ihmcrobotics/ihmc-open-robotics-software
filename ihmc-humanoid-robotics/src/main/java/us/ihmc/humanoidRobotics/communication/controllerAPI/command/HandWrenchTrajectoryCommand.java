package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.HandWrenchTrajectoryMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

import java.util.Random;

public class HandWrenchTrajectoryCommand implements Command<HandWrenchTrajectoryCommand, HandWrenchTrajectoryMessage>, FrameBasedCommand<HandWrenchTrajectoryMessage>,
      EpsilonComparable<HandWrenchTrajectoryCommand>
{
   private long sequenceId;
   private RobotSide robotSide;
   private final WrenchTrajectoryControllerCommand wrenchTrajectory;

   public HandWrenchTrajectoryCommand()
   {
      wrenchTrajectory = new WrenchTrajectoryControllerCommand();
   }

   public HandWrenchTrajectoryCommand(RobotSide robotSide, ReferenceFrame dataFrame, ReferenceFrame trajectoryFrame)
   {
      robotSide = robotSide;
      wrenchTrajectory = new WrenchTrajectoryControllerCommand(dataFrame, trajectoryFrame);
   }

   public HandWrenchTrajectoryCommand(Random random)
   {
      wrenchTrajectory = new WrenchTrajectoryControllerCommand(random);
      robotSide = RobotSide.generateRandomRobotSide(random);
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      wrenchTrajectory.clear();
      robotSide = null;
   }

   public void clear(ReferenceFrame referenceFrame)
   {
      wrenchTrajectory.clear(referenceFrame);
      robotSide = null;
   }

   @Override
   public void set(HandWrenchTrajectoryCommand other)
   {
      sequenceId = other.sequenceId;
      wrenchTrajectory.set(other.wrenchTrajectory);
      robotSide = other.robotSide;
   }

   /**
    * Same as {@link #set(HandWrenchTrajectoryCommand)} but does not change the trajectory points.
    *
    * @param other
    */
   public void setPropertiesOnly(HandWrenchTrajectoryCommand other)
   {
      sequenceId = other.sequenceId;
      wrenchTrajectory.setPropertiesOnly(other.wrenchTrajectory);
      robotSide = other.robotSide;
   }

   @Override
   public void setFromMessage(HandWrenchTrajectoryMessage message)
   {
      FrameBasedCommand.super.setFromMessage(message);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, HandWrenchTrajectoryMessage message)
   {
      sequenceId = message.getSequenceId();
      wrenchTrajectory.set(resolver, message.getWrenchTrajectory());
      robotSide = RobotSide.fromByte(message.getRobotSide());
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public WrenchTrajectoryControllerCommand getWrenchTrajectory()
   {
      return wrenchTrajectory;
   }

   @Override
   public Class<HandWrenchTrajectoryMessage> getMessageClass()
   {
      return HandWrenchTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && wrenchTrajectory.isCommandValid();
   }

   @Override
   public boolean epsilonEquals(HandWrenchTrajectoryCommand other, double epsilon)
   {
      return robotSide == other.robotSide && wrenchTrajectory.epsilonEquals(other.wrenchTrajectory, epsilon);
   }

   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }

   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      wrenchTrajectory.setExecutionDelayTime(delayTime);
   }

   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      wrenchTrajectory.setExecutionTime(adjustedExecutionTime);
   }

   @Override
   public double getExecutionDelayTime()
   {
      return wrenchTrajectory.getExecutionDelayTime();
   }

   @Override
   public double getExecutionTime()
   {
      return wrenchTrajectory.getExecutionTime();
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
