package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.SoleTrajectoryMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

import java.util.Random;

public class SoleTrajectoryCommand implements Command<SoleTrajectoryCommand, SoleTrajectoryMessage>, FrameBasedCommand<SoleTrajectoryMessage>, EpsilonComparable<SoleTrajectoryCommand>
{
   private RobotQuadrant robotQuadrant;
   private final EuclideanTrajectoryControllerCommand positionTrajectory;

   public SoleTrajectoryCommand()
   {
      positionTrajectory = new EuclideanTrajectoryControllerCommand();
   }

   public SoleTrajectoryCommand(Random random)
   {
      positionTrajectory = new EuclideanTrajectoryControllerCommand(random);
      robotQuadrant = RobotQuadrant.generateRandomRobotQuadrant(random);
   }

   @Override
   public void clear()
   {
      positionTrajectory.clear();
      robotQuadrant = null;
   }

   @Override
   public void setFromMessage(SoleTrajectoryMessage message)
   {
      positionTrajectory.setFromMessage(message.getPositionTrajectory());
      robotQuadrant = RobotQuadrant.fromByte(message.getRobotQuadrant());
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, SoleTrajectoryMessage message)
   {
      positionTrajectory.set(resolver, message.getPositionTrajectory());
      robotQuadrant = RobotQuadrant.fromByte(message.getRobotQuadrant());
   }

   @Override
   public void set(SoleTrajectoryCommand other)
   {
      positionTrajectory.set(other.positionTrajectory);
      setPropertiesOnly(other);
   }

   /**
    * Same as {@link #set(SoleTrajectoryCommand)} but does not change the trajectory points.
    *
    * @param other
    */
   public void setPropertiesOnly(SoleTrajectoryCommand other)
   {
      positionTrajectory.setPropertiesOnly(other.positionTrajectory);
      robotQuadrant = other.robotQuadrant;
   }

   public void setRobotQuadrant(RobotQuadrant robotQuadrant)
   {
      this.robotQuadrant = robotQuadrant;
   }

   public RobotQuadrant getRobotQuadrant()
   {
      return robotQuadrant;
   }

   public EuclideanTrajectoryControllerCommand getPositionTrajectory()
   {
      return positionTrajectory;
   }

   @Override
   public Class<SoleTrajectoryMessage> getMessageClass()
   {
      return SoleTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotQuadrant != null && positionTrajectory.isCommandValid();
   }

   @Override
   public boolean epsilonEquals(SoleTrajectoryCommand other, double epsilon)
   {
      return robotQuadrant == other.robotQuadrant && positionTrajectory.epsilonEquals(other.positionTrajectory, epsilon);
   }

   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }

   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      positionTrajectory.setExecutionDelayTime(delayTime);
   }

   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      positionTrajectory.setExecutionTime(adjustedExecutionTime);
   }

   @Override
   public double getExecutionDelayTime()
   {
      return positionTrajectory.getExecutionDelayTime();
   }

   @Override
   public double getExecutionTime()
   {
      return positionTrajectory.getExecutionTime();
   }
}
